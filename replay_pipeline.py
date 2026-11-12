#!/usr/bin/env python3
"""Offline conversion. No device SDK, publishers, or motion commands are imported."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
SIDES = ('left', 'right')
ALIGNMENT = 'wuji_beta_local_z_left_p90_right_m90_v1'


def default_config(name=None):
    return HERE/'config'/('human_replay.json' if name in ('human','human_camera','prepare-camera') else 'replay.json')


def read_json(path):
    return json.loads(Path(path).read_text())


def write_json(path, data):
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def provenance(path):
    p = Path(path).resolve()
    return {'path': str(p), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}


def transform(value, name='transform'):
    a = np.asarray(value, dtype=float)
    if a.shape[-2:] != (4, 4) or not np.all(np.isfinite(a)):
        raise ValueError(f'{name}: expected finite (...,4,4)')
    if not np.allclose(a[..., 3, :], [0, 0, 0, 1], atol=1e-8):
        raise ValueError(f'{name}: invalid bottom row')
    r = a[..., :3, :3]
    if not np.allclose(np.swapaxes(r, -1, -2) @ r, np.eye(3), atol=1e-6) or not np.allclose(np.linalg.det(r), 1, atol=1e-6):
        raise ValueError(f'{name}: invalid SO(3) rotation')
    return a


def inverse(a):
    a = transform(a)
    out = np.zeros_like(a)
    out[..., :3, :3] = np.swapaxes(a[..., :3, :3], -1, -2)
    out[..., :3, 3] = -np.einsum('...ij,...j->...i', out[..., :3, :3], a[..., :3, 3])
    out[..., 3, 3] = 1
    return out


def config(path):
    path = Path(path).resolve()
    c = read_json(path)
    if c.get('schema_version') != 1:
        raise ValueError('Unsupported config schema')
    def resolve(v):
        p = Path(v)
        return str(p if p.is_absolute() else (path.parent / p).resolve())
    for k in ('source_episode_root', 'table_mapping', 'current_table_observation', 'source_franka_handeye', 'camera_trajectory_root', 'camera_table_mapping', 'camera_source_observation'):
        if c.get(k): c[k] = resolve(c[k])
    for side in SIDES:
        for k in ('handeye', 'urdf'): c['sides'][side][k] = resolve(c['sides'][side][k])
    c['_config_path'] = str(path)
    return c


def ordered(q, actual_names, expected_names):
    names = list(map(str, actual_names))
    if len(names) != len(set(names)) or set(names) != set(expected_names):
        raise ValueError('Joint names missing, extra, or duplicated')
    a = np.asarray(q, dtype=float)
    if a.ndim != 2 or a.shape[1] != len(names) or not np.all(np.isfinite(a)):
        raise ValueError('Invalid joint array')
    return a[:, [names.index(n) for n in expected_names]]


def check_times(t, ns):
    t, ns = np.asarray(t, dtype=float), np.asarray(ns)
    if t.ndim != 1 or len(t) < 2 or ns.shape != t.shape or not np.all(np.isfinite(t)) or not np.all(np.diff(t) > 0) or not np.all(np.diff(ns) > 0):
        raise ValueError('Invalid timestamps')
    if not np.allclose(t - t[0], (ns - ns[0]) * 1e-9, rtol=0, atol=1e-6):
        raise ValueError('Relative time and timestamps disagree')
    return t - t[0]


def origin(joint):
    o = joint.find('origin')
    t = np.eye(4)
    if o is not None:
        t[:3, 3] = np.fromstring(o.get('xyz', '0 0 0'), sep=' ')
        t[:3, :3] = Rotation.from_euler('xyz', np.fromstring(o.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    return t


def franka_fk(assembly, side, q, names):
    joints = ET.parse(assembly).getroot().findall('joint')
    by_child = {j.find('child').get('link'): j for j in joints}
    link, base, chain = f'{side}_hand_C_MC', f'{side}_fr3v2_link0', []
    while link != base:
        if link not in by_child: raise ValueError(f'No chain from {base} to hand')
        j = by_child[link]
        chain.insert(0, j)
        link = j.find('parent').get('link')
    expected = [f'{side}_fr3_joint{i}' for i in range(1, 8)]
    q = ordered(q, names, expected)
    out = np.tile(np.eye(4), (len(q), 1, 1))
    used = []
    for j in chain:
        out = out @ origin(j)
        if j.get('type') == 'fixed': continue
        if j.get('type') != 'revolute': raise ValueError('Unexpected source joint type')
        name = j.get('name').replace('_fr3v2_joint', '_fr3_joint')
        index = expected.index(name)
        used.append(index)
        axis = np.fromstring(j.find('axis').get('xyz'), sep=' ')
        rot = np.tile(np.eye(4), (len(q), 1, 1))
        rot[:, :3, :3] = Rotation.from_rotvec(q[:, index, None] * axis).as_matrix()
        out = out @ rot
    if sorted(used) != list(range(7)): raise ValueError('Source chain is not exactly seven named joints')
    return transform(out)


def hand_contract(urdf, side):
    fingers = ('thumb', 'index_finger', 'middle_finger', 'ring_finger', 'pinky')
    names = []
    for f in fingers:
        suffix = ('cmc_flex', 'cmc_abd', 'mcp', 'ip') if f == 'thumb' else ('mcp_flex', 'mcp_abd', 'pip', 'dip')
        names += [f'{side[0]}_{f}_{s}' for s in suffix]
    by_name = {j.get('name'): j for j in ET.parse(urdf).getroot().findall('joint')}
    lo, hi = [], []
    for n in names:
        limit = by_name[n].find('limit')
        lo.append(float(limit.get('lower')))
        hi.append(float(limit.get('upper')))
    wire = [f'{side}_finger{f}_joint{j}' for f in range(1, 6) for j in range(1, 5)]
    return names, wire, np.asarray(lo), np.asarray(hi)


def dynamics(q, t):
    velocity = np.diff(q, axis=0) / np.diff(t)[:, None]
    acceleration = np.diff(velocity, axis=0) / ((np.diff(t)[:-1] + np.diff(t)[1:]) * .5)[:, None]
    return {'max_step_rad': float(np.max(np.abs(np.diff(q, axis=0)))),
            'max_velocity_rad_s': float(np.max(np.abs(velocity))),
            'max_acceleration_rad_s2': float(np.max(np.abs(acceleration))) if len(acceleration) else 0.0}


def calibrated_chain(c, mapping_path=None, source_camera_id='franka-sharpa'):
    m = read_json(mapping_path or c['table_mapping'])
    if m.get('source_camera_id') != source_camera_id or m.get('target_camera_id') != 'current-table-20261112':
        raise ValueError('Wrong source/target table mapping')
    obs = read_json(c['current_table_observation'])
    if obs.get('board_fingerprint') != m.get('board_fingerprint'):
        raise ValueError('Table board identity differs')
    mapping = transform(m['target_camera_T_source_camera'], 'camera mapping')
    transform(m['source_camera_T_target_camera'])
    if not np.allclose(mapping @ np.asarray(m['source_camera_T_target_camera']), np.eye(4), atol=1e-7):
        raise ValueError('Mapping inverse mismatch')
    out = {}
    for side in SIDES:
        h = read_json(c['sides'][side]['handeye'])
        if h.get('parent_frame') != f'{side}_aubo_base' or h.get('child_frame') != 'camera_color_optical_frame':
            raise ValueError('Wrong hand-eye frame')
        if h.get('mode') != 'eye_to_hand': raise ValueError('Expected fixed-camera eye-to-hand')
        if not np.allclose(h['metadata']['K'], obs['camera_intrinsics']['K'], atol=1e-5):
            raise ValueError('Table/hand-eye camera intrinsics differ')
        if h['metadata'].get('image_size') != [obs['camera_intrinsics']['height'], obs['camera_intrinsics']['width']]:
            raise ValueError('Table/hand-eye camera resolution differs')
        out[side] = transform(h['T_base_camera']) @ mapping
    return out


def prepare_episode(c, episode, output):
    ep = Path(c['source_episode_root']) / f'episode{episode}'
    assembly = ep.parent / 'assets/motion/source_assembly.urdf'
    files = [ep/'source_sharpa.npz', ep/'wuji_trajectory.npz', assembly, Path(c['_config_path']), Path(c['table_mapping']), Path(c['current_table_observation'])]
    with np.load(files[0], allow_pickle=False) as source, np.load(files[1], allow_pickle=False) as z:
        if str(z['alignment_version']) != ALIGNMENT: raise ValueError('Unknown root alignment; refusing implicit correction')
        t = check_times(z['time_s'], z['timestamp_ns'])
        if not np.array_equal(source['timestamp_ns'], z['timestamp_ns']): raise ValueError('Arm/hand clocks differ')
        base_T_source_camera = calibrated_chain(c)
        arrays = {'time_s': t, 'timestamp_ns': z['timestamp_ns'], 'alignment_version': z['alignment_version']}
        blockers = []
        if not c.get('source_assembly_mounts_match_recording'): blockers.append('source_franka_to_sharpa_mount_not_confirmed')
        old = read_json(c['source_franka_handeye']) if c.get('source_franka_handeye') else None
        if old is None: blockers.append('source_franka_camera_extrinsics_missing')
        elif not c.get('source_franka_handeye_matches_recording'): blockers.append('source_franka_camera_extrinsics_not_confirmed')
        if old is not None: files.append(Path(c['source_franka_handeye']))
        report = {'episode': str(episode), 'frames': len(t), 'duration_s': float(t[-1]), 'execution_enabled': False,
                  'artifact_type': 'offline_aubo_wuji_preparation', 'joint_unit': 'radian', 'position_unit': 'metre',
                  'configuration': provenance(c['_config_path']),
                  'mount_semantics': 'flange_T_wuji_base maps Wuji l/r_base_link into Aubo physical flange',
                  'root_alignment': 'preserved exactly; NO extra +/-90 degree rotation', 'sides': {}}
        for side in SIDES:
            cfg = c['sides'][side]
            hand_urdf = ep.parent / f'assets/wuji/urdf/{side}.urdf'
            files += [hand_urdf, Path(cfg['handeye'])]
            names, wire, lo, hi = hand_contract(hand_urdf, side)
            hand = ordered(z[f'{side}_wuji_joints_rad'], z[f'{side}_wuji_joint_names'], names)
            if hand.shape != (len(t),20): raise ValueError('Hand frame count mismatch')
            if np.any(hand < lo - 1e-6) or np.any(hand > hi + 1e-6): raise ValueError('Wuji position limit violation')
            fk = franka_fk(assembly, side, source[f'{side}_arm_q'], source[f'{side}_arm_names'])
            local = np.tile(np.eye(4), (len(t),1,1))
            local[:, :3, :3] = Rotation.from_rotvec(z[f'{side}_root_rotvec_in_sharpa_rad']).as_matrix()
            local[:, :3, 3] = z[f'{side}_root_xyz_in_sharpa_m']
            source_base_T_wuji = transform(fk @ local)
            arrays[f'{side}_source_franka_base_T_wuji_base'] = source_base_T_wuji
            arrays[f'{side}_wuji_joint_positions_rad'] = hand
            arrays[f'{side}_wuji_model_joint_names'] = np.asarray(names)
            arrays[f'{side}_wuji_wire_joint_names'] = np.asarray(wire)
            report['sides'][side] = {'hand_dynamics': dynamics(hand,t), 'hand_position_limits_passed': True,
                                     'target_handeye': provenance(cfg['handeye']), 'target_base_frame': f'{side}_aubo_base'}
            if old is not None:
                camera_T_base = transform(old['transforms'][f'camera_T_{side}_base'])
                arrays[f'{side}_source_camera_T_wuji_base'] = camera_T_base @ source_base_T_wuji
                arrays[f'{side}_aubo_base_T_wuji_base'] = base_T_source_camera[side] @ camera_T_base @ source_base_T_wuji
            mount = cfg.get('flange_T_wuji_base')
            if mount is None: blockers.append(f'{side}_flange_to_wuji_mount_missing')
            elif not cfg.get('mount_confirmed'): blockers.append(f'{side}_flange_to_wuji_mount_not_confirmed')
            if mount is not None:
                mount = transform(mount)
                if mount.shape != (4,4): raise ValueError('Mount must be one fixed 4x4 transform')
                key=f'{side}_aubo_base_T_wuji_base'
                if key in arrays: arrays[f'{side}_aubo_base_T_flange'] = arrays[key] @ inverse(mount)
            if not cfg.get('model_confirmed'): blockers.append(f'{side}_aubo_model_not_confirmed')
        report['blockers'] = blockers
        report['arm_ik_inputs_ready'] = not blockers
        report['arm_ik_performed'] = False
        report['collision_check'] = {'performed': False, 'scope_required': ['arms', 'hands', 'table', 'environment']}
        report['sources'] = [provenance(p) for p in files]
        output = Path(output);output.mkdir(parents=True,exist_ok=True)
        dest = output/f'episode{episode}_prepared.npz'
        np.savez_compressed(dest, **arrays)
        report['artifact'] = provenance(dest)
        write_json(output/f'episode{episode}_report.json',report)
        return report


def audit(c):
    chain = calibrated_chain(c)
    result = {'execution_enabled': False, 'calibrations': {}, 'episodes': {}}
    for side in SIDES:
        path = c['sides'][side]['handeye'];h = read_json(path)
        result['calibrations'][side] = {'source': provenance(path), 'created_unix_s': h['created_unix_s'],
            'held_out_validation': h['held_out_validation'], 'T_aubo_base_source_camera': chain[side].tolist()}
    for ep in ('594','615','616'):
        with np.load(Path(c['source_episode_root'])/f'episode{ep}'/'wuji_trajectory.npz',allow_pickle=False) as z:
            t=check_times(z['time_s'],z['timestamp_ns'])
            result['episodes'][ep]={'frames':len(t),'duration_s':float(t[-1]),'alignment_version':str(z['alignment_version'])}
    human=config(default_config('human_camera'))
    result['camera_stream'] = {'path':human['camera_trajectory_root'],'configuration':human['_config_path'],
        'source_camera_id':human['camera_source_id'], 'mapping':human['camera_table_mapping'],
        'source_observation':human['camera_source_observation'],'note':human['camera_source_identity_note']}
    return result


def prepare_camera(c, output):
    if not c.get('camera_source_observation'):raise ValueError('Human trajectories require an explicit source observation; use config/human_replay.json')
    root = Path(c['camera_trajectory_root'])
    files = []
    def load(name):
        path = root / (name + '.npy');files.append(path)
        return np.load(path, allow_pickle=False)
    if str(load('frame')) != 'undistorted_color_optical': raise ValueError('Unknown camera trajectory frame')
    if str(load('joint_unit')) != 'radian' or str(load('position_unit')) != 'metre' or str(load('quaternion_order')) != 'wxyz':
        raise ValueError('Unexpected camera trajectory units/convention')
    target_label = str(load('pose_target'))
    if 'l_base_link' not in target_label or 'r_base_link' not in target_label: raise ValueError('Unknown controlled frames')
    ns = load('timestamp_ns');t = check_times(load('time_s'), ns)
    matrices = calibrated_chain(c, c['camera_table_mapping'], c['camera_source_id'])
    if c.get('camera_source_observation'):
        source=read_json(c['camera_source_observation']);target=read_json(c['current_table_observation']);mapping=read_json(c['camera_table_mapping'])
        if source['camera_id']!=c['camera_source_id'] or source['board_fingerprint']!=target['board_fingerprint']:raise ValueError('Human source observation identity differs')
        expected=transform(target['camera_T_board'])@inverse(transform(source['camera_T_board']))
        if not np.allclose(mapping['target_camera_T_source_camera'],expected,atol=1e-8):raise ValueError('Human mapping does not match supplied source observation')
        if mapping.get('source_observation_sha256')!=provenance(c['camera_source_observation'])['sha256']:raise ValueError('Human source observation changed')
        if mapping.get('target_observation_sha256')!=provenance(c['current_table_observation'])['sha256']:raise ValueError('Human target observation changed')
        files.append(Path(c['camera_source_observation']))
    arrays = {'time_s':t, 'timestamp_ns':ns, 'alignment_version':load('alignment_version')}
    report = {'artifact_type':'offline_aubo_wuji_preparation', 'input_type':'human_camera', 'frames':len(t),
              'duration_s':float(t[-1]), 'execution_enabled':False, 'configuration':provenance(c['_config_path']),
              'source_camera_id':c['camera_source_id'], 'source_camera_identity_note':c['camera_source_identity_note'],
              'root_alignment':'Already fitted camera root; no +/-90 degree correction applied', 'sides':{}}
    blockers=[]
    for side in SIDES:
        cfg=c['sides'][side]
        poses=transform(load(side+'_root_pose'))
        if poses.shape!=(len(t),4,4):raise ValueError('Camera root frame count mismatch')
        position=load(side+'_position_m');quat=load(side+'_quaternion_wxyz')
        if position.shape!=(len(t),3) or quat.shape!=(len(t),4) or not np.all(np.isfinite(quat)) or not np.allclose(np.linalg.norm(quat,axis=1),1,atol=1e-6):raise ValueError('Invalid camera quaternion/position')
        if not np.allclose(poses[:,:3,3],position,atol=1e-8) or not np.allclose(poses[:,:3,:3],Rotation.from_quat(quat[:,[1,2,3,0]]).as_matrix(),atol=1e-7):
            raise ValueError('Camera matrix and quaternion disagree')
        hand_urdf=Path(c['source_episode_root'])/f'assets/wuji/urdf/{side}.urdf'
        names,wire,lo,hi=hand_contract(hand_urdf,side)
        hand=ordered(load(side+'_joint_positions_rad'),load(side+'_joint_names'),names)
        if hand.shape!=(len(t),20) or np.any(hand<lo-1e-6) or np.any(hand>hi+1e-6): raise ValueError('Camera hand frame count or position limits invalid')
        arrays[f'{side}_source_camera_T_wuji_base']=poses
        arrays[f'{side}_aubo_base_T_wuji_base']=matrices[side]@poses
        arrays[f'{side}_wuji_joint_positions_rad']=hand
        arrays[f'{side}_wuji_model_joint_names']=np.asarray(names)
        arrays[f'{side}_wuji_wire_joint_names']=np.asarray(wire)
        mount=cfg.get('flange_T_wuji_base')
        if mount is None:blockers.append(f'{side}_flange_to_wuji_mount_missing')
        else:
            mount=transform(mount)
            if mount.shape!=(4,4):raise ValueError('Mount must be a fixed 4x4 matrix')
            arrays[f'{side}_aubo_base_T_flange']=arrays[f'{side}_aubo_base_T_wuji_base']@inverse(mount)
            if not cfg.get('mount_confirmed'):blockers.append(f'{side}_flange_to_wuji_mount_not_confirmed')
        if not cfg.get('model_confirmed'):blockers.append(f'{side}_aubo_model_not_confirmed')
        report['sides'][side]={'hand_position_limits_passed':True,'hand_dynamics':dynamics(hand,t),'target_handeye':provenance(cfg['handeye'])}
        files.extend([hand_urdf,Path(cfg['handeye'])])
    files.extend([Path(c['_config_path']),Path(c['camera_table_mapping']),Path(c['current_table_observation'])])
    report.update(blockers=blockers,arm_ik_inputs_ready=not blockers,arm_ik_performed=False,collision_check={'performed':False},sources=[provenance(f) for f in files])
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    dest=output/'human_camera_prepared.npz';np.savez_compressed(dest,**arrays)
    report['artifact']=provenance(dest);write_json(output/'human_camera_report.json',report)
    return report


def solve_ik(c, prepared, report_path, output):
    report=read_json(report_path)
    if provenance(c['_config_path']) != report.get('configuration'): raise ValueError('Active configuration differs from preparation')
    if provenance(prepared)['sha256'] != report['artifact']['sha256']: raise ValueError('Preparation artifact hash mismatch')
    for p in report['sources']:
        if provenance(p['path'])['sha256'] != p['sha256']: raise ValueError('Preparation input changed: '+p['path'])
    if not report.get('arm_ik_inputs_ready') or report.get('blockers'): raise ValueError('Unresolved inputs: '+', '.join(report['blockers']))
    # Import only after coordinate and mounting checks succeed.
    import pinocchio as pin
    from scipy.optimize import least_squares
    with np.load(prepared,allow_pickle=False) as z:
        t=z['time_s'];out={k:z[k] for k in z.files};checks={}
        for side in SIDES:
            cfg=c['sides'][side]
            if not cfg.get('model_confirmed') or not cfg.get('mount_confirmed'): raise ValueError('Model/mount not confirmed')
            model=pin.buildModelFromUrdf(cfg['urdf']);data=model.createData()
            if model.nq!=6 or model.nv!=6 or not model.existFrame(cfg['flange_frame']): raise ValueError('Expected 6-axis Aubo and named flange frame')
            fid=model.getFrameId(cfg['flange_frame'])
            q=np.asarray(cfg['initial_q_rad'],dtype=float)
            if q.shape!=(6,) or not np.all(np.isfinite(q)): raise ValueError('Provide explicit six-joint initial_q_rad')
            lower=model.lowerPositionLimit;upper=model.upperPositionLimit
            if np.any(q<lower) or np.any(q>upper): raise ValueError('Seed outside joint limits')
            trajectory=[];errors=[]
            for i,target in enumerate(transform(z[f'{side}_aubo_base_T_flange'])):
                target_se3=pin.SE3(target[:3,:3],target[:3,3])
                def residual(x):
                    pin.forwardKinematics(model,data,x);pin.updateFramePlacements(model,data)
                    return pin.log6(data.oMf[fid].actInv(target_se3)).vector
                solved=least_squares(residual,q,bounds=(lower,upper),max_nfev=200,ftol=1e-10,xtol=1e-10,gtol=1e-10)
                error=residual(solved.x)
                if np.linalg.norm(error[:3])>0.001 or np.linalg.norm(error[3:])>0.01: raise ValueError(f'{side} IK failed at frame {i}')
                if i and np.max(np.abs(solved.x-q))>0.2: raise ValueError(f'{side} joint jump at frame {i}')
                q=solved.x;trajectory.append(q.copy());errors.append(error.copy())
            a=np.asarray(trajectory);out[f'{side}_aubo_joint_positions_rad']=a;out[f'{side}_aubo_joint_names']=np.asarray(list(model.names)[1:])
            v=np.diff(a,axis=0)/np.diff(t)[:,None]
            if np.any(np.abs(v)>model.velocityLimit+1e-6): raise ValueError(f'{side} URDF velocity violation; retiming required')
            e=np.asarray(errors);checks[side]={'dynamics':dynamics(a,t),'max_translation_log_error_m':float(np.max(np.linalg.norm(e[:,:3],axis=1))),
                'max_rotation_error_rad':float(np.max(np.linalg.norm(e[:,3:],axis=1))), 'model':provenance(cfg['urdf']),
                'acceleration_limits_checked':False}
    np.savez_compressed(output,**out)
    write_json(Path(output).with_suffix('.json'),{'execution_enabled':False,'artifact_type':'offline_aubo_wuji_ik',
        'trajectory':provenance(output),'sides':checks,'collision_check_performed':False,'controller_model_equivalence_verified':False,
        'note':'Not executable approval. Collision, controller limits, acceleration, timing, mounting and hardware validation still required.'})


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path)
    sub=p.add_subparsers(dest='command',required=True)
    a=sub.add_parser('audit');a.add_argument('--out',type=Path,default=HERE/'output/audit.json')
    a=sub.add_parser('prepare');a.add_argument('episodes',nargs='+',choices=['594','615','616']);a.add_argument('--out',type=Path,default=HERE/'output')
    a=sub.add_parser('prepare-camera');a.add_argument('--out',type=Path,default=HERE/'output')
    a=sub.add_parser('ik');a.add_argument('prepared',type=Path);a.add_argument('report',type=Path);a.add_argument('out',type=Path)
    args=p.parse_args();c=config(args.config or default_config(args.command))
    if args.command=='audit':
        result=audit(c);args.out.parent.mkdir(parents=True,exist_ok=True);write_json(args.out,result);print('Wrote',args.out)
    elif args.command=='prepare':
        for ep in args.episodes:
            r=prepare_episode(c,ep,args.out);print(f'episode{ep}: {r["frames"]} frames; IK ready={r["arm_ik_inputs_ready"]}; blockers={r["blockers"]}')
    elif args.command=='prepare-camera':
        r=prepare_camera(c,args.out);print(f'Human camera: {r["frames"]} frames; IK ready={r["arm_ik_inputs_ready"]}; blockers={r["blockers"]}')
    else: solve_ik(c,args.prepared,args.report,args.out)

if __name__=='__main__': main()
