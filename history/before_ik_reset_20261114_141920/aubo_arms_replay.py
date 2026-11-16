#!/usr/bin/env python3
"""Checked camera-trajectory to real Aubo arms. Default: read-only preparation."""
import argparse
import concurrent.futures
import copy
import os
from pathlib import Path
import time
import uuid
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from replay_pipeline import HERE, SIDES, config, read_json, write_json, prepare_camera, provenance
from device_backends import Aubo
from read_hands import Hands
from replay_control import acquire_replay_lock, read_arms, execute, require_start
from runtime_plan import check_controller, sample_replay, transition, dynamics, split, direct_transition
from collision_scene import Scene, DEFAULT_CLEARANCE_M
from dh_kinematics import fk
from aubo_continuous_ik import actual_model, cartesian_targets, wind_near, track
from collision_planning import PlanningFailure, combine_arm_graphs, coherent_winding
from aubo_cartesian_planner import plan_arm, validate_saved_geometry

SETTINGS = dict(rate_hz=100, speed=.2, minimum_transition_s=10., arm_velocity_fraction=.2,
                arm_acceleration_fraction=.15, feedback_timeout_s=.25, maximum_tick_lateness_s=.03,
                arm_tracking_error_rad=.2, fixed_hand_position_tolerance_rad=.03, warmup_s=2., mesh_clearance_m=DEFAULT_CLEARANCE_M)


def scoped_scene(scene, side):
    result = copy.copy(scene)
    result.shapes = [s for s in scene.shapes if s['side'] == side]
    result.pairs = [(i, j) for i in range(len(result.shapes)) for j in range(i+1, len(result.shapes))
                    if frozenset((result.shapes[i]['name'], result.shapes[j]['name'])) not in result.disabled]
    return result


def initial_solutions(c, side, target, model, count):
    """SDK discovers branches; actual-DH numerical refinement verifies every candidate."""
    arm = Aubo(c['sides'][side]);snapshot = arm.snapshot();current = np.asarray(snapshot['joint_positions_rad'])
    rng = np.random.default_rng(20261113)
    seeds = [current, (current+np.pi)%(2*np.pi)-np.pi, np.asarray(c['sides'][side]['initial_q_rad']), np.zeros(6)]
    seeds += list(rng.uniform(-np.pi, np.pi, (32, 6)))
    candidates = [];near_candidates = []
    try:
        for seed in seeds:
            try:
                q = arm.ik(target, seed)
                for reference in (np.zeros(6), current):
                    x = wind_near(q, reference, model.lower, model.upper)
                    def residual(v):
                        a = model.fk('flange', v)
                        return np.r_[a[:3, 3]-target[:3, 3], Rotation.from_matrix(target[:3, :3].T@a[:3, :3]).as_rotvec()]
                    fit = least_squares(residual, x, bounds=(model.lower, model.upper), max_nfev=100,
                                        ftol=1e-12, xtol=1e-12, gtol=1e-12)
                    x = fit.x;e = residual(x);sdk = arm.fk(x)
                    if np.linalg.norm(e[:3])>1e-5 or np.linalg.norm(e[3:])>1e-4:continue
                    if np.linalg.norm(sdk[:3, 3]-target[:3, 3])>1e-5 or Rotation.from_matrix(sdk[:3, :3].T@target[:3, :3]).magnitude()>1e-4:continue
                    bucket = candidates if np.array_equal(reference,np.zeros(6)) else near_candidates
                    if not any(np.max(abs(x-old))<.01 for old in bucket):bucket.append(x.copy())
            except (ValueError, RuntimeError):continue
        for x in near_candidates:
            if not any(np.max(abs(x-old))<.01 for old in candidates):candidates.append(x.copy())
        return candidates[:count]
    finally:arm.close()


def solve_side(c, side, arrays, controllers, hands, out, backend, branches):
    model = actual_model(controllers[side], out/f'{side}_actual_dh.urdf')
    times = np.unique(np.r_[arrays['time_s'], np.arange(arrays['time_s'][0], arrays['time_s'][-1], .01)])
    targets = cartesian_targets(arrays['time_s'], arrays[side+'_aubo_base_T_flange'], times)
    scene = scoped_scene(Scene(c, controllers), side)
    static_hands = {s:np.asarray(hands[s]['positions_rad']) for s in SIDES}
    def collision(q):
        arms = {s:np.asarray(controllers[s]['joint_positions_rad']) for s in SIDES};arms[side] = q
        return scene.check(arms, static_hands)
    candidates = initial_solutions(c, side, targets[0], model, branches)
    trials = [];successes = []
    for index, seed in enumerate(candidates):
        print(side, 'branch', index+1, '/', len(candidates), flush=True)
        qs, result = track(model, targets, times, seed, backend, collision)
        result['initial_q_rad'] = seed.tolist();trials.append(result)
        if qs is not None:
            # Preserve every successful branch for subsequent cross-arm checks.
            path = out/f'{side}_branch{index}_offline.npz'
            np.savez_compressed(path, time_s=times, joint_positions_rad=qs, execution_enabled=False)
            successes.append((times, qs, path))
        write_json(out/f'{side}_ik.json', {'backend':backend, 'candidate_count':len(candidates), 'trials':trials,
                                         'whole_trajectory_solutions':len(successes), 'execution_enabled':False})
    return successes


def runtime_pair(c, controllers, hands, left, right, settings, arrays):
    if settings.get('entry_planner','direct') != 'direct':
        raise ValueError('Independent entry planning has been removed; use direct transition')
    times, lq, _ = left;rt, rq, _ = right
    np.testing.assert_array_equal(times, rt)
    start_arm=np.r_[controllers['left']['joint_positions_rad'],controllers['right']['joint_positions_rad']]
    arm_lower=np.r_[controllers['left']['joint_min_rad'],controllers['right']['joint_min_rad']]
    arm_upper=np.r_[controllers['left']['joint_max_rad'],controllers['right']['joint_max_rad']]
    arm_path,winding_turns=coherent_winding(np.c_[lq,rq],start_arm,arm_lower,arm_upper)
    lq,rq=arm_path[:,:6],arm_path[:,6:]
    source = np.concatenate([lq, np.tile(hands['left']['positions_rad'], (len(times), 1)),
                             rq, np.tile(hands['right']['positions_rad'], (len(times), 1))], axis=1)
    start = np.concatenate([controllers['left']['joint_positions_rad'], hands['left']['positions_rad'],
                            controllers['right']['joint_positions_rad'], hands['right']['positions_rad']])
    lo = [];hi = [];vlim = [];alim = []
    for side in SIDES:
        s = controllers[side]
        lo.extend(s['joint_min_rad']);lo.extend([-np.inf]*20)
        hi.extend(s['joint_max_rad']);hi.extend([np.inf]*20)
        vlim.extend(np.asarray(s['joint_max_speed_rad_s'])*settings['arm_velocity_fraction']);vlim.extend([1.]*20)
        alim.extend(np.asarray(s['joint_max_acceleration_rad_s2'])*settings['arm_acceleration_fraction']);alim.extend([1.]*20)
    lo, hi, vlim, alim = map(np.asarray, (lo, hi, vlim, alim));speed = settings['speed'];rate = settings['rate_hz']
    for _ in range(16):
        ticks, replay = sample_replay(times, source, rate, speed);v, a = dynamics(replay, rate)
        if np.all(v<=vlim+1e-6) and np.all(a<=alim+1e-6):break
        speed *= .8
    else:raise ValueError('Unable to retime replay within arm velocity/acceleration limits')
    scene = Scene(c, controllers, settings['mesh_clearance_m'])
    ramp_t, ramp, entry_report = direct_transition(start, source[0], vlim, alim, settings)
    trajectory = np.concatenate([ramp, np.tile(source[0], (rate, 1)), replay[1:]])
    v, a = dynamics(trajectory, rate)
    if not np.isfinite(trajectory).all() or np.any(trajectory<lo-1e-6) or np.any(trajectory>hi+1e-6):raise ValueError('Runtime position limit violation')
    if np.any(v>vlim+1e-6) or np.any(a>alim+1e-6):raise ValueError('Combined runtime dynamic limit violation')
    bad = [];mesh_checked=0
    for i, row in enumerate(trajectory):
        mesh_checked+=1
        arms, fingers = split(row);findings = scene.check(arms, fingers)
        if findings:
            bad.append({'frame':i, 'runtime_time_s':i/rate, 'phase':'transition' if i<len(ramp) else 'replay', 'findings':findings})
            if len(bad)>=20:break
        if i%1000 == 0:print('full assembly mesh', i, '/', len(trajectory), flush=True)
    max_pe = None;max_ae = None
    if not bad:
        # Check interpolated joint commands against the requested Cartesian curve as well.
        query = np.minimum(times[-1], times[0]+ticks*speed)
        max_pe = 0.;max_ae = 0.
        for side in SIDES:
            desired = cartesian_targets(arrays['time_s'], arrays[side+'_aubo_base_T_flange'], query)
            for i, target in enumerate(desired):
                arms, _ = split(replay[i]);actual = fk(arms[side], controllers[side]['kinematics_real'])
                pe = np.linalg.norm(actual[:3, 3]-target[:3, 3]);ae = Rotation.from_matrix(actual[:3, :3].T@target[:3, :3]).magnitude()
                max_pe = max(max_pe, float(pe));max_ae = max(max_ae, float(ae))
        if max_pe>.001 or max_ae>.01:raise ValueError(f'Runtime interpolated EEF error {max_pe}m/{max_ae}rad')
    return trajectory, {'ready_for_execution':not bad, 'collision_checked':True, 'collision_passed':not bad,
                        'collision_failures':bad, 'replay_winding_turns':winding_turns.tolist(), 'entry_planning':entry_report, 'mesh_clearance_m':settings['mesh_clearance_m'], 'checked_mesh_samples':mesh_checked, 'effective_speed':speed,
                        'duration_s':(len(trajectory)-1)/rate, 'transition_s':float(ramp_t[-1]),
                        'max_velocity_rad_s':v.tolist(), 'max_acceleration_rad_s2':a.tolist(),
                        'max_replay_eef_position_error_m':max_pe, 'max_replay_eef_rotation_error_rad':max_ae, 'eef_checked':not bad}


def require_plan(c, plan, trajectory, reader):
    if not all(plan.get(k) is True for k in ('ready_for_execution', 'collision_checked', 'collision_passed')):
        raise ValueError('Preflight has not passed')
    if plan.get('arms_only') is not True:raise ValueError('Not an arms-only plan')
    if provenance(c['_config_path']) != plan['configuration']:raise ValueError('Configuration changed')
    for item in plan['sources']:
        if provenance(item['path']) != item:raise ValueError('Source or implementation changed: '+item['path'])
    if provenance(plan['trajectory']['path']) != plan['trajectory']:raise ValueError('Trajectory changed')
    with np.load(plan['trajectory']['path'], allow_pickle=False) as z:
        if not np.array_equal(z['joint_positions_left_right_rad'], trajectory):raise ValueError('Trajectory in memory differs')
    live=read_arms(c);fresh_hands=reader.snapshot()
    require_start(plan,live,fresh_hands)
    for side in SIDES:
        if np.max(abs(np.asarray(live[side]['joint_positions_rad'])-np.asarray(plan['controllers'][side]['joint_positions_rad'])))>.002:
            raise ValueError('Arm start drifted after planning; rerun preflight: '+side)
        if np.max(abs(np.asarray(fresh_hands[side]['positions_rad'])-np.asarray(plan['hand_start'][side]['positions_rad'])))>.005:
            raise ValueError('Fixed hand start drifted after planning; rerun preflight: '+side)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectory', type=Path)
    parser.add_argument('--config', type=Path, default=HERE/'config/human_replay.json')
    parser.add_argument('--backend', choices=['continuous', 'differential'], default='continuous')
    parser.add_argument('--branches', type=int, default=16)
    parser.add_argument('--waypoint-stride',type=int,default=1,help='Use every Nth source frame as a graph anchor; all original Cartesian constraints remain checked on edges')
    parser.add_argument('--planner', choices=['collision_graph','candidate_track'], default='collision_graph')
    parser.add_argument('--entry-planner', choices=['direct'], default='direct', help='Direct minimum-jerk transition from measured start to frame zero')
    parser.add_argument('--planned-path',type=Path,help='Recheck a saved offline Nx12 geometric path against the requested source and current scene; never reuses acceptance flags')
    parser.add_argument('--speed', type=float, default=.2)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if not 0<args.speed<=.2 or not 1<=args.branches<=32:parser.error('speed must be (0,0.2]; branches 1..32')
    if args.waypoint_stride<1:parser.error('waypoint stride must be a positive integer')
    if args.planned_path and args.waypoint_stride!=1:parser.error('--waypoint-stride applies to new graph searches, not --planned-path')
    if args.planner!='collision_graph' and args.waypoint_stride!=1:parser.error('--waypoint-stride requires --planner collision_graph')
    if not args.trajectory.is_dir():parser.error('trajectory must be a camera trajectory folder')
    lock = acquire_replay_lock();reader = None
    name = 'aubo_arms_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]
    out = HERE/'output'/name;out.mkdir();settings = dict(SETTINGS, speed=args.speed, path_planner=args.planner, entry_planner=args.entry_planner, edge_resolution_rad=.01,waypoint_stride=args.waypoint_stride)
    summary = {'ready_for_execution':False, 'execution_attempted':False, 'arms_only':True, 'name':name, 'path_planner':args.planner, 'entry_planner':args.entry_planner, 'waypoint_stride':args.waypoint_stride}
    def session(status, **extra):write_json(HERE/'output/replay_session.json', dict(pid=os.getpid(), name=name, status=status, updated_unix_s=time.time(), **extra))
    session('preflight');print('OUTPUT', out, flush=True)
    try:
        c = config(args.config)
        controllers = read_arms(c)
        for side in SIDES:check_controller(c, side, controllers[side])
        reader = Hands(read_json(HERE/'config/runtime.json')['hands']);hands = reader.snapshot()
        settings['hands'] = read_json(HERE/'config/runtime.json')['hands']
        c.pop('_config_path');c['camera_trajectory_root'] = str(args.trajectory.resolve())
        for side in SIDES:
            c['sides'][side]['model_confirmed'] = True
            c['sides'][side]['model_confirmation_source'] = 'Live SDK robot_type=aubo_i7H; actual DH used and FK verified'
        write_json(out/'config.json', c);c = config(out/'config.json')
        write_json(out/'planning_snapshot.json',{'controllers':controllers,'hands':hands,'read_only':True})
        prepared = prepare_camera(c, out)
        if prepared['blockers']:raise ValueError('Unresolved coordinate/mount inputs: '+str(prepared['blockers']))
        with np.load(out/'human_camera_prepared.npz', allow_pickle=False) as z:arrays = {k:z[k].copy() for k in z.files}
        sources = prepared['sources']+[provenance(p) for p in sorted(args.trajectory.glob('*.npy'))]
        sources += [provenance(p) for p in (Path(__file__), HERE/'aubo_continuous_ik.py', HERE/'replay_control.py', HERE/'collision_scene.py', HERE/'device_backends.py', HERE/'runtime_plan.py', HERE/'dh_kinematics.py', HERE/'read_hands.py', HERE/'replay_pipeline.py', HERE/'collision_planning.py', HERE/'aubo_cartesian_planner.py')]
        sources += [provenance(p) for p in sorted((HERE/'vendor/replay_ik/src/eef_replay/ik').glob('*.py'))]
        for side in SIDES:
            urdf = Path(c['sides'][side]['urdf'])
            sources += [provenance(urdf), provenance(urdf.with_suffix('.srdf'))]
        sources += [provenance(p) for p in sorted((HERE/'vendor/pyaubo_sdk_0_26').rglob('*.so'))]
        # Mesh files affect collision results and belong to the execution fingerprint.
        for root in (Path(c['sides']['left']['urdf']).parents[1]/'meshes/aubo_i7H', Path(c['source_episode_root'])/'assets/wuji'):
            sources += [provenance(p) for p in sorted(root.rglob('*.stl'))]
        sources.append(provenance(HERE/'requirements-planning.txt'))
        sources += [provenance(p) for p in sorted((HERE/'vendor/planning_python/networkx').rglob('*.py')) if p.is_file()]
        if args.planned_path:
            sources.append(provenance(args.planned_path))
            pairs=validate_saved_geometry(args.planned_path,arrays,c,controllers,hands,out,resolution_rad=settings['edge_resolution_rad'])
            summary['geometry_reused_from']=provenance(args.planned_path)
        else:
            solutions = {}
            for side in SIDES:
                if args.planner == 'collision_graph':
                    mesh = scoped_scene(Scene(c,controllers,settings['mesh_clearance_m']),side)
                    solutions[side] = plan_arm(c,side,arrays,controllers,hands,out,mesh,backend=args.backend,branches=args.branches,max_paths=8,edge_step_rad=settings['edge_resolution_rad'],waypoint_stride=args.waypoint_stride)
                else:
                    solutions[side] = solve_side(c, side, arrays, controllers, hands, out, args.backend, args.branches)
                if not solutions[side]:
                    summary['whole_trajectory_solutions'] = {s:len(v) for s,v in solutions.items()}
                    raise ValueError('No complete continuous collision-free solution on '+side+'; see '+side+'_ik.json')
            summary['whole_trajectory_solutions'] = {s:len(solutions[s]) for s in SIDES}
            if any(not solutions[s] for s in SIDES):raise ValueError('No complete continuous collision-free solution on both arms; see left_ik.json/right_ik.json')
            pairs=[]
            if args.planner == 'collision_graph':
                pair_scene=Scene(c,controllers,settings['mesh_clearance_m'])
                static={s:hands[s]['positions_rad'] for s in SIDES}
                def full_valid(q):return not pair_scene.check({'left':q[:6],'right':q[6:]},static)
                def pair_progress(i,n,k):
                    if i%20==0:print('dual-arm product graph',i,'/',n,'reachable',k,flush=True)
                try:
                    joint_paths,joint_report=combine_arm_graphs(solutions['left'].waypoint_times,solutions['left'].graph,solutions['right'].graph,full_valid,max_paths=8,resolution_rad=settings['edge_resolution_rad'],progress=pair_progress)
                except PlanningFailure as error:
                    write_json(out/'dual_arm_graph.json',error.report);raise
                write_json(out/'dual_arm_graph.json',joint_report)
                for i,(t,q) in enumerate(joint_paths):
                    dest=out/f'dual_graph_path{i}_offline.npz';np.savez_compressed(dest,time_s=t,joint_positions_rad=q,execution_enabled=False)
                    pairs.append(((t,q[:,:6],dest),(t,q[:,6:],dest)))
            else:pairs=[(left,right) for left in solutions['left'] for right in solutions['right']]
        attempts = [];chosen = None
        for left,right in pairs:
            try:trajectory, plan = runtime_pair(c, controllers, hands, left, right, settings, arrays)
            except (ValueError,PlanningFailure) as error:
                attempts.append({'ready_for_execution':False,'error':str(error),'planning_failure':getattr(error,'report',None)})
                write_json(out/'pair_checks.json',attempts);continue
            attempts.append(plan)
            if plan['ready_for_execution']:chosen = trajectory, plan;break
        write_json(out/'pair_checks.json', attempts)
        if chosen is None:raise ValueError('All branch pairs blocked by full assembly/transition checks')
        trajectory, plan = chosen
        # Final independent controller FK verification of both complete selected branches.
        for side, solution in zip(SIDES, (left, right)):
            arm = Aubo(c['sides'][side]);times, qs, path = solution
            try:
                targets = cartesian_targets(arrays['time_s'], arrays[side+'_aubo_base_T_flange'], times)
                turns=np.asarray(plan['replay_winding_turns'])[:6] if side=='left' else np.asarray(plan['replay_winding_turns'])[6:]
                for q, target in zip(qs, targets):
                    actual = arm.fk(q+2*np.pi*turns)
                    if np.linalg.norm(actual[:3, 3]-target[:3, 3])>1e-5 or Rotation.from_matrix(actual[:3, :3].T@target[:3, :3]).magnitude()>1e-4:raise ValueError('Controller FK validation failed: '+side)
            finally:arm.close()
        dest = out/'runtime.npz';np.savez_compressed(dest, joint_positions_left_right_rad=trajectory, execution_enabled=False)
        plan.update(name=name, arms_only=True, execution_enabled=False, configuration=provenance(c['_config_path']),
                    sources=sources, trajectory=provenance(dest), controllers=controllers, hand_start=hands, settings=settings)
        write_json(out/'runtime.json', plan);summary.update(ready_for_execution=True, runtime=provenance(out/'runtime.json'))
        print('PREFLIGHT PASS', out/'runtime.json', flush=True)
        if args.execute:
            require_plan(c, plan, trajectory, reader);summary['execution_attempted'] = True
            execute(c, settings, trajectory, plan, reader, arms_only=True)
        else:print('Read-only preflight complete. No movement commands sent.', flush=True)
    except BaseException as e:
        summary['error'] = str(e);print('FAILED:' if summary['execution_attempted'] else 'BLOCKED:', str(e), flush=True)
        raise
    finally:
        write_json(out/'summary.json', summary)
        # Do not overwrite a stop-not-confirmed state produced by execution cleanup.
        current = read_json(HERE/'output/replay_session.json')
        if current.get('pid') == os.getpid() and current.get('status') == 'preflight':session('stopped', error=summary.get('error'))
        if reader is not None:reader.close()
        lock.close()

if __name__ == '__main__':
    try:main()
    except (ValueError, RuntimeError):raise SystemExit(2)
    except KeyboardInterrupt:raise SystemExit(130)
