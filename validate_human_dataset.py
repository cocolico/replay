#!/usr/bin/env python3
"""Prepare an isolated human dataset and perform read-only controller IK and mesh checks."""
import argparse
import concurrent.futures
import copy
from pathlib import Path
import time

import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from replay_pipeline import SIDES, config, prepare_camera, provenance, write_json
from diagnose_ik import diagnose
from device_backends import Aubo
from dh_kinematics import fk
from collision_scene import Scene


def solve_all(c, side, arrays):
    """Record every failed frame and discontinuity without producing an executable plan."""
    robot = Aubo(c['sides'][side])
    try:
        snap = robot.snapshot()
        if snap['robot_type'] != 'aubo_i7H' or not np.allclose(snap['tcp_offset_sdk'], 0):
            raise ValueError('Unexpected robot model or TCP')
        lo, hi = np.asarray(snap['joint_min_rad']), np.asarray(snap['joint_max_rad'])
        previous = np.asarray(c['sides'][side]['initial_q_rad'])
        qs = np.full((len(arrays['time_s']), 6), np.nan)
        rows, jumps = [], []
        for i, target in enumerate(arrays[f'{side}_aubo_base_T_flange']):
            errors = []
            for seed in (previous, np.asarray(c['sides'][side]['initial_q_rad']), np.zeros(6)):
                try:
                    q = robot.ik(target, seed)
                    for j in range(6):
                        choices = np.array([q[j] - 2*np.pi, q[j], q[j] + 2*np.pi])
                        choices = choices[(choices >= lo[j]) & (choices <= hi[j])]
                        if not len(choices):
                            raise ValueError('No in-limit joint winding')
                        q[j] = choices[np.argmin(abs(choices - previous[j]))]
                    def residual(x):
                        actual = fk(x, snap['kinematics_real'])
                        return np.r_[actual[:3, 3] - target[:3, 3], Rotation.from_matrix(target[:3, :3].T @ actual[:3, :3]).as_rotvec()]
                    q = least_squares(residual, q, bounds=(lo, hi), max_nfev=30,
                                      ftol=1e-12, xtol=1e-12, gtol=1e-12).x
                    actual = robot.fk(q)
                    pe = float(np.linalg.norm(actual[:3, 3] - target[:3, 3]))
                    ae = float(Rotation.from_matrix(actual[:3, :3].T @ target[:3, :3]).magnitude())
                    if pe > 1e-5 or ae > 1e-4:
                        raise ValueError(f'FK residual {pe}m/{ae}rad')
                    step = float(np.max(abs(q - previous)))
                    if i and np.all(np.isfinite(qs[i-1])) and step > .4:
                        jumps.append({'frame': i, 'time_s': float(arrays['time_s'][i]), 'max_joint_step_rad': step})
                    qs[i] = q
                    previous = q.copy()
                    rows.append({'frame': i, 'passed': True, 'position_error_m': pe, 'orientation_error_rad': ae})
                    break
                except Exception as error:
                    errors.append(str(error))
            else:
                rows.append({'frame': i, 'time_s': float(arrays['time_s'][i]), 'passed': False, 'errors': errors})
            if i % 50 == 0:
                print(side, 'controller IK', i, '/', len(qs), flush=True)
        return qs, {'snapshot': snap, 'checked_frames': len(qs), 'failed_frames': sum(not r['passed'] for r in rows),
                    'frames': rows, 'branch_discontinuities': jumps}
    finally:
        robot.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--out', required=True, type=Path)
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    c = config(args.config)
    prepared = out / 'human_camera_prepared.npz'
    prepare_camera(c, out)
    candidate = out / 'human_camera_ik_candidate.npz'
    diagnostic = diagnose(c, prepared, out / 'human_camera_report.json', 1)
    write_json(out / 'human_camera_prepared_ik_diagnostic.json', diagnostic)
    print('Nominal IK failed samples', {s: diagnostic['sides'][s]['failed_samples'] for s in SIDES}, flush=True)
    with np.load(prepared, allow_pickle=False) as z:
        arrays = {k: z[k].copy() for k in z.files}
    for side in SIDES:
        arrays[f'{side}_aubo_joint_positions_rad'] = np.tile(c['sides'][side]['initial_q_rad'], (len(arrays['time_s']), 1))
    arrays['artifact_type'] = np.asarray('offline_controller_ik_input_with_seed')
    arrays['execution_enabled'] = np.asarray(False)
    np.savez_compressed(candidate, **arrays)
    sides = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        pending = {s: pool.submit(solve_all, c, s, arrays) for s in SIDES}
        errors = {}
        for side in SIDES:
            try:
                joints, report = pending[side].result()
                arrays[f'{side}_aubo_joint_positions_rad'] = joints
                sides[side] = report
            except Exception as error:
                errors[side] = str(error)
    if errors:
        write_json(out / 'human_camera_controller_ik_failure.json', {
            'execution_enabled': False, 'passed': False, 'errors': errors,
            'configuration': provenance(c['_config_path']),
            'collision_check_performed': False})
        raise RuntimeError(f'Controller IK failed: {errors}')
    arrays['joint_positions_left_right_rad'] = np.concatenate([
        arrays['left_aubo_joint_positions_rad'], arrays['left_wuji_joint_positions_rad'],
        arrays['right_aubo_joint_positions_rad'], arrays['right_wuji_joint_positions_rad']], axis=1)
    arrays['artifact_type'] = np.asarray('controller_ik_offline_trajectory')
    arrays['execution_enabled'] = np.asarray(False)
    trajectory = out / 'human_camera_controller_ik.npz'
    np.savez_compressed(trajectory, **arrays)
    write_json(trajectory.with_suffix('.json'), {
        'execution_enabled': False, 'trajectory': provenance(trajectory),
        'source': provenance(candidate), 'configuration': provenance(c['_config_path']),
        'method': 'Read-only controller IK/FK with calibrated DH refinement', 'sides': sides,
        'runtime_acceptance': False})
    scene = Scene(c, {s: sides[s]['snapshot'] for s in SIDES})
    # A hand-only scene remains valid even when a frame has no verified arm IK.
    hand_scene = copy.copy(scene)
    hand_scene.shapes = [s for s in scene.shapes if s['kind'] == 'hand']
    hand_scene.pairs = [(i, j) for i in range(len(hand_scene.shapes)) for j in range(i+1, len(hand_scene.shapes))
                        if frozenset((hand_scene.shapes[i]['name'], hand_scene.shapes[j]['name'])) not in scene.disabled]
    models = {s: pin.buildModelFromUrdf(str(Path(c['source_episode_root']) / f'assets/wuji/urdf/{s}.urdf')) for s in SIDES}
    data = {s: models[s].createData() for s in SIDES}
    current_hand_poses = []
    hand_scene.shape_poses = lambda arms, hands: current_hand_poses
    failures = []
    hand_failures = []
    full_checked = 0
    minimum = None
    start = time.monotonic()
    for i, t in enumerate(arrays['time_s']):
        arms = {s: arrays[s + '_aubo_joint_positions_rad'][i] for s in SIDES}
        hands = {s: arrays[s + '_wuji_joint_positions_rad'][i] for s in SIDES}
        roots = {}
        for side in SIDES:
            model = models[side]
            q = np.zeros(model.nq)
            for name, value in zip(arrays[f'{side}_wuji_model_joint_names'], hands[side], strict=True):
                q[model.joints[model.getJointId(str(name))].idx_q] = value
            pin.forwardKinematics(model, data[side], q)
            pin.updateFramePlacements(model, data[side])
            roots[side] = scene.base[side] @ arrays[f'{side}_aubo_base_T_wuji_base'][i]
        current_hand_poses = [roots[s['side']] @ data[s['side']].oMf[models[s['side']].getFrameId(s['link'])].homogeneous @ s['offset'] for s in hand_scene.shapes]
        hand_findings = hand_scene.check(arms, hands, max_findings=10000)
        if hand_findings:
            hand_failures.append({'frame': i, 'time_s': float(t), 'findings': hand_findings})
        if all(np.all(np.isfinite(arms[s])) for s in SIDES):
            full_checked += 1
            findings = scene.check(arms, hands, max_findings=10000)
            if findings:
                failures.append({'frame': i, 'time_s': float(t), 'findings': findings})
        for shape, pose in zip(hand_scene.shapes, current_hand_poses, strict=True):
            vertices = shape['hull'] @ pose[:3, :3].T + pose[:3, 3]
            height = float(np.min(-(vertices - scene.plane_point) @ scene.normal))
            if minimum is None or height < minimum['clearance_m']:
                minimum = {'frame': i, 'time_s': float(t), 'body': shape['name'], 'clearance_m': height}
        if i % 50 == 0:
            print('Collision check', i, '/', len(arrays['time_s']), 'failed', len(failures), flush=True)
    result = {
        'trajectory': provenance(trajectory), 'configuration': provenance(c['_config_path']),
        'execution_enabled': False, 'passed': not failures and not hand_failures and full_checked == len(arrays['time_s']) and not any(sides[s]['branch_discontinuities'] for s in SIDES),
        'checked_frames': full_checked, 'total_frames': len(arrays['time_s']), 'duration_s': float(arrays['time_s'][-1]),
        'failed_frames': len(failures), 'failures': failures,
        'hand_only_checked_frames': len(arrays['time_s']), 'hand_only_failed_frames': len(hand_failures), 'hand_only_failures': hand_failures,
        'minimum_hand_table_clearance': minimum, 'clearance_m': scene.clearance,
        'scope': ['calibrated DH arm meshes', 'early-beta Wuji meshes',
                  'self/inter-arm/inter-hand', 'infinite calibrated table halfspace'],
        'limitations': ['Original samples only; start transition and resampled playback not checked.',
                        'Unmodeled scene objects are not checked.'],
        'elapsed_s': time.monotonic() - start}
    write_json(out / 'human_camera_collisions.json', result)
    print('FINISHED', 'passed', result['passed'], 'failed_frames', len(failures),
          'minimum_hand_table_clearance', minimum, flush=True)


if __name__ == '__main__':
    main()
