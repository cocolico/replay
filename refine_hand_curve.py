#!/usr/bin/env python3
"""Insert hand-only transition knots and verify exact mesh samples. No device access."""
import argparse
from pathlib import Path

import numpy as np
from scipy.interpolate import PchipInterpolator, CubicHermiteSpline
from scipy.optimize import minimize

from replay_pipeline import HERE, SIDES, config, read_json, write_json, provenance
from collision_scene import Scene
from repair_human_hand_collision import HandGeometry, feasible_fallback
from runtime_plan import sample_replay


def interpolator(t, q):
    derivative = PchipInterpolator(t, q, axis=0).derivative()(t)
    derivative[[0, -1]] = 0
    return CubicHermiteSpline(t, q, derivative, axis=0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    c = config(args.config)
    controller_report = HERE/'output/human_camera_1/human_camera_controller_ik.json'
    snapshot = read_json(controller_report)
    scene = Scene(c, {s: snapshot['sides'][s]['snapshot'] for s in SIDES})
    root = Path(c['camera_trajectory_root'])
    original_t = np.load(root/'time_s.npy')
    geos = {s: HandGeometry(scene, s, Path(c['source_episode_root'])/f'assets/wuji/urdf/{s}.urdf') for s in SIDES}
    t = original_t.copy()
    q = {s: np.load(root/f'{s}_joint_positions_rad.npy') for s in SIDES}
    for side in SIDES:
        if np.load(root/f'{side}_joint_names.npy').tolist() != geos[side].names:
            raise ValueError('Unexpected joint order')
    history = []
    for iteration in range(9):
        failed = {}
        insertions = set()
        for side in SIDES:
            ticks, dense = sample_replay(t, q[side], 100, .2)
            bad = []
            for i, row in enumerate(dense):
                if geos[side].mesh_findings(row):
                    bad.append(i)
            failed[side] = bad
            by_interval = {}
            for i in bad:
                at = min(float(ticks[i]*.2), float(t[-1]))
                interval = int(np.clip(np.searchsorted(t, at)-1, 0, len(t)-2))
                by_interval.setdefault(interval, []).append(at)
            for values in by_interval.values():
                insertions.add(values[len(values)//2])
            print('iteration', iteration, side, 'dense failed', len(bad), '/', len(dense), flush=True)
        history.append({'iteration': iteration, 'knots': len(t), 'failed_samples': {s: len(failed[s]) for s in SIDES}})
        passed = not any(failed.values())
        curve = args.out/'hand_curve.npz'
        np.savez_compressed(curve, time_s=t, left_joint_positions_rad=q['left'], right_joint_positions_rad=q['right'],
                            left_joint_names=np.asarray(geos['left'].names), right_joint_names=np.asarray(geos['right'].names),
                            joint_unit=np.asarray('radian'),
                            execution_enabled=np.asarray(False), artifact_type=np.asarray('offline_repaired_hand_curve'))
        report = {'passed': passed, 'execution_enabled': False, 'scope': 'Finger self-collision, exact triangle meshes',
                  'rate_hz': 100, 'speed': .2, 'checked_samples_per_hand': len(dense), 'history': history,
                  'curve': provenance(curve), 'configuration': provenance(c['_config_path']),
                  'original_key_times_retained': bool(all(np.any(t == v) for v in original_t)),
                  'limitations': ['Hand-only curve; robot arm collision/IK continuity and live start transition remain blocked.']}
        write_json(args.out/'hand_curve_validation.json', report)
        if passed:
            print('HAND CURVE PASSED', len(t), 'knots', flush=True)
            return
        if iteration == 8:
            break
        new_t = np.unique(np.r_[t, list(insertions)])
        new_q = {}
        for side in SIDES:
            geo = geos[side]
            values = interpolator(t, q[side])(new_t)
            for i, at in enumerate(new_t):
                if np.any(t == at):
                    values[i] = q[side][np.flatnonzero(t == at)[0]]
                    continue
                target = values[i].copy()
                if geo.distances(target).min() >= .001:
                    continue
                j = int(np.clip(np.searchsorted(t, at)-1, 0, len(t)-2))
                best = None
                for seed in [q[side][j], q[side][j+1], np.zeros(20)]:
                    result = minimize(lambda x: np.sum((x-target)**2), seed,
                                      jac=lambda x: 2*(x-target), method='SLSQP',
                                      bounds=list(zip(geo.lo+1e-7, geo.hi-1e-7)),
                                      constraints=[{'type': 'ineq', 'fun': lambda x: geo.distances(x)-.0011}],
                                      options={'maxiter': 100, 'ftol': 1e-8})
                    if geo.distances(result.x).min() >= .001:
                        best = result.x.copy()
                        break
                if best is None:
                    best = feasible_fallback(geo, target, [q[side][j], q[side][j+1], np.zeros(20)], .001)
                if best is None:
                    raise RuntimeError(f'No feasible hand knot at {side} {at}')
                values[i] = best
            new_q[side] = values
        t, q = new_t, new_q
    raise RuntimeError('Adaptive hand curve still has collisions; inspect report')


if __name__ == '__main__':
    main()
