#!/usr/bin/env python3
"""Offline finger correction; preserve original roots, timestamps, and source files."""
import argparse
from pathlib import Path
import shutil
import time

import coal
import numpy as np
import pinocchio as pin
from scipy.optimize import minimize
from scipy.spatial import ConvexHull

from replay_pipeline import HERE, SIDES, config, read_json, write_json, hand_contract, provenance
from collision_scene import Scene, tf


class HandGeometry:
    def __init__(self, scene, side, urdf):
        self.model = pin.buildModelFromUrdf(str(urdf))
        self.data = self.model.createData()
        self.names, _, self.lo, self.hi = hand_contract(urdf, side)
        self.indices = [self.model.joints[self.model.getJointId(n)].idx_q for n in self.names]
        self.shapes = [s for s in scene.shapes if s['side'] == side and s['kind'] == 'hand']
        self.frames = [self.model.getFrameId(s['link']) for s in self.shapes]
        self.convex = []
        for shape in self.shapes:
            vertices = shape['hull']
            hull = ConvexHull(vertices)
            points, triangles = coal.StdVec_Vec3s(), coal.StdVec_Triangle()
            for point in vertices:
                points.append(point)
            for face, equation in zip(hull.simplices, hull.equations, strict=True):
                a, b, d = map(int, face)
                if np.dot(np.cross(vertices[b]-vertices[a], vertices[d]-vertices[a]), equation[:3]) < 0:
                    b, d = d, b
                triangles.append(coal.Triangle(a, b, d))
            self.convex.append(coal.Convex(points, triangles))
        self.pairs = [(i, j) for i in range(len(self.shapes)) for j in range(i+1, len(self.shapes))
                      if frozenset((self.shapes[i]['name'], self.shapes[j]['name'])) not in scene.disabled]
        self.request = coal.DistanceRequest()
        self.request.enable_signed_distance = True

    def matrices(self, q):
        p = np.zeros(self.model.nq)
        p[self.indices] = q
        pin.forwardKinematics(self.model, self.data, p)
        pin.updateFramePlacements(self.model, self.data)
        return [self.data.oMf[f].homogeneous @ s['offset'] for f, s in zip(self.frames, self.shapes)]

    def poses(self, q):
        return [tf(m) for m in self.matrices(q)]

    def mesh_findings(self, q):
        matrices = self.matrices(q)
        poses = [tf(m) for m in matrices]
        bounds = []
        for s, m in zip(self.shapes, matrices, strict=True):
            v = s['hull'] @ m[:3, :3].T + m[:3, 3]
            bounds.append((v.min(0), v.max(0)))
        request = coal.CollisionRequest()
        request.enable_contact = False
        request.num_max_contacts = 1
        findings = []
        for i, j in self.pairs:
            if np.any(bounds[i][0] > bounds[j][1]) or np.any(bounds[j][0] > bounds[i][1]):
                continue
            if coal.collide(self.shapes[i]['geometry'], poses[i], self.shapes[j]['geometry'], poses[j], request, coal.CollisionResult()):
                findings.append([self.shapes[i]['name'], self.shapes[j]['name']])
        return findings

    def distances(self, q):
        poses = self.poses(q)
        return np.array([coal.distance(self.convex[i], poses[i], self.convex[j], poses[j], self.request, coal.DistanceResult()) for i, j in self.pairs])


def feasible_fallback(geo, target, seeds, margin):
    """Find a feasible segment endpoint, then approach the desired pose from it."""
    best = None
    for seed in seeds:
        if geo.distances(seed).min() < margin:
            continue
        low, high = 0., 1.
        for _ in range(25):
            middle = (low+high)/2
            q = seed + middle*(target-seed)
            if geo.distances(q).min() >= margin:
                low = middle
            else:
                high = middle
        q = seed + low*(target-seed)
        if best is None or np.linalg.norm(q-target) < np.linalg.norm(best-target):
            best = q
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--controller-report', type=Path, required=True)
    parser.add_argument('--out-data', type=Path, required=True)
    parser.add_argument('--out-report', type=Path, required=True)
    parser.add_argument('--margin', type=float, default=.001)
    args = parser.parse_args()
    c = config(args.config)
    source = Path(c['camera_trajectory_root'])
    dest = args.out_data.resolve()
    if dest == source.resolve() or dest.exists():
        raise ValueError('Destination must be a new directory distinct from the original')
    report = read_json(args.controller_report)
    scene = Scene(c, {s: report['sides'][s]['snapshot'] for s in SIDES})
    dest.mkdir(parents=True)
    for path in source.glob('*.npy'):
        shutil.copy2(path, dest / path.name)
    summary = {'execution_enabled': False, 'original_directory': str(source), 'corrected_directory': str(dest),
               'method': 'SLSQP nearest-joint correction with convex-hull signed distance constraints',
               'constraint_margin_m': args.margin, 'root_poses_and_time_unchanged': True, 'sides': {}}
    start = time.monotonic()
    for side in SIDES:
        geo = HandGeometry(scene, side, Path(c['source_episode_root']) / f'assets/wuji/urdf/{side}.urdf')
        original = np.load(source / f'{side}_joint_positions_rad.npy', allow_pickle=False)
        source_names = np.load(source / f'{side}_joint_names.npy', allow_pickle=False).tolist()
        order = [source_names.index(n) for n in geo.names]
        q0 = original[:, order]
        corrected = q0.copy()
        failures, rows = [], []
        for i, target in enumerate(q0):
            if geo.distances(target).min() >= args.margin:
                rows.append({'frame': i, 'changed': False})
                continue
            previous = corrected[i-1] if i else target
            best = None
            # Start from the previous corrected frame and the original pose;
            # both are local seeds, with no claim of global optimality.
            for seed in (previous, target, .5*target, np.zeros(20)):
                objective = lambda q: np.sum((q-target)**2) + .03*np.sum((q-previous)**2)
                gradient = lambda q: 2*(q-target) + .06*(q-previous)
                result = minimize(objective, np.clip(seed, geo.lo+1e-7, geo.hi-1e-7), jac=gradient,
                                  bounds=list(zip(geo.lo+1e-7, geo.hi-1e-7)),
                                  constraints=[{'type': 'ineq', 'fun': lambda q: geo.distances(q)-args.margin}],
                                  method='SLSQP', options={'maxiter': 100, 'ftol': 1e-8})
                minimum = float(geo.distances(result.x).min())
                if minimum >= args.margin - 1e-6:
                    best = result.x.copy()
                    break
            if best is None:
                best = feasible_fallback(geo, target, [previous, np.zeros(20)], args.margin)
            if best is None:
                failures.append(i)
            else:
                corrected[i] = best
            rows.append({'frame': i, 'changed': best is not None,
                         'max_change_rad': float(np.max(abs(corrected[i]-target))),
                         'minimum_convex_distance_m': float(geo.distances(corrected[i]).min())})
            if i % 10 == 0:
                print(side, i, '/', len(q0), 'failures', len(failures), flush=True)
        updated = original.copy()
        updated[:, order] = corrected
        np.save(dest / f'{side}_joint_positions_rad.npy', updated)
        summary['sides'][side] = {'failed_frames': failures, 'changed_frames': int(np.any(abs(corrected-q0)>1e-8, axis=1).sum()),
                                 'max_change_rad': float(np.max(abs(corrected-q0))),
                                 'rms_change_rad': float(np.sqrt(np.mean((corrected-q0)**2))), 'frames': rows}
        write_json(args.out_report, summary)
        print('FINISHED', side, summary['sides'][side]['changed_frames'], 'modified frames', flush=True)
    summary['elapsed_s'] = time.monotonic()-start
    summary['all_frame_constraints_passed'] = all(not summary['sides'][s]['failed_frames'] for s in SIDES)
    summary['source_configuration'] = provenance(args.config)
    summary['acceptance'] = 'Requires independent exact mesh and interpolated-time validation; not authorized for execution.'
    write_json(args.out_report, summary)


if __name__ == '__main__':
    main()
