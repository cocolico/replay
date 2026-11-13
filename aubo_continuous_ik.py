"""Adapt the replay continuous/differential IK to controller-calibrated Aubo DH."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from replay_pipeline import HERE
from dh_kinematics import fk

REPLAY_SOURCE = HERE/'vendor/replay_ik/src'
sys.path.insert(0, str(REPLAY_SOURCE))
from eef_replay.ik.model import RobotModel
from eef_replay.ik.continuous import ContinuousIK
from eef_replay.ik.differential import DifferentialIK


def actual_model(snapshot, path):
    """Modified-DH: Tx(a) Rx(alpha) Rz(theta+q) Tz(d) Ry(beta)."""
    tree = ET.Element('robot', name='aubo_actual_dh')
    ET.SubElement(tree, 'link', name='base')
    parent = 'base'
    def add(name, child, kind, transform=None, index=None):
        nonlocal parent
        ET.SubElement(tree, 'link', name=child)
        j = ET.SubElement(tree, 'joint', name=name, type=kind)
        ET.SubElement(j, 'parent', link=parent)
        ET.SubElement(j, 'child', link=child)
        if transform is not None:
            ET.SubElement(j, 'origin', xyz=' '.join(map(str, transform[:3, 3])),
                          rpy=' '.join(map(str, Rotation.from_matrix(transform[:3, :3]).as_euler('xyz'))))
        if index is not None:
            ET.SubElement(j, 'axis', xyz='0 0 1')
            ET.SubElement(j, 'limit', lower=str(snapshot['joint_min_rad'][index]),
                          upper=str(snapshot['joint_max_rad'][index]), effort='100',
                          velocity=str(snapshot['joint_max_speed_rad_s'][index]))
        parent = child
    dh = snapshot['kinematics_real']
    for i in range(6):
        alpha, a, d, theta, beta = [float(dh[k][i]) for k in ('alpha', 'a', 'd', 'theta', 'beta')]
        x = np.eye(4);x[:3, :3] = Rotation.from_euler('x', alpha).as_matrix();x[0, 3] = a
        z = np.eye(4);z[:3, :3] = Rotation.from_euler('z', theta).as_matrix();z[2, 3] = d
        add(f'pre{i}', f'pre_link{i}', 'fixed', x@z)
        add(f'joint{i+1}', f'axis_link{i}', 'revolute', index=i)
        b = np.eye(4);b[:3, :3] = Rotation.from_euler('y', beta).as_matrix()
        add(f'beta{i}', 'flange' if i == 5 else f'link{i}', 'fixed', b)
    ET.ElementTree(tree).write(path, encoding='unicode')
    model = RobotModel(path, tuple(f'joint{i+1}' for i in range(6)))
    for q in [snapshot['joint_positions_rad'], np.zeros(6), np.linspace(-.8, 1., 6)]:
        np.testing.assert_allclose(model.fk('flange', q), fk(q, dh), atol=1e-10)
    return model


def cartesian_targets(times, targets, query):
    result = np.tile(np.eye(4), (len(query), 1, 1))
    result[:, :3, :3] = Slerp(times, Rotation.from_matrix(targets[:, :3, :3]))(query).as_matrix()
    for j in range(3):result[:, j, 3] = np.interp(query, times, targets[:, j, 3])
    return result


def wind_near(q, reference, lower, upper):
    result = np.asarray(q, dtype=float).copy()
    for j in range(6):
        values = result[j]+2*np.pi*np.arange(-3, 4)
        values = values[(values >= lower[j]) & (values <= upper[j])]
        if not len(values):raise ValueError('No in-limit winding')
        result[j] = values[np.argmin(abs(values-reference[j]))]
    return result


def track(model, targets, times, initial, backend='continuous', collision=None):
    solver = {'continuous':ContinuousIK, 'differential':DifferentialIK}[backend](model, 'flange', range(6))
    previous = model.check_q(initial).copy();rows = []
    for i, target in enumerate(targets):
        try:
            previous, result = solver.solve_near(target, previous, max_step=.12)
            if collision is not None:
                findings = collision(previous)
                if findings:raise ValueError('Modeled collision: '+str(findings))
        except (RuntimeError, ValueError) as error:
            return None, {'passed':False, 'failed_frame':i, 'source_time_s':float(times[i]), 'reason':str(error)}
        rows.append(previous.copy())
        if i and i%1000 == 0:print(backend, 'tracked', i, '/', len(times), flush=True)
    return np.asarray(rows), {'passed':True, 'frames':len(rows)}
