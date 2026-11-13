"""Check saved alignment, actions, keypoints, reports and repeat-update behavior."""
import argparse
import copy
import json
import numpy as np
from retarget import HERE, ALIGNMENT_VERSION, Hand, Rotation, apply_confirmed_alignment


def verify(episode):
    with np.load(episode/'wuji_trajectory.npz') as archive:
        z={key:archive[key].copy() for key in archive.files}
    report=json.loads((episode/'validation.json').read_text())
    assert str(z['alignment_version'])==ALIGNMENT_VERSION
    assert np.array_equal(z['alignment_local_z_deg_left_right'],[90,-90])
    max_fk=0.
    for side in ['left','right']:
        r=Rotation.from_rotvec(z[f'{side}_root_rotvec_in_sharpa_rad']).as_matrix()
        previous=Rotation.from_rotvec(z[f'{side}_unaligned_root_rotvec_in_sharpa_rad']).as_matrix()
        # Geometric sign check: +90 around local Z maps new X to old Y.
        np.testing.assert_allclose(r[:,:,2],previous[:,:,2],atol=1e-12)
        np.testing.assert_allclose(r[:,:,0],previous[:,:,1]*(1 if side=='left' else -1),atol=1e-12)
        hand=Hand('wuji',side)
        local=np.array([hand.fk(q) for q in z[f'{side}_wuji_joints_rad']])
        points=np.einsum('nij,nkj->nki',r,local)+z[f'{side}_root_xyz_in_sharpa_m'][:,None,:]
        max_fk=max(max_fk,float(abs(points-z[f'{side}_wuji_points_in_sharpa_m']).max()))
        np.testing.assert_allclose(points,z[f'{side}_wuji_points_in_sharpa_m'],atol=1e-9)
        errors=np.linalg.norm(points-z[f'{side}_source_points_m'],axis=-1)
        np.testing.assert_allclose(errors,z[f'{side}_error_m'],atol=1e-9)
        np.testing.assert_allclose(report['sides'][side]['tip_error_mm']['mean'],errors[:,[4,8,12,16,20]].mean()*1000,atol=1e-8)
        action=z[f'{side}_wuji_action_local']
        np.testing.assert_array_equal(action[:,:3],z[f'{side}_root_xyz_in_sharpa_m'])
        np.testing.assert_array_equal(action[:,3:6],z[f'{side}_root_rotvec_in_sharpa_rad'])
        np.testing.assert_array_equal(action[:,6:],z[f'{side}_wuji_joints_rad'])
    np.testing.assert_array_equal(np.load(episode/'wuji_actions_local_right_left.npy'),np.c_[z['right_wuji_action_local'],z['left_wuji_action_local']])
    np.testing.assert_array_equal(np.load(episode/'wuji_hand_joints_left_right.npy'),np.c_[z['left_wuji_joints_rad'],z['right_wuji_joints_rad']])
    repeated=copy.deepcopy(z);apply_confirmed_alignment(repeated,copy.deepcopy(report))
    for key,value in z.items():
        if value.dtype.kind=='f':np.testing.assert_allclose(repeated[key],value,atol=1e-12,rtol=0)
        else:np.testing.assert_array_equal(repeated[key],value)
    return dict(frames=len(z['time_s']),alignment_version=ALIGNMENT_VERSION,
                local_axis_and_sign_verified=True,repeat_update_idempotent=True,
                mujoco_points_max_abs_m=max_fk,actions_and_errors_consistent=True)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('episodes',nargs='*');args=ap.parse_args()
    results={name:verify(HERE/name) for name in (args.episodes or ['episode594','episode615','episode616'])}
    print(json.dumps(results,indent=2))
