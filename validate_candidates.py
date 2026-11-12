#!/usr/bin/env python3
"""Independent full-frame FK and data-preservation check for offline candidates."""
from pathlib import Path
import numpy as np
import pinocchio as pin
from replay_pipeline import HERE, SIDES, config, provenance, read_json, write_json,default_config


def main():
    c=config(HERE/'config/replay.json');summary={'execution_enabled':False,'candidate_mount_rotation_confirmed':all(c['sides'][s].get('mount_confirmed') for s in SIDES),'sources':{}}
    for name in ('episode594','episode615','episode616','human_camera'):
        c=config(default_config(name))
        path=HERE/'output'/f'{name}_ik_candidate.npz';prepared=HERE/'output'/f'{name}_prepared.npz'
        diagnostic=read_json(HERE/'output'/f'{name}_prepared_ik_diagnostic.json')
        if provenance(path)!=diagnostic['candidate_artifact']:raise ValueError('Candidate checksum mismatch')
        entry={'candidate':provenance(path),'sides':{}}
        with np.load(path,allow_pickle=False) as a,np.load(prepared,allow_pickle=False) as src:
            if bool(a['execution_enabled']):raise ValueError('Candidate incorrectly marked executable')
            if str(a['artifact_type'])!='nominal_model_ik_candidate':raise ValueError('Wrong artifact kind')
            for side in SIDES:
                np.testing.assert_array_equal(a[f'{side}_flange_T_wuji_base'],c['sides'][side]['flange_T_wuji_base'])
            for key in ('time_s','timestamp_ns'):
                np.testing.assert_array_equal(a[key],src[key])
            entry['frames']=len(a['time_s']);entry['duration_s']=float(a['time_s'][-1]);entry['combined_shape']=list(a['joint_positions_left_right_rad'].shape)
            if a['joint_positions_left_right_rad'].shape!=(entry['frames'],52) or len(set(a['joint_names_left_right']))!=52:raise ValueError('Wrong combined shape or names')
            for side in SIDES:
                np.testing.assert_array_equal(a[f'{side}_wuji_joint_positions_rad'],src[f'{side}_wuji_joint_positions_rad'])
                cfg=c['sides'][side];model=pin.buildModelFromUrdf(cfg['urdf']);data=model.createData();fid=model.getFrameId(cfg['flange_frame'])
                q=a[f'{side}_aubo_joint_positions_rad'];targets=a[f'{side}_aubo_base_T_flange']
                if not np.all(np.isfinite(q)) or np.any(q<model.lowerPositionLimit-1e-8) or np.any(q>model.upperPositionLimit+1e-8):raise ValueError('Bad candidate joint positions')
                max_position=0.;max_angle=0.
                for qi,target in zip(q,targets,strict=True):
                    pin.forwardKinematics(model,data,qi);pin.updateFramePlacements(model,data);actual=data.oMf[fid]
                    position=float(np.linalg.norm(actual.translation-target[:3,3]));angle=float(np.linalg.norm(pin.log3(actual.rotation.T@target[:3,:3])))
                    max_position=max(max_position,position);max_angle=max(max_angle,angle)
                if max_position>.001 or max_angle>.01:raise ValueError('Candidate FK does not match intended candidate flange target')
                entry['sides'][side]={'max_euclidean_position_error_m':max_position,'max_orientation_error_rad':max_angle,
                    'urdf_velocity_limits_passed':diagnostic['sides'][side]['urdf_velocity_limits_passed'],
                    'joint_dynamics':diagnostic['sides'][side]['dynamics']}
            rebuilt=np.concatenate([a['left_aubo_joint_positions_rad'],a['left_wuji_joint_positions_rad'],a['right_aubo_joint_positions_rad'],a['right_wuji_joint_positions_rad']],axis=1)
            np.testing.assert_array_equal(rebuilt,a['joint_positions_left_right_rad'])
        summary['sources'][name]=entry
        print(name,entry['frames'],'frames: independent FK, hand/time preservation, schema PASS')
    summary['mounts']={s:{'rotation_z_deg':c['sides'][s].get('mount_rotation_z_deg'),'flange_T_wuji_base':c['sides'][s]['flange_T_wuji_base']} for s in SIDES}
    summary['limitations']=['Right nominal model differs from controller FK by about 1.42 mm.','No collision validation or hardware motion.','Candidate dynamics are not full controller/hand acceptance.']
    write_json(HERE/'output/conversion_summary.json',summary)

if __name__=='__main__':main()
