#!/usr/bin/env python3
"""Offline sampled IK feasibility for candidate mounts. Never outputs executable trajectories."""
import argparse
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from replay_pipeline import HERE, SIDES, config, transform, provenance, read_json, write_json, dynamics, default_config


def diagnose(c, artifact, report_path, stride, candidate=None):
    import pinocchio as pin
    source=read_json(report_path)
    if source['artifact'] != provenance(artifact):raise ValueError('Artifact changed')
    if source['configuration'] != provenance(c['_config_path']):raise ValueError('Configuration changed')
    for item in source['sources']:
        if provenance(item['path'])!=item:raise ValueError('Source changed: '+item['path'])
    result={'artifact_type':'nominal_ik_diagnostic','execution_enabled':False,'source':provenance(artifact),
            'configuration':provenance(c['_config_path']),'unresolved_inputs':source['blockers'],
            'mounts':{s:{'confirmed':bool(c['sides'][s].get('mount_confirmed')),'flange_T_wuji_base':c['sides'][s]['flange_T_wuji_base']} for s in SIDES},
            'collision_check_performed':False,'sides':{}}
    with np.load(artifact,allow_pickle=False) as z:
        candidate_arrays={k:z[k] for k in z.files} if candidate else None
        indices=sorted(set(range(0,len(z['time_s']),stride))|{len(z['time_s'])-1})
        result['sampled_frames']=len(indices);result['total_frames']=len(z['time_s'])
        for side in SIDES:
            cfg=c['sides'][side];model=pin.buildModelFromUrdf(cfg['urdf']);data=model.createData()
            frame=model.getFrameId(cfg['flange_frame']);q=np.asarray(cfg['initial_q_rad']);rows=[];trajectory=[]
            lower,upper=model.lowerPositionLimit,model.upperPositionLimit
            for i in indices:
                T=transform(z[f'{side}_aubo_base_T_flange'][i]);target=pin.SE3(T[:3,:3],T[:3,3])
                def residual(x):
                    pin.forwardKinematics(model,data,x);pin.updateFramePlacements(model,data)
                    return pin.log6(data.oMf[frame].actInv(target)).vector
                seeds=[q]
                if not rows:seeds += [np.zeros(6),np.array([0.,-.8,1.5,-.7,1.5,0.])]
                best=None
                for seed in seeds:
                    r=least_squares(residual,np.clip(seed,lower+1e-9,upper-1e-9),bounds=(lower,upper),max_nfev=100,ftol=1e-9,gtol=1e-9,xtol=1e-9)
                    e=residual(r.x).copy()
                    score=float(np.linalg.norm(e))
                    if best is None or score<best[0]:best=(score,r.x.copy(),e)
                    if np.linalg.norm(e[:3])<.001 and np.linalg.norm(e[3:])<.01:break
                _,q,e=best
                trajectory.append(q.copy())
                rows.append({'frame':i,'time_s':float(z['time_s'][i]),'translation_log_error_m':float(np.linalg.norm(e[:3])),
                    'rotation_error_rad':float(np.linalg.norm(e[3:])),'converged':bool(np.linalg.norm(e[:3])<.001 and np.linalg.norm(e[3:])<.01)})
            result['sides'][side]={'urdf':provenance(cfg['urdf']),'converged_samples':sum(r['converged'] for r in rows),
                'failed_samples':sum(not r['converged'] for r in rows),'samples':rows,
                'interpretation':'Local numerical IK only. Failed samples do not prove global unreachability. Full-frame mode reports step/velocity diagnostics; acceleration acceptance and collision checking remain absent.'}
            if stride==1:
                a=np.asarray(trajectory);dt=np.diff(z['time_s'])
                result['sides'][side]['dynamics']=dynamics(a,z['time_s'])
                result['sides'][side]['urdf_velocity_limits_passed']=bool(np.all(np.abs(np.diff(a,axis=0)/dt[:,None])<=model.velocityLimit+1e-6))
                result['sides'][side]['max_joint_step_under_0p2_rad']=bool(np.max(np.abs(np.diff(a,axis=0)))<=.2)
                if candidate_arrays is not None:
                    candidate_arrays[f'{side}_aubo_joint_positions_rad']=a
                    candidate_arrays[f'{side}_aubo_joint_names']=np.asarray(list(model.names)[1:])
        if candidate:
            if stride!=1 or any(result['sides'][s]['failed_samples'] for s in SIDES):raise ValueError('Candidate export requires full-frame convergence on both sides')
            candidate_arrays['execution_enabled']=np.asarray(False)
            candidate_arrays['artifact_type']=np.asarray('nominal_model_ik_candidate')
            for side in SIDES:
                candidate_arrays[f'{side}_flange_T_wuji_base']=np.asarray(c['sides'][side]['flange_T_wuji_base'])
                candidate_arrays[f'{side}_mount_confirmed']=np.asarray(bool(c['sides'][side].get('mount_confirmed')))
            candidate_arrays['joint_positions_left_right_rad']=np.concatenate([
                candidate_arrays['left_aubo_joint_positions_rad'],candidate_arrays['left_wuji_joint_positions_rad'],
                candidate_arrays['right_aubo_joint_positions_rad'],candidate_arrays['right_wuji_joint_positions_rad']],axis=1)
            candidate_arrays['joint_names_left_right']=np.asarray(
                ['left_'+str(n) for n in candidate_arrays['left_aubo_joint_names']]+list(candidate_arrays['left_wuji_wire_joint_names'])+
                ['right_'+str(n) for n in candidate_arrays['right_aubo_joint_names']]+list(candidate_arrays['right_wuji_wire_joint_names']))
            candidate_arrays['joint_group_order']=np.asarray(['left_arm_6','left_hand_20','right_arm_6','right_hand_20'])
            np.savez_compressed(candidate,**candidate_arrays)
            result['candidate_artifact']=provenance(candidate)
            result['candidate_note']='OFFLINE CANDIDATE ONLY: configured mounting, nominal URDF, collision not checked, current-state transition absent. See mounts and unresolved_inputs.'
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('prepared',type=Path);p.add_argument('report',type=Path);p.add_argument('--stride',type=int,default=30);p.add_argument('--config',type=Path);p.add_argument('--out',type=Path);p.add_argument('--candidate-out',type=Path)
    a=p.parse_args()
    if a.stride<1:p.error('--stride must be positive')
    r=diagnose(config(a.config or default_config("human_camera" if a.prepared.stem.startswith("human_camera") else None)),a.prepared,a.report,a.stride,a.candidate_out)
    dest=a.out or a.prepared.with_name(a.prepared.stem+'_ik_diagnostic.json');write_json(dest,r)
    print(a.prepared.name,'sampled frames',r['sampled_frames'],{s:r['sides'][s]['converged_samples'] for s in SIDES},flush=True)

if __name__=='__main__':main()
