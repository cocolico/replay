#!/usr/bin/env python3
"""Use connected Aubo controllers for read-only IK/FK; no motion/Runtime calls."""
from pathlib import Path
import argparse,concurrent.futures
import numpy as np
from scipy.spatial.transform import Rotation
from replay_pipeline import HERE,SIDES,config,read_json,write_json,provenance,dynamics,default_config
from device_backends import Aubo
from dh_kinematics import fk as calibrated_fk
from scipy.optimize import least_squares


def solve_side(c,side,z):
    robot=Aubo(c['sides'][side])
    try:
        snap=robot.snapshot()
        if snap['robot_type']!='aubo_i7H':raise ValueError('Unexpected robot type')
        lo,hi=np.asarray(snap['joint_min_rad']),np.asarray(snap['joint_max_rad'])
        prev=np.asarray(z[f'{side}_aubo_joint_positions_rad'][0]);qs=[];position=[];angle=[]
        for i,target in enumerate(z[f'{side}_aubo_base_T_flange']):
            q=robot.ik(target,prev)
            for j in range(6):
                choices=np.array([q[j]-2*np.pi,q[j],q[j]+2*np.pi]);choices=choices[(choices>=lo[j])&(choices<=hi[j])]
                if not len(choices):raise ValueError('No in-limit IK winding')
                q[j]=choices[np.argmin(abs(choices-prev[j]))]
            def residual(x):
                actual=calibrated_fk(x,snap['kinematics_real'])
                return np.r_[actual[:3,3]-target[:3,3],Rotation.from_matrix(target[:3,:3].T@actual[:3,:3]).as_rotvec()]
            q=least_squares(residual,q,bounds=(lo,hi),max_nfev=30,ftol=1e-12,xtol=1e-12,gtol=1e-12).x
            actual=robot.fk(q)
            pe=float(np.linalg.norm(actual[:3,3]-target[:3,3]));ae=float(Rotation.from_matrix(actual[:3,:3].T@target[:3,:3]).magnitude())
            if pe>1e-5 or ae>1e-4:raise ValueError(f'{side} frame{i} controller IK residual {pe}m/{ae}rad')
            if i and np.max(abs(q-prev))>.4:raise ValueError(f'{side} frame{i} branch discontinuity')
            qs.append(q.copy());position.append(pe);angle.append(ae);prev=q
            if i%300==0:print(side,'frame',i,'/',len(z['time_s']),flush=True)
        return np.array(qs),{'snapshot':snap,'max_position_error_m':max(position),'max_orientation_error_rad':max(angle),
               'dynamics':dynamics(np.asarray(qs),z['time_s'])}
    finally:robot.close()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('names',nargs='+',choices=['episode594','episode615','episode616','human_camera']);p.add_argument('--config',type=Path);a=p.parse_args()
    for name in a.names:
        c=config(a.config or default_config(name))
        path=HERE/'output'/f'{name}_ik_candidate.npz';diagnostic=read_json(HERE/'output'/f'{name}_prepared_ik_diagnostic.json')
        if provenance(path)!=diagnostic['candidate_artifact'] or provenance(c['_config_path'])!=diagnostic['configuration']:raise ValueError('Candidate/config changed')
        if not all(c['sides'][s].get('mount_confirmed') for s in SIDES):raise ValueError('Mount not confirmed')
        with np.load(path,allow_pickle=False) as z:arrays={k:z[k].copy() for k in z.files}
        results={}
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
            pending={s:pool.submit(solve_side,c,s,arrays) for s in SIDES}
            for side in SIDES:
                qs,report=pending[side].result();arrays[f'{side}_aubo_joint_positions_rad']=qs;results[side]=report
        arrays['joint_positions_left_right_rad']=np.concatenate([arrays['left_aubo_joint_positions_rad'],arrays['left_wuji_joint_positions_rad'],arrays['right_aubo_joint_positions_rad'],arrays['right_wuji_joint_positions_rad']],axis=1)
        arrays['artifact_type']=np.asarray('controller_ik_offline_trajectory');arrays['execution_enabled']=np.asarray(False)
        dest=HERE/'output'/f'{name}_controller_ik.npz';np.savez_compressed(dest,**arrays)
        write_json(dest.with_suffix('.json'),{'artifact_type':'controller_ik_validation','execution_enabled':False,'trajectory':provenance(dest),
            'source':provenance(path),'configuration':provenance(c['_config_path']),'method':'Controller inverseToolKinematics + forwardToolKinematics; read-only',
            'sides':results,'collision_checked':False,'runtime_acceptance':False})
        print('FINISHED',name,len(arrays['time_s']),'frames',flush=True)

if __name__=='__main__':main()
