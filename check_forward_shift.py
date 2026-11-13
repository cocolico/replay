#!/usr/bin/env python3
"""Read-only forward-shift search. Never starts Runtime, Servo or hand motors."""
from pathlib import Path
import argparse,os,time,uuid
import numpy as np
from scipy.spatial.transform import Rotation
from replay_pipeline import HERE,config,read_json,write_json,prepare_camera,provenance
from replay_control import acquire_replay_lock,read_arms,require_start
from read_hands import Hands
from runtime_plan import check_controller
from collision_scene import Scene
from aubo_continuous_ik import actual_model,cartesian_targets,track
from aubo_arms_replay import initial_solutions,scoped_scene,runtime_pair,SETTINGS
from shift_camera_trajectory import shift
from device_backends import Aubo


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,default=HERE/'resources/trajectories/variants/lifted_try_2_6p5cm')
    p.add_argument('--config',type=Path,default=HERE/'config/human_replay_try_2.json')
    p.add_argument('--distances-cm',nargs='+',type=float,default=[5.,10.,15.,20.])
    a=p.parse_args()
    if not a.source.is_dir() or not a.distances_cm or any(not np.isfinite(v) or v<=0 for v in a.distances_cm):p.error('Expected source folder and positive finite distances')
    lock=acquire_replay_lock();out=HERE/'output'/('forward_scan_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]);out.mkdir()
    summary={'ready_for_execution':False,'execution_attempted':False,'arms_only':True,'source':str(a.source.resolve()),'mesh_clearance_m':SETTINGS['mesh_clearance_m'],'direction':'Away from the midpoint of both arm bases toward the mean task roots, projected into calibrated table plane','attempts':[]}
    def session(status):write_json(HERE/'output/replay_session.json',{'pid':os.getpid(),'name':out.name,'status':status,'updated_unix_s':time.time()})
    session('preflight');print('OUTPUT',out,flush=True)
    try:
        c=config(a.config)
        controllers=read_arms(c)
        for side in ('left','right'):check_controller(c,side,controllers[side])
        reader=Hands(read_json(HERE/'config/runtime.json')['hands'])
        try:hands=reader.snapshot()
        finally:reader.close()
        write_json(out/'start_snapshot.json',{'controllers':controllers,'hands':hands})
        c.pop('_config_path');c['camera_trajectory_root']=str(a.source.resolve())
        for side in ('left','right'):c['sides'][side]['model_confirmed']=True
        write_json(out/'base_config.json',c);c=config(out/'base_config.json');prep=prepare_camera(c,out/'base')
        if prep['blockers']:raise ValueError('Unresolved coordinate/mount inputs: '+str(prep['blockers']))
        with np.load(out/'base/human_camera_prepared.npz') as z:base={k:z[k].copy() for k in z.files}
        scene=Scene(c,controllers);normal=scene.normal
        base_center=np.mean([scene.base[s][:3,3] for s in ('left','right')],axis=0)
        task_center=np.mean(np.concatenate([(scene.base[s]@base[s+'_aubo_base_T_wuji_base'])[:,:3,3] for s in ('left','right')]),axis=0)
        forward=task_center-base_center;forward-=normal*np.dot(forward,normal)
        if np.linalg.norm(forward)<.01:raise ValueError('Cannot determine common forward direction from bases and task')
        forward/=np.linalg.norm(forward);summary['forward_current_camera']=forward.tolist();summary['forward_board_axes']=(scene.board[:3,:3].T@forward).tolist()
        print('FORWARD current camera',forward,'board',summary['forward_board_axes'],flush=True)
        models={s:actual_model(controllers[s],out/(s+'_actual_dh.urdf')) for s in ('left','right')};static={s:hands[s]['positions_rad'] for s in ('left','right')}
        for cm in a.distances_cm:
            folder=out/('forward_'+str(cm).replace('.','p')+'cm');folder.mkdir()
            shift(a.source,folder/'trajectory',c,forward*(cm/100.),f'Forward {cm} cm away from arm bases; preserve source lift of 6.5cm')
            cc=dict(c);cc.pop('_config_path');cc['camera_trajectory_root']=str(folder/'trajectory');write_json(folder/'config.json',cc);cc=config(folder/'config.json');report=prepare_camera(cc,folder)
            with np.load(folder/'human_camera_prepared.npz') as z:arrays={k:z[k].copy() for k in z.files}
            times=np.unique(np.r_[arrays['time_s'],np.arange(arrays['time_s'][0],arrays['time_s'][-1],.01)])
            attempt={'forward_cm':cm,'candidate':str(folder/'trajectory'),'sides':{},'pair_checks':[],'ready_for_execution':False};solutions={}
            for side in ('left','right'):
                mesh=scoped_scene(scene,side);targets=cartesian_targets(arrays['time_s'],arrays[side+'_aubo_base_T_flange'],times)
                candidates=initial_solutions(cc,side,targets[0],models[side],32);trials=[];solutions[side]=[]
                for index,seed in enumerate(candidates):
                    def collision(q):
                        arms={s:controllers[s]['joint_positions_rad'] for s in ('left','right')};arms[side]=q;return mesh.check(arms,static)
                    entry_failure=None;start=np.asarray(controllers[side]['joint_positions_rad'])
                    for fraction in np.linspace(0,1,101):
                        findings=collision(start+fraction*(seed-start))
                        if findings:entry_failure={'fraction':float(fraction),'findings':findings};break
                    if entry_failure:
                        trials.append({'candidate_index':index,'passed':False,'phase':'entry_screen','failure':entry_failure});continue
                    print('FORWARD',cm,side,'tracking candidate',index,flush=True)
                    qs,status=track(models[side],targets,times,seed,collision=collision);status.update(candidate_index=index,phase='full_single_arm_tracking');trials.append(status)
                    if qs is not None:
                        path=folder/f'{side}_branch{index}.npz';np.savez_compressed(path,time_s=times,joint_positions_rad=qs,execution_enabled=False);solutions[side].append((times,qs,path))
                    write_json(folder/(side+'_ik.json'),{'candidates':len(candidates),'trials':trials,'complete_paths':len(solutions[side])})
                write_json(folder/(side+'_ik.json'),{'candidates':len(candidates),'trials':trials,'complete_paths':len(solutions[side])})
                attempt['sides'][side]={'candidates':len(candidates),'entry_screen_passed':sum(t.get('phase')!='entry_screen' for t in trials),'complete_paths':len(solutions[side])}
                if not solutions[side]:break
            if len(solutions)==2 and all(solutions.values()):
                for left in solutions['left']:
                    for right in solutions['right']:
                        print('FORWARD',cm,'PAIR',left[2].name,right[2].name,flush=True)
                        try:trajectory,plan=runtime_pair(cc,controllers,hands,left,right,dict(SETTINGS),arrays)
                        except ValueError as error:plan={'ready_for_execution':False,'error':str(error)}
                        plan['branches']=[left[2].name,right[2].name];attempt['pair_checks'].append(plan);write_json(folder/'pair_checks.json',attempt['pair_checks'])
                        if plan['ready_for_execution']:
                            for s,solution in zip(('left','right'),(left,right)):
                                arm=Aubo(cc['sides'][s])
                                try:
                                    desired=cartesian_targets(arrays['time_s'],arrays[s+'_aubo_base_T_flange'],solution[0])
                                    for q,target in zip(solution[1],desired):
                                        actual=arm.fk(q)
                                        if np.linalg.norm(actual[:3,3]-target[:3,3])>1e-5 or Rotation.from_matrix(actual[:3,:3].T@target[:3,:3]).magnitude()>1e-4:raise ValueError('Controller FK failed: '+s)
                                finally:arm.close()
                            reader=Hands(read_json(HERE/'config/runtime.json')['hands'])
                            try:require_start({'controllers':controllers,'hand_start':hands},read_arms(cc),reader.snapshot())
                            finally:reader.close()
                            np.savez_compressed(folder/'runtime.npz',joint_positions_left_right_rad=trajectory,execution_enabled=False)
                            attempt.update(ready_for_execution=True,controller_fk_passed=True,start_rechecked=True,plan=plan);break
                    if attempt['ready_for_execution']:break
            summary['attempts'].append(attempt);summary['ready_for_execution']=attempt['ready_for_execution'];write_json(out/'summary.json',summary)
            print('FORWARD RESULT',cm,attempt['ready_for_execution'],attempt['sides'],flush=True)
            if attempt['ready_for_execution']:break
    except BaseException as error:
        summary['error']=str(error);raise
    finally:
        write_json(out/'summary.json',summary);session('stopped');lock.close()

if __name__=='__main__':
    main()
