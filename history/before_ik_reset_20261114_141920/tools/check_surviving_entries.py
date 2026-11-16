"""Fully validate candidates surviving the exact direct-entry condition; never execute."""
from pathlib import Path
import argparse,os,sys,time
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from replay_pipeline import HERE,config,read_json,write_json,provenance
from replay_control import acquire_replay_lock
from aubo_arms_replay import runtime_pair,SETTINGS,require_plan
from aubo_continuous_ik import cartesian_targets
from device_backends import Aubo
from read_hands import Hands


def check(folder):
    folder=Path(folder).resolve();out=folder/'surviving_entry_validation';out.mkdir(exist_ok=True);lock=acquire_replay_lock()
    summary={'execution_attempted':False,'ready_for_execution':False,'entry_checks':provenance(folder/'direct_entry_first_checks.json'),'checks':[]}
    def session(status):write_json(HERE/'output/replay_session.json',{'pid':os.getpid(),'name':folder.name+'_surviving_entry','status':status,'updated_unix_s':time.time()})
    session('preflight')
    try:
        c=config(folder/'config.json');snap=read_json(folder/'planning_snapshot.json');controllers=snap['controllers'];hands=snap['hands']
        prep=read_json(folder/'human_camera_report.json')
        for item in prep['sources']:
            if provenance(item['path'])!=item:raise ValueError('Source/calibration changed: '+item['path'])
        with np.load(folder/'human_camera_prepared.npz') as z:arrays={k:z[k].copy() for k in z.files}
        pairs=[v for v in read_json(folder/'direct_entry_first_checks.json')['pairs'] if v['entry_passed']]
        for record in pairs:
            paths=[]
            for name in record['branches']:
                path=folder/name
                with np.load(path) as z:paths.append((z['time_s'].copy(),z['joint_positions_rad'].copy(),path))
            print('FULL CHECK',record['branches'],flush=True)
            try:trajectory,plan=runtime_pair(c,controllers,hands,paths[0],paths[1],dict(SETTINGS),arrays)
            except ValueError as error:summary['checks'].append({'branches':record['branches'],'ready_for_execution':False,'error':str(error)});write_json(out/'summary.json',summary);continue
            plan['branches']=record['branches'];summary['checks'].append(plan);write_json(out/'summary.json',summary)
            if not plan['ready_for_execution']:continue
            sdk_errors={}
            for side,path in zip(('left','right'),paths):
                arm=Aubo(c['sides'][side]);pe=0.;ae=0.
                try:
                    targets=cartesian_targets(arrays['time_s'],arrays[side+'_aubo_base_T_flange'],path[0])
                    for q,target in zip(path[1],targets):
                        actual=arm.fk(q);pe=max(pe,float(np.linalg.norm(actual[:3,3]-target[:3,3])));ae=max(ae,float(Rotation.from_matrix(actual[:3,:3].T@target[:3,:3]).magnitude()))
                        if pe>1e-5 or ae>1e-4:raise ValueError('Controller FK failed: '+side)
                    sdk_errors[side]={'max_position_error_m':pe,'max_orientation_error_rad':ae}
                finally:arm.close()
            dest=out/'runtime.npz';np.savez_compressed(dest,joint_positions_left_right_rad=trajectory,execution_enabled=False)
            code=[HERE/name for name in ('aubo_arms_replay.py','runtime_plan.py','collision_scene.py','replay_control.py','device_backends.py','aubo_continuous_ik.py')]
            plan.update(name=folder.name+'_surviving_entry',arms_only=True,execution_enabled=False,configuration=provenance(c['_config_path']),sources=prep['sources']+[provenance(p[2]) for p in paths]+[provenance(p) for p in code]+[provenance(Path(__file__))],trajectory=provenance(dest),controllers=controllers,hand_start=hands,settings=dict(SETTINGS))
            reader=Hands(read_json(HERE/'config/runtime.json')['hands'])
            try:require_plan(c,plan,trajectory,reader)
            finally:reader.close()
            write_json(out/'runtime.json',plan);summary.update(ready_for_execution=True,sdk_fk_errors=sdk_errors,live_start_rechecked=True,runtime=provenance(out/'runtime.json'));break
        if not summary['ready_for_execution']:summary['reason']='All surviving direct-entry candidate pairs fail full runtime checks'
        print('FINAL VALIDATION',summary['ready_for_execution'],flush=True)
    except BaseException as error:
        summary['error']=str(error);raise
    finally:
        write_json(out/'summary.json',summary);session('stopped');lock.close()
    return summary

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path);a=p.parse_args();check(a.folder)
