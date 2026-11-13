"""Finish cached lifted IK preflight offline; no movement entry point."""
from pathlib import Path
import sys,os,time
import numpy as np
from scipy.spatial.transform import Rotation
BASE=Path(__file__).resolve().parent;sys.path.insert(0,str(BASE.parents[1]))
from replay_pipeline import HERE,config,read_json,write_json,provenance
from replay_control import acquire_replay_lock,read_arms,require_start
from read_hands import Hands
from runtime_plan import check_controller
from aubo_arms_replay import SETTINGS,runtime_pair
from aubo_continuous_ik import RobotModel,cartesian_targets
from dh_kinematics import fk
from device_backends import Aubo
lock=acquire_replay_lock();out=BASE/'combined_recheck';out.mkdir(exist_ok=True);summary={'ready_for_execution':False,'execution_attempted':False,'arms_only':True,'lift_cm':6.5,'checks':[]}
def state(status):write_json(HERE/'output/replay_session.json',{'pid':os.getpid(),'name':BASE.name+'_combined','status':status,'updated_unix_s':time.time()})
state('preflight')
try:
 c=config(BASE/'config.json');controllers=read_arms(c)
 for side in ('left','right'):check_controller(c,side,controllers[side])
 reader=Hands(read_json(HERE/'config/runtime.json')['hands'])
 try:hands=reader.snapshot()
 finally:reader.close()
 write_json(out/'start_snapshot.json',{'controllers':controllers,'hands':hands})
 prepared=read_json(BASE/'human_camera_report.json')
 for p in prepared['sources']:
  if provenance(p['path'])!=p:raise ValueError('Input/calibration changed: '+p['path'])
 with np.load(BASE/'human_camera_prepared.npz') as z:arrays={k:z[k].copy() for k in z.files}
 paths={}
 for side in ('left','right'):
  model=RobotModel(BASE/(side+'_actual_dh.urdf'))
  for q in (np.zeros(6),np.linspace(-.8,1.,6)):
   np.testing.assert_allclose(model.fk('flange',q),fk(q,controllers[side]['kinematics_real']),atol=1e-10)
  paths[side]=[]
  for p in sorted(BASE.glob(side+'_branch*_offline.npz')):
   with np.load(p) as z:paths[side].append((z['time_s'].copy(),z['joint_positions_rad'].copy(),p))
 summary['complete_ik_branches']={s:len(paths[s]) for s in paths};chosen=None
 for left in paths['left']:
  for right in paths['right']:
   print('PAIR',left[2].name,right[2].name,flush=True)
   try:trajectory,plan=runtime_pair(c,controllers,hands,left,right,dict(SETTINGS),arrays)
   except ValueError as e:plan={'ready_for_execution':False,'error':str(e)}
   plan['branches']=[left[2].name,right[2].name];summary['checks'].append(plan);write_json(out/'summary.json',summary)
   if plan['ready_for_execution']:chosen=trajectory,plan,left,right;break
  if chosen:break
 if chosen is not None:
  trajectory,plan,left,right=chosen
  for side,sol in zip(('left','right'),(left,right)):
   arm=Aubo(c['sides'][side])
   try:
    targets=cartesian_targets(arrays['time_s'],arrays[side+'_aubo_base_T_flange'],sol[0])
    for q,target in zip(sol[1],targets):
     actual=arm.fk(q)
     if np.linalg.norm(actual[:3,3]-target[:3,3])>1e-5 or Rotation.from_matrix(actual[:3,:3].T@target[:3,:3]).magnitude()>1e-4:raise ValueError('SDK FK failed: '+side)
   finally:arm.close()
  reader=Hands(read_json(HERE/'config/runtime.json')['hands'])
  try:require_start({'controllers':controllers,'hand_start':hands},read_arms(c),reader.snapshot())
  finally:reader.close()
  np.savez_compressed(out/'runtime.npz',joint_positions_left_right_rad=trajectory,execution_enabled=False)
  summary.update(ready_for_execution=True,controller_fk_passed=True,start_rechecked=True,plan=plan)
 else:summary['reason']='All complete IK branch combinations fail full-assembly or transition checks'
 print('RESULT',summary['ready_for_execution'],summary.get('reason','PASS'),flush=True)
finally:
 write_json(out/'summary.json',summary);state('stopped');lock.close()
