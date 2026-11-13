"""Offline Aubo adapter for replay f1b5ed8 IK; never sends hardware commands."""
import sys,json,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
ROOT=Path(__file__).resolve().parents[4];sys.path[:0]=[str(ROOT/'replay/code'),str(ROOT/'replay/replay/src')]
from replay_pipeline import config,HERE,read_json,write_json,provenance,SIDES
from dh_kinematics import fk
from eef_replay.ik.model import RobotModel
from eef_replay.ik.continuous import ContinuousIK
from eef_replay.ik.differential import DifferentialIK
from collision_scene import Scene
OUT=Path(__file__).resolve().parent
C=config(HERE/'config/human_replay_try_2.json');OLD=HERE/'output/human_camera_try_2';report=read_json(OLD/'diagnostic_controller_ik.json');prepared=read_json(OLD/'human_camera_report.json')
for source in prepared['sources']:
 assert provenance(source['path'])==source,'Source changed: '+source['path']
with np.load(OLD/'diagnostic_controller_ik.npz') as z:A={k:z[k].copy() for k in z.files}

def make_model(side):
 s=report['sides'][side]['snapshot'];dh=s['kinematics_real'];tree=ET.Element('robot',name=side+'_actual_dh');ET.SubElement(tree,'link',name='base');parent='base'
 def add(name,child,kind,T=None,index=None):
  nonlocal parent
  ET.SubElement(tree,'link',name=child);j=ET.SubElement(tree,'joint',name=name,type=kind);ET.SubElement(j,'parent',link=parent);ET.SubElement(j,'child',link=child)
  if T is not None:ET.SubElement(j,'origin',xyz=' '.join(map(str,T[:3,3])),rpy=' '.join(map(str,Rotation.from_matrix(T[:3,:3]).as_euler('xyz'))))
  if index is not None:
   ET.SubElement(j,'axis',xyz='0 0 1');ET.SubElement(j,'limit',lower=str(s['joint_min_rad'][index]),upper=str(s['joint_max_rad'][index]),velocity=str(s['joint_max_speed_rad_s'][index]),effort='100')
  parent=child
 for i in range(6):
  alpha,a,d,theta,beta=[float(dh[k][i]) for k in ('alpha','a','d','theta','beta')]
  X=np.eye(4);X[:3,:3]=Rotation.from_euler('x',alpha).as_matrix();X[0,3]=a
  Z=np.eye(4);Z[:3,:3]=Rotation.from_euler('z',theta).as_matrix();Z[2,3]=d
  add(f'pre{i}',f'pre_link{i}','fixed',X@Z);add(f'joint{i+1}',f'axis_link{i}','revolute',index=i)
  B=np.eye(4);B[:3,:3]=Rotation.from_euler('y',beta).as_matrix();add(f'beta{i}','flange' if i==5 else f'link{i}','fixed',B)
 path=OUT/f'{side}_actual_dh.urdf';ET.ElementTree(tree).write(path,encoding='unicode');model=RobotModel(path)
 max_error=0.
 for q in A[side+'_aubo_joint_positions_rad'][::10]:max_error=max(max_error,float(abs(model.fk('flange',q)-fk(q,dh)).max()))
 assert max_error<1e-10,(side,max_error)
 return model,max_error

summary={'repository_commit':'f1b5ed8','source_configuration':provenance(C['_config_path']),'execution_attempted':False,'ready_for_execution':False,'model_fk_checks':{},'runs':[]}
models={}
for s in SIDES:models[s],summary['model_fk_checks'][s]=make_model(s)
for bound in (.12,.4):
 for backend,cls in [('continuous',ContinuousIK),('differential',DifferentialIK)]:
  run={'backend':backend,'max_source_step_rad':bound,'sides':{},'whole_trajectory_passed':False}
  for side in SIDES:
   model=models[side];solver=cls(model,'flange',range(6));seed=A[side+'_aubo_joint_positions_rad'][0].copy();seed=(seed+np.pi)%(2*np.pi)-np.pi
   qs=[seed.copy()];results=[];status={'first_frame_from':'previous controller-verified frame0, equivalent centered angles'}
   # Refine first frame with the new solver, rather than trusting an imported first solution.
   try:
    seed,first=solver.solve_near(A[side+'_aubo_base_T_flange'][0],seed,max_step=bound);qs=[seed.copy()]
    for i in range(1,len(A['time_s'])):
     step=np.minimum(bound,model.velocity*(A['time_s'][i]-A['time_s'][i-1])/.2)
     seed,res=solver.solve_near(A[side+'_aubo_base_T_flange'][i],seed,max_step=step);qs.append(seed.copy());results.append(res.to_dict())
    status.update(passed=True,frames=len(qs),max_step_rad=float(abs(np.diff(qs,axis=0)).max()))
    np.savez_compressed(OUT/f'{backend}_{bound}_{side}_offline.npz',time_s=A['time_s'],joint_positions_rad=qs,execution_enabled=False)
   except Exception as e:status.update(passed=False,failed_frame=len(qs),source_time_s=float(A['time_s'][min(len(qs),len(A['time_s'])-1)]),error=str(e),solved_frames=len(qs))
   run['sides'][side]=status;print(backend,bound,side,status,flush=True)
  run['whole_trajectory_passed']=all(v['passed'] for v in run['sides'].values());summary['runs'].append(run);write_json(OUT/'summary.json',summary)
print('DONE',OUT/'summary.json',flush=True)
