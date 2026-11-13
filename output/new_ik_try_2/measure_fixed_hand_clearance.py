"""Offline fixed hand root geometry measurement, independent of arm IK branches."""
from pathlib import Path
import sys,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from replay_pipeline import HERE,config,read_json,write_json
from collision_scene import Scene
from aubo_continuous_ik import cartesian_targets
from dh_kinematics import fk
folder=HERE/'output/aubo_arms_20261113_105902_493ce1';c=config(folder/'config.json')
# The exact hand start/arm snapshot was not persisted by failed preflight. Read fresh states, read-only.
from replay_control import acquire_replay_lock,read_arms
from read_hands import Hands
lock=acquire_replay_lock();controllers=read_arms(c);reader=Hands(read_json(HERE/'config/runtime.json')['hands'])
try:
 hands=reader.snapshot();scene=Scene(c,controllers);poses=scene.shape_poses({s:controllers[s]['joint_positions_rad'] for s in ('left','right')},{s:hands[s]['positions_rad'] for s in ('left','right')})
 with np.load(folder/'human_camera_prepared.npz') as z:
  times=np.unique(np.r_[z['time_s'],np.arange(z['time_s'][0],z['time_s'][-1],.01)])
  minimum=(1e9,None,None);per_side={}
  for side in ('left','right'):
   camera_T_base=scene.base[side];current_root=camera_T_base@fk(controllers[side]['joint_positions_rad'],controllers[side]['kinematics_real'])@np.asarray(c['sides'][side]['flange_T_wuji_base'])
   desired=cartesian_targets(z['time_s'],z[side+'_aubo_base_T_wuji_base'],times);rootlocal=[]
   for shape,T in zip(scene.shapes,poses):
    if shape['kind']=='hand' and shape['side']==side:
     local=np.linalg.inv(current_root)@T;points=shape['hull']@local[:3,:3].T+local[:3,3];rootlocal.append((shape['name'],points))
   low=(1e9,None,None)
   for i,Tbase in enumerate(desired):
    T=camera_T_base@Tbase
    for name,points in rootlocal:
     world=points@T[:3,:3].T+T[:3,3];h=float(np.min(-(world-scene.plane_point)@scene.normal))
     if h<low[0]:low=(h,float(times[i]),name)
   per_side[side]={'minimum_height_m':low[0],'source_time_s':low[1],'shape':low[2]};minimum=min(minimum,low)
 result={'scope':'All fixed hand mesh vertices along desired roots; excludes arm shape and reachability','per_side':per_side,'required_lift_for_5mm_cm':max(0.,(.005-minimum[0])*100),'controllers':controllers,'hands':hands,'samples_per_side':len(times)}
 write_json(HERE/'output/new_ik_try_2/fixed_hand_lift_measurement.json',result);print(per_side);print('required cm',result['required_lift_for_5mm_cm'])
finally:reader.close();lock.close()
