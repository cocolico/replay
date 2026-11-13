"""Offline full tracking of branches whose entry screening passed; no motion commands."""
from pathlib import Path
import sys,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from replay_pipeline import HERE,config,read_json,write_json
from aubo_continuous_ik import actual_model,cartesian_targets,track
from aubo_arms_replay import scoped_scene
from collision_scene import Scene
out=HERE/'output/lift_entry_scan';c=config(HERE/'config/human_replay_try_2.json');snapshot=read_json(out/'snapshot.json');snap=snapshot['controllers'];hands=snapshot['hands'];scene=Scene(c,snap);models={s:actual_model(snap[s],out/(s+'_tracking.urdf')) for s in ('left','right')};static={s:hands[s]['positions_rad'] for s in ('left','right')}
with np.load(HERE/'output/human_camera_try_2/human_camera_prepared.npz') as z:a={k:z[k].copy() for k in z.files}
times=np.unique(np.r_[a['time_s'],np.arange(a['time_s'][0],a['time_s'][-1],.01)]);runs=[]
for height in read_json(out/'summary.json')['heights']:
 cm=height['lift_cm'];result={'lift_cm':cm,'sides':{},'both_passed':False}
 for side in ('left','right'):
  mesh=scoped_scene(scene,side);T=a[side+'_aubo_base_T_flange'].copy();R=np.asarray(read_json(c['sides'][side]['handeye'])['T_base_camera'])[:3,:3];T[:,:3,3]+=(R@(-scene.normal))*(cm/100.)
  targets=cartesian_targets(a['time_s'],T,times);trials=[]
  for candidate in height['sides'][side]['records']:
   if not candidate['screen_passed']:continue
   def collision(q):
    arms={s:snap[s]['joint_positions_rad'] for s in ('left','right')};arms[side]=q;return mesh.check(arms,static)
   qs,status=track(models[side],targets,times,candidate['initial_q_rad'],collision=collision)
   status['candidate_index']=candidate['candidate_index'];trials.append(status);print('LIFT',cm,side,status,flush=True)
   if qs is not None:
    np.savez_compressed(out/f'lift_{cm}_{side}_safe_entry.npz',time_s=times,joint_positions_rad=qs,execution_enabled=False);break
  result['sides'][side]={'passed':any(v['passed'] for v in trials),'trials':trials}
  if not result['sides'][side]['passed']:break
 result['both_passed']=len(result['sides'])==2 and all(v['passed'] for v in result['sides'].values());runs.append(result);write_json(out/'tracking_summary.json',{'execution_enabled':False,'runs':runs})
 if result['both_passed']:break
