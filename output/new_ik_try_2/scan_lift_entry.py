"""Read-only height/initial-branch screening; no execution approval or commands."""
from pathlib import Path
import sys,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from replay_pipeline import HERE,config,read_json,write_json
from replay_control import acquire_replay_lock,read_arms
from read_hands import Hands
from aubo_arms_replay import initial_solutions,scoped_scene
from aubo_continuous_ik import actual_model
from collision_scene import Scene
out=HERE/'output/lift_entry_scan';out.mkdir(exist_ok=True);c=config(HERE/'config/human_replay_try_2.json');lock=acquire_replay_lock();snap=read_arms(c);reader=Hands(read_json(HERE/'config/runtime.json')['hands'])
try:
 hands=reader.snapshot();scene=Scene(c,snap);models={s:actual_model(snap[s],out/(s+'.urdf')) for s in ('left','right')}
 write_json(out/'snapshot.json',{'controllers':snap,'hands':hands})
 with np.load(HERE/'output/human_camera_try_2/human_camera_prepared.npz') as z:targets={s:z[s+'_aubo_base_T_flange'][0].copy() for s in ('left','right')}
 up=-scene.normal;settings={s:scoped_scene(scene,s) for s in ('left','right')};static={s:hands[s]['positions_rad'] for s in ('left','right')};summary=[]
 for cm in (2.,3.,4.,5.,6.5,8.,10.):
  row={'lift_cm':cm,'sides':{}}
  for side in ('left','right'):
   T=targets[side].copy();R=np.asarray(read_json(c['sides'][side]['handeye'])['T_base_camera'])[:3,:3];T[:3,3]+=R@up*(cm/100.)
   candidates=initial_solutions(c,side,T,models[side],24);start=np.asarray(snap[side]['joint_positions_rad']);records=[]
   for i,q in enumerate(candidates):
    first=None
    for step,phase in enumerate(np.linspace(0,1,101)):
     arms={s:snap[s]['joint_positions_rad'] for s in ('left','right')};arms[side]=start+phase*(q-start);findings=settings[side].check(arms,static)
     if findings:first={'fraction':float(phase),'findings':findings};break
    records.append({'candidate_index':i,'screen_passed':first is None,'initial_q_rad':q.tolist(),'failure':first})
   row['sides'][side]={'candidates':len(records),'screen_passed':sum(x['screen_passed'] for x in records),'records':records}
  summary.append(row);write_json(out/'summary.json',{'screen_only':True,'execution_enabled':False,'samples_per_entry':101,'heights':summary})
  print('HEIGHT',cm,{s:row['sides'][s]['screen_passed'] for s in ('left','right')},flush=True)
finally:reader.close();lock.close()
