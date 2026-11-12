"""Read-only arm-only mesh preflight, with no hand or motion commands."""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from replay_pipeline import HERE,config,read_json,write_json,provenance,SIDES
from replay_control import acquire_replay_lock,read_arms
from runtime_plan import sample_replay,transition,identity,check_controller
from collision_scene import Scene
out=Path(__file__).resolve().parent;c=config(HERE/'config/human_replay_3.json');lock=acquire_replay_lock();controllers=read_arms(c)
r=read_json(out/'diagnostic_controller_ik.json')
assert r['configuration']==provenance(c['_config_path'])
for s in SIDES:
 check_controller(c,s,controllers[s])
 assert not r['sides'][s]['jumps_over_0_4_rad']
 assert controllers[s]['kinematics_real']==r['sides'][s]['snapshot']['kinematics_real']
with np.load(out/'diagnostic_controller_ik.npz') as z:
 t=z['time_s'].copy();q=np.concatenate([z[s+'_aubo_joint_positions_rad'] for s in SIDES],axis=1)
rate=100;speed=.2;lo=np.concatenate([controllers[s]['joint_min_rad'] for s in SIDES]);hi=np.concatenate([controllers[s]['joint_max_rad'] for s in SIDES]);vlim=np.concatenate([controllers[s]['joint_max_speed_rad_s'] for s in SIDES])*.2;alim=np.concatenate([controllers[s]['joint_max_acceleration_rad_s2'] for s in SIDES])*.15
for _ in range(12):
 ticks,replay=sample_replay(t,q,rate,speed)
 if np.all(abs(np.diff(replay,axis=0)*rate)<=vlim+1e-6) and np.all(abs(np.diff(replay,n=2,axis=0)*rate**2)<=alim+1e-6):break
 speed*=.8
else:raise ValueError('Cannot retime arm limits')
start=np.concatenate([controllers[s]['joint_positions_rad'] for s in SIDES]);duration=max(10.,float(np.max(1.9*abs(q[0]-start)/vlim)),float(np.max(np.sqrt(6*abs(q[0]-start)/alim))))
_,ramp=transition(start,replay[0],rate,duration);plan=np.concatenate([ramp,np.tile(replay[0],(rate,1)),replay[1:]])
assert np.all(plan>=lo-1e-6) and np.all(plan<=hi+1e-6)
assert np.all(abs(np.diff(plan,axis=0)*rate)<=vlim+1e-6) and np.all(abs(np.diff(plan,n=2,axis=0)*rate**2)<=alim+1e-6)
scene=Scene(c,controllers,.005);scene.shapes=[x for x in scene.shapes if x['kind']=='arm'];scene.pairs=[(i,j) for i in range(len(scene.shapes)) for j in range(i+1,len(scene.shapes)) if frozenset((scene.shapes[i]['name'],scene.shapes[j]['name'])) not in scene.disabled]
bad=[];zero={s:np.zeros(20) for s in SIDES}
for i,row in enumerate(plan):
 f=scene.check({'left':row[:6],'right':row[6:]},zero,max_findings=100)
 if f:bad.append({'frame':i,'time_s':i/rate,'phase':'transition' if i<len(ramp) else 'replay','source_time_s':None if i<len(ramp)+rate else min(float(t[-1]),(i-len(ramp)-rate+1)/rate*speed),'findings':f})
 if i%1000==0:print('arm-only mesh',i,'/',len(plan),'failed',len(bad),flush=True)
report={'ready_for_execution':not bad,'execution_attempted':False,'scope':'Arm meshes only; no hand geometry or hand actions','controllers':controllers,'configuration':provenance(c['_config_path']),'requested_speed':.2,'effective_speed':speed,'rate_hz':rate,'transition_s':duration,'checked_frames':len(plan),'failed_frames':len(bad),'failed_transition_frames':sum(x['phase']=='transition' for x in bad),'failures':bad,'dynamic_limits_passed':True}
write_json(out/'arms_only_preflight.json',report);print('ARM-ONLY', 'BLOCKED' if bad else 'PASS', 'failed',len(bad),'transition',report['failed_transition_frames'],flush=True)
