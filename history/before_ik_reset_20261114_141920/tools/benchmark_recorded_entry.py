"""Offline check of the original direct entry; does not search for a detour."""
from pathlib import Path
import sys,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from replay_pipeline import HERE,config,read_json,write_json
from collision_scene import Scene
from collision_planning import coherent_winding
from runtime_plan import transition,dynamics
p=HERE/'output/lift_entry_scan/snapshot.json';snapshot=read_json(p);snap=snapshot['controllers'];hands={s:snapshot['hands'][s]['positions_rad'] for s in ('left','right')};c=config(HERE/'config/human_replay.json');scene=Scene(c,snap)
start=np.r_[snap['left']['joint_positions_rad'],snap['right']['joint_positions_rad']];lo=np.r_[snap['left']['joint_min_rad'],snap['right']['joint_min_rad']];hi=np.r_[snap['left']['joint_max_rad'],snap['right']['joint_max_rad']]
base=HERE/'output/aubo_arms_20261113_110506_d987ef';out=HERE/'output/direct_recorded_entry';out.mkdir(exist_ok=True);results=[]
v=np.r_[snap['left']['joint_max_speed_rad_s'],snap['right']['joint_max_speed_rad_s']]*.2;a=np.r_[snap['left']['joint_max_acceleration_rad_s2'],snap['right']['joint_max_acceleration_rad_s2']]*.15
for name in ('left_branch5_offline.npz','left_branch6_offline.npz'):
    with np.load(base/name) as left,np.load(base/'right_branch3_offline.npz') as right:
        source,_=coherent_winding(np.c_[left['joint_positions_rad'],right['joint_positions_rad']],start,lo,hi);goal=source[0]
    duration=max(10.,float(np.max(1.9*abs(goal-start)/v)),float(np.max(np.sqrt(6*abs(goal-start)/a))))
    for _ in range(8):
        ticks,rows=transition(start,goal,100,duration);speed,acc=dynamics(rows,100)
        if np.all(speed<=v+1e-6) and np.all(acc<=a+1e-6):break
        duration*=1.3
    else:raise ValueError('Direct transition exceeds dynamic limits')
    first=None
    for i,q in enumerate(rows):
        findings=scene.check({'left':q[:6],'right':q[6:]},hands)
        if findings:first={'frame':i,'time_s':float(ticks[i]),'findings':findings};break
    result={'left':name,'right':'right_branch3_offline.npz','planner':'direct_joint_interpolation','independently_planned':False,'execution_attempted':False,'passed':first is None,'first_collision':first,'duration_s':float(ticks[-1])}
    results.append(result);write_json(out/'summary.json',results);print(result,flush=True)
