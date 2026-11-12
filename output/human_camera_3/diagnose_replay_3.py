import sys,concurrent.futures,time
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path('replay/code').resolve()))
from replay_pipeline import HERE,config,write_json,provenance,SIDES
from replay_control import acquire_replay_lock
from device_backends import Aubo
from dh_kinematics import fk
from collision_scene import Scene
c=config(HERE/'config/human_replay_3.json');out=HERE/'output/human_camera_3';lock=acquire_replay_lock()
from replay_pipeline import prepare_camera
r=prepare_camera(c,out);print('PREPARED',r['frames'],r['duration_s'],flush=True)
with np.load(out/'human_camera_prepared.npz') as z:a={k:z[k].copy() for k in z.files}
def solve(side):
 arm=Aubo(c['sides'][side]);snap=arm.snapshot();prev=np.asarray(snap['joint_positions_rad']);lo=np.asarray(snap['joint_min_rad']);hi=np.asarray(snap['joint_max_rad']);qs=[];jumps=[];errors=[]
 try:
  for i,target in enumerate(a[side+'_aubo_base_T_flange']):
   q=arm.ik(target,prev)
   for j in range(6):
    choices=q[j]+2*np.pi*np.arange(-2,3);choices=choices[(choices>=lo[j])&(choices<=hi[j])];q[j]=choices[np.argmin(abs(choices-prev[j]))]
   def residual(x):
    actual=fk(x,snap['kinematics_real']);return np.r_[actual[:3,3]-target[:3,3],Rotation.from_matrix(target[:3,:3].T@actual[:3,:3]).as_rotvec()]
   q=least_squares(residual,q,bounds=(lo,hi),max_nfev=30,ftol=1e-12,xtol=1e-12,gtol=1e-12).x
   actual=arm.fk(q);pe=float(np.linalg.norm(actual[:3,3]-target[:3,3]));ae=float(Rotation.from_matrix(actual[:3,:3].T@target[:3,:3]).magnitude())
   if pe>1e-5 or ae>1e-4:raise ValueError(f'frame {i} residual {pe}/{ae}')
   step=float(abs(q-prev).max())
   if i and step>.4:jumps.append({'frame':i,'time_s':float(a['time_s'][i]),'max_joint_step_rad':step})
   qs.append(q.copy());prev=q;errors.append((pe,ae))
  return np.array(qs),{'snapshot':snap,'jumps_over_0_4_rad':jumps,'max_position_error_m':max(x[0] for x in errors),'max_orientation_error_rad':max(x[1] for x in errors)}
 finally:arm.close()
reports={}
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
 jobs={s:pool.submit(solve,s) for s in SIDES}
 for s,f in jobs.items():a[s+'_aubo_joint_positions_rad'],reports[s]=f.result();print(s,'IK complete, jumps',len(reports[s]['jumps_over_0_4_rad']),flush=True)
a['execution_enabled']=np.asarray(False);np.savez_compressed(out/'diagnostic_controller_ik.npz',**a)
write_json(out/'diagnostic_controller_ik.json',{'execution_enabled':False,'ready_for_execution':False,'sides':reports,'configuration':provenance(c['_config_path']),'note':'Diagnostic only; discontinuities remain blocked.'})
scene=Scene(c,{s:reports[s]['snapshot'] for s in SIDES});bad=[]
for i in range(len(a['time_s'])):
 f=scene.check({s:a[s+'_aubo_joint_positions_rad'][i] for s in SIDES},{s:a[s+'_wuji_joint_positions_rad'][i] for s in SIDES},max_findings=100)
 if f:bad.append({'frame':i,'time_s':float(a['time_s'][i]),'findings':f})
 if i%50==0:print('collision check',i,'bad',len(bad),flush=True)
write_json(out/'diagnostic_collisions.json',{'passed':not bad,'checked_frames':len(a['time_s']),'failed_frames':len(bad),'failures':bad,'execution_enabled':False})
handself=lambda f:':hand:' in f['a'] and ':hand:' in f['b'] and f['a'].split(':')[0]==f['b'].split(':')[0]
summary={'ready_for_execution':False,'execution_attempted':False,'frames':len(a['time_s']),'duration_s':float(a['time_s'][-1]),'controller_ik_fk_passed':True,'joint_jumps':{s:reports[s]['jumps_over_0_4_rad'] for s in SIDES},'failed_collision_frames':len(bad),'hand_self_collision_frames':sum(any(handself(f) for f in b['findings']) for b in bad),'arm_collision_frames':sum(any(':arm:' in f['a'] or ':arm:' in f['b'] for f in b['findings']) for b in bad),'table_collision_frames':sum(any(f['b']=='calibrated_table_halfspace' for f in b['findings']) for b in bad),'first_failures':bad[:3],'runtime_dense_check_performed':False}
write_json(out/'summary.json',summary);print({k:v for k,v in summary.items() if k not in ('joint_jumps','first_failures')},flush=True)
