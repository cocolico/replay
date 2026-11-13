"""Arm-only source-frame diagnostic; never sends motion commands."""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from replay_pipeline import HERE,config,read_json,write_json,SIDES
from collision_scene import Scene
out=Path(__file__).resolve().parent;c=config(HERE/'config/human_replay_try_3.json');r=read_json(out/'diagnostic_controller_ik.json')
scene=Scene(c,{s:r['sides'][s]['snapshot'] for s in SIDES});scene.shapes=[x for x in scene.shapes if x['kind']=='arm'];scene.pairs=[(i,j) for i in range(len(scene.shapes)) for j in range(i+1,len(scene.shapes)) if frozenset((scene.shapes[i]['name'],scene.shapes[j]['name'])) not in scene.disabled];bad=[]
with np.load(out/'diagnostic_controller_ik.npz') as z:
 for i in range(len(z['time_s'])):
  f=scene.check({s:z[s+'_aubo_joint_positions_rad'][i] for s in SIDES},{s:np.zeros(20) for s in SIDES},max_findings=100)
  if f:bad.append({'frame':i,'source_time_s':float(z['time_s'][i]),'findings':f})
 def category(f):
  if f['b']=='calibrated_table_halfspace':return 'table'
  return 'self' if f['a'].split(':')[0]==f['b'].split(':')[0] else 'cross_arm'
 counts={kind:sum(any(category(f)==kind for f in b['findings']) for b in bad) for kind in ('table','self','cross_arm')}
 report={'scope':'Arm meshes only; hand geometry excluded','ready_for_execution':False,'execution_attempted':False,'checked_source_frames':len(z['time_s']),'duration_s':float(z['time_s'][-1]),'joint_jumps':{s:r['sides'][s]['jumps_over_0_4_rad'] for s in SIDES},'failed_collision_frames':len(bad),'category_failed_frames':counts,'failures':bad,'runtime_dense_check_performed':False}
write_json(out/'arms_only_preflight.json',report)
print('failed',len(bad),'categories',counts);print('first',bad[0] if bad else None)
for kind in ('table','self','cross_arm'):
 matches=[(f['clearance_m'],b['frame'],b['source_time_s'],f['a'],f['b']) for b in bad for f in b['findings'] if category(f)==kind];print(kind,'minimum',min(matches) if matches else None)
print('jumps',{s:len(report['joint_jumps'][s]) for s in SIDES})
root=Path(c['camera_trajectory_root']);old=HERE.parent/'wuji2_camera_replay_trajectory(2)';files=list(root.glob('*.npy'));print('identical to (2)',old.exists() and all((old/p.name).exists() and p.read_bytes()==(old/p.name).read_bytes() for p in files))
