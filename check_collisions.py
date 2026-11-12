#!/usr/bin/env python3
"""Check original controller-IK samples against the calibrated mesh scene."""
import argparse,time
from pathlib import Path
import numpy as np
from replay_pipeline import HERE,SIDES,config,read_json,write_json,provenance,default_config
from collision_scene import Scene


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('names',nargs='+',choices=['episode594','episode615','episode616','human_camera']);p.add_argument('--config',type=Path);a=p.parse_args()
    for name in a.names:
        c=config(a.config or default_config(name))
        controllers={s:v["snapshot"] for s,v in read_json(HERE/"output"/f"{name}_controller_ik.json")["sides"].items()}
        scene=Scene(c,controllers)
        path=HERE/'output'/f'{name}_controller_ik.npz';bad=[];count=0;t0=time.monotonic()
        with np.load(path) as z:
            for i in range(len(z['time_s'])):
                collisions=scene.check({s:z[s+'_aubo_joint_positions_rad'][i] for s in SIDES},{s:z[s+'_wuji_joint_positions_rad'][i] for s in SIDES})
                if collisions:
                    count+=1
                    if len(bad)<20:bad.append({'frame':i,'time_s':float(z['time_s'][i]),'findings':collisions})
                if i%200==0:print(name,i,'/',len(z['time_s']),'failed_frames',count,flush=True)
            result={'trajectory':provenance(path),'execution_enabled':False,'passed':count==0,'checked_frames':len(z['time_s']),
                'failed_frames':count,'first_failures':bad,'clearance_m':scene.clearance,'elapsed_s':time.monotonic()-t0,
                'scope':['calibrated DH arm meshes','early-beta Wuji meshes','self/inter-arm/inter-hand','infinite calibrated table halfspace'],
                'limitations':['Original samples only; resampled replay and start transition need separate checks.','Unmodeled objects are not checked.'],
                'exclusions':['Aubo SRDF disabled pairs','same fixed hand cluster / same-branch two-axis knuckle covers','arm flange against attached hand root','base mounting face against table'],
                'hand_self_clearance_m':0.0}
        write_json(HERE/'output'/f'{name}_collisions.json',result);print('FINISHED',name,'failed_frames',count,flush=True)

if __name__=='__main__':main()
