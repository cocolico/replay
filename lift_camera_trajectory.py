#!/usr/bin/env python3
"""Create a translated camera trajectory, preserving rotations, fingers and timestamps."""
from pathlib import Path
import argparse,shutil
import numpy as np
from replay_pipeline import HERE,config,read_json,write_json,provenance


def lift(source, destination, c, height_cm):
    if not np.isfinite(height_cm) or height_cm<0:raise ValueError('height_cm must be finite and nonnegative')
    source=Path(source).resolve();destination=Path(destination).resolve()
    if destination.exists():raise ValueError('Destination already exists; refusing to overwrite')
    board=np.asarray(read_json(c['current_table_observation'])['camera_T_board'])
    mapping=np.asarray(read_json(c['camera_table_mapping'])['target_camera_T_source_camera'])
    up_current=-board[:3,2];up_source=mapping[:3,:3].T@up_current
    delta=up_source*(height_cm/100.)
    files=sorted(source.glob('*.npy'))
    if not files:raise ValueError('No trajectory NPY files')
    for side in ('left','right'):
        q=np.load(source/f'{side}_root_pose.npy',allow_pickle=False)
        p=np.load(source/f'{side}_position_m.npy',allow_pickle=False)
        if not np.allclose(q[:,:3,3],p,atol=1e-8):raise ValueError('Input position and root disagree')
    destination.mkdir(parents=True)
    for p in files:shutil.copy2(p,destination/p.name)
    for side in ('left','right'):
        q=np.load(source/f'{side}_root_pose.npy',allow_pickle=False).astype(float)
        p=np.load(source/f'{side}_position_m.npy',allow_pickle=False).astype(float)
        q[:,:3,3]+=delta;p+=delta
        np.save(destination/f'{side}_root_pose.npy',q);np.save(destination/f'{side}_position_m.npy',p)
    manifest={'artifact_type':'offline_lifted_camera_trajectory','execution_enabled':False,'lift_cm':height_cm,
              'source':str(source),'destination':str(destination),'delta_source_camera_m':delta.tolist(),
              'up_current_camera':up_current.tolist(),'sources':[provenance(p) for p in files]+[provenance(c[k]) for k in ('_config_path','current_table_observation','camera_table_mapping')],
              'outputs':[provenance(p) for p in sorted(destination.glob('*.npy'))]}
    write_json(destination/'lift_manifest.json',manifest)
    return manifest

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('destination',type=Path);p.add_argument('--lift-cm',required=True,type=float);p.add_argument('--config',type=Path,default=HERE/'config/human_replay.json');a=p.parse_args()
    print(lift(a.source,a.destination,config(a.config),a.lift_cm)['delta_source_camera_m'])
