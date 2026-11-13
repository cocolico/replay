"""Offline rigid translation of a camera trajectory in calibrated current-camera axes."""
from pathlib import Path
import shutil
import numpy as np
from replay_pipeline import read_json,write_json,provenance


def shift(source,destination,c,delta_current_m,description):
    source=Path(source).resolve();destination=Path(destination).resolve()
    delta=np.asarray(delta_current_m,dtype=float)
    if delta.shape!=(3,) or not np.isfinite(delta).all():raise ValueError('Expected finite xyz translation')
    if destination.exists():raise ValueError('Destination already exists')
    mapping=np.asarray(read_json(c['camera_table_mapping'])['target_camera_T_source_camera'])
    delta_source=mapping[:3,:3].T@delta
    files=sorted(source.glob('*.npy'))
    if not files:raise ValueError('No trajectory arrays')
    for side in ('left','right'):
        pose=np.load(source/f'{side}_root_pose.npy',allow_pickle=False);p=np.load(source/f'{side}_position_m.npy',allow_pickle=False)
        if not np.allclose(pose[:,:3,3],p,atol=1e-8):raise ValueError('Input pose/position mismatch')
    destination.mkdir(parents=True)
    for p in files:shutil.copy2(p,destination/p.name)
    for side in ('left','right'):
        pose=np.load(source/f'{side}_root_pose.npy',allow_pickle=False).astype(float);p=np.load(source/f'{side}_position_m.npy',allow_pickle=False).astype(float)
        pose[:,:3,3]+=delta_source;p+=delta_source
        np.save(destination/f'{side}_root_pose.npy',pose);np.save(destination/f'{side}_position_m.npy',p)
    inputs=files+[Path(c[k]) for k in ('_config_path','camera_table_mapping','current_table_observation')]
    if (source/'lift_manifest.json').exists():inputs.append(source/'lift_manifest.json')
    manifest={'execution_enabled':False,'source':str(source),'description':description,'delta_current_camera_m':delta.tolist(),
              'delta_source_camera_m':delta_source.tolist(),'sources':[provenance(p) for p in inputs],
              'outputs':[provenance(p) for p in sorted(destination.glob('*.npy'))]}
    write_json(destination/'shift_manifest.json',manifest);return manifest
