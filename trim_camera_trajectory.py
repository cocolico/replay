#!/usr/bin/env python3
"""Trim camera-trajectory samples and rebase both clocks; no device access."""
import argparse,shutil
from pathlib import Path
import numpy as np
from replay_pipeline import check_times,provenance,write_json


def trim(source,destination,seconds):
    source=Path(source).resolve();destination=Path(destination).resolve()
    if not np.isfinite(seconds) or seconds<0:raise ValueError('Trim seconds must be finite and nonnegative')
    if destination.exists():raise ValueError('Destination already exists; source is never overwritten')
    times=np.load(source/'time_s.npy',allow_pickle=False);ns=np.load(source/'timestamp_ns.npy',allow_pickle=False);check_times(times,ns)
    index=int(np.searchsorted(times,times[0]+seconds,side='left'))
    if len(times)-index<2:raise ValueError('Trim must retain at least two frames')
    channels={'time_s','timestamp_ns'}|{side+'_'+name for side in ('left','right') for name in ('root_pose','position_m','quaternion_wxyz','joint_positions_rad')}
    files=sorted(source.glob('*.npy'))
    for name in channels:
        a=np.load(source/(name+'.npy'),allow_pickle=False)
        if not a.ndim or a.shape[0]!=len(times):raise ValueError('Frame count mismatch: '+name)
    destination.mkdir(parents=True)
    for p in files:
        if p.stem in channels:
            a=np.load(p,allow_pickle=False)[index:].copy()
            if p.stem in ('time_s','timestamp_ns'):a-=a[0]
            np.save(destination/p.name,a)
        else:shutil.copy2(p,destination/p.name)
    check_times(np.load(destination/'time_s.npy'),np.load(destination/'timestamp_ns.npy'))
    result={'execution_enabled':False,'source':str(source),'requested_drop_start_s':seconds,'removed_frames':index,'retained_frames':len(times)-index,
            'original_first_kept_time_s':float(times[index]),'original_first_kept_timestamp_ns':int(ns[index]),'duration_s':float(times[-1]-times[index]),
            'clock_rebased':True,'sources':[provenance(p) for p in files],'outputs':[provenance(p) for p in sorted(destination.glob('*.npy'))]}
    write_json(destination/'trim_manifest.json',result);return result

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('destination',type=Path);p.add_argument('--drop-start-s',type=float,default=3.);a=p.parse_args()
    print(trim(a.source,a.destination,a.drop_start_s))
