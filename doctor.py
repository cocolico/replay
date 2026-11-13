#!/usr/bin/env python3
"""Local dependency/resource check. Never constructs a controller or hand connection."""
import argparse,importlib,json,platform,tempfile
from pathlib import Path
from replay_pipeline import HERE,config,read_json,prepare_camera,calibrated_chain


def check():
    report={'hardware_connected':False,'modules':{},'configs':{},'problems':[]}
    for name in ('numpy','scipy','pinocchio','coal','pyaubo_sdk','wuji_sdk'):
        try:
            module=importlib.import_module(name);report['modules'][name]={'version':str(getattr(module,'__version__','SDK binary')),'path':str(module.__file__)}
        except ImportError as error:report['problems'].append(name+': '+str(error))
    for p in sorted((HERE/'config').glob('*.json')):
        d=read_json(p)
        if 'source_episode_root' not in d or 'sides' not in d:continue
        c=config(p);paths=[c[k] for k in ('source_episode_root','source_franka_handeye','table_mapping','current_table_observation','camera_trajectory_root','camera_table_mapping','camera_source_observation') if c.get(k)]
        paths += [entry[k] for entry in c['sides'].values() for k in ('handeye','urdf')]
        for value in paths:
            target=Path(value)
            if not target.exists():report['problems'].append(p.name+': missing '+str(target))
            elif not target.resolve().is_relative_to(HERE.resolve()):report['problems'].append(p.name+': external resource '+str(target))
        report['configs'][p.name]={'all_resources_local':all(Path(v).exists() and Path(v).resolve().is_relative_to(HERE.resolve()) for v in paths)}
        if not report['problems']:
            try:
                if c.get('camera_source_observation'):
                    with tempfile.TemporaryDirectory() as out:result=prepare_camera(c,Path(out));report['configs'][p.name]['frames']=result['frames']
                else:calibrated_chain(c)
            except Exception as error:report['problems'].append(p.name+': '+str(error))
    report['ok']=not report['problems'];return report

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--offline',action='store_true',help='Explicitly select local checks only (also the default)');p.parse_args()
    result=check();print(json.dumps(result,ensure_ascii=False,indent=2));raise SystemExit(0 if result['ok'] else 1)
