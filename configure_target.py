#!/usr/bin/env python3
"""Import a target workcell's calibration/identities and activate local relative configs."""
import argparse,copy,hashlib,json,os,shutil
from pathlib import Path
import numpy as np
from replay_pipeline import HERE,config,read_json,write_json,provenance,transform,inverse,calibrated_chain


def configure(spec_path,root=HERE):
    root=Path(root).resolve();spec_path=Path(spec_path).resolve();spec=read_json(spec_path)
    def source(name):
        value=spec.get(name)
        if not isinstance(value,str) or not value.strip():raise ValueError('Fill target spec field: '+name)
        p=Path(value);p=(spec_path.parent/p).resolve() if not p.is_absolute() else p
        if not p.is_file():raise ValueError('Missing target file: '+str(p))
        return p
    obs_source=source('table_observation');obs=read_json(obs_source);transform(obs['camera_T_board'])
    if not obs.get('camera_id'):raise ValueError('Target table observation requires camera_id')
    settings=read_json(root/'config/runtime.json');sides={};eye_sources={}
    for side in ('left','right'):
        data=spec.get('sides',{}).get(side,{})
        for key in ('ip','robot_name','hand_address','hand_serial','handeye'):
            if not isinstance(data.get(key),str) or not data[key].strip():raise ValueError('Fill sides.'+side+'.'+key)
        if data.get('mount_confirmed') is not True:raise ValueError('Confirm actual installed flange_T_wuji_base for '+side)
        mount=transform(data['flange_T_wuji_base'])
        if mount.shape!=(4,4):raise ValueError('Expected fixed 4x4 mount')
        eye=Path(data['handeye']);eye=(spec_path.parent/eye).resolve() if not eye.is_absolute() else eye
        h=read_json(eye);transform(h['T_base_camera'])
        if h.get('parent_frame')!=side+'_aubo_base' or h.get('child_frame')!='camera_color_optical_frame' or h.get('mode')!='eye_to_hand':raise ValueError('Wrong target hand-eye frames: '+side)
        if not np.allclose(h['metadata']['K'],obs['camera_intrinsics']['K'],atol=1e-5) or h['metadata']['image_size']!=[obs['camera_intrinsics']['height'],obs['camera_intrinsics']['width']]:raise ValueError('Target hand-eye/table intrinsics or resolution mismatch: '+side)
        sides[side]=dict(data,flange_T_wuji_base=mount.tolist());eye_sources[side]=eye
        settings['hands'][side]={'address':data['hand_address'],'serial':data['hand_serial']}
    if settings['hands']['left']['serial']==settings['hands']['right']['serial']:raise ValueError('Left and right hand serial numbers must differ')
    # Validate source-board relationships before writing any activated configuration.
    human=config(root/'config/human_replay.json');source_paths={human['camera_source_id']:Path(human['camera_source_observation']),
        'franka-sharpa':root/'resources/calibration/source_board_observations/franka-sharpa/observation.json'}
    for key,p in source_paths.items():
        src=read_json(p)
        if src['camera_id']!=key or src['board_fingerprint']!=obs['board_fingerprint']:raise ValueError('Source/target printed board frame mismatch: '+key)
    target=root/'resources/calibration/target';target.mkdir(parents=True,exist_ok=True)
    backup=root/'history/reference_configs'
    if not backup.exists():shutil.copytree(root/'config',backup)
    shutil.copy2(obs_source,target/'observation.json')
    for side,p in eye_sources.items():shutil.copy2(p,target/(side+'_handeye.json'))
    mapping_paths={}
    for key,source_path in source_paths.items():
        src=read_json(source_path);T=transform(obs['camera_T_board'])@inverse(transform(src['camera_T_board']))
        m={'schema_version':1,'source_camera_id':key,'target_camera_id':obs['camera_id'],'board_fingerprint':obs['board_fingerprint'],
           'target_camera_T_source_camera':T.tolist(),'source_camera_T_target_camera':inverse(T).tolist(),
           'source_observation_sha256':provenance(source_path)['sha256'],'target_observation_sha256':provenance(target/'observation.json')['sha256'],
           'interpretation':'Corresponding task-board frames between recording and target workcell; not a surveyed inter-site transform.'}
        dest=target/('mapping_from_'+key+'.json');write_json(dest,m);mapping_paths[key]=dest
    changed=[]
    for p in sorted((root/'config').glob('*.json')):
        data=read_json(p)
        if 'sides' not in data or 'source_episode_root' not in data:continue
        # Existing config paths are already relative; retain dataset and source calibration selection.
        data['current_table_observation']=os.path.relpath(target/'observation.json',p.parent)
        data['table_mapping']=os.path.relpath(mapping_paths['franka-sharpa'],p.parent)
        if data.get('camera_source_observation'):data['camera_table_mapping']=os.path.relpath(mapping_paths[data['camera_source_id']],p.parent)
        for side,entry in sides.items():
            data['sides'][side].update(ip=entry['ip'],rpc_port=int(entry.get('rpc_port',30004)),robot_name=entry['robot_name'],
                handeye=os.path.relpath(target/(side+'_handeye.json'),p.parent),flange_T_wuji_base=entry['flange_T_wuji_base'],
                mount_confirmed=True,model_confirmed=False,initial_q_rad=[0.]*6,
                seed_source='Numerical seed only; fresh controller state and identity are read before planning')
        write_json(p,data);changed.append(p)
    write_json(root/'config/runtime.json',settings)
    c=config(root/'config/human_replay.json');calibrated_chain(c,c['camera_table_mapping'],c['camera_source_id'])
    return {'configured':True,'profiles_updated':len(changed),'devices_contacted':False,'human_config':str(root/'config/human_replay.json'),'runtime_config':str(root/'config/runtime.json')}

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--spec',type=Path,required=True);a=p.parse_args()
    try:print(json.dumps(configure(a.spec),ensure_ascii=False,indent=2))
    except (ValueError,KeyError,OSError) as e:raise SystemExit(str(e))
