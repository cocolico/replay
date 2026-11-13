#!/usr/bin/env python3
"""Read-only controller inventory. No power, Runtime, TCP/payload setters or motion APIs."""
import argparse
import json
import os
from pathlib import Path
import sys
import time
import numpy as np

HERE=Path(__file__).resolve().parent
SDK=HERE/'vendor/pyaubo_sdk_0_26'
sys.path.append(str(SDK))
from replay_pipeline import config, write_json


def read_controller(c):
    import pyaubo_sdk
    rpc=pyaubo_sdk.RpcClient()
    rpc.setRequestTimeout(2000)
    try:
        rpc.connect(c['ip'],int(c['rpc_port']))
        if not rpc.hasConnected():raise RuntimeError('RPC connection failed')
        rpc.login('aubo',os.environ.get('AUBO_PASSWORD','123456'))
        if not rpc.hasLogined():raise RuntimeError('RPC login failed; set AUBO_PASSWORD')
        names=list(rpc.getRobotNames());name=c['robot_name']
        if name not in names:raise ValueError('Configured robot name absent')
        robot=rpc.getRobotInterface(name);state=robot.getRobotState();cfg=robot.getRobotConfig()
        result={'ip':c['ip'],'robot_name':name,'captured_unix_s':time.time(),'read_only':True}
        getters={'robot_type':(cfg,'getRobotType'),'payload':(cfg,'getPayload'),'tcp_offset_sdk':(cfg,'getTcpOffset'),
            'joint_min_rad':(cfg,'getJointMinPositions'),'joint_max_rad':(cfg,'getJointMaxPositions'),
            'joint_max_speed_rad_s':(cfg,'getJointMaxSpeeds'),'joint_max_acceleration_rad_s2':(cfg,'getJointMaxAccelerations'),
            'joint_positions_rad':(state,'getJointPositions'),'tcp_pose_sdk':(state,'getTcpPose'),
            'mode':(state,'getRobotModeType'),'safety':(state,'getSafetyModeType'),'steady':(state,'isSteady')}
        for label,(obj,method) in getters.items():
            try:
                value=getattr(obj,method)()
                try:json.dumps(value)
                except TypeError:value=str(value)
                result[label]=value
            except Exception as e:result[label+'_error']=str(e)
        try:
            math_api=rpc.getMath();algorithm=robot.getRobotAlgorithm()
            q=np.asarray(result['joint_positions_rad'],dtype=float)
            fk_samples=[]
            for offset in (np.zeros(6),np.full(6,0.03),np.array([.02,-.03,.01,-.02,.03,-.01])):
                sample_q=q+offset
                pose,error=algorithm.forwardKinematics(sample_q.tolist())
                if error!=0:raise ValueError(f'SDK FK error {error}')
                xyz=np.asarray(pose[:3]);T=np.eye(4);T[:3,3]=xyz
                for index in range(3):
                    shift=[0.,0.,0.,0.,0.,0.];shift[index]=.01
                    T[:3,index]=(np.asarray(math_api.poseTrans(pose,shift)[:3])-xyz)/.01
                fk_samples.append({'q_rad':sample_q.tolist(),'base_T_tcp':T.tolist()})
            result['sdk_fk_samples']=fk_samples
        except Exception as e:result['sdk_fk_samples_error']=str(e)
        return result
    finally:
        try:
            if rpc.hasLogined():rpc.logout()
        finally:
            if rpc.hasConnected():rpc.disconnect()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,default=HERE/'config/replay.json');p.add_argument('--out',type=Path,default=HERE/'output/controller_snapshot.json');a=p.parse_args()
    c=config(a.config);result={'read_only':True,'sides':{}}
    for side in ('left','right'):
        try:
            result['sides'][side]=read_controller(c['sides'][side]);d=result['sides'][side]
            print(side,'type=',d.get('robot_type'),'TCP=',d.get('tcp_offset_sdk'),'payload=',d.get('payload'))
        except Exception as e:result['sides'][side]={'error':str(e)};print(side,'ERROR',e)
    a.out.parent.mkdir(parents=True,exist_ok=True);write_json(a.out,result)
    return int(any('error'in d for d in result['sides'].values()))

if __name__=='__main__':raise SystemExit(main())
