"""Device adapters. Construction and all Aubo kinematics methods are read-only."""
from pathlib import Path
import sys,os,time,threading
import numpy as np
from scipy.spatial.transform import Rotation
from replay_pipeline import HERE,transform
MATCHED_SDK=HERE/'vendor/pyaubo_sdk_0_26'
if MATCHED_SDK.exists():sys.path.insert(0,str(MATCHED_SDK))
WUJI_SDK=HERE/'vendor/wuji_sdk_2026_11_9'
if WUJI_SDK.exists():sys.path.insert(0,str(WUJI_SDK))


class Aubo:
    def __init__(self,cfg):
        import pyaubo_sdk
        self.cfg=cfg;self.rpc=pyaubo_sdk.RpcClient();self.rpc.setRequestTimeout(2000)
        try:
            self.rpc.connect(cfg['ip'],int(cfg['rpc_port']))
            if not self.rpc.hasConnected():raise RuntimeError('Aubo connection failed')
            self.rpc.login('aubo',os.environ.get('AUBO_PASSWORD','123456'))
            if not self.rpc.hasLogined():raise RuntimeError('Aubo login failed')
            self.robot=self.rpc.getRobotInterface(cfg['robot_name']);self.state=self.robot.getRobotState()
            self.config=self.robot.getRobotConfig();self.algorithm=self.robot.getRobotAlgorithm();self.math=self.rpc.getMath()
            identity=np.asarray(self.math.rpyToQuaternion([0.,0.,0.]))
            if np.allclose(abs(identity),[1,0,0,0]):self.quaternion_order='wxyz'
            elif np.allclose(abs(identity),[0,0,0,1]):self.quaternion_order='xyzw'
            else:raise ValueError('Unknown SDK quaternion order')
        except Exception:
            self.close();raise

    def close(self):
        try:
            if self.rpc.hasLogined():self.rpc.logout()
        finally:
            if self.rpc.hasConnected():self.rpc.disconnect()

    def pose_matrix(self,pose):
        xyz=np.asarray(pose[:3]);T=np.eye(4);T[:3,3]=xyz
        for i in range(3):
            shift=[0.,0.,0.,0.,0.,0.];shift[i]=.01
            T[:3,i]=(np.asarray(self.math.poseTrans(list(pose),shift)[:3])-xyz)/.01
        return transform(T)

    def matrix_pose(self,T):
        T=transform(T);q=Rotation.from_matrix(T[:3,:3]).as_quat()
        if self.quaternion_order=='wxyz':q=q[[3,0,1,2]]
        pose=T[:3,3].tolist()+list(self.math.quaternionToRpy(q.tolist()))
        if not np.allclose(self.pose_matrix(pose),T,atol=1e-8):raise ValueError('SDK pose conversion roundtrip failed')
        return pose

    def fk(self,q):
        pose,err=self.algorithm.forwardToolKinematics(np.asarray(q,dtype=float).tolist())
        if err!=0:raise ValueError(f'Aubo FK error {err}')
        return self.pose_matrix(pose)

    def ik(self,T,seed):
        q,err=self.algorithm.inverseToolKinematics(np.asarray(seed,dtype=float).tolist(),self.matrix_pose(T))
        if err!=0:raise ValueError(f'Aubo IK error {err}')
        q=np.asarray(q,dtype=float)
        if q.shape!=(6,) or not np.all(np.isfinite(q)):raise ValueError('Bad Aubo IK result')
        return q

    def snapshot(self):
        cfg=self.config;s=self.state
        return {'captured_unix_s':time.time(),'ip':self.cfg['ip'],'robot_name':self.cfg['robot_name'],
            'robot_type':cfg.getRobotType(),'payload':cfg.getPayload(),'tcp_offset_sdk':cfg.getTcpOffset(),
            'joint_positions_rad':s.getJointPositions(),'joint_min_rad':cfg.getJointMinPositions(),
            'joint_max_rad':cfg.getJointMaxPositions(),'joint_max_speed_rad_s':cfg.getJointMaxSpeeds(),
            'joint_max_acceleration_rad_s2':cfg.getJointMaxAccelerations(),'kinematics_real':cfg.getKinematicsParam(True),
            'kinematics_nominal':cfg.getKinematicsParam(False),'mode':str(s.getRobotModeType()),'safety':str(s.getSafetyModeType()),
            'steady':s.isSteady(),'runtime_status':str(self.rpc.getRuntimeMachine().getStatus()),
            'servo_mode_enabled':bool(self.robot.getMotionControl().isServoModeEnabled()),'base_T_flange':self.fk(s.getJointPositions()).tolist()}
