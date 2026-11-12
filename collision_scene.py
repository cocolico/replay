"""Calibrated-DH arm + Wuji triangle-mesh collision model in current camera frame."""
from pathlib import Path
import struct,xml.etree.ElementTree as ET
import numpy as np
import coal
import pinocchio as pin
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from replay_pipeline import SIDES,origin,inverse,read_json
from dh_kinematics import placements


def stl_mesh(path):
    raw=Path(path).read_bytes();n=struct.unpack('<I',raw[80:84])[0]
    if len(raw)!=84+50*n:raise ValueError('Expected binary STL '+str(path))
    triangles=np.frombuffer(raw,offset=84,count=n,dtype=np.dtype([('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')]))['v'].astype(float)
    v,indices=np.unique(triangles.reshape(-1,3),axis=0,return_inverse=True);f=indices.reshape(-1,3)
    vv=coal.StdVec_Vec3s();ff=coal.StdVec_Triangle()
    for point in v:vv.append(point)
    for face in f:ff.append(coal.Triangle(*map(int,face)))
    bvh=coal.BVHModelOBBRSS();bvh.beginModel(len(f),len(v));bvh.addSubModel(vv,ff);bvh.endModel()
    hull=v[ConvexHull(v).vertices]
    return bvh,hull


def tf(T):return coal.Transform3s(np.asarray(T[:3,:3]),np.asarray(T[:3,3]))


class Scene:
    def __init__(self,c,controllers,clearance=.005):
        self.c=c;self.controllers=controllers;self.clearance=clearance;self.shapes=[];self.arm_corrections={};self.hand_trees={};self.disabled=set();self.base={}
        for side in SIDES:
            cfg=c['sides'][side];self.base[side]=inverse(np.asarray(read_json(cfg['handeye'])['T_base_camera']))
            urdf=Path(cfg['urdf']);root=urdf.parents[1];tree=ET.parse(urdf).getroot();model=pin.buildModelFromUrdf(str(urdf));data=model.createData()
            links=['shoulder_Link','upperArm_Link','foreArm_Link','wrist1_Link','wrist2_Link','wrist3_Link']
            pin.forwardKinematics(model,data,np.zeros(6));pin.updateFramePlacements(model,data)
            nominal=placements(np.zeros(6),controllers[side]['kinematics_nominal'])
            self.arm_corrections[side]={name:inverse(nominal[i])@data.oMf[model.getFrameId(name)].homogeneous for i,name in enumerate(links)}
            test=np.array([.3,-.4,.8,.2,1.1,-.3]);poses=placements(test,controllers[side]['kinematics_nominal']);pin.forwardKinematics(model,data,test);pin.updateFramePlacements(model,data)
            for i,name in enumerate(links):
                if not np.allclose(poses[i]@self.arm_corrections[side][name],data.oMf[model.getFrameId(name)].homogeneous,atol=1e-8):raise ValueError('DH/URDF intermediate-frame alignment failed')
            arm_adjacency=set()
            for d in ET.parse(urdf.with_suffix('.srdf')).getroot().findall('disable_collisions'):
                arm_adjacency.add(frozenset((f'{side}:arm:{d.get("link1")}',f'{side}:arm:{d.get("link2")}')))
            self.disabled|=arm_adjacency
            for link in tree.findall('link'):
                for coll in link.findall('collision'):
                    m=coll.find('geometry/mesh')
                    if m is None:continue
                    path=root/m.get('filename').split('package://aubo_description/')[1]
                    self.add(side,'arm',link.get('name'),path,origin(coll))
            hand=Path(c['source_episode_root'])/f'assets/wuji/urdf/{side}.urdf';tree=ET.parse(hand).getroot();joints=tree.findall('joint');self.hand_trees[side]=joints
            parent={link.get('name'):link.get('name') for link in tree.findall('link')}
            def find(n):
                while parent[n]!=n:n=parent[n]
                return n
            for j in joints:
                if j.get('type')=='fixed':parent[find(j.find('child').get('link'))]=find(j.find('parent').get('link'))
            connected=set()
            for j in joints:connected.add(frozenset((find(j.find('parent').get('link')),find(j.find('child').get('link')))))
            # Each finger's two-axis knuckle has overlapping joint-cover meshes.
            # Exempt graph-distance two only on the same kinematic branch, never across fingers.
            ancestors={find(n):set() for n in parent}
            for j in joints:
                a,b=find(j.find('parent').get('link')),find(j.find('child').get('link'))
                if a!=b:ancestors[b].add(a)
            for child,parents in list(ancestors.items()):
                for p in list(parents):
                    for grandparent in ancestors[p]:connected.add(frozenset((child,grandparent)))
            link_names=list(parent)
            for i,a in enumerate(link_names):
                for b in link_names[i+1:]:
                    if find(a)==find(b) or frozenset((find(a),find(b))) in connected:self.disabled.add(frozenset((f'{side}:hand:{a}',f'{side}:hand:{b}')))
            for link in tree.findall('link'):
                name=link.get('name')
                if find(name)==find(side[0]+'_base_link'):
                    self.disabled.add(frozenset((f'{side}:arm:wrist3_Link',f'{side}:hand:{name}')))
                for coll in link.findall('collision'):
                    m=coll.find('geometry/mesh')
                    if m is not None:self.add(side,'hand',name,hand.parent/m.get('filename'),origin(coll))
        obs=read_json(c['current_table_observation']);self.board=np.asarray(obs['camera_T_board']);self.normal=self.board[:3,2];self.plane_point=self.board[:3,3]
        self.pairs=[(i,j) for i in range(len(self.shapes)) for j in range(i+1,len(self.shapes)) if frozenset((self.shapes[i]['name'],self.shapes[j]['name'])) not in self.disabled]

    def add(self,side,kind,link,path,offset):
        bvh,hull=stl_mesh(path)
        self.shapes.append({'side':side,'kind':kind,'link':link,'name':f'{side}:{kind}:{link}','geometry':bvh,'hull':hull,'offset':offset})

    def shape_poses(self,arms,hands):
        world={}
        for side in SIDES:
            chain=placements(arms[side],self.controllers[side]['kinematics_real']);base=self.base[side]
            for i,name in enumerate(['shoulder_Link','upperArm_Link','foreArm_Link','wrist1_Link','wrist2_Link','wrist3_Link']):world[(side,'arm',name)]=base@chain[i]@self.arm_corrections[side][name]
            world[(side,'arm','base_link')]=base
            joint_names=[f'{side[0]}_{f}_{s}' for f in ('thumb','index_finger','middle_finger','ring_finger','pinky') for s in (('cmc_flex','cmc_abd','mcp','ip') if f=='thumb' else ('mcp_flex','mcp_abd','pip','dip'))]
            q=dict(zip(joint_names,hands[side],strict=True));root=side[0]+'_base_link';poses={root:base@chain[-1]@np.asarray(self.c['sides'][side]['flange_T_wuji_base'])}
            pending=list(self.hand_trees[side])
            while pending:
                remaining=[]
                for j in pending:
                    parent=j.find('parent').get('link');child=j.find('child').get('link')
                    if parent not in poses:remaining.append(j);continue
                    local=origin(j)
                    if j.get('type')!='fixed':
                        v=np.eye(4);axis=np.fromstring(j.find('axis').get('xyz'),sep=' ');v[:3,:3]=Rotation.from_rotvec(axis*q[j.get('name')]).as_matrix();local=local@v
                    poses[child]=poses[parent]@local
                if len(remaining)==len(pending):raise ValueError('Disconnected hand tree')
                pending=remaining
            for name,T in poses.items():world[(side,'hand',name)]=T
        return [world[(s['side'],s['kind'],s['link'])]@s['offset'] for s in self.shapes]

    def check(self,arms,hands,max_findings=10):
        poses=self.shape_poses(arms,hands);bounds=[];findings=[]
        for shape,T in zip(self.shapes,poses,strict=True):
            v=shape['hull']@T[:3,:3].T+T[:3,3];bounds.append((v.min(0),v.max(0)))
            # Robot bases are mounting interfaces; the rest is checked against an infinite calibrated board plane.
            if not(shape['kind']=='arm' and shape['link']=='base_link'):
                min_height=float(np.min(-(v-self.plane_point)@self.normal))
                if min_height<self.clearance:
                    findings.append({'a':shape['name'],'b':'calibrated_table_halfspace','clearance_m':min_height})
                    if len(findings)>=max_findings:return findings
        request=coal.DistanceRequest();request.enable_nearest_points=False
        contact_request=coal.CollisionRequest();contact_request.num_max_contacts=1;contact_request.enable_contact=False
        for i,j in self.pairs:
            lo_i,hi_i=bounds[i];lo_j,hi_j=bounds[j]
            if np.any(lo_i>hi_j+self.clearance) or np.any(lo_j>hi_i+self.clearance):continue
            hand_self=(self.shapes[i]['side']==self.shapes[j]['side'] and self.shapes[i]['kind']==self.shapes[j]['kind']=='hand')
            if hand_self:
                contact_result=coal.CollisionResult()
                hit=coal.collide(self.shapes[i]['geometry'],tf(poses[i]),self.shapes[j]['geometry'],tf(poses[j]),contact_request,contact_result)>0
                distance=0.0
            else:
                result=coal.DistanceResult();distance=coal.distance(self.shapes[i]['geometry'],tf(poses[i]),self.shapes[j]['geometry'],tf(poses[j]),request,result)
                hit=distance<=self.clearance
            if hit:
                findings.append({'a':self.shapes[i]['name'],'b':self.shapes[j]['name'],'clearance_m':float(distance)})
                if len(findings)>=max_findings:return findings
        return findings
