#!/usr/bin/env python3
"""Render measured arm pose plus assumed zero-rotation, direct-mounted Wuji model."""
from pathlib import Path
import json, struct, tarfile
import numpy as np
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import pinocchio as pin
from replay_pipeline import HERE, config, inverse, origin, write_json

ROOT=HERE/'resources'
OUT=HERE/'visuals';OUT.mkdir(exist_ok=True)
FONT=FontProperties(fname=str(HERE/'resources/fonts/NotoSansCJK-Regular.ttc'))
plt.rcParams['font.family']=FONT.get_name();plt.rcParams['axes.unicode_minus']=False


def restore_model_links():
    # All referenced meshes are real local files in the portable bundle.
    return


def stl(path):
    raw=Path(path).read_bytes();n=struct.unpack('<I',raw[80:84])[0]
    if len(raw)!=84+50*n:raise ValueError('Expected binary STL '+str(path))
    d=np.frombuffer(raw,offset=84,count=n,dtype=np.dtype([('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')]))
    return d['v'].astype(float)


def reduce_mesh(tris, voxel):
    flat=tris.reshape(-1,3);cells=np.round(flat/voxel).astype(int)
    unique,inv=np.unique(cells,axis=0,return_inverse=True)
    counts=np.bincount(inv);vertices=np.column_stack([np.bincount(inv,weights=flat[:,i])/counts for i in range(3)])
    faces=inv.reshape(-1,3);keep=(faces[:,0]!=faces[:,1])&(faces[:,1]!=faces[:,2])&(faces[:,0]!=faces[:,2]);faces=faces[keep]
    _,ids=np.unique(np.sort(faces,axis=1),axis=0,return_index=True);faces=faces[np.sort(ids)]
    return vertices,faces


def mesh_at(vertices,T):return vertices@T[:3,:3].T+T[:3,3]


def link_visuals(urdf,q,package_root=None):
    model=pin.buildModelFromUrdf(str(urdf));data=model.createData();pin.forwardKinematics(model,data,np.asarray(q));pin.updateFramePlacements(model,data)
    meshes=[]
    for link in ET.parse(urdf).getroot().findall('link'):
        if not model.existFrame(link.get('name')):continue
        T=data.oMf[model.getFrameId(link.get('name'))].homogeneous
        # The Aubo URDF uses the collision STL geometry also used by its model viewer.
        element=link.find('collision' if package_root else 'visual')
        if element is None:continue
        mesh=element.find('geometry/mesh')
        if mesh is None:continue
        name=mesh.get('filename')
        path=package_root/name.split('package://aubo_description/')[1] if name.startswith('package://') else urdf.parent/name
        tris=stl(path);scale=np.fromstring(mesh.get('scale','1 1 1'),sep=' ');tris*=scale
        placement=T@origin(element)
        vertices,faces=reduce_mesh(tris,.006 if package_root else .003)
        meshes.append({'name':link.get('name'),'v':mesh_at(vertices,placement),'f':faces})
    return model,data,meshes


def axes(ax,T,size=.08):
    for i,(label,col) in enumerate(zip('XYZ',['#db4c48','#2e9e58','#3978ce'])):
        p=T[:3,3];v=T[:3,i]*size;ax.quiver(*p,*v,color=col,linewidth=2,arrow_length_ratio=.14);ax.text(*(p+v*1.12),label,color=col,fontsize=11,weight='bold')


def add_mesh(ax,meshes,T=np.eye(4),color='#dda858',alpha=1):
    from matplotlib.colors import to_rgb
    for m in meshes:
        v=mesh_at(m['v'],T);tri=v[m['f']]
        normal=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);normal/=np.maximum(np.linalg.norm(normal,axis=1,keepdims=True),1e-10)
        light=np.array([.4,-.5,.75]);light/=np.linalg.norm(light)
        intensity=.55+.4*np.abs(normal@light)
        base=np.array(to_rgb('#c374ab' if 'thumb' in m['name'] else color))
        colors=np.clip(intensity[:,None]*base[None,:],0,1)
        poly=Poly3DCollection(tri,facecolor=colors,edgecolor='none',alpha=alpha,zsort='average');ax.add_collection3d(poly)


def frame(ax,center,radius,elev=22,azim=-60):
    ax.set_xlim(center[0]-radius,center[0]+radius);ax.set_ylim(center[1]-radius,center[1]+radius);ax.set_zlim(center[2]-radius,center[2]+radius)
    ax.set_box_aspect([1,1,1]);ax.view_init(elev=elev,azim=azim);ax.set_axis_off()


def main():
    restore_model_links();c=config(HERE/'config/replay.json');snap=json.loads((HERE/'output/controller_snapshot.json').read_text());data={}
    for side in ('left','right'):
        cfg=c['sides'][side];q=snap['sides'][side]['joint_positions_rad']
        model,d,arm=link_visuals(Path(cfg['urdf']),q,ROOT/'models/aubo_description')
        flange=d.oMf[model.getFrameId(cfg['flange_frame'])].homogeneous.copy()
        hand_urdf=Path(c['source_episode_root'])/f'assets/wuji/urdf/{side}.urdf'
        _,_,hand=link_visuals(hand_urdf,np.zeros(20))
        camera_T_base=inverse(np.asarray(json.loads(Path(cfg['handeye']).read_text())['T_base_camera']))
        data[side]={'arm':arm,'hand':hand,'flange':flange,'camera_T_base':camera_T_base}
    fig=plt.figure(figsize=(14,7.5),facecolor='#f4f6f8');fig.suptitle('当前装配假设：Aubo i7H + Wuji 早期 Beta\n法兰中心 = 手根中心；X/Y/Z 同向（待你核对）',fontproperties=FONT,fontsize=17,y=.98)
    for index,side in enumerate(('left','right')):
        d=data[side];ax=fig.add_subplot(1,2,index+1,projection='3d');ax.set_facecolor('#f4f6f8')
        add_mesh(ax,d['arm'],color='#aebbc7');add_mesh(ax,d['hand'],d['flange'],color='#58a4c4' if side=='left' else '#dfa152')
        axes(ax,np.eye(4),.14);axes(ax,d['flange'],.10)
        verts=np.concatenate([m['v'] for m in d['arm']]+[mesh_at(m['v'],d['flange']) for m in d['hand']]);lo,hi=verts.min(0),verts.max(0);frame(ax,(lo+hi)/2,float(max(hi-lo)*.55),26,25 if side=='left' else -65)
        ax.set_proj_type('persp',focal_length=.8);ax.dist=7
        ax.set_title(('左臂 LEFT' if side=='left' else '右臂 RIGHT')+'｜各自在自身基座系展示',fontproperties=FONT,fontsize=14)
    fig.text(.5,.025,'机械臂：上次只读关节快照；手指：模型零位示意，非实时手型。红 X · 绿 Y · 蓝 Z；不代表碰撞或运动仿真。',ha='center',fontproperties=FONT,fontsize=11)
    fig.savefig(OUT/'assembly-overview.png',dpi=165,bbox_inches='tight');plt.close(fig)
    fig=plt.figure(figsize=(15,8),facecolor='#f4f6f8');fig.suptitle('法兰端面方向对照｜从手指端朝安装面看（沿 −Z 看）',fontproperties=FONT,fontsize=17,y=.98)
    angles=[0,90,180,-90]
    for row,side in enumerate(('left','right')):
        for column,angle in enumerate(angles):
            ax=fig.add_subplot(2,4,row*4+column+1,projection='3d');ax.set_facecolor('#f4f6f8');T=np.eye(4);T[:3,:3]=Rotation.from_euler('z',angle,degrees=True).as_matrix()
            add_mesh(ax,data[side]['hand'],T,color='#58a4c4' if side=='left' else '#dfa152');axes(ax,np.eye(4),.095)
            circle=np.linspace(0,2*np.pi,100);ax.plot(.034*np.cos(circle),.034*np.sin(circle),np.zeros(100),color='#697682',linewidth=1)
            frame(ax,np.array([0,0,.08]),.13,90,-90);ax.dist=7
            ax.set_title(('左手' if side=='left' else '右手')+f'：绕法兰 Z {angle:+d}°',fontproperties=FONT,fontsize=12)
    fig.text(.5,.035,'坐标箭头固定属于 Aubo 法兰；只有手模型绕 Z 旋转。正角按右手定则，当前程序采用两侧 0°。',ha='center',fontproperties=FONT,fontsize=11)
    fig.savefig(OUT/'mount-angle-options.png',dpi=165,bbox_inches='tight');plt.close(fig)
    payload={}
    for side,d in data.items():
        payload[side]={k:[{'n':m['name'],'v':np.round(m['v'],5).tolist(),'f':m['f'].tolist()} for m in d[k]] for k in ('arm','hand')}
        payload[side]['flange']=d['flange'].round(9).tolist()
    (OUT/'mount-geometry.json').write_text(json.dumps(payload,separators=(',',':')))
    print('Geometry bytes', (OUT/'mount-geometry.json').stat().st_size)
    print('Rendered',OUT)

if __name__=='__main__':main()
