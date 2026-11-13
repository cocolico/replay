"""Move both Wuji hands with recorded arm FK and retained retarget root offsets.

Scene frame is the local assembled URDF's lychee_root. Its nominal base mounts
are not claimed to be calibrated to the episode RGB. No arm IK is generated.
"""
import argparse
import json
import hashlib
import copy
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path
from retarget import HERE, Hand, Rotation, np, mujoco, pose_attrs, stat
from visualize import viewer_model, text, letterbox, project

ASSEMBLY=HERE/'assets/motion/source_assembly.urdf'


def arm_chains():
    r=ET.parse(ASSEMBLY).getroot();joints=r.findall('joint');bychild={j.find('child').get('link'):j for j in joints}
    chains={}
    for side in ['left','right']:
        link=f'{side}_hand_C_MC';chain=[]
        while link in bychild:
            j=bychild[link];chain.insert(0,j);link=j.find('parent').get('link')
        assert link=='lychee_root';assert sum(j.get('type')=='revolute' for j in chain)==7
        chains[side]=chain
    return chains


def prepare(ep):
    source=np.load(ep/'source_sharpa.npz');z=np.load(ep/'wuji_trajectory.npz');n=len(z['time_s']);chains=arm_chains();out={'time_s':z['time_s'],'timestamp_ns':z['timestamp_ns'],'alignment_version':z['alignment_version']}
    doc=ET.Element('mujoco');ET.SubElement(doc,'compiler',angle='radian',autolimits='true',fusestatic='false');world=ET.SubElement(doc,'worldbody');bodies={'lychee_root':ET.SubElement(world,'body',name='lychee_root')}
    for side,chain in chains.items():
        for j in chain:
            child=j.find('child').get('link');parent=j.find('parent').get('link')
            if child in bodies:continue
            b=ET.SubElement(bodies[parent],'body',name=child,**pose_attrs(j.find('origin')));bodies[child]=b
            ET.SubElement(b,'inertial',mass='.01',pos='0 0 0',diaginertia='.001 .001 .001')
            if j.get('type')=='revolute':ET.SubElement(b,'joint',name=j.get('name'),axis=j.find('axis').get('xyz'))
    model=mujoco.MjModel.from_xml_string(ET.tostring(doc).decode());data=mujoco.MjData(model);reports={}
    for side,chain in chains.items():
        names=list(source[f'{side}_arm_names']);q=source[f'{side}_arm_q'][:,[names.index(f'{side}_fr3_joint{i}') for i in range(1,8)]]
        # Independent homogeneous-transform chain, using named measured angles.
        transform=np.tile(np.eye(4),(n,1,1));ji=0
        for j in chain:
            o=j.find('origin');a=np.eye(4)
            if o is not None:
                a[:3,3]=np.fromstring(o.get('xyz','0 0 0'),sep=' ');a[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix()
            b=np.tile(np.eye(4),(n,1,1))
            if j.get('type')=='revolute':
                b[:,:3,:3]=Rotation.from_rotvec(q[:,ji,None]*np.fromstring(j.find('axis').get('xyz'),sep=' ')).as_matrix();ji+=1
            transform=transform@a@b
        root=np.tile(np.eye(4),(n,1,1));root[:,:3,:3]=Rotation.from_rotvec(z[f'{side}_root_rotvec_in_sharpa_rad']).as_matrix();root[:,:3,3]=z[f'{side}_root_xyz_in_sharpa_m']
        wuji=transform@root
        actual=np.einsum('nij,nkj->nki',transform[:,:3,:3],z[f'{side}_wuji_points_in_sharpa_m'])+transform[:,:3,3,None].transpose(0,2,1)
        target=np.einsum('nij,nkj->nki',transform[:,:3,:3],z[f'{side}_source_points_m'])+transform[:,:3,3,None].transpose(0,2,1)
        arm_joint_names=[j.get('name') for j in chain if j.get('type')=='revolute'];maxpos=0.;maxrot=0.
        for fi in range(n):
            for name,value in zip(arm_joint_names,q[fi]):data.qpos[model.joint(name).qposadr[0]]=value
            mujoco.mj_forward(model,data);bid=model.body(f'{side}_hand_C_MC').id
            maxpos=max(maxpos,float(abs(data.xpos[bid]-transform[fi,:3,3]).max()));maxrot=max(maxrot,float(abs(data.xmat[bid].reshape(3,3)-transform[fi,:3,:3]).max()))
        assert maxpos<1e-8 and maxrot<1e-8
        out.update({f'{side}_scene_T_sharpa_root':transform,f'{side}_scene_T_wuji_base':wuji,f'{side}_wuji_points_scene_m':actual,f'{side}_sharpa_points_scene_m':target})
        displacement=np.linalg.norm(wuji[:,:3,3]-wuji[0,:3,3],axis=-1)
        rotation=np.rad2deg(Rotation.from_matrix(wuji[:,:3,:3]@wuji[0,:3,:3].T).magnitude())
        reports[side]=dict(independent_arm_fk_position_max_abs_m=maxpos,independent_arm_fk_rotation_max_abs=maxrot,max_root_displacement_from_start_m=float(displacement.max()),max_root_rotation_from_start_deg=float(rotation.max()),root_step_m=stat(np.linalg.norm(np.diff(wuji[:,:3,3],axis=0),axis=-1)))
    np.savez_compressed(ep/'wuji_motion_scene.npz',**out)
    report=dict(scene_frame='lychee_root from local nominal assembled dual FR3+Sharpa URDF',source_assembly_original='/home/descfly/lpy/li-agent-mono/lia-sim/src/lia_sim/assets/robots/franka_sharpa/franka_sharpa.urdf',assembly_sha256=hashlib.sha256(ASSEMBLY.read_bytes()).hexdigest(),method='recorded 7 arm joint states -> URDF FK to Sharpa root -> retained Sharpa-root-to-Wuji-base transform',frames=n,sides=reports,limitations=['Nominal inter-arm mounting and Sharpa mounting from local assembly, not verified against recorded workcell hash.','No camera extrinsics applied; visualization is not a calibrated RGB overlay.','Reference floor/grid is for orientation only, not a reconstructed table.','No target arm IK, contact or collision validation.'])
    report['alignment_version']=str(z['alignment_version'])
    report['orientation_note']='Already corrected in conversion: left local Z +90 deg, right -90 deg. No additional rotation in playback.'
    (ep/'motion_validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)


def make_scene(floor_z):
    # Reuse the verified two-hand viewer assets and add a visual reference grid.
    doc=ET.Element('mujoco');ET.SubElement(doc,'compiler',angle='radian',autolimits='true');ET.SubElement(doc,'option',gravity='0 0 0')
    vis=ET.SubElement(doc,'visual');ET.SubElement(vis,'global',offwidth='1280',offheight='1000');ET.SubElement(vis,'headlight',ambient='.5 .5 .5',diffuse='.7 .7 .7');ET.SubElement(vis,'rgba',haze='.05 .07 .10 1')
    asset=ET.SubElement(doc,'asset');world=ET.SubElement(doc,'worldbody')
    ET.SubElement(asset,'texture',name='grid',type='2d',builtin='checker',rgb1='.14 .18 .23',rgb2='.18 .23 .29',width='512',height='512')
    ET.SubElement(asset,'material',name='floor',texture='grid',texrepeat='12 12',reflectance='.08')
    ET.SubElement(world,'geom',name='reference_floor',type='plane',size='2 2 .02',pos=f'0 0 {floor_z}',material='floor',contype='0',conaffinity='0')
    ET.SubElement(world,'light',pos='1 0 2',dir='-0.5 0 -1',directional='true',diffuse='.6 .6 .6')
    for side in ['left','right']:
        h=Hand('wuji',side,True);tree=ET.fromstring(h.xml)
        for mesh in tree.findall('asset/mesh'):mesh.set('name',side+'_'+mesh.get('name'));asset.append(mesh)
        body=copy.deepcopy(tree.find('worldbody/body'));body.insert(0,ET.Element('freejoint',name=side+'_floating'));ET.SubElement(body,'inertial',mass='.1',pos='0 0 0',diaginertia='.001 .001 .001')
        for geom in body.findall('.//geom'):
            if geom.get('mesh'):geom.set('mesh',side+'_'+geom.get('mesh'))
            geom.set('rgba','.40 .68 .92 1' if side=='left' else '.93 .65 .37 1')
        for site in body.findall('.//site'):site.set('name',side+'_'+site.get('name'))
        world.append(body)
    return mujoco.MjModel.from_xml_string(ET.tostring(doc).decode())


def set_frame(model,data,z,motion,fi):
    for side in ['left','right']:
        t=motion[f'{side}_scene_T_wuji_base'][fi];a=model.joint(side+'_floating').qposadr[0]
        data.qpos[a:a+3]=t[:3,3];data.qpos[a+3:a+7]=Rotation.from_matrix(t[:3,:3]).as_quat()[[3,0,1,2]]
        for name,value in zip(z[f'{side}_wuji_joint_names'],z[f'{side}_wuji_joints_rad'][fi]):data.qpos[model.joint(str(name)).qposadr[0]]=value
    mujoco.mj_forward(model,data)
    for side in ['left','right']:
        actual=np.array([data.site_xpos[model.site(f'{side}_point{i}').id] for i in range(21)])
        assert abs(actual-motion[f'{side}_wuji_points_scene_m'][fi]).max()<1e-8


def render(ep,rgb_dir,preview=False):
    import cv2
    import imageio_ffmpeg
    z=np.load(ep/'wuji_trajectory.npz');motion=np.load(ep/'wuji_motion_scene.npz');n=len(z['time_s'])
    points=np.concatenate([motion[f'{s}_wuji_points_scene_m'].reshape(-1,3) for s in ['left','right']]);low,high=points.min(0),points.max(0)
    model=make_scene(float(low[2]-.08));data=mujoco.MjData(model);renderer=mujoco.Renderer(model,height=520,width=1280)
    camera=mujoco.MjvCamera();mujoco.mjv_defaultCamera(camera);camera.azimuth=0;camera.elevation=-30;camera.lookat[:]=(low+high)/2;camera.distance=max(float(np.max(high-low))*.5,.6)
    opt=mujoco.MjvOption();opt.sitegroup[:]=0;selected=np.linspace(0,n-1,6).round().astype(int).tolist();shots=[];writer=None
    # Fit full mesh bounding spheres, including the mounting plates beyond the
    # 21 landmarks. Every output frame participates in the camera fit.
    geometry_points=[];ids=np.flatnonzero(model.geom_type==mujoco.mjtGeom.mjGEOM_MESH)
    sphere_offsets=model.geom_rbound[ids,None,None]*np.concatenate([np.eye(3),-np.eye(3)])[None,:,:]
    for fi in range(n):
        set_frame(model,data,z,motion,fi);geometry_points.append((data.geom_xpos[ids,None,:]+sphere_offsets).reshape(-1,3))
    fit_points=np.concatenate(geometry_points)
    set_frame(model,data,z,motion,0)
    for _ in range(15):
        renderer.update_scene(data,camera=camera,scene_option=opt);pp=project(np.empty((520,1280,3)),renderer.scene,fit_points)
        if (pp.min(0)>[70,60]).all() and (pp.max(0)<[1210,460]).all():break
        camera.distance*=1.1
    else:raise RuntimeError('Cannot fit full hand motion into scene view')
    if not preview:
        temporary_video=ep/f'.motion-{uuid.uuid4().hex}.mp4'
        writer=imageio_ffmpeg.write_frames(str(temporary_video),(1280,960),fps=30,codec='libx264',pix_fmt_in='rgb24',output_params=['-crf','19','-preset','fast']);writer.send(None)
    for fi in (selected if preview else range(n)):
        set_frame(model,data,z,motion,fi);renderer.update_scene(data,camera=camera,scene_option=opt);im=renderer.render().copy()
        for side,col in [('left',(85,174,255)),('right',(255,174,85))]:
            trail=motion[f'{side}_scene_T_wuji_base'][max(0,fi-60):fi+1,:3,3];pp=project(im,renderer.scene,trail)
            for p1,p2 in zip(pp[:-1],pp[1:]):cv2.line(im,tuple(p1),tuple(p2),col,2,cv2.LINE_AA)
            text(im,side.upper(),tuple(pp[-1]+[6,-8]),.5,col)
        frame=np.full((960,1280,3),(18,24,34),np.uint8)
        text(frame,f'{ep.name.upper()} | BOTH HANDS: TRANSLATION + ROTATION + FINGER MOTION',(20,28),.64)
        text(frame,f'Frame {fi+1}/{n}    Time {z["time_s"][fi]:.2f} s    Original timing: 30 Hz',(20,54),.57)
        rgb=cv2.imread(str(rgb_dir/f'{int(z["rgb_index"][fi]):05d}.jpg'));assert rgb is not None
        frame[64:384,:640]=letterbox(cv2.cvtColor(rgb,cv2.COLOR_BGR2RGB),640,320);frame[420:940]=im
        text(frame,'RECORDED RGB',(18,100),.55)
        for row,line in enumerate(['Blue: LEFT Wuji hand | Orange: RIGHT Wuji hand','Wrist motion recovered from recorded arm joints','Confirmed local Z: LEFT +90 deg / RIGHT -90 deg','Corrected poses loaded directly from output data','Scene: local nominal dual-arm URDF assembly','Grid is a visual reference, not the actual table','No target-arm IK or contact simulation']):text(frame,line,(650,100+35*row),.52)
        text(frame,'WUJI BIMANUAL MOTION | Fixed view of the complete trajectory',(20,410),.58)
        text(frame,'Two-second wrist trails | RGB shown separately (not a calibrated overlay)',(20,955),.49)
        if writer is not None:writer.send(np.ascontiguousarray(frame))
        if fi in selected:
            cv2.imwrite(str(ep/f'motion_preview_{fi:04d}.jpg'),cv2.cvtColor(frame,cv2.COLOR_RGB2BGR));shots.append(cv2.resize(frame,(640,480)))
        if fi%200==0:print(ep.name,'motion render',fi,n,flush=True)
    renderer.close()
    if writer is not None:
        writer.close();sheet=np.concatenate([np.concatenate(shots[i:i+3],axis=1) for i in [0,3]],axis=0);cv2.imwrite(str(ep/'motion_contact_sheet.jpg'),cv2.cvtColor(sheet,cv2.COLOR_RGB2BGR))
        cap=cv2.VideoCapture(str(temporary_video));check=dict(frames=int(cap.get(cv2.CAP_PROP_FRAME_COUNT)),fps=float(cap.get(cv2.CAP_PROP_FPS)),samples={})
        for fi in selected:cap.set(cv2.CAP_PROP_POS_FRAMES,fi);ok,a=cap.read();check['samples'][str(fi)]=bool(ok and a is not None)
        cap.release();assert check['frames']==n and all(check['samples'].values())
        temporary_video.replace(ep/'both_hands_motion.mp4')
        report=json.loads((ep/'motion_validation.json').read_text());report['video_validation']=check;report['framing_includes_all_frames_mesh_bounds']=True;(ep/'motion_validation.json').write_text(json.dumps(report,indent=2))


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('episode',type=Path);ap.add_argument('--rgb',required=True,type=Path);ap.add_argument('--preview',action='store_true');ap.add_argument('--render-only',action='store_true');args=ap.parse_args()
    if not args.render_only:prepare(args.episode)
    render(args.episode,args.rgb,args.preview)
