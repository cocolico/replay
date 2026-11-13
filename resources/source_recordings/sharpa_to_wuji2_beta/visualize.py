"""Synchronized RGB / source Sharpa / target Wuji kinematic comparison."""
import argparse
import json
from pathlib import Path
import time
import copy
import xml.etree.ElementTree as ET
from retarget import Hand, HERE, TIPS, PARENTS, Rotation, np, mujoco

COLORS=[(250,122,61),(250,204,56),(56,212,158),(69,166,255),(199,125,255)]


def text(image,s,xy,size=.55,color=(224,232,242)):
    import cv2
    cv2.putText(image,s,xy,cv2.FONT_HERSHEY_SIMPLEX,size,color,1,cv2.LINE_AA)


def project(image,scene,points):
    a,b=scene.camera;pos=(a.pos+b.pos)/2;forward=(a.forward+b.forward)/2;up=(a.up+b.up)/2;right=np.cross(forward,up)
    focal=image.shape[0]*a.frustum_near/(a.frustum_top-a.frustum_bottom);v=points-pos;depth=v@forward
    return np.round(np.c_[image.shape[1]/2+focal*(v@right)/depth,image.shape[0]/2-focal*(v@up)/depth]).astype(int)


def overlay(im,scene,target,pred=None):
    import cv2
    tt=project(im,scene,target)
    for f,col in enumerate(COLORS):
        for k in range(1+4*f,5+4*f):
            if k!=1+4*f:cv2.line(im,tuple(tt[PARENTS[k]]),tuple(tt[k]),col,1,cv2.LINE_AA)
        tip=TIPS[f];cv2.circle(im,tuple(tt[tip]),5,col,1,cv2.LINE_AA)
    if pred is not None:
        pp=project(im,scene,pred)
        for tip in TIPS:
            cv2.line(im,tuple(tt[tip]),tuple(pp[tip]),(255,75,75),1,cv2.LINE_AA);cv2.circle(im,tuple(pp[tip]),2,(255,255,255),-1,cv2.LINE_AA)
    return im


def letterbox(im,w,h):
    import cv2
    s=min(w/im.shape[1],h/im.shape[0]);nw,nh=round(im.shape[1]*s),round(im.shape[0]*s)
    out=np.full((h,w,3),20,np.uint8);out[(h-nh)//2:(h+nh)//2,(w-nw)//2:(w+nw)//2]=cv2.resize(im,(nw,nh));return out


def render(episode,frames_dir,preview=False):
    import cv2
    import imageio_ffmpeg
    z=np.load(episode/'wuji_trajectory.npz');report=json.loads((episode/'validation.json').read_text());n=len(z['time_s'])
    models={(k,s):Hand(k,s,True) for k in ['sharpa','wuji'] for s in ['left','right']}
    renderers={key:mujoco.Renderer(h.model,height=300,width=640) for key,h in models.items()}
    opt=mujoco.MjvOption();opt.sitegroup[:]=0
    cam=mujoco.MjvCamera();mujoco.mjv_defaultCamera(cam);cam.azimuth=10;cam.elevation=-8;cam.distance=.41;cam.lookat[:]=[.005,0,.103]
    writer=None
    if not preview:
        writer=imageio_ffmpeg.write_frames(str(episode/'comparison.mp4'),(1280,1120),fps=30,codec='libx264',pix_fmt_in='rgb24',output_params=['-crf','20','-preset','fast']);writer.send(None)
    selected=np.linspace(0,n-1,6).round().astype(int).tolist();snapshots=[]
    for fi in ([0,n//2,n-1] if preview else range(n)):
        frame=np.full((1120,1280,3),(18,24,34),np.uint8)
        text(frame,f'{episode.name.upper()} | SHARPA -> WUJI HAND 2 BETA',(20,28),.76)
        text(frame,f'Frame {fi+1}/{n}   t={z["time_s"][fi]:.2f}s   30 Hz | synchronized measured joints',(20,53),.55)
        rgb=cv2.imread(str(frames_dir/f'{int(z["rgb_index"][fi]):05d}.jpg'));assert rgb is not None
        frame[64:424,:640]=letterbox(cv2.cvtColor(rgb,cv2.COLOR_BGR2RGB),640,360)
        text(frame,'RECORDED HEAD CAMERA',(18,88),.53)
        for row,line in enumerate(['TARGET: official early hand2_beta','20 joints / hand, OpenPI joint order','IK + confirmed local-Z orientation correction','LEFT +90 deg / RIGHT -90 deg, applied once','Coloured rings: unchanged Sharpa fingertips','White: corrected Wuji tips; red: deviation','Arm motion is shown in RGB only','Kinematic check; no contact simulation']):
            text(frame,line,(665,94+29*row),.54)
        for si,side in enumerate(['left','right']):
            x=si*640;err=z[f'{side}_error_m'][fi,TIPS]*1000
            text(frame,f'{side.upper()} | SHARPA measured pose',(x+18,447),.6)
            text(frame,f'{side.upper()} | WUJI   tip deviation {err.mean():.2f} / max {err.max():.2f} mm',(x+18,777),.50)
            for kind,y in [('sharpa',454),('wuji',784)]:
                hand=models[kind,side];rr=renderers[kind,side]
                if kind=='wuji':
                    bid=hand.model.body(hand.root).id
                    hand.model.body_pos[bid]=z[f'{side}_root_xyz_in_sharpa_m'][fi]
                    hand.model.body_quat[bid]=Rotation.from_rotvec(z[f'{side}_root_rotvec_in_sharpa_rad'][fi]).as_quat()[[3,0,1,2]]
                rendered_points=hand.fk(z[f'{side}_{kind}_joints_rad'][fi])
                expected=z[f'{side}_wuji_points_in_sharpa_m'][fi] if kind=='wuji' else z[f'{side}_source_points_m'][fi]
                assert np.max(np.abs(rendered_points-expected))<1e-8
                rr.update_scene(hand.data,camera=cam,scene_option=opt)
                im=rr.render().copy();overlay(im,rr.scene,z[f'{side}_source_points_m'][fi],z[f'{side}_wuji_points_in_sharpa_m'][fi] if kind=='wuji' else None)
                frame[y:y+300,x:x+640]=im
        text(frame,'Wuji root offsets are retained in the output. Hand joints alone do not reproduce fingertip positions.',(20,1108),.53)
        if writer is not None:writer.send(np.ascontiguousarray(frame))
        if fi in selected or preview:
            cv2.imwrite(str(episode/f'preview_{fi:04d}.jpg'),cv2.cvtColor(frame,cv2.COLOR_RGB2BGR));snapshots.append(cv2.resize(frame,(640,560)))
        if fi%200==0:print(episode.name,'render',fi,'/',n,flush=True)
    if writer is not None:writer.close()
    for rr in renderers.values():rr.close()
    if not preview:
        sheet=np.concatenate([np.concatenate(snapshots[i:i+3],axis=1) for i in [0,3]],axis=0);cv2.imwrite(str(episode/'contact_sheet.jpg'),cv2.cvtColor(sheet,cv2.COLOR_RGB2BGR))
        cap=cv2.VideoCapture(str(episode/'comparison.mp4'));validation=dict(frames=int(cap.get(cv2.CAP_PROP_FRAME_COUNT)),fps=float(cap.get(cv2.CAP_PROP_FPS)),width=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),height=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),sample_decodes={})
        for i in selected:
            cap.set(cv2.CAP_PROP_POS_FRAMES,i);ok,im=cap.read();validation['sample_decodes'][str(i)]=bool(ok and im is not None)
        cap.release();assert validation['frames']==n and all(validation['sample_decodes'].values())
        report['video_validation']=validation;(episode/'validation.json').write_text(json.dumps(report,indent=2));print(validation,flush=True)


def viewer_model():
    doc=ET.Element('mujoco');ET.SubElement(doc,'compiler',angle='radian',autolimits='true');asset=ET.SubElement(doc,'asset');world=ET.SubElement(doc,'worldbody')
    for si,side in enumerate(['left','right']):
        h=Hand('wuji',side,True);tree=ET.fromstring(h.xml)
        for m in tree.findall('asset/mesh'):m.set('name',side+'_'+m.get('name'));asset.append(m)
        b=copy.deepcopy(tree.find('worldbody/body'));b.insert(0,ET.Element('freejoint',name=side+'_floating'))
        ET.SubElement(b,'inertial',mass='.1',pos='0 0 0',diaginertia='.001 .001 .001')
        for g in b.findall('.//geom'):
            if g.get('mesh'):g.set('mesh',side+'_'+g.get('mesh'))
        for site in b.findall('.//site'):site.set('name',side+'_'+site.get('name'))
        world.append(b)
    return mujoco.MjModel.from_xml_string(ET.tostring(doc).decode())


def viewer(episode):
    import mujoco.viewer
    z=np.load(episode/'wuji_trajectory.npz');n=len(z['time_s']);model=viewer_model();data=mujoco.MjData(model);state=dict(t=0.,paused=False,speed=1.)
    def key(k):
        if k==32:state['paused']=not state['paused']
        elif k in [262,263]:state['t']=float(np.clip(state['t']+(1 if k==262 else -1)/30,0,z['time_s'][-1]));state['paused']=True
        elif k in [82,114]:state['t']=0.
        elif k in [61,334]:state['speed']=min(4,state['speed']*1.25)
        elif k in [45,333]:state['speed']=max(.1,state['speed']/1.25)
    with mujoco.viewer.launch_passive(model,data,key_callback=key) as v:
        v.cam.azimuth=10;v.cam.elevation=-8;v.cam.distance=.7;v.cam.lookat[:]=[0,0,.1];last=time.monotonic()
        while v.is_running():
            now=time.monotonic()
            if not state['paused']:state['t']=(state['t']+(now-last)*state['speed'])%(n/30)
            last=now;fi=min(n-1,int(state['t']*30))
            with v.lock():
                for si,side in enumerate(['left','right']):
                    adr=model.joint(side+'_floating').qposadr[0];data.qpos[adr:adr+3]=z[f'{side}_root_xyz_in_sharpa_m'][fi]+[0,.15 if si==0 else -.15,0]
                    data.qpos[adr+3:adr+7]=Rotation.from_rotvec(z[f'{side}_root_rotvec_in_sharpa_rad'][fi]).as_quat()[[3,0,1,2]]
                    for name,val in zip(z[f'{side}_wuji_joint_names'],z[f'{side}_wuji_joints_rad'][fi]):data.qpos[model.joint(str(name)).qposadr[0]]=val
                mujoco.mj_forward(model,data)
            v.sync();time.sleep(.005)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('episode',type=Path);ap.add_argument('--rgb',type=Path);ap.add_argument('--preview',action='store_true');ap.add_argument('--viewer',action='store_true');args=ap.parse_args()
    if args.viewer:viewer(args.episode)
    else:
        if args.rgb is None:ap.error('--rgb is required to render')
        render(args.episode,args.rgb,args.preview)
