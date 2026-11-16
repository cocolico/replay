"""Render saved failed arm candidate, offline only, with exact mesh distance."""
import argparse, sys, subprocess, xml.etree.ElementTree as ET
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import cv2, coal
from PIL import Image, ImageDraw, ImageFont
from replay_pipeline import config, read_json
from collision_scene import Scene, tf
from runtime_plan import sample_replay
from render_mount_preview import stl, reduce_mesh
from render_repaired_hand_video import camera, project, draw_mesh


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('folder',type=Path);a=ap.parse_args()
    folder=a.folder.resolve();out=folder/'collision_preview';out.mkdir(exist_ok=True)
    c=config(folder/'config.json');snap=read_json(folder/'planning_snapshot.json')
    plan=read_json(folder/'surviving_entry_validation/summary.json')['checks'][0]
    scene=Scene(c,snap['controllers']);meshes=[]
    for shape in scene.shapes:
        side=shape['side'];arm=shape['kind']=='arm'
        urdf=Path(c['sides'][side]['urdf']) if arm else Path(c['source_episode_root'])/f'assets/wuji/urdf/{side}.urdf'
        link=ET.parse(urdf).getroot().find(f"link[@name='{shape['link']}']")
        m=link.find('collision/geometry/mesh');name=m.get('filename')
        path=urdf.parents[1]/name.split('package://aubo_description/')[1] if name.startswith('package://') else urdf.parent/name
        meshes.append(reduce_mesh(stl(path),.004 if arm else .003))
    paths=[np.load(folder/n) for n in plan['branches']];times=paths[0]['time_s']
    qs=np.c_[paths[0]['joint_positions_rad'],paths[1]['joint_positions_rad']]+2*np.pi*np.asarray(plan['replay_winding_turns'])
    _,runtime=sample_replay(times,qs,100,plan['effective_speed'])
    hands={s:snap['hands'][s]['positions_rad'] for s in ('left','right')}
    def state(t):
        index=min(len(runtime)-1,int(round(t/plan['effective_speed']*100)))
        q=runtime[index];return scene.shape_poses({'left':q[:6],'right':q[6:]},hands)
    ids=[next(i for i,s in enumerate(scene.shapes) if s['name']==n) for n in ('left:arm:foreArm_Link','right:arm:wrist1_Link')]
    req=coal.DistanceRequest();req.enable_nearest_points=True
    def distance(poses):
        result=coal.DistanceResult();i,j=ids
        d=coal.distance(scene.shapes[i]['geometry'],tf(poses[i]),scene.shapes[j]['geometry'],tf(poses[j]),req,result)
        return float(d),np.array(result.getNearestPoint1()),np.array(result.getNearestPoint2())
    flip=np.diag([1.,1.,-1.]);board_inv=np.linalg.inv(scene.board)
    def display(v):return (v@board_inv[:3,:3].T+board_inv[:3,3])@flip
    def triangles(poses):
        tris=[];colors=[]
        for k,(shape,T,(v,f)) in enumerate(zip(scene.shapes,poses,meshes)):
            tris.append(display(v@T[:3,:3].T+T[:3,3])[f])
            color=(75,172,225) if shape['side']=='left' else (239,179,78)
            if k in ids:color=(245,90,110) if k==ids[0] else (195,126,245)
            colors.append(np.tile(color,(len(f),1)))
        return np.concatenate(tris),np.concatenate(colors)
    failure=plan['collision_failures'][0];hit=(failure['runtime_time_s']-plan['transition_s']-1)*plan['effective_speed']
    start,end=hit-.8,hit+.65
    # Overview uses a fixed framing across the entire clip, with a fixed contact close-up.
    points=np.concatenate([triangles(state(t))[0].reshape(-1,3) for t in (start,hit,end)])
    lo,hi=points.min(0),points.max(0);center=(lo+hi)/2
    globalview=camera((0,120,800,620),center,[.7,1.,.7],2.)
    p=(points-center)@globalview[2];width=max(np.ptp(p[:,0])*1.12,np.ptp(p[:,1])*800/620*1.12)
    globalview=camera((0,120,800,620),center,[.7,1.,.7],width)
    poses=state(hit);d,p1,p2=distance(poses);contact=display((p1+p2)/2)
    localview=camera((805,120,795,620),contact,[1.,-.8,.55],.43)
    fps=20
    # Playback at 0.1x source speed, with a two-second freeze at first threshold failure.
    sequence=list(np.linspace(start,hit,int((hit-start)*fps/.1),endpoint=False))+[hit]*40+list(np.linspace(hit,end,int((end-hit)*fps/.1)+1))
    fontpath=str(Path(__file__).resolve().parents[1]/'resources/fonts/NotoSansCJK-Regular.ttc')
    fonts={n:ImageFont.truetype(fontpath,n) for n in (22,27,32)}
    video=out/'collision_closeup.mp4'
    encoder=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pixel_format','bgr24','-video_size','1600x900','-framerate',str(fps),'-i','-','-an','-c:v','libx264','-preset','fast','-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(video)],stdin=subprocess.PIPE)
    records=[]
    for frame,t in enumerate(sequence):
        poses=state(t);d,p1,p2=distance(poses);tris,colors=triangles(poses)
        image=np.full((900,1600,3),(27,25,22),np.uint8)
        for view in (globalview,localview):
            rect=view[0];x,y,w,h=rect;panel=np.full_like(image,(27,25,22))
            for g in np.arange(-1.5,1.6,.1):
                for line in (np.array([[g,-1.5,0],[g,1.5,0]]),np.array([[-1.5,g,0],[1.5,g,0]])):
                    xy,_=project(line,view);cv2.line(panel,tuple(xy[0].astype(int)),tuple(xy[1].astype(int)),(52,48,43),1,cv2.LINE_AA)
            draw_mesh(panel,tris,colors,view)
            image[y:y+h,x:x+w]=panel[y:y+h,x:x+w]
        for view in (globalview,localview):
            xy,_=project(display(np.array([p1,p2])),view);xy=np.round(xy).astype(int)
            for pt in xy:
                if view[0][0]<=pt[0]<view[0][0]+view[0][2] and 120<=pt[1]<740:cv2.circle(image,tuple(pt),7,(80,80,255),2,cv2.LINE_AA)
        pil=Image.fromarray(cv2.cvtColor(image,cv2.COLOR_BGR2RGB));draw=ImageDraw.Draw(pil)
        status='间距不足 2 mm' if d<.002 else '间距满足 2 mm'
        if d<=0:status='模型发生接触 / 相交'
        lines=[(25,14,'双臂轨迹碰撞预览 · 总前移 15 cm / 总上移 4 cm / 删除前 3 秒',32,(235,239,245)),
               (25,60,f'原轨迹 {t+3.00208:.3f} s  |  裁剪后 {t:.3f} s  |  0.1 倍源时间播放',27,(185,203,219)),
               (25,760,'双臂全景：左臂蓝色，右臂黄色',27,(220,229,239)),
               (825,760,'特写：左前臂粉红色，右腕 1 紫色',27,(220,229,239)),
               (25,810,f'精确碰撞网格间距：{max(d,0)*1000:.3f} mm  |  {status}',32,(255,115,115) if d<.002 else (141,226,173)),
               (25,860,'仅离线几何预览，无真机运动。显示网格已简化；间距使用预检的原始网格计算。',22,(169,184,202))]
        if abs(t-hit)<1e-9:lines.append((825,102,'首次低于 2 mm（定格）',27,(255,115,115)))
        for x,y,value,size,color in lines:draw.text((x,y),value,font=fonts[size],fill=color)
        image=cv2.cvtColor(np.array(pil),cv2.COLOR_RGB2BGR);encoder.stdin.write(image.tobytes())
        records.append({'source_trimmed_s':float(t),'distance_m':d})
        if frame==int((hit-start)*fps/.1):cv2.imwrite(str(out/'first_clearance_failure.png'),image)
        if frame%40==0:print('render',frame,'/',len(sequence),'gap_mm',d*1000,flush=True)
    encoder.stdin.close()
    if encoder.wait():raise RuntimeError('ffmpeg failed')
    import json
    (out/'preview_report.json').write_text(json.dumps({'offline_only':True,'source_run':str(folder),'video':str(video),'fps':fps,'frames':len(sequence),'source_time_speed':.1,'first_threshold_source_trimmed_s':hit,'samples':records},ensure_ascii=False,indent=2))
    print('VIDEO',video,flush=True)


if __name__=='__main__':main()
