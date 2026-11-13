"""Offline ROS2 bag extraction. Run with system Python after sourcing ROS Jazzy."""
import argparse
import json
from pathlib import Path
import sqlite3
import numpy as np
import cv2
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState, Image


def extract(ep, output, frames_dir, fps=30):
    output.mkdir(parents=True, exist_ok=True)
    frames_dir.mkdir(parents=True, exist_ok=True)
    state = json.loads((ep/'collection_state.json').read_text())
    report = state['validation_report']
    start, end = report['source_effective_start_time_ns'], report['source_effective_end_time_ns']
    times = np.arange(start, end+1, round(1e9/fps), dtype=np.int64)
    conn = sqlite3.connect(f'file:{next(ep.glob("*.db3"))}?mode=ro', uri=True)
    topics = {name: id for id, name in conn.execute('select id,name from topics')}
    out = dict(timestamp_ns=times, time_s=(times-times[0])/1e9)
    diagnostics = dict(episode=str(ep), fps=fps, alignment='ROS header source timestamps; linear joint interpolation, nearest RGB', source_start_ns=int(start), source_end_ns=int(end), streams={})
    for side in ['left','right']:
        for kind, topic in [('hand',f'/sharpa/{side}/joint_states'),('arm',f'/{side}/franka/joint_states')]:
            stamps, rows, names = [], [], None
            for payload, in conn.execute('select data from messages where topic_id=? order by timestamp',(topics[topic],)):
                msg = deserialize_message(payload, JointState)
                if names is None: names = list(msg.name)
                assert set(names)==set(msg.name)
                mapping = dict(zip(msg.name,msg.position))
                rows.append([mapping[n] for n in names])
                stamps.append(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec)
            t, indices = np.unique(np.asarray(stamps,dtype=np.int64),return_index=True)
            q = np.asarray(rows)[indices]
            assert np.isfinite(q).all() and t[0]<=times[0] and t[-1]>=times[-1]
            ix = np.clip(np.searchsorted(t,times),1,len(t)-1)
            gap = (t[ix]-t[ix-1])/1e6
            assert gap.max()<50, (topic,gap.max())
            x=(t-start)/1e9
            out[f'{side}_{kind}_q'] = np.column_stack([np.interp(out['time_s'],x,q[:,i]) for i in range(q.shape[1])])
            out[f'{side}_{kind}_names'] = np.array(names)
            diagnostics['streams'][topic] = dict(samples=len(t),joints=len(names),max_interpolation_bracket_ms=float(gap.max()))
    # Decode only the head RGB stream and retain the frames selected by the output clock.
    rgb_stamps, rgb_ids = [], []
    for i,(payload,) in enumerate(conn.execute('select data from messages where topic_id=? order by timestamp',(topics['/cam0/color/image_raw'],))):
        msg=deserialize_message(payload,Image)
        rgb_stamps.append(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec)
        a=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
        if msg.encoding=='rgb8':a=cv2.cvtColor(a,cv2.COLOR_RGB2BGR)
        else:assert msg.encoding=='bgr8',msg.encoding
        cv2.imwrite(str(frames_dir/f'{i:05d}.jpg'),a,[cv2.IMWRITE_JPEG_QUALITY,92]);rgb_ids.append(i)
    rt=np.array(rgb_stamps,dtype=np.int64);ix=np.clip(np.searchsorted(rt,times),1,len(rt)-1)
    ix=np.where(abs(rt[ix]-times)<abs(rt[ix-1]-times),ix,ix-1)
    age=abs(rt[ix]-times)/1e6
    assert age.max()<60,age.max()
    out['rgb_index']=ix;out['rgb_timestamp_ns']=rt[ix]
    diagnostics['rgb_max_alignment_ms']=float(age.max())
    diagnostics['frames']=len(times);diagnostics['duration_s']=float(out['time_s'][-1])
    np.savez_compressed(output/'source_sharpa.npz',**out)
    (output/'extraction.json').write_text(json.dumps(diagnostics,indent=2))
    print(json.dumps(diagnostics),flush=True)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('episode',type=Path);ap.add_argument('output',type=Path);ap.add_argument('frames',type=Path);args=ap.parse_args()
    extract(args.episode,args.output,args.frames)
