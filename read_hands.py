#!/usr/bin/env python3
"""Read Wuji state/identity only. Never enables motors or creates a command publisher."""
import argparse,json,time,threading
from pathlib import Path
import numpy as np
from replay_pipeline import HERE,write_json
import device_backends


class Hands:
    def __init__(self,settings):
        import wuji_sdk
        self.sdk=wuji_sdk;self.manager=wuji_sdk.SdkManager.instance();self.devices={};self.subs=[];self.latest={};self.lock=threading.Lock()
        try:
            for side in ('left','right'):
                cfg=settings[side]
                h=self.manager.connect(address=cfg['address'],device_name='aubo_replay_'+side,options=wuji_sdk.ConnectOptions(enable_bridge=False))
                self.devices[side]=h
                if str(h.serial_number)!=cfg['serial']:raise ValueError('Wuji serial mismatch: '+side)
                handed=str(h.handedness().get()).split('.')[-1].lower()
                if handed!=side or int(h.online_joints_count().get())!=20:raise ValueError('Wuji side/online count mismatch')
                def callback(frame,side=side,hand=h):
                    try:
                        q=[None]*20
                        for item in frame.joints:
                            i=int(hand.nid_to_joint_index(int(item.nid)))
                            if i<0 or i>=20 or q[i] is not None:raise ValueError('Bad Wuji joint id')
                            q[i]=float(item.position)
                        if len(frame.joints)!=20 or not all(v is not None and np.isfinite(v) for v in q):raise ValueError('Incomplete Wuji feedback')
                        with self.lock:self.latest[side]={'positions_rad':q,'received_monotonic':time.monotonic(),'captured_unix_s':time.time()}
                    except Exception as e:
                        with self.lock:self.latest[side]={'error':str(e),'received_monotonic':time.monotonic()}
                self.subs.append(h.joint_states().subscribe_with_callback(callback))
        except Exception:self.close();raise

    def snapshot(self,timeout=3.,max_age=.25):
        deadline=time.monotonic()+timeout
        while True:
            with self.lock:data={s:dict(v) for s,v in self.latest.items()}
            if all(s in data and 'error' not in data[s] and time.monotonic()-data[s]['received_monotonic']<max_age for s in ('left','right')):return data
            if time.monotonic()>deadline:raise TimeoutError('Fresh complete hand feedback unavailable: '+str(data))
            time.sleep(.01)

    def close(self):
        for sub in self.subs:
            try:sub.close()
            except Exception:pass
        for hand in self.devices.values():
            try:hand.disconnect()
            except Exception:pass


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,default=HERE/'output/hand_snapshot_latest.json');a=p.parse_args()
    settings={}
    for side in ('left','right'):
        old=json.loads(Path('/tmp/wuji_'+side+'_state.json').read_text());settings[side]={'address':old['address'],'serial':old['serial_number']}
    reader=Hands(settings)
    try:
        states=reader.snapshot();out={'read_only':True,'settings':settings,'sides':states}
        for side,h in reader.devices.items():
            states[side]['control_mode']=str(h.control_mode().get())
            states[side]['effort_limit']=h.effort_limit().get()
            states[side]['mit_params']=[{'kp':float(v.kp),'kd':float(v.kd)} for v in h.mit_params().get()]
        write_json(a.out,out);print('Fresh feedback from both Wuji hands, 20 joints each. No commands sent.')
    finally:reader.close()

if __name__=='__main__':main()
