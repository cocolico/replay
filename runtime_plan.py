"""Retiming and start-transition preparation. No device commands."""
from pathlib import Path
import hashlib,json,math,time
import numpy as np
from scipy.interpolate import PchipInterpolator,CubicHermiteSpline
from replay_pipeline import HERE,SIDES,config,read_json,write_json,provenance,hand_contract
from collision_scene import Scene


def combined(arrays):
    return np.concatenate([arrays['left_aubo_joint_positions_rad'],arrays['left_wuji_joint_positions_rad'],arrays['right_aubo_joint_positions_rad'],arrays['right_wuji_joint_positions_rad']],axis=1)


def split(q):return ({'left':q[:6],'right':q[26:32]},{'left':q[6:26],'right':q[32:]})


def identity(snapshot):
    values={k:snapshot[k] for k in ['ip','robot_name','robot_type','payload','tcp_offset_sdk','kinematics_real','joint_min_rad','joint_max_rad','joint_max_speed_rad_s','joint_max_acceleration_rad_s2']}
    return json.loads(json.dumps(values,allow_nan=False))


def check_controller(c,side,snapshot):
    if snapshot['robot_type']!='aubo_i7H' or snapshot['ip']!=c['sides'][side]['ip']:raise ValueError('Aubo identity mismatch')
    if not snapshot['mode'].endswith('.Running') or not snapshot['safety'].endswith(('.Normal','.ReducedMode')) or not snapshot['steady']:raise ValueError('Controller must be Running, steady, and in Normal/ReducedMode')
    if float(snapshot['payload'][0])<=0:raise ValueError('Active payload must be positive')
    if not np.allclose(snapshot['tcp_offset_sdk'],0,atol=1e-8):raise ValueError('Expected unchanged zero TCP offset')
    if snapshot.get('servo_mode_enabled'):raise ValueError('Servo mode already active; refusing to take over another command source')


def limits(c,controllers,settings):
    lo=[];hi=[];v=[];a=[]
    for side in SIDES:
        d=controllers[side]
        names,wire,hl,hu=hand_contract(Path(c['source_episode_root'])/f'assets/wuji/urdf/{side}.urdf',side)
        lo.extend(d['joint_min_rad']);lo.extend(hl);hi.extend(d['joint_max_rad']);hi.extend(hu)
        v.extend(np.array(d['joint_max_speed_rad_s'])*settings['arm_velocity_fraction']);v.extend([settings['hand_velocity_rad_s']]*20)
        a.extend(np.array(d['joint_max_acceleration_rad_s2'])*settings['arm_acceleration_fraction']);a.extend([settings['hand_acceleration_rad_s2']]*20)
    return [np.asarray(x,dtype=float) for x in (lo,hi,v,a)]


def sample_replay(t,q,rate,speed):
    if not 0<speed<=1 or not 20<=rate<=200:raise ValueError('Speed must be (0,1], rate 20..200Hz')
    t=(np.asarray(t)-t[0])/speed;duration=float(t[-1]);ticks=np.arange(math.ceil(duration*rate)+1)/rate
    # Extend the final sample to the next exact scheduler tick as a hold.
    query=np.minimum(ticks,duration);p=PchipInterpolator(t,q,axis=0);derivative=p.derivative()(t);derivative[[0,-1]]=0
    spline=CubicHermiteSpline(t,q,derivative,axis=0);sampled=spline(query);sampled[-1]=q[-1]
    return ticks,sampled


def transition(start,target,rate,duration):
    ticks=np.arange(math.ceil(duration*rate)+1)/rate;phase=np.clip(ticks/duration,0,1);blend=10*phase**3-15*phase**4+6*phase**5
    return ticks,start[None,:]+blend[:,None]*(target-start)[None,:]


def dynamics(q,rate):
    v=np.diff(q,axis=0)*rate;a=np.diff(v,axis=0)*rate
    return np.max(abs(v),axis=0),np.max(abs(a),axis=0)


def validate_numbers(q,rate,lo,hi,vlim,alim):
    if q.ndim!=2 or q.shape[1]!=52 or not np.all(np.isfinite(q)):raise ValueError('Expected finite Nx52 trajectory')
    if np.any(q<lo-1e-6) or np.any(q>hi+1e-6):raise ValueError('Joint position limits violated')
    vmax,amax=dynamics(q,rate)
    return vmax,amax, bool(np.all(vmax<=vlim+1e-6) and np.all(amax<=alim+1e-6))


def make_plan(name,c,settings,controllers,hands,speed=None,check_mesh=True):
    path=HERE/'output'/f'{name}_controller_ik.npz';report=read_json(path.with_suffix('.json'))
    if provenance(path)!=report['trajectory'] or provenance(c['_config_path'])!=report['configuration']:raise ValueError('Controller IK/config hashes changed')
    source_report=read_json(HERE/'output'/f'{name}_report.json')
    for source in source_report['sources']:
        if provenance(source['path'])!=source:raise ValueError('Source/calibration changed: '+source['path'])
    collision_report_path=HERE/'output'/f'{name}_collisions.json'
    if collision_report_path.exists():
        known=read_json(collision_report_path)
        if known['trajectory']!=provenance(path):raise ValueError('Collision report is stale; rebuild collision checks')
        if not known['passed']:
            write_json(HERE/'output'/f'{name}_runtime.json',{'ready_for_execution':False,'execution_enabled':False,'collision_checked':True,'collision_passed':False,
                'reason':'Known source Wuji poses collide in the modeled scene; no automatic modification of the desired motion.',
                'collision_report':provenance(collision_report_path),'collision_failures':known['first_failures'],'failed_source_frames':known['failed_frames']})
            raise ValueError(f'{name}: {known["failed_frames"]} source frames fail modeled collision checks; inspect {collision_report_path}')
    for side in SIDES:
        check_controller(c,side,controllers[side])
        old=report['sides'][side]['snapshot']
        if old['kinematics_real']!=controllers[side]['kinematics_real']:raise ValueError('Controller kinematic calibration changed; rebuild IK')
    with np.load(path,allow_pickle=False) as z:
        source=combined(z);times=z['time_s'].copy()
    lo,hi,vlim,alim=limits(c,controllers,settings);rate=settings['rate_hz'];requested=float(speed if speed is not None else settings['speed']);chosen=requested
    start=np.concatenate([np.array(controllers['left']['joint_positions_rad']),hands['left']['positions_rad'],controllers['right']['joint_positions_rad'],hands['right']['positions_rad']])
    if not np.all(np.isfinite(start)) or np.any(start<lo-1e-3) or np.any(start>hi+1e-3):raise ValueError('Measured start outside model joint range')
    for _ in range(12):
        ticks,replay=sample_replay(times,source,rate,chosen);vmax,amax,ok=validate_numbers(replay,rate,lo,hi,vlim,alim)
        if ok:break
        chosen*=.8
    else:raise ValueError('Unable to retime within dynamic limits')
    duration=max(settings['minimum_transition_s'],float(np.max(1.9*np.abs(source[0]-start)/vlim)),float(np.max(np.sqrt(6*np.abs(source[0]-start)/alim))))
    for _ in range(8):
        ramp_t,ramp=transition(start,replay[0],rate,duration);_,_,ok=validate_numbers(ramp,rate,lo,hi,vlim,alim)
        if ok:break
        duration*=1.3
    else:raise ValueError('Start transition exceeds dynamic limits')
    hold=np.tile(replay[0],(rate,1));trajectory=np.concatenate([ramp,hold,replay[1:]],axis=0);ticks=np.arange(len(trajectory))/rate
    vmax,amax,ok=validate_numbers(trajectory,rate,lo,hi,vlim,alim)
    if not ok:raise ValueError('Combined transition/replay exceeds dynamic limits')
    result={'artifact_type':'runtime_preflight','execution_enabled':False,'name':name,'source':provenance(path),
        'configuration':provenance(c['_config_path']),'settings':settings,'requested_speed':requested,'effective_speed':chosen,'rate_hz':rate,
        'frames':len(trajectory),'transition_s':float(ramp_t[-1]),'replay_start_frame':len(ramp)+rate-1,'duration_s':float(ticks[-1]),
        'controllers':controllers,'hand_start':hands,'max_velocity_rad_s':vmax.tolist(),'max_acceleration_rad_s2':amax.tolist(),
        'sources':source_report['sources'],
        'collision_checked':False,'collision_passed':False,'ready_for_execution':False,'collision_failures':[]}
    if check_mesh:
        scene=Scene(c,controllers,settings['mesh_clearance_m']);checked=0;bad=[];last=None
        for i,q in enumerate(trajectory):
            arms,fingers=split(q);finding=scene.check(arms,fingers);checked+=1;last=q.copy()
            if finding:
                bad.append({'frame':i,'time_s':float(ticks[i]),'phase':'transition' if i<len(ramp) else 'replay','findings':finding})
                if len(bad)>=20:break
            if checked%1000==0:print(name,'mesh samples',checked,'/',i,'of',len(trajectory),flush=True)
        result.update(collision_checked=True,collision_passed=not bad,checked_mesh_samples=checked,collision_failures=bad,ready_for_execution=not bad)
    dest=HERE/'output'/f'{name}_runtime.npz';np.savez_compressed(dest,time_s=ticks,joint_positions_left_right_rad=trajectory,execution_enabled=np.asarray(False),artifact_type=np.asarray('validated_runtime_plan'))
    result['trajectory']=provenance(dest);write_json(dest.with_suffix('.json'),result)
    return trajectory,result
