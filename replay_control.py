#!/usr/bin/env python3
"""Checked Aubo/Wuji synchronized replay. Default is read-only preflight; --execute moves hardware."""
from pathlib import Path
import argparse,concurrent.futures,fcntl,json,os,signal,threading,time
import numpy as np
from device_backends import Aubo
from read_hands import Hands
from replay_pipeline import HERE,SIDES,config,read_json,write_json,provenance,default_config
from runtime_plan import make_plan,split,identity


def acquire_replay_lock(directory=None):
    directory=Path(directory) if directory is not None else HERE/'output'
    directory.mkdir(parents=True,exist_ok=True)
    lock=open(directory/'replay.lock','a+')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        detail=''
        try:
            session=read_json(directory/'replay_session.json')
            labels={'holding':'末帧保持','running':'回放中','preflight':'预检中','stopped':'已停止登记'}
            detail=f'\n当前登记：PID {session["pid"]}，{session.get("name","")}，{labels.get(session.get("status"),session.get("status","未知状态"))}。'
        except (OSError,ValueError,KeyError):pass
        raise SystemExit('已有回放或预检进程占用设备，不能同时启动第二个。'+detail+
            '\n如果是之前的末帧保持，请先支撑好抓持物，再运行 ./stop_replay；停止确认后重新启动。\n如果另一个终端正在预检，请等待其结束。') from None
    lock.seek(0);lock.truncate();lock.write(str(os.getpid())+'\n');lock.flush()
    return lock


def read_arms(c):
    out={}
    for side in SIDES:
        arm=Aubo(c['sides'][side])
        try:out[side]=arm.snapshot()
        finally:arm.close()
    return out


def require_start(plan,arms,hands):
    for side in SIDES:
        if identity(arms[side])!=identity(plan['controllers'][side]):raise ValueError('Controller model/payload/TCP/limits changed: '+side)
        if not arms[side]['mode'].endswith('.Running') or not arms[side]['safety'].endswith(('.Normal','.ReducedMode')) or not arms[side]['steady']:raise ValueError('Controller not ready: '+side)
        if arms[side].get('servo_mode_enabled'):raise ValueError('Servo mode already controlled by another source: '+side)
        if np.max(abs(np.asarray(arms[side]['joint_positions_rad'])-np.asarray(plan['controllers'][side]['joint_positions_rad'])))>.015:raise ValueError('Arm moved after preparation: '+side)
        if np.max(abs(np.asarray(hands[side]['positions_rad'])-np.asarray(plan['hand_start'][side]['positions_rad'])))>.03:raise ValueError('Hand moved after preparation: '+side)


def check_hand_controls(reader,settings):
    result={}
    for side in SIDES:
        hand=reader.devices[side]
        if str(hand.control_mode().get()).split('.')[-1].lower()!='mit':raise ValueError('Wuji must use the preflight MIT profile: '+side)
        efforts=np.asarray(hand.effort_limit().get(),dtype=float);gains=hand.mit_params().get()
        kp=np.asarray([float(g.kp) for g in gains]);kd=np.asarray([float(g.kd) for g in gains])
        if efforts.shape!=(20,) or kp.shape!=(20,) or kd.shape!=(20,) or not np.all(np.isfinite(np.r_[efforts,kp,kd])):raise ValueError('Incomplete Wuji control profile')
        if not np.allclose(efforts,settings['expected_hand_effort_limit_a'],atol=1e-6) or not np.allclose(kp,settings['expected_hand_mit_kp'],atol=1e-6) or not np.allclose(kd,settings['expected_hand_mit_kd'],atol=1e-6):raise ValueError('Wuji MIT parameters changed; review runtime settings against actual device profile: '+side)
        result[side]={'kp':kp.tolist(),'kd':kd.tolist(),'effort_limit_a':efforts.tolist()}
    return result


def checked_plan(name,c,settings,arms,hands):
    plan=read_json(HERE/'output'/f'{name}_runtime.json')
    if not plan.get('ready_for_execution') or not plan.get('collision_checked') or not plan.get('collision_passed'):raise ValueError('No previously checked plan')
    if plan['configuration']!=provenance(c['_config_path']) or plan['settings']!=settings:raise ValueError('Checked plan configuration/settings changed')
    if provenance(plan['trajectory']['path'])!=plan['trajectory']:raise ValueError('Checked trajectory changed')
    for source in plan.get('sources',[]):
        if provenance(source['path'])!=source:raise ValueError('Checked source/calibration changed')
    require_start(plan,arms,hands)
    for side in SIDES:
        if np.max(abs(np.asarray(arms[side]['joint_positions_rad'])-plan['controllers'][side]['joint_positions_rad']))>.002:raise ValueError('Checked start moved; rerun full preflight')
        if np.max(abs(np.asarray(hands[side]['positions_rad'])-plan['hand_start'][side]['positions_rad']))>.005:raise ValueError('Checked hand start moved; rerun full preflight')
    with np.load(plan['trajectory']['path'],allow_pickle=False) as z:trajectory=z['joint_positions_left_right_rad'].copy()
    return trajectory,plan


def stop_arm_verified(arm,reconnect=None,timeout=2.):
    warnings=[];fresh=None;result={};consecutive=0
    def request(device):
        motion=device.robot.getMotionControl()
        for name,args in [('stopJoint',(2.,)),('setServoModeSelect',(0,))]:
            try:
                value=getattr(motion,name)(*args)
                if value not in (None,0):warnings.append(f'{name}: {value}')
            except Exception as e:warnings.append(f'{name}: {e}')
    request(arm);started=time.monotonic();deadline=started+timeout;active=arm
    try:
        while time.monotonic()<deadline:
            try:
                motion=active.robot.getMotionControl();speed=np.asarray(active.state.getJointSpeeds(),dtype=float)
                result={'steady':bool(active.state.isSteady()),'servo_mode':int(motion.getServoModeSelect()),
                        'max_speed_rad_s':float(np.max(abs(speed))) if speed.shape==(6,) and np.all(np.isfinite(speed)) else None}
                stopped=result['steady'] and result['servo_mode']==0 and result['max_speed_rad_s'] is not None and result['max_speed_rad_s']<.01
                consecutive=consecutive+1 if stopped else 0
                if consecutive>=3:return {'confirmed':True,'readback':result,'command_warnings':warnings}
            except Exception as e:warnings.append('readback: '+str(e));consecutive=0
            if reconnect is not None and fresh is None and time.monotonic()-started>.15:
                fresh=reconnect();active=fresh;request(fresh)
            time.sleep(.03)
        raise RuntimeError('Stop state not confirmed: '+str(result)+'; commands='+str(warnings))
    finally:
        if fresh is not None:fresh.close()


class ArmMonitor:
    def __init__(self,c):
        self.c=c;self.latest={};self.lock=threading.Lock();self.stop=threading.Event();self.threads=[]
        for side in SIDES:
            t=threading.Thread(target=self.loop,args=(side,),daemon=True);t.start();self.threads.append(t)
    def loop(self,side):
        arm=None
        try:
            arm=Aubo(self.c['sides'][side])
            while not self.stop.is_set():
                positions=np.asarray(arm.state.getJointPositions());mode=str(arm.state.getRobotModeType());safety=str(arm.state.getSafetyModeType())
                limits=bool(arm.state.isWithinSafetyLimits())
                with self.lock:self.latest[side]={'q':positions,'mode':mode,'safety':safety,'limits':limits,'at':time.monotonic()}
                self.stop.wait(.05)
        except Exception as e:
            with self.lock:self.latest[side]={'error':str(e),'at':time.monotonic()}
        finally:
            if arm:
                try:arm.close()
                except Exception:pass
    def check(self,expected,settings):
        with self.lock:d={s:dict(v) for s,v in self.latest.items()}
        for side in SIDES:
            v=d.get(side)
            if not v or 'error' in v or time.monotonic()-v['at']>settings['feedback_timeout_s']:raise RuntimeError('Arm feedback stale/unavailable: '+side)
            if not v['limits'] or not v['mode'].endswith('.Running') or not v['safety'].endswith(('.Normal','.ReducedMode')):raise RuntimeError('Arm mode/safety fault: '+side)
            if not np.all(np.isfinite(v['q'])) or np.max(abs(v['q']-expected[side]))>settings['arm_tracking_error_rad']:raise RuntimeError('Arm tracking error: '+side)
    def close(self):
        self.stop.set()
        for t in self.threads:t.join(timeout=3.)


def execute(c,settings,trajectory,plan,reader,hold_last=False,loop_hook=None,arms_only=False):
    if plan.get('ready_for_execution') is not True or not plan.get('collision_passed') or not plan.get('collision_checked'):raise ValueError('Preflight has not passed')
    if 'configuration' in plan and provenance(c['_config_path'])!=plan['configuration']:raise ValueError('Configuration changed after preflight')
    for source in plan.get('sources',[]):
        if provenance(source['path'])!=source:raise ValueError('Source/calibration changed after preflight')
    require_start(plan,read_arms(c),reader.snapshot())
    arms={};stoppers={};pubs={};diagnostics=[];monitor=None;enabled=[];servo=[];started_runtime=[];pool=None
    cleanup_errors=[];tick_errors=[];run_error=None;stop_event=threading.Event();startup_stop=threading.Event();startup_commands={s:0 for s in SIDES};stop_results={}
    old_signal={s:signal.getsignal(s) for s in (signal.SIGINT,signal.SIGTERM)}
    def on_signal(signum,frame):stop_event.set()
    for s in old_signal:signal.signal(s,on_signal)
    def send_hand(side,q):pubs[side].send([reader.sdk.JointCommand(float(v),0.,0.) for v in q])
    try:
        if not arms_only:check_hand_controls(reader,settings)
        for side in SIDES:
            arms[side]=Aubo(c['sides'][side]);stoppers[side]=Aubo(c['sides'][side])
        monitor=ArmMonitor(c);deadline=time.monotonic()+3
        while True:
            try:monitor.check(split(trajectory[0])[0],settings);break
            except RuntimeError:
                if time.monotonic()>deadline:raise
                time.sleep(.02)
        # No power-on, payload, TCP, gains or effort-limit setters are called.
        for side in SIDES:
            runtime=arms[side].rpc.getRuntimeMachine();status=str(runtime.getStatus())
            if status.endswith('.Stopped'):
                started_runtime.append(side)
                result=runtime.start()
                if result not in (None,0):raise RuntimeError('Runtime start failed: '+side)
                deadline=time.monotonic()+3.
                while not str(runtime.getStatus()).endswith('.Running'):
                    if time.monotonic()>deadline:raise TimeoutError('Runtime did not enter Running: '+side)
                    time.sleep(.01)
            elif not status.endswith('.Running'):raise RuntimeError('Runtime is neither stopped nor running: '+side)
        if not arms_only:
            enabled_states={};diag_lock=threading.Lock()
            for side in SIDES:
                def callback(frame,side=side):
                    with diag_lock:enabled_states[side]=(time.monotonic(),len(frame.joints),sum(int(j.status_word.ext_state)==2 for j in frame.joints))
                diagnostics.append(reader.devices[side].joint_diagnostics().subscribe_with_callback(callback))
                enabled.append(side);reader.devices[side].enable()
            deadline=time.monotonic()+5
            while True:
                with diag_lock:state=dict(enabled_states)
                if all(s in state and time.monotonic()-state[s][0]<.25 and state[s][1:]==(20,20) for s in SIDES):break
                if stop_event.is_set():raise InterruptedError('Operator stop')
                if time.monotonic()>deadline:raise TimeoutError('Not all 40 hand joints enabled')
                time.sleep(.01)
            # Enabling must not have changed the measured start pose.
            post=reader.snapshot()
            for side in SIDES:
                if np.max(abs(np.asarray(post[side]['positions_rad'])-np.asarray(plan['hand_start'][side]['positions_rad'])))>.03:raise RuntimeError('Hand moved during enable: '+side)
                pubs[side]=reader.devices[side].joint_command().publish()
        pool=concurrent.futures.ThreadPoolExecutor(max_workers=4);rate=settings['rate_hz'];dt=1/rate;count=0
        ready={s:threading.Event() for s in SIDES};initial_arms=split(trajectory[0])[0]
        def enter_servo(side):
            servo.append(side);motion=arms[side].robot.getMotionControl()
            result=motion.setServoModeSelect(1)
            if result not in (None,0):raise RuntimeError(f'Servo mode-select failed: {side}, code={result}')
            deadline=time.monotonic()+3.
            while motion.getServoModeSelect()!=1:
                if time.monotonic()>deadline or stop_event.is_set():raise TimeoutError('Servo enable timeout: '+side)
                time.sleep(.005)
            # Keep a stationary target stream alive while the other arm enters Servo.
            while not startup_stop.is_set():
                value=motion.servoJoint(initial_arms[side].tolist(),0.,0.,dt,.1,150.)
                if value not in (None,0):raise RuntimeError(f'Stationary Servo failed: {side}, code={value}')
                startup_commands[side]+=1;ready[side].set();startup_stop.wait(dt)
        starters={s:pool.submit(enter_servo,s) for s in SIDES};deadline=time.monotonic()+4.
        while not all(ready[s].is_set() for s in SIDES):
            for future in starters.values():
                if future.done():future.result()
            if time.monotonic()>deadline or stop_event.is_set():raise TimeoutError('Both arms did not enter stationary Servo')
            time.sleep(.005)
        startup_stop.set()
        for future in starters.values():future.result(timeout=.1)
        start=time.monotonic()
        warmup_ticks=round(settings.get('warmup_s',2.)*rate)
        print('REPLAY START: '+('arms only; no hand commands' if arms_only else 'both arms + both hands')+'; Ctrl-C requests synchronized stop.',flush=True)
        while count<len(trajectory)+warmup_ticks or hold_last:
            if stop_event.is_set():raise InterruptedError('Operator stop')
            index=max(0,min(count-warmup_ticks,len(trajectory)-1));target=trajectory[index];arm_q,hand_q=split(target)
            deadline=start+count*dt;delay=deadline-time.monotonic()
            if delay>0:time.sleep(delay)
            late=time.monotonic()-deadline
            if late>settings['maximum_tick_lateness_s']:raise RuntimeError(f'Scheduler late by {late:.4f}s; refusing catch-up burst')
            reference=trajectory[max(0,min(index-1,len(trajectory)-1))];monitor.check(split(reference)[0],settings)
            feedback=reader.snapshot(timeout=.001,max_age=settings['feedback_timeout_s'])
            for side in SIDES:
                if np.max(abs(np.asarray(feedback[side]['positions_rad'])-split(reference)[1][side]))>(settings.get('fixed_hand_position_tolerance_rad',.03) if arms_only else settings['hand_tracking_error_rad']):raise RuntimeError('Hand tracking error: '+side)
            jobs=[]
            for side in SIDES:
                jobs.append((side,pool.submit(arms[side].robot.getMotionControl().servoJoint,arm_q[side].tolist(),0.,0.,dt,.1,150.)))
                if not arms_only:jobs.append((side+'_hand',pool.submit(send_hand,side,hand_q[side])))
            for label,job in jobs:
                value=job.result(timeout=.1)
                if value not in (None,0):raise RuntimeError(f'{label} rejected command: {value}')
            tick_errors.append(late);count+=1
            if count%rate==0:
                print(f'progress {min(index/rate,plan["duration_s"]):.1f}/{plan["duration_s"]:.1f}s',flush=True)
                write_json(HERE/'output/replay_session.json',{'pid':os.getpid(),'name':plan['name'],'status':'holding' if count>=len(trajectory)+warmup_ticks else 'running',
                    'playback_completed':count>=len(trajectory)+warmup_ticks,'progress_s':min(index/rate,plan['duration_s']),'duration_s':plan['duration_s'],'updated_unix_s':time.time()})
            if count==len(trajectory)+warmup_ticks and hold_last:print('REPLAY COMPLETE; holding last frame. Ctrl-C stops arms and disables hands.',flush=True)
    except BaseException as e:
        run_error=e
    finally:
        startup_stop.set()
        # Independent preconnected stop clients avoid waiting behind a blocked command request.
        def stop_arm(side):
            if side not in servo:return
            stop_results['arm_'+side]=stop_arm_verified(stoppers[side],lambda:Aubo(c['sides'][side]))
        def stop_hand(side):
            requested=time.monotonic();warnings=[]
            try:reader.devices[side].disable()
            except Exception as e:warnings.append(str(e))
            deadline=time.monotonic()+2.
            while time.monotonic()<deadline:
                with diag_lock:value=enabled_states.get(side)
                if value and value[0]>=requested and value[1:]==(20,0):
                    stop_results['hand_'+side]={'confirmed':True,'joints':20,'enabled':0,'command_warnings':warnings};return
                time.sleep(.02)
            raise RuntimeError('Fresh complete diagnostics did not confirm all 20 hand motors disabled')
        stop_pool=concurrent.futures.ThreadPoolExecutor(max_workers=4)
        stopping=[('arm_'+s,stop_pool.submit(stop_arm,s)) for s in servo]
        stopping += [('hand_'+s,stop_pool.submit(stop_hand,s)) for s in enabled]
        for label,future in stopping:
            try:future.result(timeout=6.)
            except BaseException as e:cleanup_errors.append(label+': '+str(e))
        stop_pool.shutdown(wait=True,cancel_futures=True)
        if pool:pool.shutdown(wait=True,cancel_futures=True)
        if monitor:monitor.close()
        for sub in diagnostics:
            try:sub.close()
            except Exception as e:cleanup_errors.append(str(e))
        for pub in pubs.values():
            try:pub.close()
            except Exception as e:cleanup_errors.append(str(e))
        # Leave user-started Runtime intact. Stop only runtimes this process started, after stop commands.
        for side in started_runtime:
            try:
                if arms[side].state.isSteady():arms[side].rpc.getRuntimeMachine().stop()
            except Exception as e:cleanup_errors.append('Runtime stop: '+str(e))
        for arm in list(arms.values())+list(stoppers.values()):
            try:arm.close()
            except Exception as e:cleanup_errors.append(str(e))
        for s,handler in old_signal.items():signal.signal(s,handler)
        playback_completed='warmup_ticks' in locals() and count>=len(trajectory)+warmup_ticks
        log={'execution_attempted':True,'arms_only':arms_only,'completed':playback_completed,'operator_stopped':isinstance(run_error,InterruptedError),'error':str(run_error) if run_error else None,'cleanup_errors':cleanup_errors,'stop_results':stop_results,'startup_hold_commands':startup_commands,'ticks':len(tick_errors),'max_tick_lateness_s':max(tick_errors,default=0),'plan_trajectory':plan['trajectory']}
        write_json(HERE/'output'/f'{plan["name"]}_execution.json',log)
        write_json(HERE/'output/replay_session.json',{'pid':os.getpid(),'name':plan['name'],'status':'stop_not_confirmed' if cleanup_errors else 'stopped','error':str(run_error) if run_error else None,'cleanup_errors':cleanup_errors,'updated_unix_s':time.time()})
    if cleanup_errors:raise RuntimeError('Stop not fully confirmed: '+str(cleanup_errors)) from run_error
    if run_error:raise run_error


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('name',choices=['594','615','616','human','episode594','episode615','episode616','human_camera']);p.add_argument('--execute',action='store_true',help='Enable devices and perform the collision-checked synchronized trajectory');p.add_argument('--speed',type=float);p.add_argument('--hold-last',action='store_true',help='Keep publishing the final pose until Ctrl-C');p.add_argument('--reuse-checked-plan',action='store_true',help='Reuse only an unchanged collision-checked plan with a tightly matching live start');p.add_argument('--config',type=Path);p.add_argument('--settings',type=Path,default=HERE/'config/runtime.json');args=p.parse_args()
    args.name='episode'+args.name if args.name.isdigit() else ('human_camera' if args.name=='human' else args.name)
    c=config(args.config or default_config(args.name));settings=read_json(args.settings)
    lock=acquire_replay_lock()
    if args.execute:write_json(HERE/'output/replay_session.json',{'pid':os.getpid(),'name':args.name,'status':'preflight','updated_unix_s':time.time()})
    reader=Hands(settings['hands'])
    try:
        controls=check_hand_controls(reader,settings)
        arms=read_arms(c);hands=reader.snapshot()
        if args.reuse_checked_plan:
            trajectory,plan=checked_plan(args.name,c,settings,arms,hands)
            if args.speed is not None and args.speed!=plan['requested_speed']:raise ValueError('Requested speed differs from checked plan')
        else:trajectory,plan=make_plan(args.name,c,settings,arms,hands,args.speed)
        plan['hand_controls']=controls
        write_json(HERE/'output'/f'{args.name}_runtime.json',plan)
        print('PREFLIGHT', 'PASS' if plan['ready_for_execution'] else 'BLOCKED', 'report:',HERE/'output'/f'{args.name}_runtime.json',flush=True)
        if not plan['ready_for_execution']:raise RuntimeError('Collision/preflight blocked. See collision_failures in the runtime report; no device commands sent.')
        if args.execute:execute(c,settings,trajectory,plan,reader,args.hold_last)
        else:print('Read-only preflight complete; no motor enable or movement commands sent.',flush=True)
    except BaseException as e:
        session=read_json(HERE/'output/replay_session.json') if (HERE/'output/replay_session.json').exists() else {}
        if args.execute and session.get('pid')==os.getpid() and session.get('status')=='preflight':
            write_json(HERE/'output/replay_session.json',{'pid':os.getpid(),'name':args.name,'status':'stopped','error':str(e),'updated_unix_s':time.time()})
        raise
    finally:reader.close();lock.close()

if __name__=='__main__':
    try:main()
    except InterruptedError:
        print('同步停止已确认，双臂已停止、双手已禁用。',flush=True)
        raise SystemExit(130)
