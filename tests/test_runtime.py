import copy,sys,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import replay_control as control
import runtime_plan as plan
from replay_pipeline import config,read_json,HERE
from dh_kinematics import fk
from collision_scene import Scene


class RuntimeTests(unittest.TestCase):
    def test_busy_lock_explains_holder_without_truncating_or_releasing_it(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);held=control.acquire_replay_lock(root)
            try:
                original=(root/'replay.lock').read_text()
                (root/'replay_session.json').write_text(__import__('json').dumps({'pid':123,'name':'episode594','status':'holding'}))
                with self.assertRaisesRegex(SystemExit,'末帧保持'):
                    control.acquire_replay_lock(root)
                self.assertEqual((root/'replay.lock').read_text(),original)
                with self.assertRaises(SystemExit):control.acquire_replay_lock(root)
            finally:held.close()
            reopened=control.acquire_replay_lock(root);reopened.close()

    def test_calibrated_dh_matches_recorded_controller_fk(self):
        states=read_json(HERE/'tests/fixtures/controller_kinematics.json')
        for side,d in states.items():
            np.testing.assert_allclose(fk(d['joint_positions_rad'],d['kinematics_real']),d['base_T_flange'],atol=1e-8)

    def test_resampling_keeps_named_channels_and_zero_endpoint_velocity(self):
        t=np.array([0.,.1,.2,.3]);q=np.tile(np.arange(52),(4,1)).astype(float);q[:,0]=[0,.1,.15,.2]
        ticks,out=plan.sample_replay(t,q,100,.2)
        np.testing.assert_allclose(out[0],q[0]);np.testing.assert_allclose(out[-1],q[-1])
        self.assertEqual(len(ticks),151);self.assertTrue(np.all(np.diff(out[:,0])>=-1e-10));self.assertTrue(np.all((out[:,0]>=0)&(out[:,0]<=.2)))
        np.testing.assert_allclose(out[:,1:],q[0,1:][None,:]+np.zeros((len(out),51)))
        for speed in (0,-1,2):
            with self.assertRaises(ValueError):plan.sample_replay(t,q,100,speed)

    def test_transition_is_continuous_bounded_and_starts_at_measured_pose(self):
        start=np.linspace(-.3,.3,52);target=start+.5
        ticks,q=plan.transition(start,target,100,10)
        np.testing.assert_array_equal(q[0],start);np.testing.assert_allclose(q[-1],target)
        self.assertTrue(np.all(q>=start-1e-10));self.assertTrue(np.all(q<=target+1e-10))
        v,a=plan.dynamics(q,100);self.assertLess(max(v),.1);self.assertLess(max(a),.04)

    def test_table_positive_board_z_is_occupied(self):
        scene=Scene.__new__(Scene);scene.clearance=.005;scene.normal=np.array([0.,0.,1.]);scene.plane_point=np.zeros(3);scene.pairs=[]
        scene.shapes=[{'name':'test','kind':'hand','link':'test','hull':np.zeros((1,3))}]
        T=np.eye(4);scene.shape_poses=lambda arms,hands:[T]
        T[2,3]=-.02;self.assertEqual(scene.check({},{}),[])
        T[2,3]=.02;self.assertEqual(scene.check({},{})[0]['b'],'calibrated_table_halfspace')

    def test_controller_payload_change_invalidates_preflight(self):
        snapshots=read_json(HERE/'tests/fixtures/controller_kinematics.json');hands={s:{'positions_rad':[0.]*20} for s in ('left','right')}
        p={'controllers':snapshots,'hand_start':hands};changed=copy.deepcopy(snapshots);changed['left']['payload'][0]+=.01
        with self.assertRaisesRegex(ValueError,'payload'):control.require_start(p,changed,hands)

    def test_live_payload_tuple_matches_json_payload_list(self):
        snapshots=read_json(HERE/'tests/fixtures/controller_kinematics.json');live=copy.deepcopy(snapshots)
        hands={s:{'positions_rad':[0.]*20} for s in ('left','right')}
        for d in live.values():d['payload']=tuple(d['payload'])
        control.require_start({'controllers':snapshots,'hand_start':hands},live,hands)

    def test_unchecked_plan_cannot_enter_device_setup(self):
        with patch.object(control,'Aubo') as mock_arm:
            with self.assertRaises(ValueError):control.execute({}, {},np.zeros((2,52)),{'ready_for_execution':False},None)
            mock_arm.assert_not_called()

    def test_stop_nonzero_return_is_recorded_but_stopped_readback_confirms(self):
        motion=NS(stopJoint=lambda acc:-3,setServoModeSelect=lambda mode:-3,getServoModeSelect=lambda:0)
        arm=NS(robot=NS(getMotionControl=lambda:motion),state=NS(isSteady=lambda:True,getJointSpeeds=lambda:[0.]*6))
        result=control.stop_arm_verified(arm,timeout=.2)
        self.assertTrue(result['confirmed']);self.assertEqual(len(result['command_warnings']),2)

    def test_stop_success_return_does_not_override_moving_feedback(self):
        motion=NS(stopJoint=lambda acc:0,setServoModeSelect=lambda mode:0,getServoModeSelect=lambda:1)
        arm=NS(robot=NS(getMotionControl=lambda:motion),state=NS(isSteady=lambda:False,getJointSpeeds=lambda:[.1]*6))
        with self.assertRaisesRegex(RuntimeError,'not confirmed'):control.stop_arm_verified(arm,timeout=.04)

    def test_stop_reconnects_when_existing_client_cannot_confirm(self):
        old=NS(robot=NS(getMotionControl=lambda:NS(stopJoint=lambda acc:-3,setServoModeSelect=lambda mode:-3,getServoModeSelect=lambda:1)),state=NS(isSteady=lambda:False,getJointSpeeds=lambda:[.1]*6))
        fresh=NS(robot=NS(getMotionControl=lambda:NS(stopJoint=lambda acc:0,setServoModeSelect=lambda mode:0,getServoModeSelect=lambda:0)),state=NS(isSteady=lambda:True,getJointSpeeds=lambda:[0.]*6),close=lambda:None)
        factory=__import__('unittest.mock',fromlist=['Mock']).Mock(return_value=fresh)
        result=control.stop_arm_verified(old,reconnect=factory,timeout=.5)
        self.assertTrue(result['confirmed']);factory.assert_called_once()

    def run_mock(self,fail_command=False,fail_feedback=False,arms_only=False):
        events=[];c=config(HERE/'config/replay.json');settings=read_json(HERE/'config/runtime.json');snapshots=read_json(HERE/'tests/fixtures/controller_kinematics.json')
        settings['warmup_s']=0.0
        for d in snapshots.values():d['joint_positions_rad']=[0.]*6;d['mode']='RobotModeType.Running';d['safety']='SafetyModeType.Normal';d['steady']=True
        hand_start={s:{'positions_rad':[0.]*20} for s in ('left','right')}
        report={'ready_for_execution':True,'collision_passed':True,'collision_checked':True,'controllers':snapshots,'hand_start':hand_start,'name':'mock','trajectory':{},'duration_s':.02}
        class Motion:
            def __init__(self,side):self.side=side;self.selected=0
            def setServoModeSelect(self,value):self.selected=value;events.append((self.side,'servo',bool(value)));return 0
            def getServoModeSelect(self):return self.selected
            def isServoModeEnabled(self):return True
            def stopJoint(self,a):events.append((self.side,'stop'));return 0
            def servoJoint(self,*args):events.append((self.side,'command'));return -1 if fail_command and self.side=='right' else 0
        class Arm:
            def __init__(self,cfg):
                self.side='left' if cfg['ip']==c['sides']['left']['ip'] else 'right';m=Motion(self.side)
                self.robot=NS(getMotionControl=lambda:m);self.state=NS(isSteady=lambda:True,getJointSpeeds=lambda:[0.]*6)
                runtime=NS(getStatus=lambda:'RuntimeState.Running');self.rpc=NS(getRuntimeMachine=lambda:runtime)
            def close(self):pass
        class Resource:
            def __init__(self,value):self.value=value
            def get(self):return self.value
        class Hand:
            def __init__(self,side):self.side=side;self.callback=None
            def control_mode(self):return Resource('mit')
            def effort_limit(self):return Resource([settings['expected_hand_effort_limit_a']]*20)
            def mit_params(self):return Resource([NS(kp=settings['expected_hand_mit_kp'],kd=settings['expected_hand_mit_kd'])]*20)
            def joint_diagnostics(self):
                def subscribe(cb):self.callback=cb;return NS(close=lambda:None)
                return NS(subscribe_with_callback=subscribe)
            def enable(self):events.append((self.side,'hand_enable'));self.callback(NS(joints=[NS(status_word=NS(ext_state=2))]*20))
            def disable(self):
                events.append((self.side,'hand_disable'))
                self.callback(NS(joints=[NS(status_word=NS(ext_state=1))]*20))
            def joint_command(self):return NS(publish=lambda:NS(send=lambda q:events.append((self.side,'hand_command')),close=lambda:None))
        reader=NS(devices={s:Hand(s) for s in ('left','right')},snapshot=lambda **kw:hand_start,sdk=NS(JointCommand=lambda *a:a))
        class Monitor:
            def __init__(self,c):self.calls=0
            def check(self,*a):
                self.calls+=1
                if fail_feedback and self.calls>1:raise RuntimeError('stale feedback')
            def close(self):pass
        with patch.object(control,'Aubo',Arm),patch.object(control,'ArmMonitor',Monitor),patch.object(control,'read_arms',return_value=snapshots),patch.object(control,'write_json'):
            if fail_command or fail_feedback:
                with self.assertRaises(RuntimeError):control.execute(c,settings,np.zeros((3,52)),report,reader,arms_only=arms_only)
            else:control.execute(c,settings,np.zeros((3,52)),report,reader,arms_only=arms_only)
        for side in ('left','right'):
            self.assertIn((side,'stop'),events);self.assertIn((side,'servo',False),events)
            if arms_only:
                for action in ('hand_enable','hand_command','hand_disable'):self.assertNotIn((side,action),events)
            else:self.assertIn((side,'hand_disable'),events)
        return events

    def test_arms_only_never_enables_publishes_or_disables_hands(self):self.run_mock(arms_only=True)
    def test_arms_only_command_failure_stops_both_arms(self):self.run_mock(arms_only=True,fail_command=True)
    def test_arms_only_feedback_failure_stops_both_arms(self):self.run_mock(arms_only=True,fail_feedback=True)

    def test_command_failure_stops_both_arms_and_hands(self):self.run_mock(fail_command=True)
    def test_feedback_failure_stops_both_arms_and_hands(self):self.run_mock(fail_feedback=True)
    def test_normal_completion_stops_both_arms_and_hands(self):
        events=self.run_mock();self.assertGreaterEqual(sum(e[-1]=='command' for e in events),8)

if __name__=='__main__':unittest.main()
