"""Arm and finger replay uses source fingers and moving-hand collision checks."""
import sys,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import aubo_arms_replay as replay
from replay_pipeline import HERE,config,read_json,hand_contract
from dh_kinematics import fk


class SyncTests(unittest.TestCase):
    def test_fingers_follow_source_and_collision_sees_moving_shape(self):
        c=config(HERE/'config/human_replay.json')
        controllers=read_json(HERE/'tests/fixtures/controller_kinematics.json')
        t=np.array([0.,.1]);hands={};arrays={'time_s':t};paths=[];seen=[]
        for side in ('left','right'):
            _,_,lo,hi=hand_contract(Path(c['source_episode_root'])/f'assets/wuji/urdf/{side}.urdf',side)
            initial=(np.asarray(lo)+np.asarray(hi))/2
            hands[side]={'positions_rad':initial}
            target=initial.copy();target[0]+=.01
            arrays[side+'_wuji_joint_positions_rad']=np.array([initial,target])
            q=controllers[side]['joint_positions_rad'];paths.append((t,np.tile(q,(2,1)),None))
            arrays[side+'_aubo_base_T_flange']=np.tile(fk(q,controllers[side]['kinematics_real']),(2,1,1))
        settings=dict(read_json(HERE/'config/runtime.json'),**replay.SETTINGS)
        settings['arms_only']=False
        def check(a,h,**kwargs):seen.append(h['left'].copy());return []
        warning={'a':'left:hand:test1','b':'left:hand:test2','clearance_m':0.0}
        scene=SimpleNamespace(check=check,check_hand_self=lambda a,h:[warning])
        with patch.object(replay,'Scene',return_value=scene):
            trajectory,plan=replay.runtime_pair(c,controllers,hands,*paths,settings,arrays)
        self.assertTrue(plan['ready_for_execution'])
        self.assertEqual(plan['hand_self_collision_policy'],'warn')
        self.assertEqual(plan['hand_self_collision_warning_frames'],2)
        np.testing.assert_allclose(trajectory[-1,6:26],arrays['left_wuji_joint_positions_rad'][-1])
        self.assertGreater(np.ptp(np.array(seen)[:,0]),.009)
        arrays['left_wuji_joint_positions_rad'][1,0]=1e5
        with patch.object(replay,'Scene',return_value=SimpleNamespace(check=lambda a,h,**kw:[],check_hand_self=lambda a,h:[])):
            with self.assertRaises(ValueError):
                replay.runtime_pair(c,controllers,hands,*paths,settings,arrays)


if __name__=='__main__':unittest.main()
