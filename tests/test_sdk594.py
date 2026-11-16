"""Regression checks for the restored episode594 SDK solver; no hardware."""
import sys, tempfile, unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import solve_controller_ik as legacy
import aubo_arms_replay as arms
from replay_pipeline import HERE, read_json
from dh_kinematics import fk


class SDK594Tests(unittest.TestCase):
    def run_solver(self, step):
        snapshot=read_json(HERE/'tests/fixtures/controller_kinematics.json')['left']
        q=np.asarray(snapshot['joint_positions_rad']);end=q.copy();end[0]+=step
        targets=np.array([fk(v,snapshot['kinematics_real']) for v in (q,end)])
        seeds=[];closed=[]
        class Robot:
            def snapshot(self):return snapshot
            def ik(self,target,seed):
                seeds.append(np.asarray(seed).copy())
                value=(q if len(seeds)==1 else end).copy()
                value[0]+=2*np.pi
                return value
            def fk(self,value):return fk(value,snapshot['kinematics_real'])
            def close(self):closed.append(True)
        arrays={'time_s':np.array([0.,.033]),'left_aubo_joint_positions_rad':np.array([q,q]),
                'left_aubo_base_T_flange':targets}
        try:
            with patch.object(legacy,'Aubo',return_value=Robot()):
                result=legacy.solve_side({'sides':{'left':{}}},'left',arrays)
            return result,seeds,q,end
        finally:self.assertEqual(closed,[True])

    def test_every_source_frame_uses_sdk_and_previous_refined_seed(self):
        (qs,report),seeds,start,end=self.run_solver(.02)
        self.assertEqual(len(seeds),2)
        np.testing.assert_allclose(seeds[0],start)
        np.testing.assert_allclose(seeds[1],qs[0])
        np.testing.assert_allclose(qs,[start,end],atol=1e-8)
        self.assertLess(report['max_position_error_m'],1e-5)

    def test_point_four_rad_branch_jump_warns_and_continues(self):
        (qs,report),_,_,end=self.run_solver(.5)
        np.testing.assert_allclose(qs[-1],end,atol=1e-8)
        self.assertEqual(len(report['branch_discontinuity_warnings']),1)
        warning=report['branch_discontinuity_warnings'][0]
        self.assertEqual(warning['frame'],1)
        self.assertEqual(warning['action'],'warn_only_continue')

    def test_camera_adapter_preserves_original_times_and_requires_later_collision_check(self):
        controllers=read_json(HERE/'tests/fixtures/controller_kinematics.json')
        times=np.array([0.,.031,.071]);q=np.tile(controllers['left']['joint_positions_rad'],(3,1))
        with tempfile.TemporaryDirectory() as d:
            folder=Path(d)
            with patch.object(arms,'sdk_solve_side',return_value=(q,{'snapshot':controllers['left']})) as solver:
                result=arms.solve_side({},'left',{'time_s':times},controllers,{},folder)
            solver.assert_called_once()
            np.testing.assert_array_equal(result[0][0],times)
            report=read_json(folder/'left_ik.json')
            self.assertTrue(report['passed']);self.assertFalse(report['collision_checked'])
            self.assertFalse(report['runtime_acceptance'])
            with np.load(result[0][2]) as z:self.assertFalse(bool(z['execution_enabled']))


if __name__=='__main__':unittest.main()
