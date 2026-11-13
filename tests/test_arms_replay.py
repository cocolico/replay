"""Arm-only SDK integration and gate tests. No real hardware commands."""
import sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import aubo_arms_replay as arms
from aubo_continuous_ik import actual_model,wind_near,track
from dh_kinematics import fk
from replay_pipeline import HERE,read_json,provenance

class ArmsTests(unittest.TestCase):
    def test_generated_model_matches_actual_dh_at_other_configurations(self):
        snaps=read_json(HERE/'tests/fixtures/controller_kinematics.json')
        with tempfile.TemporaryDirectory() as d:
            for side,s in snaps.items():
                model=actual_model(s,Path(d)/(side+'.urdf'))
                for q in np.random.default_rng(17).uniform(-2.,2.,(8,6)):
                    np.testing.assert_allclose(model.fk('flange',q),fk(q,s['kinematics_real']),atol=1e-10)

    def test_winding_selection_does_not_rewrite_command_during_execution(self):
        q=np.zeros(6);q[4]=np.deg2rad(350)
        center=wind_near(q,np.zeros(6),np.full(6,-2*np.pi),np.full(6,2*np.pi))
        self.assertAlmostEqual(center[4],np.deg2rad(-10))
        near=wind_near(q,q,np.full(6,-2*np.pi),np.full(6,2*np.pi))
        np.testing.assert_allclose(near,q)

    def test_tracker_rejects_collision_instead_of_returning_partial_path(self):
        s=read_json(HERE/'tests/fixtures/controller_kinematics.json')['left']
        with tempfile.TemporaryDirectory() as d:
            model=actual_model(s,Path(d)/'left.urdf');q=np.asarray(s['joint_positions_rad']);target=model.fk('flange',q)
            trajectory,status=track(model,np.array([target,target]),np.array([0.,.1]),q,collision=lambda q:[{'a':'arm','b':'table'}])
            self.assertIsNone(trajectory);self.assertFalse(status['passed']);self.assertEqual(status['failed_frame'],0)

    def test_known_collision_blocks_without_running_unneeded_eef_scan(self):
        snapshots=read_json(HERE/'tests/fixtures/controller_kinematics.json')
        hands={s:{'positions_rad':[0.]*20} for s in ('left','right')}
        t=np.array([0.,.1])
        left=(t,np.tile(snapshots['left']['joint_positions_rad'],(2,1)),None)
        right=(t,np.tile(snapshots['right']['joint_positions_rad'],(2,1)),None)
        scene=__import__('types').SimpleNamespace(check=lambda a,h:[{'a':'arm','b':'table'}])
        with patch.object(arms,'Scene',return_value=scene) as scene_factory:
            q,plan=arms.runtime_pair({},snapshots,hands,left,right,dict(arms.SETTINGS),{})
        scene_factory.assert_called_once_with({},snapshots,.002)
        self.assertFalse(plan['ready_for_execution']);self.assertFalse(plan['eef_checked'])
        self.assertTrue(plan['collision_checked']);self.assertFalse(plan['collision_passed'])
        self.assertEqual(plan['checked_mesh_samples'],20)

    def test_missing_acceptance_blocks_before_reading_live_devices(self):
        with patch.object(arms,'read_arms') as read:
            with self.assertRaisesRegex(ValueError,'Preflight'):arms.require_plan({}, {},np.zeros((2,52)),None)
            read.assert_not_called()

    def test_changed_trajectory_blocks_before_hardware_setup(self):
        with tempfile.TemporaryDirectory() as d:
            cfg=Path(d)/'config.json';cfg.write_text('{}')
            npz=Path(d)/'plan.npz';np.savez(npz,joint_positions_left_right_rad=np.zeros((2,52)))
            plan=dict(ready_for_execution=True,collision_checked=True,collision_passed=True,arms_only=True,
                      configuration=provenance(cfg),sources=[],trajectory=provenance(npz))
            np.savez(npz,joint_positions_left_right_rad=np.ones((2,52)))
            with patch.object(arms,'read_arms') as read:
                with self.assertRaisesRegex(ValueError,'Trajectory changed'):arms.require_plan({'_config_path':str(cfg)},plan,np.zeros((2,52)),None)
                read.assert_not_called()

if __name__=='__main__':unittest.main()
