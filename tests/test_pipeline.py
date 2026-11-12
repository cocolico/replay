import copy
import json
from pathlib import Path
import tempfile
import unittest
import sys
import numpy as np
from scipy.spatial.transform import Rotation

HERE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(HERE))
import replay_pipeline as p


class ConversionTests(unittest.TestCase):
    def test_numbered_and_human_configurations_are_separate(self):
        self.assertEqual(p.default_config('episode594'),HERE/'config/replay.json')
        self.assertEqual(p.default_config('prepare-camera'),HERE/'config/human_replay.json')
        c=p.config(p.default_config('human_camera'))
        self.assertEqual(c['camera_source_observation'],'/home/descfly/ty/desk/observation.json')
        self.assertEqual(c['camera_source_id'],'orbbec_gemini_335')

    def test_human_without_explicit_source_observation_is_rejected(self):
        c=p.config(HERE/'config/replay.json')
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaisesRegex(ValueError,'explicit source observation'):p.prepare_camera(c,d)

    def test_transform_rejects_reflection_and_invalid_bottom_row(self):
        t=np.eye(4);t[0,0]=-1
        with self.assertRaises(ValueError):p.transform(t)
        t=np.eye(4);t[3,0]=1
        with self.assertRaises(ValueError):p.transform(t)

    def test_clock_rejects_mixed_timebases(self):
        with self.assertRaises(ValueError):p.check_times([0,.1,.2],np.array([0,100000000,300000000]))
        with self.assertRaises(ValueError):p.check_times([0,.1,.1],np.array([0,100000000,200000000]))

    def test_mapping_mount_composition_has_expected_direction(self):
        a=np.eye(4);a[:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix();a[:3,3]=[1,2,3]
        camera=np.eye(4);camera[:3,3]=[.5,0,0]
        root=np.eye(4);root[:3,3]=[0,0,1]
        mount=np.eye(4);mount[:3,3]=[0,0,.2]
        flange=a@camera@root@p.inverse(mount)
        np.testing.assert_allclose(flange[:3,3],[1,2.5,3.8])
        np.testing.assert_allclose(flange@mount,a@camera@root)

    def test_joint_reordering_rejects_duplicate_names(self):
        with self.assertRaises(ValueError):p.ordered([[1,2]],['a','a'],['a','b'])
        np.testing.assert_array_equal(p.ordered([[2,1]],['b','a'],['a','b']),[[1,2]])

    def test_all_recorded_fk_matches_delivered_scene_without_nominal_base_mount(self):
        c=p.config(HERE/'config/replay.json');root=Path(c['source_episode_root'])
        tree=p.ET.parse(root/'assets/motion/source_assembly.urdf').getroot()
        fixed={j.find('child').get('link'):j for j in tree.findall('joint')}
        for episode in ('594','615','616'):
            ep=root/f'episode{episode}'
            with np.load(ep/'source_sharpa.npz') as source,np.load(ep/'wuji_trajectory.npz') as z,np.load(ep/'wuji_motion_scene.npz') as scene:
                for side in p.SIDES:
                    q=source[f'{side}_arm_q'];names=source[f'{side}_arm_names']
                    fk=p.franka_fk(root/'assets/motion/source_assembly.urdf',side,q,names)
                    # Recorded source messages use a different array order; named conversion must be invariant.
                    permutation=np.arange(7)[::-1]
                    shuffled=p.franka_fk(root/'assets/motion/source_assembly.urdf',side,q[:,permutation],names[permutation])
                    np.testing.assert_allclose(fk,shuffled,atol=1e-12)
                    local=np.tile(np.eye(4),(len(q),1,1));local[:,:3,:3]=Rotation.from_rotvec(z[f'{side}_root_rotvec_in_sharpa_rad']).as_matrix();local[:,:3,3]=z[f'{side}_root_xyz_in_sharpa_m']
                    link=f'{side}_fr3v2_link0';chain=[]
                    while link in fixed:
                        joint=fixed[link];self.assertEqual(joint.get('type'),'fixed');chain.insert(0,joint);link=joint.find('parent').get('link')
                    scene_T_base=np.eye(4)
                    for joint in chain:scene_T_base=scene_T_base@p.origin(joint)
                    expected=p.inverse(scene_T_base)@scene[f'{side}_scene_T_wuji_base']
                    np.testing.assert_allclose(fk@local,expected,atol=1e-10)

    def test_default_preparation_is_blocked_without_mount_and_source_extrinsics(self):
        c=p.config(HERE/'config/replay.json')
        c['source_franka_handeye']=None
        with tempfile.TemporaryDirectory() as d:
            r=p.prepare_episode(c,'594',d)
            self.assertFalse(r['arm_ik_inputs_ready']);self.assertFalse(r['execution_enabled'])
            self.assertIn('source_franka_camera_extrinsics_missing',r['blockers'])
            with self.assertRaisesRegex(ValueError,'Unresolved'):
                p.solve_ik(c,Path(d)/'episode594_prepared.npz',Path(d)/'episode594_report.json',Path(d)/'ik.npz')
            with np.load(Path(d)/'episode594_prepared.npz') as z:
                self.assertEqual(z['left_wuji_joint_positions_rad'].shape,(965,20))
                self.assertNotIn('left_aubo_base_T_flange',z.files)
            path=Path(d)/'episode594_prepared.npz';path.write_bytes(path.read_bytes()+b'tampered')
            with self.assertRaisesRegex(ValueError,'hash mismatch'):
                p.solve_ik(c,path,Path(d)/'episode594_report.json',Path(d)/'ik.npz')

    def test_human_camera_uses_supplied_gemini335_observation_and_preserves_time(self):
        c=p.config(p.default_config('human_camera'))
        c['sides']['left']['mount_confirmed']=False
        with tempfile.TemporaryDirectory() as d:
            report=p.prepare_camera(c,d)
            self.assertEqual(report['frames'],150)
            self.assertEqual(report['source_camera_id'],'orbbec_gemini_335')
            self.assertIn('/home/descfly/ty/desk/observation.json',[s['path'] for s in report['sources']])
            self.assertFalse(report['execution_enabled'])
            self.assertIn('left_flange_to_wuji_mount_not_confirmed',report['blockers'])
            with np.load(Path(d)/'human_camera_prepared.npz') as z:
                root=Path(c['camera_trajectory_root'])
                np.testing.assert_array_equal(z['timestamp_ns'],np.load(root/'timestamp_ns.npy'))
                for side in p.SIDES:
                    base=p.transform(p.read_json(c['sides'][side]['handeye'])['T_base_camera'])
                    mapping=p.transform(p.read_json(c['camera_table_mapping'])['target_camera_T_source_camera'])
                    source=np.load(root/f'{side}_root_pose.npy')
                    np.testing.assert_allclose(z[f'{side}_aubo_base_T_wuji_base'],base@mapping@source,atol=1e-12)
            c['camera_source_id']='franka-sharpa'
            with self.assertRaisesRegex(ValueError,'Wrong source/target'):p.prepare_camera(c,d)

    def test_synthetic_aubo_ik_roundtrip_remains_offline(self):
        import pinocchio as pin
        c=copy.deepcopy(p.config(HERE/'config/replay.json'));arrays={'time_s':np.array([0.,.5,1.])}
        for side in p.SIDES:
            cfg=c['sides'][side];cfg['model_confirmed']=True;cfg['mount_confirmed']=True
            cfg['initial_q_rad']=[.2,-.5,.8,-.3,1.,.2]
            model=pin.buildModelFromUrdf(cfg['urdf']);data=model.createData();fid=model.getFrameId(cfg['flange_frame']);targets=[]
            for step in (0.,.01,.02):
                q=np.array(cfg['initial_q_rad'])+step
                pin.forwardKinematics(model,data,q);pin.updateFramePlacements(model,data);targets.append(data.oMf[fid].homogeneous.copy())
            arrays[f'{side}_aubo_base_T_flange']=np.array(targets)
        with tempfile.TemporaryDirectory() as d:
            base=Path(d);prepared=base/'synthetic.npz';np.savez_compressed(prepared,**arrays)
            p.write_json(base/'report.json',{'artifact':p.provenance(prepared),'configuration':p.provenance(c['_config_path']),'sources':[],'arm_ik_inputs_ready':True,'blockers':[]})
            p.solve_ik(c,prepared,base/'report.json',base/'ik.npz')
            report=p.read_json(base/'ik.json');self.assertFalse(report['execution_enabled']);self.assertFalse(report['collision_check_performed'])
            for side in p.SIDES:self.assertLess(report['sides'][side]['max_translation_log_error_m'],1e-5)

    def test_full_frame_candidate_export_has_explicit_52_joint_order_and_is_disabled(self):
        import diagnose_ik
        c=p.config(p.default_config('human_camera'))
        with tempfile.TemporaryDirectory() as d:
            base=Path(d)
            p.prepare_camera(c,base)
            artifact=base/'human_camera_prepared.npz'
            with np.load(artifact) as z:
                arrays={k:(z[k][:3] if z[k].ndim and z[k].shape[0]==150 else z[k]) for k in z.files}
            np.savez_compressed(artifact,**arrays)
            report=p.read_json(base/'human_camera_report.json');report['artifact']=p.provenance(artifact);p.write_json(base/'report.json',report)
            r=diagnose_ik.diagnose(c,artifact,base/'report.json',1,base/'candidate.npz')
            self.assertFalse(r['execution_enabled'])
            with np.load(base/'candidate.npz') as z:
                self.assertFalse(bool(z['execution_enabled']))
                self.assertEqual(z['joint_positions_left_right_rad'].shape,(3,52))
                self.assertEqual(len(set(z['joint_names_left_right'])),52)
                np.testing.assert_array_equal(z['joint_positions_left_right_rad'][:,6:26],z['left_wuji_joint_positions_rad'])
                np.testing.assert_array_equal(z['joint_positions_left_right_rad'][:,32:52],z['right_wuji_joint_positions_rad'])

if __name__=='__main__':unittest.main()
