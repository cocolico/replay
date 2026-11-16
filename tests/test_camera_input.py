"""NPZ packaging preserves the same arrays and provenance as NPY input."""
import sys,tempfile,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from camera_trajectory_input import read_camera_input
from replay_pipeline import HERE,config,prepare_camera


class CameraInputTests(unittest.TestCase):
    def test_npz_matches_npy_and_archive_is_fingerprinted(self):
        c=config(HERE/'config/human_replay.json')
        original,_,_=read_camera_input(c['camera_trajectory_root'])
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);archive=root/'wuji2_camera_replay_trajectory.npz'
            np.savez_compressed(archive,**original)
            loaded,sources,kind=read_camera_input(root)
            self.assertEqual(kind,'wuji_npz');self.assertEqual(sources,[archive])
            for key in original:np.testing.assert_array_equal(original[key],loaded[key])
            c['camera_trajectory_root']=str(root)
            report=prepare_camera(c,root/'prepared')
            self.assertIn(str(archive),[p['path'] for p in report['sources']])
            with np.load(root/'prepared/human_camera_prepared.npz') as data:
                np.testing.assert_array_equal(data['left_source_camera_T_wuji_base'],original['left_root_pose'])

    def test_sharpa_archive_is_not_interpreted_as_wuji(self):
        with tempfile.TemporaryDirectory() as directory:
            np.savez(Path(directory)/'sharpa_camera_replay_trajectory.npz',time_s=[0.,1.])
            with self.assertRaisesRegex(ValueError,'Sharpa'):
                read_camera_input(directory)

    def test_missing_npz_fields_are_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            np.savez(Path(directory)/'wuji2_camera_replay_trajectory.npz',time_s=[0.,1.])
            with self.assertRaisesRegex(ValueError,'NPZ缺少轨迹字段'):
                read_camera_input(directory)


if __name__=='__main__':unittest.main()
