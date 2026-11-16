import tempfile,unittest,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from trim_camera_trajectory import trim

class TrimTests(unittest.TestCase):
    def fixture(self,p):
        t=np.array([0.,1.,2.,3.002,4.,5.]);np.save(p/'time_s.npy',t);np.save(p/'timestamp_ns.npy',(t*1e9).astype(np.int64)+123)
        for side in ('left','right'):
            for name,width in [('root_pose',(4,4)),('position_m',(3,)),('quaternion_wxyz',(4,)),('joint_positions_rad',(20,))]:np.save(p/(side+'_'+name+'.npy'),np.arange(6*np.prod(width)).reshape((6,)+width))
            np.save(p/(side+'_joint_names.npy'),np.array(['j'+str(i) for i in range(20)]))
        np.save(p/'fps.npy',10.)
    def test_trim_aligns_all_channels_and_rebases_both_clocks(self):
        with tempfile.TemporaryDirectory() as d:
            src=Path(d)/'source';src.mkdir();self.fixture(src);dst=Path(d)/'trimmed';before={p.name:p.read_bytes() for p in src.glob('*.npy')}
            result=trim(src,dst,3.);self.assertEqual(result['removed_frames'],3);self.assertEqual(result['retained_frames'],3)
            np.testing.assert_allclose(np.load(dst/'time_s.npy'),[0.,.998,1.998]);np.testing.assert_array_equal(np.load(dst/'timestamp_ns.npy'),[0,998000000,1998000000])
            for side in ('left','right'):
                np.testing.assert_array_equal(np.load(dst/(side+'_root_pose.npy')),np.load(src/(side+'_root_pose.npy'))[3:])
                self.assertEqual((src/(side+'_joint_names.npy')).read_bytes(),(dst/(side+'_joint_names.npy')).read_bytes())
            self.assertEqual(before,{p.name:p.read_bytes() for p in src.glob('*.npy')})
            with self.assertRaises(ValueError):trim(src,dst,3.)
    def test_trim_rejects_removing_all_frames_before_creating_output(self):
        with tempfile.TemporaryDirectory() as d:
            src=Path(d)/'source';src.mkdir();self.fixture(src);dst=Path(d)/'trimmed'
            with self.assertRaises(ValueError):trim(src,dst,5.)
            self.assertFalse(dst.exists())
