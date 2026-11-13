import sys,json,tempfile,unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from shift_camera_trajectory import shift

class ShiftTests(unittest.TestCase):
    def test_forward_translation_preserves_height_orientation_fingers_and_timestamps(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);src=root/'source';src.mkdir();out=root/'shifted'
            board=np.eye(4);board[:3,:3]=Rotation.from_euler('xyz',[.3,.4,.2]).as_matrix()
            mapping=np.eye(4);mapping[:3,:3]=Rotation.from_euler('xyz',[-.2,.1,.5]).as_matrix()
            for name,obj in [('board',{'camera_T_board':board.tolist()}),('mapping',{'target_camera_T_source_camera':mapping.tolist()}),('config',{})]:(root/(name+'.json')).write_text(json.dumps(obj))
            c={'current_table_observation':str(root/'board.json'),'camera_table_mapping':str(root/'mapping.json'),'_config_path':str(root/'config.json')}
            for side in ('left','right'):
                pose=np.tile(np.eye(4),(3,1,1));pose[:,:3,3]=np.arange(9).reshape(3,3)/10
                np.save(src/f'{side}_root_pose.npy',pose);np.save(src/f'{side}_position_m.npy',pose[:,:3,3]);np.save(src/f'{side}_joint_positions_rad.npy',np.ones((3,20)))
            np.save(src/'time_s.npy',[0.,.1,.2])
            delta=-board[:3,1]*.1;shift(src,out,c,delta,'forward 10 cm')
            for side in ('left','right'):
                old=np.load(src/f'{side}_root_pose.npy');new=np.load(out/f'{side}_root_pose.npy');d=(new[:,:3,3]-old[:,:3,3])@mapping[:3,:3].T
                np.testing.assert_allclose(d,np.tile(delta,(3,1)),atol=1e-12)
                np.testing.assert_allclose(d@board[:3,2],0.,atol=1e-12)
                np.testing.assert_array_equal(new[:,:3,:3],old[:,:3,:3])
                self.assertEqual((src/f'{side}_joint_positions_rad.npy').read_bytes(),(out/f'{side}_joint_positions_rad.npy').read_bytes())
            self.assertEqual((src/'time_s.npy').read_bytes(),(out/'time_s.npy').read_bytes())
            with self.assertRaises(ValueError):shift(src,out,c,delta,'repeat')

if __name__=='__main__':unittest.main()
