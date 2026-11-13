import sys,json,tempfile,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from lift_camera_trajectory import lift
from scipy.spatial.transform import Rotation

class LiftTests(unittest.TestCase):
    def test_lift_uses_table_up_and_preserves_other_arrays(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);src=root/'source';src.mkdir();out=root/'lifted'
            board=np.eye(4);board[:3,:3]=Rotation.from_euler('xyz',[.3,.4,.2]).as_matrix()
            mapping=np.eye(4);mapping[:3,:3]=Rotation.from_euler('xyz',[-.2,.1,.5]).as_matrix()
            for name,value in [('board',{'camera_T_board':board.tolist()}),('mapping',{'target_camera_T_source_camera':mapping.tolist()}),('config',{})]:(root/(name+'.json')).write_text(json.dumps(value))
            for side in ('left','right'):
                poses=np.tile(np.eye(4),(3,1,1));poses[:,:3,3]=np.arange(9).reshape(3,3)/10
                np.save(src/f'{side}_root_pose.npy',poses);np.save(src/f'{side}_position_m.npy',poses[:,:3,3]);np.save(src/f'{side}_joint_positions_rad.npy',np.ones((3,20)))
            np.save(src/'time_s.npy',[0.,.1,.2])
            c={'current_table_observation':str(root/'board.json'),'camera_table_mapping':str(root/'mapping.json'),'_config_path':str(root/'config.json')}
            lift(src,out,c,6.5)
            for side in ('left','right'):
                old=np.load(src/f'{side}_root_pose.npy');new=np.load(out/f'{side}_root_pose.npy');delta=new[:,:3,3]-old[:,:3,3]
                np.testing.assert_allclose((delta@mapping[:3,:3].T)@(-board[:3,2]),.065,atol=1e-12)
                np.testing.assert_array_equal(new[:,:3,:3],old[:,:3,:3])
                self.assertEqual((src/f'{side}_joint_positions_rad.npy').read_bytes(),(out/f'{side}_joint_positions_rad.npy').read_bytes())
            self.assertEqual((src/'time_s.npy').read_bytes(),(out/'time_s.npy').read_bytes())
            with self.assertRaises(ValueError):lift(src,out,c,6.5)

if __name__=='__main__':unittest.main()
