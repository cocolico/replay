"""Check the user-requested model clearance without disabling real collisions."""
import sys,unittest,itertools
from pathlib import Path
import numpy as np
import coal
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from collision_scene import Scene,DEFAULT_CLEARANCE_M
from replay_pipeline import read_json,HERE
from aubo_arms_replay import SETTINGS

class ClearanceTests(unittest.TestCase):
    def test_all_active_defaults_use_two_mm(self):
        self.assertEqual(DEFAULT_CLEARANCE_M,.002)
        self.assertEqual(SETTINGS['mesh_clearance_m'],.002)
        self.assertEqual(read_json(HERE/'config/runtime.json')['mesh_clearance_m'],.002)

    def scene(self,center_distance,clearance):
        s=Scene.__new__(Scene);s.clearance=clearance;s.normal=np.array([0.,0.,1.]);s.plane_point=np.zeros(3);s.pairs=[(0,1)]
        hull=np.asarray(list(itertools.product([-.01,.01],repeat=3)))
        s.shapes=[dict(name=f'left:arm:test{i}',kind='arm',side='left',link=f'test{i}',hull=hull,geometry=coal.Sphere(.01)) for i in range(2)]
        a=np.eye(4);a[2,3]=-1.;b=a.copy();b[0,3]=center_distance;s.shape_poses=lambda arms,hands:[a,b]
        return s

    def test_three_mm_gap_passes_two_mm_but_overlap_still_blocks(self):
        self.assertEqual(self.scene(.023,.002).check({},{}),[])
        self.assertTrue(self.scene(.023,.005).check({},{}))
        self.assertTrue(self.scene(.019,.002).check({},{}))

if __name__=='__main__':unittest.main()
