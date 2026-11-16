"""Planning tests exercise graph look-ahead, edge obstacles and winding."""
import sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from collision_planning import cartesian_graph,combine_arm_graphs,edge_valid,coherent_winding,PlanningFailure
from runtime_plan import dynamics
from replay_pipeline import HERE,config,read_json
from collision_scene import Scene

class CartesianGraphTests(unittest.TestCase):
    def test_global_graph_keeps_alternative_prefix_when_cheapest_dead_ends(self):
        # Layer 1's cheap node 0 cannot reach the goal, but prefix 4 -> 5 can.
        layers=[[[0.],[4.]],[[0.],[5.]],[[6.]]]
        valid_edges={(0.,0.),(4.,5.),(5.,6.)}
        def connect(i,a,b):
            if (a[0],b[0]) not in valid_edges:return None
            return np.array([i-1.,i]),np.array([a,b])
        paths,report=cartesian_graph(np.arange(3.),[None]*3,lambda i,t,p:[np.array(q) for q in layers[i]],lambda q:True,connect)
        np.testing.assert_allclose(paths[0][1].ravel(),[4.,5.,6.]);np.testing.assert_allclose(paths[0][0],[0.,1.,2.])
        self.assertTrue(report['passed']);self.assertGreater(report['edges_rejected'],0)

    def test_collision_nodes_are_excluded_during_search(self):
        def candidates(i,t,p):return [np.array([0.]),np.array([1.])]
        def connect(i,a,b):return np.array([0.,1.]),np.array([a,b])
        paths,report=cartesian_graph([0.,1.],[None,None],candidates,lambda q:q[0]>0,connect)
        self.assertEqual(report['collision_nodes_rejected'],2);np.testing.assert_allclose(paths[0][1],1.)

    def test_collision_between_valid_waypoints_blocks_graph(self):
        valid=lambda q:not abs(q[0])<.2
        def connect(i,a,b):return (np.array([0.,1.]),np.array([a,b])) if edge_valid(a,b,valid,.05) else None
        with self.assertRaises(PlanningFailure) as result:
            cartesian_graph([0.,1.],[None,None],lambda i,t,p:[np.array([-1. if i==0 else 1.])],valid,connect)
        self.assertEqual(result.exception.report['failed_frame'],1)

    def test_product_graph_checks_inter_arm_collision_before_selection(self):
        def make(layers):
            return cartesian_graph([0.,1.],[None,None],lambda i,t,p:[np.array([q]) for q in layers[i]],lambda q:True,
                                   lambda i,a,b:(np.array([0.,1.]),np.array([a,b])),retain_graph=True)[2]
        left=make([[-1.],[0.,1.]]);right=make([[1.],[0.,2.]])
        valid=lambda q:abs(q[0]-q[1])>.15
        paths,report=combine_arm_graphs([0.,1.],left,right,valid,resolution_rad=.02)
        self.assertTrue(report['full_scene_collision_during_search'])
        self.assertGreater(report['collision_nodes_rejected'],0)
        for t,q in paths:
            self.assertTrue(all(edge_valid(a,b,valid,.01) for a,b in zip(q[:-1],q[1:])))
        np.testing.assert_allclose(paths[0][1][-1],[0.,2.])

    def test_connector_cannot_teleport_between_branches(self):
        with self.assertRaisesRegex(ValueError,'reach its nodes'):
            cartesian_graph([0.,1.],[None,None],lambda i,t,p:[np.array([i*1.])],lambda q:True,lambda i,a,b:(np.array([0.,1.]),np.array([a,a])))

class ReducedWaypointsTests(unittest.TestCase):
    def test_223_source_points_become_57_anchors_with_both_endpoints(self):
        from aubo_cartesian_planner import planning_waypoint_indices
        indices=planning_waypoint_indices(223,4)
        self.assertEqual(len(indices),57);self.assertEqual(indices[0],0);self.assertEqual(indices[-1],222)
        np.testing.assert_array_equal(planning_waypoint_indices(223,1),np.arange(223))

    def test_connector_keeps_original_times_without_near_duplicate_grid_times(self):
        from aubo_cartesian_planner import connector_query
        source=np.array([0.,.1,.2,.30000000000000004,.4])
        query=connector_query(source,source[0],source[-1])
        for t in source:self.assertIn(t,query)
        self.assertTrue(np.all(np.diff(query)>1e-9));self.assertLessEqual(np.max(np.diff(query)),.01+1e-9)

    def test_removed_anchor_does_not_remove_a_cartesian_collision_constraint(self):
        import aubo_cartesian_planner as planner
        from types import SimpleNamespace
        class Factory:
            def __init__(self,*args):self.stats={}
            def __call__(self,*args):return [np.zeros(6)]
            def close(self):pass
        class Solver:
            def __init__(self,*args):pass
            def solve_near(self,target,q,**kw):
                result=np.zeros(6);result[1]=target[1,3];return result,None
        targets=np.tile(np.eye(4),(3,1,1));targets[1,1,3]=2.
        arrays={'time_s':np.array([0.,.1,.2]),'left_aubo_base_T_flange':targets}
        controllers={side:{'joint_positions_rad':[0.]*6} for side in ('left','right')}
        hands={side:{'positions_rad':[0.]*20} for side in ('left','right')}
        scene=SimpleNamespace(check=lambda a,h:[{'a':'arm','b':'table'}] if a['left'][1]>1.5 else [])
        with tempfile.TemporaryDirectory() as folder,patch.object(planner,'SDKCandidates',Factory),patch.object(planner,'actual_model',return_value=object()),patch.object(planner,'ContinuousIK',Solver):
            with self.assertRaises(PlanningFailure) as error:
                planner.plan_arm({'sides':{'left':{}}},'left',arrays,controllers,hands,Path(folder),scene,waypoint_stride=2)
        self.assertEqual(error.exception.report['failed_source_frame'],2)
        self.assertGreater(error.exception.report['edge_checks']['edge_collision'],0)

class WindingTests(unittest.TestCase):
    def test_one_winding_is_used_for_the_entire_path(self):
        q=np.deg2rad(np.array([[-151.,-157.],[-3.,-98.]]));start=np.deg2rad([219.,111.])
        shifted,turns=coherent_winding(q,start,np.full(2,-2*np.pi),np.full(2,2*np.pi))
        np.testing.assert_array_equal(turns,[1,1]);np.testing.assert_allclose(np.diff(q,axis=0),np.diff(shifted,axis=0))
        self.assertTrue(np.all(shifted<=2*np.pi));self.assertLess(abs(shifted[0,0]-start[0]),abs(q[0,0]-start[0]))

    def test_internal_jump_is_not_hidden_by_per_frame_modulo(self):
        q=np.deg2rad(np.array([[179.],[-179.]]));shifted,_=coherent_winding(q,np.array([0.]),np.array([-2*np.pi]),np.array([2*np.pi]))
        self.assertAlmostEqual(float(np.rad2deg(shifted[1,0]-shifted[0,0])),-358.)

    def test_winding_must_fit_all_frames_not_only_first_frame(self):
        with self.assertRaises(PlanningFailure):coherent_winding(np.array([[-10.],[10.]]),np.array([0.]),np.array([-2*np.pi]),np.array([2*np.pi]))

class SavedGeometryTests(unittest.TestCase):
    def fixture(self,root,times=(0.,.1)):
        path=Path(root)/'geometry.npz';np.savez(path,time_s=times,joint_positions_rad=np.zeros((len(times),12)),execution_enabled=False,ready_for_execution=True)
        a={'time_s':np.array([0.,.1])}
        for side in ('left','right'):a[side+'_aubo_base_T_flange']=np.tile(np.eye(4),(2,1,1))
        hands={s:{'positions_rad':[0.]*20} for s in ('left','right')}
        return path,a,hands

    def test_cached_acceptance_flag_cannot_bypass_current_collision(self):
        from types import SimpleNamespace
        from aubo_cartesian_planner import validate_saved_geometry
        with tempfile.TemporaryDirectory() as root:
            path,a,hands=self.fixture(root)
            with patch('aubo_cartesian_planner.actual_model',return_value=SimpleNamespace(fk=lambda f,q:np.eye(4))),patch('collision_scene.Scene',return_value=SimpleNamespace(check=lambda a,h:[{'a':'arm','b':'table'}])):
                with self.assertRaisesRegex(PlanningFailure,'current full-scene'):validate_saved_geometry(path,a,{}, {'left':{},'right':{}},hands,Path(root))

    def test_cached_path_must_match_current_requested_poses(self):
        from types import SimpleNamespace
        from aubo_cartesian_planner import validate_saved_geometry
        with tempfile.TemporaryDirectory() as root:
            path,a,hands=self.fixture(root);a['left_aubo_base_T_flange'][:,0,3]=.1
            with patch('aubo_cartesian_planner.actual_model',return_value=SimpleNamespace(fk=lambda f,q:np.eye(4))):
                with self.assertRaisesRegex(ValueError,'does not match requested'):validate_saved_geometry(path,a,{}, {'left':{},'right':{}},hands,Path(root))

    def test_cached_path_cannot_omit_original_timestamps(self):
        from aubo_cartesian_planner import validate_saved_geometry
        with tempfile.TemporaryDirectory() as root:
            path,a,hands=self.fixture(root,(0.,.1));a['time_s']=np.array([0.,.05,.1])
            with self.assertRaisesRegex(ValueError,'omits an original'):validate_saved_geometry(path,a,{}, {'left':{},'right':{}},hands,Path(root))

class FixedGeometryTests(unittest.TestCase):
    def test_cached_hand_transforms_match_independent_pinocchio_fk(self):
        import pinocchio as pin
        c=config(HERE/'config/human_replay.json');snap=read_json(HERE/'tests/fixtures/controller_kinematics.json');scene=Scene(c,snap)
        for side in ('left','right'):
            urdf=Path(c['source_episode_root'])/f'assets/wuji/urdf/{side}.urdf';model=pin.buildModelFromUrdf(str(urdf));data=model.createData()
            names=[f'{side[0]}_{f}_{s}' for f in ('thumb','index_finger','middle_finger','ring_finger','pinky') for s in (('cmc_flex','cmc_abd','mcp','ip') if f=='thumb' else ('mcp_flex','mcp_abd','pip','dip'))]
            for q in (np.zeros(20),np.random.default_rng(5).uniform(-.1,.2,20)):
                qp=np.zeros(model.nq)
                for name,value in zip(names,q):qp[model.joints[model.getJointId(name)].idx_q]=value
                pin.framesForwardKinematics(model,data,qp);poses=scene.hand_local_poses(side,q)
                for name,T in poses.items():np.testing.assert_allclose(T,data.oMf[model.getFrameId(name)].homogeneous,atol=1e-10)
                self.assertIs(poses,scene.hand_local_poses(side,q))

if __name__=='__main__':unittest.main()
