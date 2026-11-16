"""Calibrated Aubo adapter for collision-constrained Cartesian waypoint graphs."""
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from replay_pipeline import SIDES,write_json
from aubo_continuous_ik import actual_model,cartesian_targets,wind_near
from collision_planning import cartesian_graph,edge_valid,PlanningFailure
from device_backends import Aubo
from aubo_continuous_ik import ContinuousIK,DifferentialIK


class ArmGraphSolutions(list):
    def __init__(self,paths,graph,waypoint_times):
        super().__init__(paths);self.graph=graph;self.waypoint_times=waypoint_times


class SDKCandidates:
    """Persistent read-only SDK connection; no Runtime/Servo/hand command calls."""
    def __init__(self,cfg,model,limit,valid):
        self.arm=Aubo(cfg);self.model=model;self.limit=limit;self.valid=valid
        self.current=np.asarray(self.arm.snapshot()['joint_positions_rad']);self.rng=np.random.default_rng(20261113)
        self.stats=dict(sdk_ik_calls=0,pose_rejected=0,collision_rejected=0)
    def close(self):self.arm.close()
    def __call__(self,index,target,previous):
        references=previous or [self.current,np.zeros(6)]
        seeds=list(previous)+[self.current,np.zeros(6)]
        seeds+=list(self.rng.uniform(-np.pi,np.pi,(24 if index==0 else 2,6)))
        pool=[]
        for seed in seeds:
            try:self.stats['sdk_ik_calls']+=1;q=self.arm.ik(target,seed)
            except (ValueError,RuntimeError):continue
            # A winding is chosen while planning the entire path, never in the command loop.
            refs=[min(references,key=lambda r:np.linalg.norm(wind_near(q,r,self.model.lower,self.model.upper)-r)),np.zeros(6)]
            for ref in refs:
                x=wind_near(q,ref,self.model.lower,self.model.upper)
                if any(np.max(abs(x-old))<1e-3 for old in pool):continue
                def residual(v):
                    a=self.model.fk('flange',v)
                    return np.r_[a[:3,3]-target[:3,3],Rotation.from_matrix(target[:3,:3].T@a[:3,:3]).as_rotvec()]
                fitted=least_squares(residual,x,bounds=(self.model.lower,self.model.upper),max_nfev=60,ftol=1e-12,xtol=1e-12,gtol=1e-12)
                x=fitted.x;e=residual(x)
                if np.linalg.norm(e[:3])>1e-5 or np.linalg.norm(e[3:])>1e-4:self.stats['pose_rejected']+=1;continue
                if not self.valid(x):self.stats['collision_rejected']+=1;continue
                if not any(np.max(abs(x-old))<1e-3 for old in pool):pool.append(x.copy())
        # Keep physical branches represented before filling equivalent windings.
        groups={}
        for q in pool:
            key=tuple(np.round((q+np.pi)%(2*np.pi)-np.pi,2))
            groups.setdefault(key,[]).append(q)
        ranked=[]
        def score(q):return min(np.linalg.norm(q-r) for r in references)
        for group in groups.values():group.sort(key=score)
        for rank in range(max((len(g) for g in groups.values()),default=0)):
            ranked.extend(sorted([g[rank] for g in groups.values() if len(g)>rank],key=score))
        return ranked[:self.limit]


def planning_waypoint_indices(count,stride=1):
    if count<2 or not isinstance(stride,(int,np.integer)) or stride<1:raise ValueError('Expected at least two source points and a positive waypoint stride')
    return np.unique(np.r_[np.arange(0,count,stride,dtype=int),count-1])


def connector_query(source_times,start,end,max_step_s=.01):
    """Reduce graph anchors, not the original Cartesian constraints on edges."""
    source_times=np.asarray(source_times,dtype=float)
    if source_times.ndim!=1 or len(source_times)<2 or np.any(np.diff(source_times)<=0) or not np.isfinite(source_times).all():raise ValueError('Invalid source timeline')
    if not start<end or max_step_s<=0:raise ValueError('Invalid connector interval')
    original=source_times[(source_times>=start)&(source_times<=end)]
    grid=np.linspace(start,end,max(1,int(np.ceil((end-start)/max_step_s)))+1)
    # Preserve original timestamps exactly; avoid artificial near-duplicate times.
    if len(original):grid=grid[np.min(abs(grid[:,None]-original[None,:]),axis=1)>1e-9]
    return np.sort(np.r_[original,grid])


def plan_arm(c,side,arrays,controllers,hands,out,scene,*,backend='continuous',branches=16,max_paths=8,edge_step_rad=.01,waypoint_stride=1):
    model=actual_model(controllers[side],out/f'{side}_actual_dh.urdf')
    collision_log=[];context={'frame':0,'phase':'candidate'}
    static={s:np.asarray(hands[s]['positions_rad']) for s in SIDES}
    def valid(q):
        arms={s:controllers[s]['joint_positions_rad'] for s in SIDES};arms[side]=q
        findings=scene.check(arms,static)
        if findings:
            collision_log.append(dict(context,findings=findings))
            if len(collision_log)>12:del collision_log[0]
        return not findings
    factory=SDKCandidates(c['sides'][side],model,branches,valid)
    solver={'continuous':ContinuousIK,'differential':DifferentialIK}[backend](model,'flange',range(6))
    source_times=arrays['time_s'];source_targets=arrays[side+'_aubo_base_T_flange']
    indices=planning_waypoint_indices(len(source_times),waypoint_stride)
    times=source_times[indices];targets=source_targets[indices]
    print(side,'source waypoints',len(source_times),'-> planning waypoints',len(indices),'stride',waypoint_stride,flush=True)
    edge_stats=dict(ik_failure=0,endpoint_mismatch=0,edge_collision=0,tracking_cache_hits=0,tracking_cache_misses=0)
    def candidates(i,target,previous):
        context.update(frame=i,phase='candidate');return factory(i,target,previous)
    tracking_cache={}
    def connect(i,start,goal):
        context.update(frame=i,phase='edge')
        # Tracking depends on the start and Cartesian curve, not on which IK
        # endpoint candidate is being compared. Reuse that deterministic work.
        key=(i,tuple(start))
        if key not in tracking_cache:
            edge_stats['tracking_cache_misses']+=1
            query=connector_query(source_times,times[i-1],times[i])
            desired=cartesian_targets(source_times,source_targets,query)
            q=start.copy();rows=[q.copy()];failed=False
            for j,target in enumerate(desired[1:],1):
                try:next_q,_=solver.solve_near(target,q,max_step=.12)
                except (ValueError,RuntimeError):edge_stats['ik_failure']+=1;failed=True;break
                # The final edge is checked against the exact child node below.
                if j<len(query)-1 and not edge_valid(q,next_q,valid,edge_step_rad):
                    edge_stats['edge_collision']+=1;failed=True;break
                rows.append(next_q.copy());q=next_q
            tracking_cache[key]=None if failed else (query,np.asarray(rows))
        else:edge_stats['tracking_cache_hits']+=1
        cached=tracking_cache[key]
        if cached is None:return None
        query,rows=cached
        if np.max(abs(rows[-1]-goal))>.002:edge_stats['endpoint_mismatch']+=1;return None
        if np.max(abs(rows[-2]-goal))>.12+1e-8:return None
        if not edge_valid(rows[-2],goal,valid,edge_step_rad):edge_stats['edge_collision']+=1;return None
        result=rows.copy();result[-1]=goal
        return query,result
    def progress(i,n,k):
        if i%20==0:
            print(side,'Cartesian graph',i,'/',n,'reachable',k,flush=True)
            write_json(out/f'{side}_graph_progress.json',dict(frame=i,frames=n,reachable=k,sdk=factory.stats,edge_checks=edge_stats))
    try:
        paths,report,graph=cartesian_graph(times,targets,candidates,valid,connect,max_paths=max_paths,progress=progress,retain_graph=True)
        report.update(backend=backend,sdk=factory.stats,edge_checks=edge_stats,edge_resolution_rad=edge_step_rad,source_frames=len(source_times),planning_waypoints=len(times),waypoint_stride=waypoint_stride,source_waypoint_indices=indices.tolist())
        results=[]
        for i,(t,q) in enumerate(paths):
            path=out/f'{side}_graph_path{i}_offline.npz';np.savez_compressed(path,time_s=t,joint_positions_rad=q,execution_enabled=False);results.append((t,q,path))
        write_json(out/f'{side}_graph.json',report);return ArmGraphSolutions(results,graph,times)
    except PlanningFailure as error:
        error.report.update(sdk=factory.stats,edge_checks=edge_stats,collision_failures=collision_log,source_frames=len(source_times),planning_waypoints=len(times),waypoint_stride=waypoint_stride,source_waypoint_indices=indices.tolist(),failed_source_frame=int(indices[error.report['failed_frame']]) if 'failed_frame' in error.report else None)
        write_json(out/f'{side}_graph.json',error.report);raise
    finally:factory.close()


def validate_saved_geometry(path,arrays,c,controllers,hands,out,*,resolution_rad=.01):
    """Reuse a geometric candidate, not its acceptance flags; recheck from scratch."""
    from collision_scene import Scene
    with np.load(path,allow_pickle=False) as z:
        if 'execution_enabled' not in z or bool(z['execution_enabled']):raise ValueError('Saved geometry must explicitly be offline')
        t=z['time_s'].copy();q=z['joint_positions_rad'].copy()
    original=arrays['time_s']
    if t.ndim!=1 or len(t)<2 or q.shape!=(len(t),12) or not np.isfinite(t).all() or not np.isfinite(q).all() or np.any(np.diff(t)<=0):raise ValueError('Invalid saved dual-arm geometry')
    if abs(t[0]-original[0])>1e-9 or abs(t[-1]-original[-1])>1e-9:raise ValueError('Saved geometry does not cover the requested source timeline')
    for value in original:
        if np.min(abs(t-value))>1e-9:raise ValueError('Saved geometry omits an original source timestamp')
    for index,side in enumerate(SIDES):
        model=actual_model(controllers[side],out/f'{side}_actual_dh.urdf');targets=cartesian_targets(original,arrays[side+'_aubo_base_T_flange'],t)
        for row,target in zip(q[:,index*6:(index+1)*6],targets):
            actual=model.fk('flange',row)
            if np.linalg.norm(actual[:3,3]-target[:3,3])>1e-5 or Rotation.from_matrix(actual[:3,:3].T@target[:3,:3]).magnitude()>1e-4:raise ValueError('Saved geometry does not match requested Cartesian targets: '+side)
    scene=Scene(c,controllers);static={s:hands[s]['positions_rad'] for s in SIDES}
    def valid(row):return not scene.check({'left':row[:6],'right':row[6:]},static)
    for i,(a,b) in enumerate(zip(q[:-1],q[1:])):
        if not edge_valid(a,b,valid,resolution_rad):raise PlanningFailure('Saved geometry fails current full-scene collision check',dict(failed_segment=i,source_time_s=float(t[i])))
        if i%500==0:print('revalidate saved dual-arm geometry',i,'/',len(q)-1,flush=True)
    from replay_pipeline import provenance
    write_json(out/'geometry_reuse.json',dict(passed=True,acceptance_flags_reused=False,source=provenance(path),frames=len(t),full_scene_rechecked=True,cartesian_targets_rechecked=True,execution_enabled=False))
    return [((t,q[:,:6],path),(t,q[:,6:],path))]
