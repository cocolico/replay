"""Collision-constrained Cartesian graph search and coherent joint winding.

No device SDK imports. Callbacks supply the calibrated robot/collision model.
All angles remain unwrapped radians. Edge checks are discrete, not CCD.
"""
from pathlib import Path
import sys
from itertools import islice
import numpy as np

VENDOR = Path(__file__).resolve().parent/'vendor/planning_python'
if VENDOR.exists():sys.path.insert(0, str(VENDOR))


class PlanningFailure(RuntimeError):
    def __init__(self, message, report=None):
        super().__init__(message)
        self.report = dict(report or {}, passed=False, reason=message)


def motion_samples(start, goal, max_joint_step):
    start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
    if start.ndim!=1 or start.shape != goal.shape or not np.isfinite([start, goal]).all():raise ValueError('Invalid edge endpoints')
    if not np.isfinite(max_joint_step) or max_joint_step <= 0:raise ValueError('Invalid edge resolution')
    count = max(1, int(np.ceil(np.max(abs(goal-start))/max_joint_step)))
    return start+(goal-start)*np.linspace(0., 1., count+1)[:, None]


def edge_valid(start, goal, valid, max_joint_step=.01):
    return all(valid(q) for q in motion_samples(start, goal, max_joint_step))


def cartesian_graph(times, targets, candidates, node_valid, connect, *, max_paths=8, progress=None, retain_graph=False):
    """Search all reachable waypoint candidates; do not commit a greedy prefix.

    candidates(index, target, previous_nodes) -> IK arrays
    connect(index, previous_q, next_q) -> (edge_times, edge_positions), or None
    Collision checks belong to node_valid and connect, before graph insertion.
    """
    import networkx as nx
    times = np.asarray(times, dtype=float)
    if len(times)<2 or len(targets)!=len(times) or not np.isfinite(times).all() or np.any(np.diff(times)<=0):raise ValueError('Invalid Cartesian timeline')
    if max_paths<1:raise ValueError('max_paths must be positive')
    graph = nx.DiGraph();source = ('source',);sink = ('sink',);previous = []
    stats = dict(planner='collision_constrained_waypoint_graph', graph_library='networkx',
                 collision_nodes_rejected=0, edges_tested=0, edges_rejected=0, layers=[])
    for i, target in enumerate(targets):
        offered = candidates(i, target, [graph.nodes[n]['q'] for n in previous])
        current = []
        for q in offered:
            q = np.asarray(q, dtype=float)
            if q.ndim!=1 or not np.isfinite(q).all():raise ValueError('Invalid IK candidate')
            if any(np.max(abs(q-graph.nodes[n]['q']))<1e-4 for n in current):continue
            if not node_valid(q):stats['collision_nodes_rejected']+=1;continue
            n = (i, len(current));graph.add_node(n, q=q.copy());current.append(n)
        if i == 0:
            for n in current:graph.add_edge(source, n, weight=0.)
        else:
            for parent in previous:
                a = graph.nodes[parent]['q']
                for child in current:
                    b = graph.nodes[child]['q'];stats['edges_tested']+=1
                    segment = connect(i, a, b)
                    if segment is None:stats['edges_rejected']+=1;continue
                    t, rows = map(np.asarray, segment)
                    if rows.shape!=(len(t),len(a)) or len(t)<2 or not np.isfinite(rows).all() or not np.isfinite(t).all():raise ValueError('Invalid planned graph edge')
                    if not np.allclose(rows[0],a,rtol=0,atol=1e-9) or not np.allclose(rows[-1],b,rtol=0,atol=1e-9):raise ValueError('Graph edge does not reach its nodes')
                    if abs(t[0]-times[i-1])>1e-9 or abs(t[-1]-times[i])>1e-9 or np.any(np.diff(t)<=0):raise ValueError('Graph edge has invalid timing')
                    graph.add_edge(parent, child, weight=float(np.linalg.norm(np.diff(rows,axis=0),axis=1).sum()), segment=(t.copy(), rows.copy()))
            current = [n for n in current if graph.in_degree(n)>0]
        stats['layers'].append(dict(frame=i, source_time_s=float(times[i]), candidates=len(offered), reachable=len(current)))
        if not current:raise PlanningFailure(f'No collision-free connected IK candidates at frame {i}', dict(stats, failed_frame=i, source_time_s=float(times[i])))
        previous = current
        if progress is not None:progress(i, len(times), len(current))
    for n in previous:graph.add_edge(n, sink, weight=0.)
    results = []
    for path in islice(nx.shortest_simple_paths(graph, source, sink, weight='weight'), max_paths):
        nodes = path[1:-1];parts = [graph.edges[a,b]['segment'] for a,b in zip(nodes[:-1],nodes[1:])]
        t = np.concatenate([parts[0][0]]+[p[0][1:] for p in parts[1:]])
        q = np.concatenate([parts[0][1]]+[p[1][1:] for p in parts[1:]])
        results.append((t,q))
    stats.update(passed=True, graph_nodes=graph.number_of_nodes(), graph_edges=graph.number_of_edges(), solutions=len(results), edge_checks='adaptive Cartesian IK plus joint-space sub-samples; no CCD')
    return (results, stats, graph) if retain_graph else (results, stats)


def combine_arm_graphs(times,left_graph,right_graph,valid,*,max_paths=8,resolution_rad=.01,progress=None):
    """Search the product graph with inter-arm collision constraints on nodes/edges."""
    import networkx as nx
    sink=('sink',)
    viable=[nx.ancestors(g,sink) for g in (left_graph,right_graph)]
    layers=[];lookup={}
    for i in range(len(times)):
        pairs=[]
        left=[n for n in viable[0] if len(n)==2 and n[0]==i]
        right=[n for n in viable[1] if len(n)==2 and n[0]==i]
        for l in sorted(left):
            for r in sorted(right):
                q=np.r_[left_graph.nodes[l]['q'],right_graph.nodes[r]['q']]
                pairs.append(q);lookup[(i,tuple(q))]=(l,r)
        layers.append(pairs)
    def connect(i,a,b):
        l0,r0=lookup[(i-1,tuple(a))];l1,r1=lookup[(i,tuple(b))]
        if not left_graph.has_edge(l0,l1) or not right_graph.has_edge(r0,r1):return None
        tl,ql=left_graph.edges[l0,l1]['segment'];tr,qr=right_graph.edges[r0,r1]['segment']
        if not np.array_equal(tl,tr):raise ValueError('Arm graph edge timelines differ')
        q=np.c_[ql,qr]
        if not all(edge_valid(x,y,valid,resolution_rad) for x,y in zip(q[:-1],q[1:])):return None
        return tl,q
    paths,report=cartesian_graph(times,[None]*len(times),lambda i,t,p:layers[i],valid,connect,max_paths=max_paths,progress=progress)
    report.update(planner='dual_arm_product_graph',dimension=len(paths[0][1][0]),full_scene_collision_during_search=True,resolution_rad=resolution_rad)
    return paths,report


def coherent_winding(path,start,lower,upper):
    """Choose one constant 2*pi shift per joint for the entire replay.

    This preserves geometry and all differences, and never hides an internal
    jump. Every point must fit controller limits after the same shift.
    """
    path=np.asarray(path,dtype=float);start,lower,upper=[np.asarray(x,dtype=float) for x in (start,lower,upper)]
    if path.ndim!=2 or len(path)<2 or path.shape[1]!=len(start) or start.shape!=lower.shape or start.shape!=upper.shape:raise ValueError('Invalid winding dimensions')
    if not np.isfinite(path).all() or not np.isfinite([start,lower,upper]).all() or np.any(lower>=upper):raise ValueError('Invalid winding input')
    period=2*np.pi;smallest=np.ceil((lower-path.min(0)-1e-10)/period).astype(int);largest=np.floor((upper-path.max(0)+1e-10)/period).astype(int)
    if np.any(smallest>largest):raise PlanningFailure('No whole-trajectory winding fits controller limits')
    preferred=np.rint((start-path[0])/period).astype(int);turns=np.clip(preferred,smallest,largest)
    shifted=path+period*turns
    if np.any(shifted<lower-1e-9) or np.any(shifted>upper+1e-9):raise PlanningFailure('Winding limit verification failed')
    return shifted,turns
