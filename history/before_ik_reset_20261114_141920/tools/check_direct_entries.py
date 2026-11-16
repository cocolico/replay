"""Entry-first necessary-condition check of complete arm-path candidates; no SDK access."""
import argparse,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from replay_pipeline import config,read_json,write_json,provenance
from collision_scene import Scene
from collision_planning import coherent_winding
from runtime_plan import direct_transition,split
from aubo_arms_replay import SETTINGS


def check(folder):
    folder=Path(folder).resolve();c=config(folder/'config.json');snapshot=read_json(folder/'planning_snapshot.json');controllers=snapshot['controllers'];hands=snapshot['hands']
    scene=Scene(c,controllers,SETTINGS['mesh_clearance_m']);solutions={}
    for side in ('left','right'):
        report=read_json(folder/(side+'_graph.json'))
        if not report.get('passed'):raise ValueError('Incomplete arm graph: '+side)
        paths=[]
        for p in sorted(folder.glob(side+'_graph_path*_offline.npz')):
            with np.load(p,allow_pickle=False) as z:paths.append((z['time_s'].copy(),z['joint_positions_rad'].copy(),p))
        if not paths:raise ValueError('No complete arm paths: '+side)
        solutions[side]=paths
    start_arm=np.r_[controllers['left']['joint_positions_rad'],controllers['right']['joint_positions_rad']]
    lower=np.r_[controllers['left']['joint_min_rad'],controllers['right']['joint_min_rad']];upper=np.r_[controllers['left']['joint_max_rad'],controllers['right']['joint_max_rad']]
    start=np.r_[start_arm[:6],hands['left']['positions_rad'],start_arm[6:],hands['right']['positions_rad']]
    vlim=[];alim=[]
    for side in ('left','right'):
        vlim.extend(np.array(controllers[side]['joint_max_speed_rad_s'])*SETTINGS['arm_velocity_fraction']);vlim.extend([1.]*20)
        alim.extend(np.array(controllers[side]['joint_max_acceleration_rad_s2'])*SETTINGS['arm_acceleration_fraction']);alim.extend([1.]*20)
    summary={'execution_attempted':False,'ready_for_execution':False,'scope':'Necessary direct-entry condition only; any surviving candidate still requires full replay/SDK/live validation','settings':SETTINGS,'start_snapshot':provenance(folder/'planning_snapshot.json'),'pairs':[]}
    for left in solutions['left']:
        for right in solutions['right']:
            np.testing.assert_array_equal(left[0],right[0]);path,turns=coherent_winding(np.c_[left[1],right[1]],start_arm,lower,upper)
            goal=np.r_[path[0,:6],hands['left']['positions_rad'],path[0,6:],hands['right']['positions_rad']]
            ticks,ramp,report=direct_transition(start,goal,vlim,alim,SETTINGS);first=None
            for i,row in enumerate(ramp):
                arms,fingers=split(row);findings=scene.check(arms,fingers)
                if findings:first={'frame':i,'runtime_time_s':float(ticks[i]),'phase':'transition','findings':findings};break
            record={'branches':[left[2].name,right[2].name],'entry_passed':first is None,'entry_planning':report,'transition_s':float(ticks[-1]),'checked_samples':i+1,'first_collision':first,'winding_turns':turns.tolist()}
            summary['pairs'].append(record);write_json(folder/'direct_entry_first_checks.json',summary);print('ENTRY',record['branches'],'PASS' if first is None else first,flush=True)
    summary['entry_passed_pairs']=sum(v['entry_passed'] for v in summary['pairs']);summary['all_tested_pairs_blocked']=summary['entry_passed_pairs']==0
    write_json(folder/'direct_entry_first_checks.json',summary);print('ENTRY RESULT',summary['entry_passed_pairs'],'/',len(summary['pairs']),flush=True)
    return summary

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path);a=p.parse_args();check(a.folder)
