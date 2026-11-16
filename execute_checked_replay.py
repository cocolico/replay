#!/usr/bin/env python3
"""Execute one unchanged, previously accepted camera replay plan without replanning."""
import argparse,sys,time
from pathlib import Path
import numpy as np
from replay_pipeline import HERE,config,read_json,write_json
from read_hands import Hands,check_current_limit
from replay_control import acquire_replay_lock,check_hand_controls,execute
from aubo_arms_replay import require_plan


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory',type=Path)
    args=parser.parse_args();folder=args.run_directory.resolve()
    plan=read_json(folder/'runtime.json');c=config(folder/'config.json')
    with np.load(folder/'runtime.npz',allow_pickle=False) as z:trajectory=z['joint_positions_left_right_rad'].copy()
    lock=acquire_replay_lock();reader=None
    try:
        reader=Hands(plan['settings']['hands']);check_hand_controls(reader,plan['settings'])
        currents=check_current_limit(reader.snapshot(),plan['settings']['maximum_hand_current_a'])
        require_plan(c,plan,trajectory,reader)
        write_json(HERE/'output/replay_session.json',{'pid':__import__('os').getpid(),'name':plan['name'],'status':'preflight','reused_checked_plan':True,'hand_current_check':currents,'updated_unix_s':time.time()})
        execute(c,plan['settings'],trajectory,plan,reader,arms_only=plan['arms_only'])
    finally:
        if reader is not None:reader.close()
        lock.close()


if __name__=='__main__':
    try:main()
    except (ValueError,RuntimeError) as error:
        print('BLOCKED:',error,flush=True);raise SystemExit(2)
    except KeyboardInterrupt:raise SystemExit(130)
