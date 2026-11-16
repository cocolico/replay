#!/usr/bin/env python3
"""Run one full camera replay preflight and select it as the latest accepted plan."""
import argparse,json,subprocess,sys
from pathlib import Path

ROOT=Path(__file__).resolve().parent
POINTER=ROOT/'output/latest_checked_replay.json'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectory',type=Path)
    parser.add_argument('--speed',type=float,default=.2)
    args=parser.parse_args()
    POINTER.parent.mkdir(parents=True,exist_ok=True)
    pending={'accepted':False,'trajectory':str(args.trajectory.resolve()),'reason':'full preflight pending or failed'}
    temporary=POINTER.with_suffix('.tmp');temporary.write_text(json.dumps(pending,ensure_ascii=False,indent=2)+'\n');temporary.replace(POINTER)
    command=[str(ROOT/'replay_arms'),str(args.trajectory.resolve()),'--speed',str(args.speed)]
    process=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
    run_directory=None
    try:
        assert process.stdout is not None
        for line in process.stdout:
            print(line,end='',flush=True)
            if line.startswith('OUTPUT '):run_directory=Path(line[7:].strip()).resolve()
        code=process.wait()
    except KeyboardInterrupt:
        process.send_signal(__import__('signal').SIGINT);process.wait();raise
    if code!=0:return code
    if run_directory is None:raise RuntimeError('Preflight did not report its output directory')
    runtime=run_directory/'runtime.json'
    report=json.loads(runtime.read_text())
    if report.get('ready_for_execution') is not True:raise RuntimeError('Preflight exited without an accepted runtime plan')
    payload={'accepted':True,'run_directory':str(run_directory),'runtime_json':str(runtime),'trajectory':str(args.trajectory.resolve()),
             'duration_s':report['duration_s'],'effective_speed':report['effective_speed'],
             'hand_self_collision_policy':report.get('hand_self_collision_policy'),
             'hand_self_collision_warning_frames':report.get('hand_self_collision_warning_frames',0)}
    temporary=POINTER.with_suffix('.tmp');temporary.write_text(json.dumps(payload,ensure_ascii=False,indent=2)+'\n');temporary.replace(POINTER)
    print('\n最新通过计划已记录。直接执行命令：',flush=True)
    print(f'  ./execute_latest_replay',flush=True)
    return 0


if __name__=='__main__':raise SystemExit(main())
