#!/usr/bin/env python3
"""Execute the latest plan selected by recalculate_replay without replanning."""
import json,os
from pathlib import Path

ROOT=Path(__file__).resolve().parent
pointer=ROOT/'output/latest_checked_replay.json'
if not pointer.exists():raise SystemExit('没有已记录的通过计划；请先运行 ./recalculate_replay TRAJECTORY')
record=json.loads(pointer.read_text())
if record.get('accepted') is not True:raise SystemExit('最近一次完整预检未通过或未完成；不能执行旧计划')
folder=Path(record['run_directory']).resolve()
if not (folder/'runtime.json').is_file() or not (folder/'runtime.npz').is_file():
    raise SystemExit('最新计划文件不完整；请重新运行 ./recalculate_replay TRAJECTORY')
os.execv(str(ROOT/'execute_checked_replay'),[str(ROOT/'execute_checked_replay'),str(folder)])
