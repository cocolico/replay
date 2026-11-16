#!/usr/bin/env python3
"""Local replay console. All robot operations use the existing checked commands."""
import json
import os
from pathlib import Path
import queue
import signal
import subprocess
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext, ttk
from camera_trajectory_input import read_camera_input

ROOT = Path(__file__).resolve().parent
HOME_COMMAND = ROOT.parent / 'client/aubo_wuji/home'


def warning_text(report):
    count = int(report.get('hand_self_collision_warning_frames', 0))
    return f'，手内自碰撞告警 {count} 个采样点（不阻止执行）' if count else ''


def accepted_plan(folder, trajectory):
    """Bind an explicitly selected accepted plan to the matching source trajectory."""
    folder = Path(folder).resolve()
    report = json.loads((folder / 'runtime.json').read_text())
    configuration = json.loads((folder / 'config.json').read_text())
    if not all(report.get(k) is True for k in ('ready_for_execution', 'collision_checked', 'collision_passed')):
        raise ValueError('本次校验未通过')
    if Path(configuration['camera_trajectory_root']).resolve() != Path(trajectory).resolve():
        raise ValueError('校验计划与所选轨迹不匹配')
    if not (folder / 'runtime.npz').is_file():
        raise ValueError('缺少已校验的运行轨迹')
    return folder, report


class ReplayWindow:
    def __init__(self, root):
        self.root = root
        self.trajectory = None
        self.plan = None
        self.process = None
        self.mode = None
        self.closing = False
        self.events = queue.Queue()
        root.title('Aubo + Wuji 轨迹回放')
        root.geometry('1050x720')
        root.minsize(780, 520)
        root.protocol('WM_DELETE_WINDOW', self.close)
        frame = ttk.Frame(root, padding=18)
        frame.pack(fill='both', expand=True)
        ttk.Label(frame, text='Aubo + Wuji 轨迹回放', font=('', 19, 'bold')).pack(anchor='w')
        ttk.Label(frame, text='载入本地文件夹 → 完整校验一次 → 执行本次通过的计划').pack(anchor='w', pady=(6, 16))
        buttons = ttk.Frame(frame)
        buttons.pack(fill='x')
        self.load_button = ttk.Button(buttons, text='① 载入轨迹', command=self.load)
        self.check_button = ttk.Button(buttons, text='② 开始校验', command=self.preflight)
        self.execute_button = ttk.Button(buttons, text='③ 执行', command=self.execute)
        for button in (self.load_button, self.check_button, self.execute_button):
            button.pack(side='left', padx=(0, 12), ipady=8)
        self.plan_button = ttk.Button(buttons, text='载入已校验计划', command=self.load_plan)
        self.plan_button.pack(side='left', padx=(0, 12), ipady=8)
        self.home_button = ttk.Button(buttons, text='Home 复位（真机）', command=self.home)
        self.home_button.pack(side='left', padx=(0, 12), ipady=8)
        self.stop_button = ttk.Button(buttons, text='停止（软件）', command=self.stop)
        self.stop_button.pack(side='right', ipady=8)
        self.path_text = tk.StringVar(value='尚未载入轨迹')
        ttk.Label(frame, textvariable=self.path_text, wraplength=980).pack(anchor='w', pady=(18, 6))
        self.status = tk.StringVar(value='请选择轨迹文件夹。')
        ttk.Label(frame, textvariable=self.status, wraplength=980).pack(anchor='w', pady=(0, 10))
        self.progress = ttk.Progressbar(frame, mode='indeterminate')
        self.progress.pack(fill='x', pady=(0, 12))
        self.log = scrolledtext.ScrolledText(frame, state='disabled', wrap='word', font=('Monospace', 10))
        self.log.pack(fill='both', expand=True)
        ttk.Label(frame, text='校验不驱动设备。手内自碰撞显示告警并继续；桌面、机械臂、双臂或双手之间的碰撞会阻止执行。停止按钮不能替代硬件急停。',
                  wraplength=980).pack(anchor='w', pady=(12, 0))
        self.restore_latest()
        self.refresh()
        root.after(100, self.poll)

    def refresh(self):
        busy = self.process is not None
        self.load_button.config(state='disabled' if busy else 'normal')
        self.plan_button.config(state='disabled' if busy else 'normal')
        self.home_button.config(state='disabled' if busy else 'normal')
        self.check_button.config(state='normal' if self.trajectory and not busy else 'disabled')
        self.execute_button.config(state='normal' if self.plan and not busy else 'disabled')
        self.stop_button.config(state='normal' if busy else 'disabled')

    def append(self, line):
        self.log.config(state='normal')
        self.log.insert('end', line)
        if int(self.log.index('end-1c').split('.')[0]) > 10000:
            self.log.delete('1.0', '1001.0')
        self.log.see('end')
        self.log.config(state='disabled')

    def use_plan(self, folder):
        folder = Path(folder).resolve()
        configuration = json.loads((folder / 'config.json').read_text())
        trajectory = Path(configuration['camera_trajectory_root']).resolve()
        plan, report = accepted_plan(folder, trajectory)
        self.trajectory, self.plan = trajectory, plan
        self.path_text.set(str(trajectory))
        self.status.set(f'已载入通过计划（{report["duration_s"]:.2f}秒{warning_text(report)}），可执行。')
        self.append(f'\n已复用校验计划：{plan}\n')
        self.refresh()

    def restore_latest(self, matching_trajectory=None):
        try:
            record = json.loads((ROOT / 'output/latest_checked_replay.json').read_text())
            if record.get('accepted') is not True:
                return
            folder = Path(record['run_directory'])
            configuration = json.loads((folder / 'config.json').read_text())
            if matching_trajectory is not None and Path(configuration['camera_trajectory_root']).resolve() != matching_trajectory:
                return
            self.use_plan(folder)
        except (OSError, ValueError, KeyError) as error:
            self.append(f'未恢复历史计划：{error}\n')

    def load_plan(self):
        path = filedialog.askdirectory(parent=self.root, title='选择已通过校验的输出目录（包含runtime.json）',
                                       initialdir=str(ROOT / 'output'), mustexist=True)
        if path:
            try:
                self.use_plan(path)
            except (OSError, ValueError, KeyError) as error:
                messagebox.showerror('无法载入计划', str(error), parent=self.root)

    def load(self):
        path = filedialog.askdirectory(parent=self.root, title='选择轨迹文件夹',
                                       initialdir=str(self.trajectory or ROOT.parent / 'try'), mustexist=True)
        if not path:
            return
        selected = Path(path).resolve()
        try:
            arrays, _, input_format = read_camera_input(selected)
        except Exception as error:
            messagebox.showerror('无法载入', str(error), parent=self.root)
            return
        self.trajectory = selected
        self.plan = None
        self.path_text.set(str(selected))
        self.status.set(f'轨迹已载入（{input_format}，{len(arrays["time_s"])}帧），请点击“开始校验”。')
        self.append(f'\n已载入：{selected}\n')
        self.restore_latest(matching_trajectory=selected)
        self.refresh()

    def preflight(self):
        if self.process is not None or not self.trajectory:
            return
        self.plan = None
        self.start('check', [str(ROOT / 'recalculate_replay'), str(self.trajectory)])

    def execute(self):
        if self.process is not None or not self.plan:
            return
        self.start('execute', [str(ROOT / 'execute_checked_replay'), str(self.plan)])

    def home(self):
        if self.process is not None:
            return
        if not HOME_COMMAND.is_file() or not os.access(HOME_COMMAND, os.X_OK):
            messagebox.showerror('Home不可用', '未找到可执行命令：' + str(HOME_COMMAND), parent=self.root)
            return
        self.plan = None
        self.append('\nHome目标：wuji_outroom_flower_episode1400 的首帧（双臂及手指），并非关节归零。\n')
        self.start('home', [str(HOME_COMMAND), '--execute'])

    def start(self, mode, command):
        self.mode = mode
        statuses = {'check':'正在完整校验，设备保持静止……',
                    'execute':'正在核对起点、电流及文件；通过后启动真机……',
                    'home':'正在校验Home进场；通过后真机移动到指定首帧……'}
        labels = {'check':'开始完整校验', 'execute':'开始执行已校验计划', 'home':'开始Home复位（真机）'}
        self.status.set(statuses[mode])
        self.append('\n' + labels[mode] + '\n')
        try:
            self.process = subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE,
                                            stderr=subprocess.STDOUT, text=True, encoding='utf-8',
                                            errors='replace', bufsize=1, start_new_session=True)
        except OSError as error:
            self.status.set(f'启动失败：{error}')
            self.refresh()
            return
        process = self.process
        def read_output():
            folder = None
            try:
                for line in process.stdout:
                    if line.startswith('OUTPUT '):
                        folder = line[7:].strip()
                    self.events.put(('line', line))
                self.events.put(('done', (process.wait(), mode, folder)))
            except Exception as error:
                self.events.put(('line', f'日志读取错误：{error}\n'))
                self.events.put(('done', (process.wait(), mode, None)))
        threading.Thread(target=read_output, daemon=True).start()
        self.progress.start(12)
        self.refresh()

    def poll(self):
        for _ in range(300):
            try:
                kind, value = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == 'line':
                self.append(value)
                if value.startswith('progress '):
                    self.status.set('真机运行中：' + value.strip())
            else:
                code, mode, folder = value
                self.process = None
                self.progress.stop()
                if mode == 'check' and code == 0 and folder:
                    try:
                        self.plan, report = accepted_plan(folder, self.trajectory)
                        self.status.set(f'校验通过，计划 {report["duration_s"]:.2f} 秒{warning_text(report)}。可点击“执行”。')
                    except Exception as error:
                        self.plan = None
                        self.status.set(f'计划不可用：{error}')
                elif mode == 'check':
                    self.plan = None
                    self.status.set('校验未通过或已中断，请查看日志。')
                elif mode == 'home':
                    self.plan = None
                    self.status.set('Home复位完成。旧起点计划已清除，请对所需轨迹重新校验。' if code == 0 else
                                    'Home未完成或已停止，请查看日志和停止确认。旧起点计划已清除。')
                else:
                    self.status.set('执行完成。计划已保留；再次执行仍会快速核对起点，起点变化时才需重新校验。' if code == 0 else
                                    '执行未完成或被停止，计划已保留；请查看日志中的错误与停止确认。')
                self.refresh()
                if self.closing:
                    self.root.destroy()
                    return
        self.root.after(100, self.poll)

    def stop(self):
        if self.process is not None:
            try:
                os.killpg(self.process.pid, signal.SIGINT)
                self.status.set('已请求软件停止，等待程序退出及设备停止确认……')
            except ProcessLookupError:
                pass

    def close(self):
        if self.process is not None:
            self.closing = True
            self.stop()
        else:
            self.root.destroy()


def main():
    root = tk.Tk()
    ReplayWindow(root)
    root.mainloop()


if __name__ == '__main__':
    main()
