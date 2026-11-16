# 恢复后的 IK 与进场计算

2026-11-14 将 IK 规划入口恢复到 Git 提交 `bd056add7676de2ad719a853a3118c03039de26f`（2026-11-13 15:03）的算法。
修改前的代码、图搜索模块、辅助脚本和依赖说明已备份至 `history/before_ik_reset_20261114_141920/`。

## 当前流程

1. 只读获取实际 DH、关节限位、当前位置及双手固定手型。
2. 使用 Aubo SDK 为首帧生成初始 IK 候选，以实际 DH 数值细化并复核控制器 FK。默认最多8个候选。
3. 每个初始候选分别通过 continuous（默认）或 differential IK 顺序跟踪整条笛卡尔轨迹；保留原始帧并插入不超过0.01秒源时间的采样。逐点检查同臂、固定手和桌面，碰撞即拒绝该分支。
4. 枚举完整左右臂分支的组合；从实测起点直接五次 minimum-jerk 关节插值到轨迹首帧，进场至少10秒，并按速度和加速度限额延长。
5. 重定时、完整100Hz装配碰撞检查、关节动态检查、插值末端位姿检查与控制器FK复核。余量仍为2mm。
6. 只有全部通过并显式加 `--execute` 时，才重新核对文件、设备起点并进入执行。

没有候选图、双臂乘积图或独立绕行进场。恢复算法仍包含原有的碰撞检查，并不会绕过碰撞。
保留了后续的固定手FK缓存、碰撞采样计数修复、规划快照及严格起点漂移复核；这些不改变IK选解或进场算法。

## 用法

```bash
# 只读校验，无运动指令
./replay_arms '/path/to/trajectory'
./replay_arms '/path/to/trajectory' --backend differential --branches 8

# 完整校验通过后执行
./replay_arms '/path/to/trajectory' --execute
```

`--planner`、`--entry-planner`、`--waypoint-stride`、`--planned-path` 参数已移除，不会静默使用另一种算法。
当前入口不需要 NetworkX 或 OMPL；原有SDK和环境依赖保持不变。

历史结果和视频仍保留，但其算法与当前入口不同，不能复用其通过标记作为执行依据。
