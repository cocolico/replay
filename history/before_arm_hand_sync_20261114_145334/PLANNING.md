# 594成功时的SDK逐帧逆解流程

2026-11-14 将相机轨迹 `replay_arms` 的求解链路恢复为594成功时使用的算法。
直接复用原有 `diagnose_ik.py` 和 `solve_controller_ik.py`，两份求解代码未修改。
修改前备份：`history/before_sdk594_reset_20261114_142926/`。

## 求解与校验

1. 将受支持的Wuji相机掌根位姿通过标定和安装变换转换到Aubo法兰目标。
2. 调用 `diagnose_ik.diagnose(..., stride=1)`，以原始帧顺序和时间轴做标称URDF数值IK，生成不可执行候选。
3. 每侧调用 `solve_controller_ik.solve_side`：首帧以标称候选首帧为种子；每帧调用SDK逆解；选取限位内最接近上一帧的±2π表示；使用实际DH做least_squares细化；SDK正解复核10微米/0.1毫弧度误差；相邻原始帧超过0.4rad则拒绝。
4. 不在IK中用碰撞图选分支。得到双臂完整关节轨迹后，固定当前实测手型，重定时并生成100Hz命令；当前关节至首帧使用直接五次minimum-jerk插值，至少10秒。
5. 检查整段限位、速度、加速度、双臂/固定手/桌面网格碰撞，以及运行插值末端误差，再独立SDK正解复核。
6. 只有全部通过且显式 `--execute`，才重新检查实测起点和文件哈希后进入执行。

这是恢复594的求解算法，不是复用594的轨迹或执行批准。相机位姿的来源及坐标变换仍与594的Franka FK数据不同。
保留用户当前设置：只运动臂、手型固定、2mm碰撞余量、默认0.2倍速和现有动态限额。594当时的双手动作、5mm余量和速度参数没有自动覆盖这些设置。
Sharpa掌根/22关节输入仍需明确转换到Wuji掌根定义，不能直接当作Wuji数据。

## 使用

```bash
./replay_arms '/path/to/wuji_camera_trajectory'
./replay_arms '/path/to/wuji_camera_trajectory' --speed 0.2 --execute
```

`--backend`、`--branches` 及此前的图搜索参数已移除，旧命令会明确报参数错误。
`./replay 594` 保留原有整机回放流程。历史报告不能代替新轨迹的校验。

## 输出

`nominal_ik_diagnostic.json`、`nominal_ik_candidate.npz` 是标称候选；
`left_ik.json/right_ik.json` 和 `left_sdk_offline.npz/right_sdk_offline.npz` 是SDK逐帧求解结果。
它们不是碰撞或执行批准；`pair_checks.json` 保存完整预检，通过后才生成 `runtime.json/.npz`。

## 用户指定阈值更新

2026-11-14：当前 `replay_arms` 桌面余量改为0mm（仍检查穿过标定平面）；
机器人模型之间余量保持2mm；运行插值末端位置误差上限改为2mm，姿态误差上限仍为0.01rad。
原有594整机入口的设置不变。
