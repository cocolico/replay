# Aubo i7H + Wuji 轨迹 replay

本目录以 **Wuji 掌根的六维位姿 + 左右各20个手指关节** 为目标，用当前左右 Aubo 控制器的实际运动学生成六轴关节轨迹。Franka 关节只用于还原源工位的 Wuji 掌根位姿，不作为 Aubo 的执行关节。

## 当前结果（2026-11-12）

| 项目 | 结果 |
| --- | --- |
| 左臂 | Aubo i7H，192.168.1.3，激活负载 **1.238 kg**，质心 `[0.00434, 0.02465, 0.0013] m` |
| 右臂 | Aubo i7H，192.168.1.2，激活负载 **0.632 kg**，质心 `[-0.0073, 0.03246, 0.15035] m` |
| TCP | 两侧当前均为零偏置，控制目标为实际法兰 |
| 手部安装 | 用户确认直连、零平移；左 Rz(0°)，右 Rz(-90°) |
| Wuji 设备 | 左 192.168.1.110:50001 / WH2JA01260923005；右 .111:50001 / WH2KA01260803001 |
| 当前手部控制参数 | 保留设备现有 MIT profile：kp=5、kd=0.1、effort_limit=1 A；不自动改写 |
| 实际模型 | 两侧实际 DH 已从控制器读取；正解与控制器数值闭合，原名义模型约1.42mm偏差不再用于执行关节逆解 |
| 四组逆解 | 共3287帧的左右臂全部完成控制器逆解 + 实际DH数值精化 + 控制器正解逐帧复核 |
| 594 | 原始采样及含当前起点过渡、100Hz插值的全部17168帧通过已建模碰撞检查；本次预检约171.67秒轨迹，含10秒过渡、1秒首帧保持和0.2倍速回放 |
| 615 | 74个原始采样帧有左中指/无名指末节模型相交，启动拦截 |
| 616 | 121个原始采样帧有左中指/无名指末节模型相交，启动拦截 |
| 人手来源 | 已改用用户指定Gemini335观测；44个原始采样帧违反桌面余量，最深约6.15cm，启动拦截 |
| 真机运动验证 | 本次未发送 motor enable、Runtime start、Servo 或手部命令；运动发送/异常停止逻辑已通过模拟测试，尚需首次低速硬件验证 |

逆解中的极小误差是两个计算模型的数值闭合误差，不是机械臂、手眼标定或实际任务的绝对定位精度承诺。

## 启动

在普通终端运行，不需要启动旧 Franka/Sharpa 执行脚本或 O30 驱动。

先只读预检（不使能、不运动）：

```bash
cd /media/descfly/新加卷/ty/replay/code
./replay 594
```

该命令读取新的臂/手状态，从当前实测52关节状态生成到第一帧的过渡，重新做整段100Hz动态/碰撞检查，写入 `output/episode594_runtime.json`。完整网格检查需要数分钟，终端打印进度。设备在预检期间保持静止；若之后发生明显移动、负载/TCP/控制参数变化，会在使能前拒绝。

确认现场活动范围清空、设备状态正常后，以下命令才会运动：

```bash
./replay 594 --execute --speed 0.2 --hold-last
```

- `--execute` 才进入执行分支；没有这个参数只读预检。
- 默认0.2倍速，双臂双手统一减速，不能分别改变时间轴。若动态限制仍不满足，准备阶段会进一步降低整体速度并记录 `effective_speed`。
- 两臂通过原生 SDK `servoJoint`（弧度）100Hz下发，双手同步发布20个弧度目标；不经过使用度数的旧 Aubo 上层 MoveJ 接口。
- 使能后先在实测起点保持2秒，检查真实命令通道/反馈，再开始到首帧的过渡；过渡也参与碰撞检查，不直接跳到第一帧。
- `--hold-last` 在轨迹结束后继续发布末帧，直到 Ctrl+C。**Ctrl+C 会请求两臂停止、退出 Servo，并禁用双手电机，抓持力会解除。** 停止前先确保抓持物有支撑。
- 不使用 `--hold-last` 时，末帧之后立即进入同样的停止/禁用清理。
- 不自动上电，不修改负载/TCP、手部增益/电流限制。若参数不符，先审阅实际设备设置，不通过改检查结果来放行。
- 615、616、human（或human_camera）目前会打印碰撞错误并退出，即使加 `--execute` 也不会运动。没有跳过碰撞的启动选项。

回放由本助手启动时，可在另一个终端用 `./stop_replay` 请求同样的同步停止；它只向本工作目录登记的真实replay进程发送SIGINT，不按进程名批量杀进程。`output/replay_session.json` 记录PID、预检/运行/末帧保持/停止状态。

同一时刻只允许一个本程序实例。不要同时运行 MANUS/Avatar 遥操、其他手命令发布器或其他臂运动源。当前两个 `/tmp/wuji_*_connection.py` 是只读反馈订阅器，本程序没有终止它们。

## 检查与执行边界

准备阶段包括：

1. 校验源数据、配置和标定哈希，读回设备身份、TCP、负载、实际DH与关节位置/速度/加速度上限。
2. 严格核对左右手地址、串号、handedness、20关节在线数及完整反馈，按设备NID转换成手指优先的顺序。
3. PCHIP导数的三次Hermite重采样，起止速度为零；minimum-jerk起点过渡和首帧保持；全部52关节统一时间轴。
4. 检查位置、速度、加速度。默认臂速度上限为控制器上限的20%、加速度15%；手速度1.5rad/s、加速度5rad/s²；见 `config/runtime.json`。
5. 对全部100Hz采样点检查实际DH下的臂碰撞网格、Wuji早期Beta手网格、双臂/双手之间以及标定桌面半空间。臂/手/桌面余量5mm；同一只手非相邻部件检查真实网格相交。
6. 使用Aubo官方SRDF排除固定/相邻关节覆盖件，使用同一分支两轴指根覆盖件排除规则，排除法兰与相连手根，以及基座安装面。没有排除跨手指末节，所以615/616的中指/无名指相交会被拦截。

这里是离散100Hz网格检查，不是连续碰撞检测，也没有实际物体/未知障碍物的环境网格。桌面采用无限标定平面，几何模型为给定URDF。启动前请移开无关障碍物；模型检查不等于完整现场安全验收。

执行阶段包括：

- 使能前重新核对实测起点，核对负载/TCP/DH/限位未变、Servo未被其他来源占用。
- 两臂两手并发发送，使用同一单调时钟，迟到超过30ms不突发补发，而是终止。
- 双臂独立反馈监视器，双手回调反馈；超过250ms无新反馈或跟踪偏差过大时终止。
- 单侧异常、Ctrl+C/SIGTERM都会通过独立连接同时请求两臂停止，并禁用双手。停止返回异常或机械臂未确认steady会明确报告“停止未完全确认”。软件停止不能替代现场硬件急停。
- 只有在 `--execute` 时才会开启必要的Runtime/Servo和手电机；只读检查不会开启这些状态。
- 源Wuji轨迹的手型和空间位姿没有自动做避碰修改。当前被拦截的三组需要另行审阅修正版或纠正标定/输入定义。

## 数据与文件

- `config/replay.json`：594/615/616的源轨迹、手眼、桌面映射及安装变换；配置内容保持不变。
- `config/human_replay.json`：人手轨迹独立配置，明确绑定 `/home/descfly/ty/desk/observation.json`。
- `config/mapping_from_human_gemini335.json`：由指定源观测和当前工位观测重新计算的映射，含源/目标观测SHA256。
- `config/runtime.json`：时间轴、动态限制、设备串号、当前MIT参数核对值。
- `output/*_controller_ik.npz`：以Wuji目标逆解得到的当前Aubo六轴结果。
- `output/*_controller_ik.json`：逐帧控制器复核最大误差、实际DH、负载和数据来源。
- `output/*_collisions.json`：原始采样帧的碰撞报告、失败帧与具体部件。
- `output/*_runtime.npz/.json`：新起点的100Hz计划和完整预检结果。
- `output/*_execution.json`：只有实际调用执行分支才会产生的执行/停止记录。本次没有真实执行记录。
- `output/controller_snapshot_latest.json`、`controller_snapshot_execution_check.json`：本次负载及当前控制器只读读回。
- `output/hand_snapshot_latest.json`：手部位置及当前MIT配置只读读回。

52维数组 `joint_positions_left_right_rad` 的顺序为 **左臂6 + 左手20 + 右臂6 + 右手20**，必须结合 `joint_names_left_right` 解析。NPZ保留 `execution_enabled=false`，表示它本身不是运动命令；只有本入口的显式 `--execute` 且当前预检通过才下发。

## 坐标和实际运动学

```text
Aubo_base_T_flange(t)
  = Aubo_base_T_current_camera
  @ current_camera_T_source_camera
  @ source_camera_T_Wuji_base(t)
  @ inverse(Aubo_flange_T_Wuji_base)
```

- Franka来源使用 `replay/handeye/handeye/franka-sharpa/handeye_calibration.json` 与 `mapping_from_franka-sharpa.json`。
- 人手来源使用 `/home/descfly/ty/desk/observation.json`（Gemini335、1280×720），映射为 `config/mapping_from_human_gemini335.json`，不经过Franka FK。之前错误推定zhuji的输出已归档在 `output/archive_human_wrong_zhuji`，不再作为当前人手结果。
- 当前左手眼为 `aubo_left_02/result.json`，右为 `aubo_right_01/result.json`，使用用户给定 `/home/descfly/ty/cam/.../data`。
- 桌面为 `/home/descfly/ty/desk/calibration/results/current-table-20261112`。
- 右手安装 Rz(-90°)，意味着法兰目标 = 手根目标 @ Rz(+90°)，不是给全部臂关节加90°。
- 机械臂使用控制器读取的 modified-DH alpha/a/d/theta/beta，先由SDK逆解，再按实际DH做数值精化，逐帧SDK `forwardToolKinematics` 验证。
- 碰撞链使用实际DH放置每个臂链接；与名义URDF帧之间的固定关系已数值核对。
- 以前 `*_ik_candidate.npz` 是名义URDF结果，保留作对照，不是新入口的执行输入。

重新生成只读控制器逆解、原始碰撞报告：

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 solve_controller_ik.py episode594 episode615 episode616 human_camera
/usr/bin/python3 check_collisions.py episode594 episode615 episode616 human_camera
```

修改源数据、标定或安装后，应先运行 `./run.sh prepare 594 615 616` / `./run.sh prepare-camera` 并重新生成候选、控制器逆解与碰撞报告。不要通过编辑 `passed` 或 `ready_for_execution` 来消除错误。

## 软件验证

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -m unittest discover -s tests -v
```

25项测试通过。覆盖控制器实际DH数值一致、原轨迹坐标链、关节名称/时间戳、插值与过渡边界、桌面半空间符号、负载变化拦截、未检查轨迹禁止进入设备设置、单侧命令失败和反馈异常后的两臂两手停止、正常结束清理。模拟测试不会代表原生Servo的实机跟踪、停止距离或操作系统实时性能已经验收。

更早的离线准备过程见 `OFFLINE_PREPARATION.md`；其中历史候选状态与初始示意图不代表当前控制器执行准备结果。

## Servo 原生接口适配

当前运行层优先加载 `vendor/pyaubo_sdk_0_26` 中的官方 pyaubo-sdk 0.26.0rc6，SDK接口与两台控制器均为0.26.0；原 aubo/.venv 未修改。旧SDK/旧setServoMode路径的首次启动被控制器拒绝，轨迹点发送数为0、双手未使能，已保留在 `output/attempts/`。两臂静止与Servo退出随后已读回确认。新SDK和 `setServoModeSelect(1/0)` 的静止模式进出测试通过。

手部就绪检查先完成，再并发开启双臂Servo，并立即发送起点保持流；这样不会在等待手部使能时触发Servo缺帧自动退出。正式轨迹前仍有2秒同步保持。官方接口说明：[MotionControl](https://docs.aubo-robotics.cn/arcs_api/en/group__MotionControl.html)。

`--reuse-checked-plan` 只复用已通过全帧碰撞检查的同一份计划，重新检查数据/配置哈希、设备标定/负载/TCP/限位，要求起点变化小于臂0.002rad、手0.005rad；任何差异拒绝复用。它不是跳过碰撞的选项。

## 提示设备被占用时

`--hold-last` 回放结束后仍保持设备和独占锁，不会自动退出。若另一个终端再次启动，会提示当前登记PID及“末帧保持”。先支撑好物体，运行 `./stop_replay`，等提示停止确认与锁释放，再启动新回放。停止脚本现在会等待清理完成，不再仅发送请求立即返回。不要删除正在使用的锁文件，删除会破坏进程间互斥。

停止确认现在会独立读回Servo模式、实测关节速度及steady，连续三次确认；返回码非零保留为warning，无法确认时才阻止放行，并可重建控制连接复核。双手需收到新的20关节诊断帧且全部非Enabled。2026-11-12的右侧-3停止返回值已通过独立只读复核确认设备实际停止，原返回码记录保留。

人手配置由 `./replay human`、`./run.sh prepare-camera`、`solve_controller_ik.py human_camera` 和 `check_collisions.py human_camera` 自动选择。显式把编号轨迹配置传给人手准备会被拒绝，避免再次混用源相机。人手原始150帧和20关节数据未修改；新映射下的150帧控制器逆解已通过，但44帧有已建模桌面冲突，仍保持拦截。
