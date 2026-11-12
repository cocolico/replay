# Aubo i7H + Wuji 双臂回放适配

本目录由离线转换、合成 IK 检查和独立的控制器只读检查组成。没有实现或触发真机运动；所有生成产物 `execution_enabled=false`。设备已连接不等于可以把离线轨迹直接执行。

## 本次实测与输入

- 左右控制器均报告 `aubo_i7H`，不是普通 i7。左臂 `192.168.1.3`，右臂 `192.168.1.2`，均为 rob1。
- SDK 接口版本 0.24.0，服务端接口版本 0.26.0；本次只读 getter/FK 调用成功，不代表运动接口兼容性验收。
- 读取时左右 TCP offset 均为六个零。因此 SDK TCP 目前对应法兰配置，而非已测定的 Wuji 掌根。
- 读取时左臂激活负载为 0 kg，右臂为 0.632 kg，质心 `[-0.0073, 0.03246, 0.15035] m`。没有修改负载；示教器中保存的负载测量任务可能未全部激活到当前控制器配置。
- 每臂用当前关节及两组数值偏移做控制器 FK 与 URDF 对比（只是计算，没有运动）：左最大位置差约 0.576 mm，右约 1.421 mm；角度差约 0.158° / 0.265°。配置仅对小于 1 mm / 0.01 rad 的模型比较标记 `model_confirmed`，因此右侧暂未通过。这一阈值是本离线实现的模型检查阈值，不是用户任务精度要求或整机精度承诺。
- 可使用实际控制器算法做后续精确 IK，或者根据控制器标定参数完善离线模型；不能为了生成可执行轨迹直接放宽模型检查。
- 完整只读结果与比较见 `output/controller_snapshot.json`、`output/model_comparison.json`。

输入并非“Franka 关节 + 任意另一段 Wuji 关节”直接拼接：

| 数据 | 语义 |
| --- | --- |
| `../sharpa_to_wuji2_beta_594_615_616/sharpa_to_wuji2_beta/episode594` | 965 帧 / 32.133333 s |
| 同目录 episode615 | 909 帧 / 30.266666 s |
| 同目录 episode616 | 1263 帧 / 42.066666 s |
| 每段 `source_sharpa.npz` | 同一时钟下的原 Franka 七轴实测角与 Sharpa 角 |
| 每段 `wuji_trajectory.npz` | 已转好的 Wuji 二十关节及相对 Sharpa 掌根的 Wuji 掌根位姿 |
| `../wuji-teleop` | ROS 驱动、控制与重定向源码，不是轨迹数据目录 |
| `../wuji2_camera_replay_trajectory` | 另一段 150 帧、约 10 Hz 的人手相机系轨迹，标记 human_camera_direct_fit_v1；单独使用 zhuji → 当前工位桌面映射，不与三个 episode 混合。zhuji 身份根据用户所指另一相机观测推定，来源说明保存在配置中 |

三段原轨迹已含左手局部 Z +90°、右手 -90° 修正。本代码原样保留，不再旋转，也不重新拟合手指。

## 坐标链

所有变换使用米、列向量，`A_T_B` 把 B 系坐标变换到 A 系。每侧最终法兰目标为：

```text
Aubo_base_T_flange(t)
  = Aubo_base_T_current_camera
  @ current_camera_T_source_camera
  @ source_camera_T_Franka_base
  @ Franka_base_T_Sharpa_root(q_FR3(t))
  @ Sharpa_root_T_Wuji_base(t)
  @ inverse(Aubo_flange_T_Wuji_base)
```

- 左手眼固定使用 `/home/descfly/ty/cam/franka_orbbec_handeye_calibration/data/aubo_left_02/result.json`（当前最新），右手眼使用同目录 `aubo_right_01/result.json`。
- 桌面映射固定使用 `/home/descfly/ty/desk/calibration/results/current-table-20261112/mapping_from_franka-sharpa.json`。另一份 `mapping_from_zhuji.json` 是另一来源，不可叠乘。
- 本次检查了桌面映射的逆矩阵、板身份，以及当前桌面观测/左右手眼的相机 K 和分辨率一致。
- 原包 `wuji_motion_scene.npz` 是名义 `lychee_root` 场景系，不是相机系。本代码从原始关节按名称做 FK，直接得到每侧独立 Franka link0 基座下的手根位姿，不使用名义左右基座布局冒充标定。
- `assets/motion/source_assembly.urdf` 中 FR3 link8 到 Sharpa 掌根的安装仍需与源录制确认一致。
- `Aubo_flange_T_Wuji_base` 目标是早期 Beta 模型的 `l_base_link/r_base_link`，不是 `l_wrist/r_wrist`、手指尖或任意“掌心”点。用户确认无转接件、法兰与手根端面中心直连，因此当前按零平移处理；URDF 根网格安装面的 z 约为 0，与这一描述相符。用户已通过可视化确认：左手绕法兰 Z 0°，右手绕法兰 Z -90°；两侧 `mount_confirmed=true`。固定变换为左侧单位阵、右侧 Rz(-90°)。因此右侧法兰目标 = 右侧手根目标 @ Rz(+90°)，不是给所有机械臂关节简单加减 90°。示教器负载/质心不能代替该朝向。
- 手眼结果的 `T_tcp_target` 是标定板相对于采样 TCP 的变换，不能当作 Wuji 安装变换。

## 已生成的产物

`output/episode{594,615,616}_prepared.npz` 保存：

- 相同 episode 的原始 `timestamp_ns` 和归零后的 `time_s`，保持原时间间隔。
- `{side}_wuji_joint_positions_rad`：N×20，弧度。
- `{side}_wuji_model_joint_names`：早期 Beta 模型关节名。
- `{side}_wuji_wire_joint_names`：`{side}_finger1_joint1` 到 `{side}_finger5_joint4`，与现有 Wuji control 契约对应。
- `{side}_source_franka_base_T_wuji_base`：N×4×4，仍在对应源 Franka 基座系，不能发给 Aubo。
- 仅在配置了源相机外参时才会出现 `source_camera_T_wuji_base`、`aubo_base_T_wuji_base`。
- 仅在进一步提供固定安装矩阵时才会出现 `aubo_base_T_flange`。

对应 `episode*_report.json` 记录输入哈希、手指限位检查、速度/加速度统计、坐标与安装缺项。三个 episode 共 3137 帧的手指位置均在所附模型限位内。速度/加速度为统计值，不代表已通过真机动态限制。

已生成四组 Aubo 六轴 + Wuji 二十关节的离线候选：`episode594_ik_candidate.npz`、`episode615_ik_candidate.npz`、`episode616_ik_candidate.npz`、`human_camera_ik_candidate.npz`。

三个 episode 共 3137 帧、人手 150 帧的左右臂名义 IK 全部收敛，全部机械臂速度差分低于 URDF velocity。已按确认的左 0°/右 -90° 安装重新生成。手指数据、时间轴及左臂结果与原 0°/0° 候选逐字节相同；右臂重新求解并通过全帧 FK 校验。结果尚未通过碰撞、实际控制器模型、同步执行或起始过渡验收。

候选文件另包含 `joint_positions_left_right_rad`（N×52）和 `joint_names_left_right`，顺序为 **左臂6 + 左手20 + 右臂6 + 右手20**。这与源 52 维掌根动作的含义不同；必须按名称和 artifact_type 解析。候选内明确标记 `execution_enabled=false`、`artifact_type=nominal_model_ik_candidate`。不得作为已经放行的硬件命令文件。

报告 `*_prepared_ik_diagnostic.json` 保存逐帧误差、速度/步长统计、候选哈希和未确认项。

## 命令

从任意目录可运行，所有离线输出默认写入本目录 `output`：

```bash
/media/descfly/新加卷/ty/replay/code/run.sh audit
/media/descfly/新加卷/ty/replay/code/run.sh prepare 594 615 616
/media/descfly/新加卷/ty/replay/code/run.sh prepare-camera
```

配置在 `config/replay.json`，相对路径相对于配置文件目录解析。用户已指定 `../handeye/handeye/franka-sharpa/handeye_calibration.json` 为源录制所需标定，现已接入。该文件与之前的旧副本 SHA256 相同；采用依据是用户本次明确给出的数据来源，而不是文件日期更新。源 FR3→Sharpa 固定链采用轨迹包自己的 source_assembly.urdf，作为源数据的安装定义，不依赖名义双臂基座布局。

只读获取当前控制器型号、TCP、负载、关节限位、动态上限和三组计算 FK：

```bash
/usr/bin/python3 /media/descfly/新加卷/ty/replay/code/read_controllers.py
```

它只连接 RPC、登录、调用 getters/数学正解、退出。不调用 poweron/startup、Runtime、setPayload/setTcpOffset、MoveJ/Servo；不会使设备运动。密码仅从 `AUBO_PASSWORD` 环境变量读取，未设置时沿用现有 SDK 登录默认值；不写入产物。

只有缺项补全并重新 `prepare` 后，才允许离线名义模型 IK：

```bash
/media/descfly/新加卷/ty/replay/code/run.sh ik \
  /media/descfly/新加卷/ty/replay/code/output/episode594_prepared.npz \
  /media/descfly/新加卷/ty/replay/code/output/episode594_report.json \
  /media/descfly/新加卷/ty/replay/code/output/episode594_ik.npz
```

当前配置下该严格命令仍会拒绝：右侧名义模型与控制器的比较未通过；安装方向已确认。IK 会检查每帧误差、关节位置、相邻跳变和 URDF 速度限制，且验证准备阶段的配置/数据哈希。它仍不包含真实控制器加速度验收、碰撞检测或执行放行。

## 后续运行层

完整 replay 还需以下工作：

1. 源外参和桌面映射已接入；左右安装旋转已确认；确认早期 Beta 模型与实手一致，并核对示教器测量负载是否激活（本代码不会自动改写）。
2. 验证控制器模型，逐帧生成双侧 Aubo 六轴 IK，再校验 FK、限位、奇异性及连续性。不能把 Franka 七轴角截成六轴角。
3. 在共同的当前相机/任务坐标中加入双臂、手指、转接件、真实桌面厚度及环境碰撞。桌面板的 +Z 进入板面，不能直接套“Z 越大离桌越高”的假设。
4. 按控制器动态上限和手部限制确定统一时间轴。若需要减速，机械臂和双手必须一起重定时，不能只把一侧逐点 MoveJ。现有 WujiClient.move_to 是单目标插值接口，也不是同步轨迹播放器。
5. 执行层使用统一单调时钟、关节反馈时效检查、双臂/双手同步停止，并单独规划当前实测姿态到第一帧的路径。避免两个 SDK 实例争用同一只手；不启动 MANUS 等其他命令源。
6. 审阅可视化及轨迹检查结果后，再进行独立授权的低速硬件验证。本文档没有提供可直接运动的 execute 入口。

## 验证

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -m unittest discover -s /media/descfly/新加卷/ty/replay/code/tests -v
```

9 项测试通过，覆盖坐标/安装矩阵方向、非法变换/错时钟/重复关节名拒绝、三段全部帧的 FK 与交付场景一致、源关节排列不变性、缺标定时拒绝 IK、篡改产物拒绝，以及合成 Aubo FK→IK 往返。没有把合成测试等同于真实轨迹验收。

## 重建离线候选（仍不执行）

以下示例执行按配置安装方向的名义模型诊断；不走严格 IK 放行入口：

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 /media/descfly/新加卷/ty/replay/code/diagnose_ik.py \
  /media/descfly/新加卷/ty/replay/code/output/episode594_prepared.npz \
  /media/descfly/新加卷/ty/replay/code/output/episode594_report.json \
  --stride 1 \
  --candidate-out /media/descfly/新加卷/ty/replay/code/output/episode594_ik_candidate.npz
```

其他 episode 替换编号，人手数据使用 `human_camera_prepared.npz` 与 `human_camera_report.json`。`--stride 1` 才能导出候选，且要求左右全部帧均达到 IK 残差阈值。以更大 stride 运行只生成抽样诊断。

人手路线直接使用 `Aubo_base_T_current_camera @ mapping_from_zhuji @ source_camera_T_wuji_base @ inverse(flange_T_wuji_base)`，不经过 Franka FK，不额外应用 Franka episode 的 +/-90° 修正，也不需要 zhuji 的机械臂手眼外参。图像去畸变按保持光学坐标轴处理，没有擅自添加相机旋转。

独立全帧复核可运行 `source /opt/ros/jazzy/setup.bash` 后执行 `/usr/bin/python3 /media/descfly/新加卷/ty/replay/code/validate_candidates.py`。它逐帧用 URDF 正解校验候选目标，核对手指数据、时间戳及 52 维顺序未变；最新汇总为 `output/conversion_summary.json`。这仍只验证已配置安装下的名义运动学。

本次安装更新验证见 `output/mount_update_verification.json`。`visuals/` 中较早的静态装配图描绘初始左 0°/右 0°，保留作历史对照；当前安装值以 `config/replay.json` 和候选内 `*_flange_T_wuji_base` 为准。
