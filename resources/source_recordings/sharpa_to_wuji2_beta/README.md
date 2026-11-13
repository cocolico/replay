# Sharpa → Wuji Hand 2 早期 Beta：三个 episode 转换与回放

目标版本与本机 OpenPI 文档指向的遥操栈一致：
`/home/descfly/llx/gello_upper_body_teleop/adapters/wuji/models/hand2_beta/`。
左右 URDF 与本机官方模型仓库 v2026.6.27 中的文件逐字节一致。不是后来的 hand2_beta1、2.1 或 2.2。
源 Sharpa 模型和可视化资产沿用「Sharpa 数据 URDF」聊天验证过的模型。

## 已确认的默认朝向（2026-11-10）

本次默认转换在几何 IK 之后，按用户确认的回放方向，对每只 Wuji 手根施加一次局部 Z 轴旋转：
**左手 +90°（逆时针），右手 −90°（顺时针）**，从各自 Z 正端看向原点判断顺逆时针。
旋转中心是 `l_base_link` / `r_base_link` 原点，列向量约定下
`R_output = R_ik @ Rz(angle)`，`p_output = p_ik`，手指关节角不变。
这是手部实际姿态的修正，不是相机旋转或世界坐标重命名。

三个 episode 的局部 52 维动作、NPZ 根姿态和关键点、场景位姿、CSV 误差及回放均按此约定更新。
40 维手指关节数组保持不变。原始 Sharpa bag 和 `source_sharpa.npz` 保持不变。
输出已有 `alignment_version` 标记；回放直接读取修正后的位姿，**不要再手动旋转一次**。
原几何 IK 结果保存在 NPZ 的 `{side}_unaligned_*` 字段，便于追溯和避免重复施加。

`retarget.py episode594` 对今后的新转换默认应用这次修正。
对旧转换执行 `retarget.py episode594 --update-alignment` 可复用原 IK 结果更新；重复执行不会叠加旋转。
更新数据后运行 `motion_replay.py`（不要加 `--render-only`）重建场景数据和运动视频，
运行 `visualize.py` 更新手型视频。运行 `verify_alignment.py` 可检查三个 episode 的方向、
全部关键点、动作数组、误差一致性及重复更新行为。

此修正会改变指尖轨迹：`validation.json`、`fingertip_errors_mm.csv` 中的误差已重新计算，
参考仍是未旋转的源 Sharpa 关键点；原 IK 的毫米级拟合误差单独保存在 `pre_alignment_fit`。
视觉朝向确认不等于建立了真实法兰到 Wuji 的固定安装关系；目标法兰轨迹和机械臂逆解尚未生成。

## 双手整体运动视频

每个 episode 的 `both_hands_motion.mp4` 显示两只 Wuji 手在同一场景中的平移、旋转和手指运动。
原始 RGB 同步显示，蓝色为左手、橙色为右手，线条为最近两秒的掌根轨迹。
完整场景采用固定视角，已检查全部轨迹关键点位于画面范围内。

运动链为：录制的左右臂实测 7 关节 → FR3 URDF 正向运动学 → Sharpa 安装链 → 已转换的 Wuji 根变换。
源安装链来自随包保存的 `assets/motion/source_assembly.urdf`（本地双 FR3+Sharpa 完整装配模型），
左右 link8 到 Sharpa 掌根分别旋转 Z 轴 π、3π/2，轴向平移均为 0.0295 m。
每帧都核对独立矩阵链和 MuJoCo 的末端位置/旋转，并检查渲染出的 Wuji 21 点与复合变换一致。

场景坐标是该装配模型的 `lychee_root`。**模型中的两臂基座布局、手部安装变换尚未与这三段录制的
workcell 配置哈希建立对应，不能将视频当作精确标定的真实工作台重建。**
没有使用旧相机外参，也未将渲染手叠加到 RGB 中；地面网格仅作空间参考。
机械臂本体未绘制，也没有为调整后的 Wuji 掌根求目标机械臂逆解。

新增文件：`wuji_motion_scene.npz` 保存源掌根/目标根在该场景下的 4×4 位姿与关键点；
`motion_validation.json` 保存运动幅度、校验和来源；`motion_contact_sheet.jpg` 为六时刻预览。
原先 `wuji_actions_local_right_left.npy` 的局部坐标定义保持不变。

```bash
/home/descfly/miniforge3/envs/bench2dex/bin/python motion_replay.py episode594 --rgb /tmp/wuji594_rgb
```

## 原手型对照视频与结果

每个 episode 目录的 `comparison.mp4` 是完整 30 Hz 对照视频：
上方为原始头部 RGB；中间为左右 Sharpa 实测手型；下方为转换后的左右 Wuji 手型。
彩色圈和骨段表示源 Sharpa 目标，白点表示 Wuji 指尖，红线表示偏差。
手部以各自源掌根坐标显示，并已施加确认的局部 Z 修正。机械臂的空间运动仅在原始 RGB 中展示，
没有把两只独立局部坐标的手伪装成共同世界坐标。`contact_sheet.jpg` 是六个时刻的总览。

**以下表格为修正前几何 IK 的拟合结果，仅用于追溯，不是当前输出误差。**

| Episode | 帧数 | 首末帧时间跨度（秒） | 左指尖平均误差（mm） | 右指尖平均误差（mm） | 全部指尖最大误差（mm） |
|---|---:|---:|---:|---:|---:|
| 594 | 965 | 32.13 | 1.558 | 1.457 | 5.492 |
| 615 | 909 | 30.27 | 1.767 | 1.630 | 5.068 |
| 616 | 1263 | 42.07 | 1.536 | 1.461 | 4.868 |


当前输出与未旋转源 Sharpa 关键点的指尖平均距离：

| Episode | 左手（mm） | 右手（mm） |
|---|---:|---:|
| 594 | 99.241 | 100.000 |
| 615 | 84.921 | 86.511 |
| 616 | 90.027 | 95.338 |

全部 3,137 帧的数值有限，Wuji 关节位置未越限；每帧每个目标链接的独立 URDF 矩阵链计算与 MuJoCo 一致。
逐帧差分速度均未超过资产中记录的 URDF velocity 上限，这不代表已验证真机控制速度。
所有帧优化器均以收敛状态结束。精确数值、速度、根位姿步长、限位及源数据同步误差见各目录 `validation.json` 和 `extraction.json`。

## 输入处理

输入为 `/home/descfly/Xflow/data/3-shouna/episode594`、`episode615`、`episode616` 原始 ROS bag。
读取 `/sharpa/{left,right}/joint_states` **实测关节**，不读取 command 作为拟合目标。
按 collection_state.json 的 source_effective_start/end_time_ns 截取有效区间，使用 ROS 消息 header 时间戳。
关节线性插值到 30 Hz；RGB 取最近源时间戳，三段最大图像对齐差均小于 25 ms。
原始机械臂关节也按相同时钟提取并保存在 source_sharpa.npz，未对机械臂求逆解。
原始 bag 不修改；未采用旧相机外参或先前聊天中的 20° 相机修正。

## 转换算法和定义

Sharpa 22 关节 → 精确 URDF 正向运动学 21 点 → Wuji 20 关节及根位姿的联合逆运动学。
保持目标点的米制坐标，不做骨长归一化。权重：指尖 1、腕点 0.1、其他点 0.05。
第一帧以中性姿态的掌指关节点刚性对齐初始化；随后使用前帧结果，增加关节时序正则（系数 2.5e-5）。
第一帧正则系数为 1e-8；每帧最多 300 次函数求值。没有后处理裁剪、重定时或滤波。

21 点依次为腕点、拇/食/中/无名/小指各四点。
Sharpa：根 hand_C_MC；拇指 CMC_VL/MCP_VL/DP/fingertip，其他手指 MCP_VL/MP/DP/fingertip。
Wuji：腕点 `{l,r}_wrist`；各指 proximal_abd/middle/distal/tip。
两种手的中间点和腕点定义并不完全同源，所以指尖误差与全 21 点误差分开报告；
全点平均误差约 8 mm，腕点差约 24–27 mm，不能把它们说成所有点都达到了指尖精度。

修正前几何 IK 的连续性调整将 episode615 右手最大单帧关节变化从 20.31° 降到 4.46°，该侧平均指尖误差增加约 0.005 mm。
修正前几何 IK 的所有 episode 的最大单帧关节变化约 7.76°，见报告；源录制也包含快速运动。

## 数据文件及接入

每个 episode 包含：

- `wuji_hand_joints_left_right.npy`：`[N,40]`，**左手 20 + 右手 20**，弧度，顺序与 OpenPI 的 hand joint_names 一致。
- `wuji_actions_local_right_left.npy`：`[N,52]`，**右手 26 + 左手 26**，每手 `[xyz_m, rotvec_rad, 20关节_rad]`。
- `wuji_trajectory.npz`：时间戳、左右关节名、源/目标关键点、拟合误差、根位姿和优化状态。
- `source_sharpa.npz`：对齐后的 Sharpa 22 关节、原机械臂 7 关节、名称、时间戳、RGB 对应索引。
- `fingertip_errors_mm.csv`：逐帧逐指误差；`joint_order.json`：两个手的准确关节顺序。
- `validation.json`：运动学、限位、连续性、视频验证报告。

**52 维数据中的根位姿是 Wuji `l_base_link`/`r_base_link` 相对于对应 Sharpa `left_hand_C_MC`/`right_hand_C_MC` 的变换。
左右手分别属于两个局部坐标系，不是相机坐标，也不是共同世界坐标。**
旋转是轴角旋转向量（弧度），不是 Euler。若已知真实源掌根世界位姿，则：

`T_world_wuji_base = T_world_sharpa_root @ T_sharpa_root_wuji_base`

当前指尖误差是在应用含朝向修正的根变换后计算的，单独取 40 维手指角、忽略根调整，会改变指尖位置。
本次没有生成“原机械臂角度 + 新手指角度”的 54 维可执行动作，因为那样不能保证拟合后的末端位置。
若需要整机回放，需补充确切安装变换，组合完整末端轨迹并求机械臂 IK。

## 交互查看及复现

在本目录执行：

```bash
bash run_viewer.sh episode594
bash run_viewer.sh episode615
bash run_viewer.sh episode616
```

空格暂停/继续，左右键逐帧，+/- 调速，R 重启，鼠标旋转缩放。两只手在查看器里人为分开以便观察。
已验证交互查看器模型的变换一致性，本次未启动桌面交互窗口；已实际渲染完整视频。
本机复用 bench2dex 的 Python 和 mujoco-env 中的 MuJoCo，不修改已有环境。
迁移时安装 requirements.txt；依赖实测版本见 runtime_versions.json。

```bash
# 从原始 bag 重新提取（需本机 ROS2 Jazzy）
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 extract_rosbag.py /home/descfly/Xflow/data/3-shouna/episode594 episode594 /tmp/wuji594_rgb
# 转换和渲染
/home/descfly/miniforge3/envs/bench2dex/bin/python retarget.py episode594
/home/descfly/miniforge3/envs/bench2dex/bin/python visualize.py episode594 --rgb /tmp/wuji594_rgb
```

模型、STL 资产、代码和动作数据均随包保存；原始大体积 ROS bag 和中间 RGB JPEG 不放入包中。
模型来源与哈希见 asset_manifest.json；第三方资产遵循原许可证。
Sharpa 缺失的部分末节外观使用此前回放中的胶囊/球显示，不影响已核对的 URDF 关节链。

## 验证范围

本次是离线运动学转换与可视化：没有验证物体接触、抓取成功、手指自碰撞、动力学或机械臂可达性；没有连接硬件。
相同的指尖位置也不保证两种手的指腹接触面一致。请结合视频检查实际抓握手型。
