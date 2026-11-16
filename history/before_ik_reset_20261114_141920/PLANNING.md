# 碰撞约束回放与直接进场

`replay_arms` 使用 Aubo 原生 SDK，未加 `--execute` 时仅做只读校验。
独立进场规划已移除；当前只使用原本的直接关节五次平滑插值。

## 默认流程

1. 读取双臂实际 DH、限位、TCP、负载和当前位置，以及双手固定手型。
2. 在笛卡尔路点生成 IK 候选，检查同臂、固定手模型和桌面碰撞。
3. 用 continuous/differential IK 跟踪相邻路点之间的位姿曲线，细分检查连接边；NetworkX 在分层图上搜索回放路径。
4. 构造双臂乘积图，在节点和连接边中检查跨臂碰撞。
5. 从当前实测关节状态，**直接**过渡到选定的首帧关节状态：五次 minimum-jerk 插值，按速度/加速度限额确定时长，至少10秒。双臂使用同一过渡时间轴，手型固定。
6. 重定时并检查完整100Hz命令轨迹、插值末端误差和SDK正解，重新核对实测起点和文件哈希。全部通过后才可进入原有同步Servo与停止确认流程。

直接进场没有绕行路点，不尝试逐臂先行或RRT搜索；发生碰撞或动态超限仍会拒绝执行。
记录中的 `entry_planning.planner` 为 `direct_joint_interpolation`，`independently_planned=false`。
旧配置若指定其他进场算法会明确拒绝。

## 用法

```bash
# 默认：碰撞约束回放候选图 + 原本直接进场；只读检查
./replay_arms resources/trajectories/try/wuji2_camera_replay_trajectory_2_inflower

# 增加回放段IK候选数量
./replay_arms /path/to/trajectory --branches 24

# 对照原有逐分支跟踪方式；进场仍是直接插值
./replay_arms /path/to/trajectory --planner candidate_track

# 全部预检通过才执行
./replay_arms /path/to/trajectory --execute
```

`--backend continuous|differential` 控制回放连接器；`--planner collision_graph` 保留为默认。
`--entry-planner` 仅接受 `direct`，无需额外传入；`--entry-timeout` 已删除。

## 复用几何路径与减少候选路点

```bash
./replay_arms /path/to/source_trajectory \
  --planned-path output/aubo_arms_PREVIOUS/dual_graph_path0_offline.npz

./replay_arms /path/to/source_trajectory --waypoint-stride 4 --branches 16
```

`--planned-path` 输入是离线N×12弧度关节数组，包含 `time_s` 和 `joint_positions_rad`，顺序左6轴、右6轴。
它不复用通过标记：重新核对原始时间轴、全部目标、当前场景及节点/连接边，然后重新计算直接进场、重定时和完整100Hz检查。

`--waypoint-stride 4` 可将223个原始路点的显式候选图减少为57层；仍保留首尾，连接器仍检查所有原始时刻和不超过0.01秒源时间步长。
它只适用于新建 `collision_graph`，不能与 `--planned-path` 同时使用。
缓存只复用同一段、同一起始状态的确定性跟踪结果；实际终点和最后连接边仍单独检查。

整段采用一致的360°绕转表示，必须满足控制器限位，并保留全部帧间角度差；不会在执行中逐帧取模。

## 输出与范围

- `planning_snapshot.json`：只读设备起点。
- `left_graph.json/right_graph.json`、`dual_arm_graph.json`：回放段候选图与拒绝统计。
- `*_graph_path*_offline.npz`：离线候选，`execution_enabled=false`。
- `pair_checks.json`：直接进场、动态和100Hz校验结果。
- `runtime.json/.npz`：整段通过才生成。
- `summary.json`：是否具备执行条件及是否进入执行分支。

边检查按最大单关节变化0.01rad离散细分；当前模型余量为 `collision_scene.DEFAULT_CLEARANCE_M`（2mm）。
这些是离散检查，不是连续碰撞检测。保留原始末端位姿和时间顺序，不自动上提或平移。
有限候选搜索失败不构成全局无解证明。

## 依赖

图搜索继续使用 NetworkX 3.4.2，不再要求安装独立进场规划库。

```bash
./python tools/setup_planning.py --python /path/to/python-with-pip
./python doctor.py
./python -m unittest discover -s tests -v
```

历史下载的第三方源码和运行报告只作追溯，不参与当前进场计算。
