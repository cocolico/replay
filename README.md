# Aubo i7H + Wuji 回放代码合集

本目录可以整体复制到另一台 Ubuntu 24.04 / x86_64 控制电脑，不依赖原来的
`/media/descfly/...`、`/home/descfly/...`、相邻 `replay` 仓库或 `/tmp/wuji_*` 文件。
这里集成代码、实际使用的 SDK 和资源，**不打包完整 Python/ROS 环境**；第三方依赖在目标机安装。

## 包含内容

- `vendor/pyaubo_sdk_0_26/`：实际验证过的官方 Python SDK 0.26.0rc6。
- `vendor/wuji_sdk_2026_11_9/`：Wuji SDK、Python 类型接口及 SDK 自带动态库。
- `vendor/replay_ik/`：新 continuous / differential IK 源码，附原仓库提交和说明。
- `resources/models/`：Aubo i7H URDF/SRDF 和引用的全部可视/碰撞网格，已解引用模型软链接。
- `resources/source_recordings/`：594/615/616 原始转换数据、Wuji/Franka 装配模型及原交付说明。
- `resources/calibration/`：源相机观测、源手眼、当前参考工位左右手眼、桌面观测和映射。
- `resources/trajectories/`：原人手轨迹、try 各版本及需要保留的修正/抬高输入。
- `config/`：全部资源路径均相对于配置文件；设备 IP、串号、MIT 核对值集中在这里。
- `tests/fixtures/`：离线测试快照，与 `output/` 中历史真机计划分开。
- `history/`、`output/`：原配置、报告、诊断和历史结果，仅作追溯。换机器、目录或配置后需重新准备，不能复用旧放行记录。

## 目标机安装依赖

自带的两个 SDK 二进制是 **Linux x86_64 / CPython 3.12**，目标电脑使用 Python 3.12。
Pinocchio、Coal、NumPy、SciPy 等在目标电脑下载，选择其中一种方式：

```bash
# 自己创建普通环境并联网安装；不要拷贝本机虚拟环境。
python3.12 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
# 需要视频/图片预览时再安装：
.venv/bin/python -m pip install -r requirements-preview.txt
```

已有 ROS Jazzy 的 Pinocchio/Coal 环境也可以使用，不必重复安装。
`./python` 优先选择 `REPLAY_PYTHON`，其次 `.venv/bin/python`，否则选择系统 Python 3.12，
并在存在 ROS Jazzy 时加载其环境。SDK 和新 IK 源码始终从本目录导入。
它不自动安装依赖；可用 `REPLAY_PYTHON=/path/to/python ./python doctor.py` 指定现有环境。

```bash
./python doctor.py             # 只检查依赖、配置、模型和源轨迹；不连接硬件
./python -m unittest discover -s tests -v
```

`requirements.txt` 是依赖清单，不是离线环境锁。若选用其他 Pinocchio/Coal 版本，安装后运行上述检查和测试。

## 换到另一套同型号真机

同型号不意味着左右手眼、桌面外参、安装变换和设备串号相同。原标定作为参考资料保留；
目标机需要自己的 `left_handeye.json`、`right_handeye.json` 和桌面 `observation.json`。
JSON 使用本目录参考文件的同一 schema：眼在手外 `T_base_camera`，桌面为 `camera_T_board`，
相同打印板定义/原点/轴约定，且左右手眼和桌面观测使用相同相机内参与分辨率。

1. 将目标标定放到 `target_input/`，复制 `config/target.example.json` 为自己的配置。
2. 填入左右控制器 IP、左右手地址和串号、手眼文件路径、实际 `flange_T_wuji_base`，确认实际安装数据。
3. 执行：

```bash
./python configure_target.py --spec config/target.example.json
./python doctor.py
```

此工具只处理文件：复制目标标定到目录内部、从源/目标桌面观测重新生成映射，并更新各 replay 配置与手的地址/串号。
它不连接设备、不修改控制器负载/TCP，也不修改手部控制参数。
原配置备份在 `history/reference_configs/`。新控制器的实际 DH、关节限位和起点在每次校验时重新读取。

## 轨迹校验与执行

目前这次适配的通用入口默认**只运动双臂**；双手只读反馈，按当前固定手型参与碰撞检查。

```bash
# 先只读预检；轨迹可换成 resources/trajectories 中的其他 camera trajectory 文件夹。
./replay_arms resources/trajectories/try/wuji2_camera_replay_trajectory_2_inflower

# 需要实机运动时，同一次调用重新完成全部检查，通过后才启动。
./replay_arms resources/trajectories/try/wuji2_camera_replay_trajectory_2_inflower --execute

# 另一个终端请求停止：
./stop_replay
```

默认0.2倍速、100Hz；臂速度上限为控制器的20%、加速度15%，必要时进一步统一减速。
当前模型间距要求为 **2mm**。起点过渡、固定手模型、双臂组合、全帧动态和插值末端误差都必须通过。
不直接对执行中的关节角取模，不自动上电或修改负载/TCP。
Ctrl+C 请求双臂同步软件停止并退出Servo，不能替代硬件急停。

`./replay` 保留原594/615/616与双手同步流程；其准备结果必须在目标工位重新生成，
不得将历史 `output/*_runtime.json` 当作新工位的有效计划。
此前几条人手轨迹仍有碰撞/逆解拦截：代码集成不改变这些轨迹的可行性结论。

## 移交

直接压缩整个 `code/` 文件夹即可；不需要相邻目录，不需要携带 `.venv/`、`__pycache__/`。
也可不携带 `output/` 和 `history/` 历史结果，运行时会生成新的输出。
请至少保留代码、`vendor/` SDK/IK、`resources/`、`config/`、`tests/` 和依赖清单。
