# 命令速查

[文档索引](README.md) · [项目架构](ARCHITECTURE.md)

以下命令在项目根目录执行，以根 [Makefile](../Makefile) 为准；独立交付包和手势识别
模块有自己的 Makefile。`make help` 查看主要命令。真机启动须先阅读对应流程，
此页不替代预检、标定或分终端启动步骤。

## 环境、构建与检查

| 命令 | 作用与边界 |
|---|---|
| `make env` | 打印机器人 Python、项目根目录和 OpenRobots 前缀 |
| `make build` | 构建 ROS 工作区，随后请求 sudo 设置 `servo_node` 网卡权限 |
| `make check` | 只读依赖检查；不启动电机或占用相机 |
| `make check-control` / `make check-record` / `make check-visualize` | 分别检查控制、采集、模型展示依赖 |
| `make test` | 使用机器人环境执行 `tests/`；部分测试依赖编译器、ROS 或可选仿真环境 |
| `make clean-build` | 仅打印说明，不自动删除构建产物 |

机器人端可覆盖 `VENV=/path/to/venv`、`OPENROBOTS_PREFIX=/path/to/sdk`。
安装说明见[环境与构建](../environments/README.md)，测试边界见[验证说明](VALIDATION.md)。

## 真机控制与采集

| 命令 | 作用与边界 |
|---|---|
| `make servo` | 底层伺服；正常启动会执行回零 |
| `make arms` | 双臂中层控制 |
| `make robot` | 上层控制 |
| `make vr` | VR 输入；控制节点运行时可驱动真机 |
| `make vr-debug` | 只打印 XR 输入诊断，不发布控制指令 |
| `make camera-list` | 列出连接的 RealSense 序列号 |
| `make preview` | 打开三路相机预览，不录制；会占用相机 |
| `make record` | 启动相机采集与三个 OpenCV 窗口；左 B 才开始 episode |
| `make record-rviz` | 采集并额外发布 ROS 图像供 RViz 使用 |
| `make watch` | 观察 `/towel/gripper_state`；可加 `WATCH_TOPIC=... WATCH_ARGS=--once` |
| `make watch-arm SIDE=both RATE=3` | 按指定频率打印单/双臂关节反馈；SIDE 为 left/right/both |
| `make gesture-check` | 手势输入和控制接口检查，不发送控制命令 |
| `make gesture` | 用手势替代 VR 输入，完整流程见[手势真机接入](GESTURE_ROBOT.md) |

正式采集按[五终端流程](DATA_COLLECTION_WORKFLOW.md)依次启动。
`preview`、`record`、策略 bridge 互斥占用相机；`gesture` 和 `vr` 不同时运行。

`make record ARGS="--dataset-root data/raw/interventions"` 可指定原始数据目录。
录制停止后，校验/导出的 `DATASET_ROOT` 必须匹配本次目录。

## 标定与反馈诊断

| 命令 | 作用与边界 |
|---|---|
| `make read-motors` | 查询配置中的电机，不回零；不取消旧命令、不提供负载支撑，不与其他 EtherCAT 主站并行 |
| `make scan-arm-can` | 被动扫描 CAN 通道反馈；详见[零位标定](ARM_ZERO_CALIBRATION.md) |
| `make calibrate SIDE=left ENDPOINT=open` | 保存一侧夹爪端点；SIDE 为 left/right，ENDPOINT 为 open/closed |
| `make calibrate-arm-zero SIDE=left` | 进入软件零位标定；所选侧零电流，须人工托稳，详见[操作风险与步骤](ARM_ZERO_CALIBRATION.md) |
| `make save-arm-zero SIDE=left TARGET=both` | 保存反馈对应的软件参考位姿；TARGET 为 joint/cartesian/both |
| `make zero-motor SIDE=left JOINT=2` | 写一个电机内部零点；会改变坐标参考，先读[单电机零位说明](MOTOR_ZERO.md) |

夹爪标定不会替你启动反馈节点，先按[夹爪标定流程](GRIPPER_CALIBRATION.md)启动所需节点。
`TARGET=preset` 是历史兼容保存入口，不是当前右 B 的配置入口。

## 数据处理

以下目标使用当前终端的 Python，不自动激活 `.venv-robot` 或 Conda；导出/筛选需安装 LeRobot。

```bash
make validate-data
make export-data OUTPUT=/absolute/path/to/lerobot_data
make filter-data DATASET=/absolute/path/to/lerobot_data MIN_JOINT_DELTA_RAD=0.02 ARGS="--dry-run"
make filter-data DATASET=/absolute/path/to/lerobot_data OUTPUT=/absolute/path/to/filtered_data MIN_JOINT_DELTA_RAD=0.02
```

校验/导出默认读 `data/raw/fold_towel`，覆盖用 `DATASET_ROOT=...`，导出配置覆盖用
`EXPORT_CONFIG=...`。筛选对象是已导出的 LeRobot 数据；字段及过滤规则见[数据集约定](DATASET.md)。

## 模型部署与数据回放

| 命令 | 作用与环境 |
|---|---|
| `make pi05` | 在 `PI05_ENV=pi05-openpi` 启动 OpenPI worker |
| `make act` | 在 `ACT_ENV=lerobot` 启动 ACT worker |
| `make pi05-bench` | 离线推理测速，不启动机器人；参数用 `BENCH_ARGS="..."` |
| `make bridge` | 机器人环境的 dry-run bridge；打开相机，不发机器人动作 |
| `make motion` | 允许 bridge 发布真实动作，先按[部署指南](DEPLOYMENT.md)预检 |
| `make vr-takeover` | 请求 VR 控制权并锚定当前反馈；先退出 motion，见[接管流程](POLICY_INTERVENTION.md) |
| `make replay ARGS="--episode 0 --max-frames 90"` | 数据回放，默认 dry-run，不需要 worker |
| `make replay ARGS="--episode 0 --max-frames 90 --rviz"` | RViz 回放预览 |

PI05 可覆盖 `PI05_MODEL`、`PI05_CHECKPOINT`、`PI05_HOST`、`PI05_PORT`、`PI05_ARGS`；
ACT 可覆盖 `ACT_CHECKPOINT`、`ACT_HOST`、`ACT_PORT`、`ACT_ARGS`。
两个 worker 默认共用 8765 端口，只启动一个。修改 worker 端口时也要同步 bridge 配置。
bridge 使用 `PI05_CONFIG=config/pi05_deployment.yaml`，额外参数用 `ARGS="..."`。

当前 `speed_only` 模式下 `PI05_SLOW_FACTOR` / `--slow-factor` 不降低实际轨迹速度，
因为反馈相对偏移裁剪已禁用。回放调整时间轴使用 `--speed`，先读[回放流程](DATASET_REPLAY_WORKFLOW.md)。
当前模型部署还禁用了速度检查，详见[当前保护配置](CONTROL_SAFETY.md#current-configuration)。

## 仿真与模型展示

| 命令 | 作用与环境 |
|---|---|
| `make visualize` | ROS/RViz 独立模型展示与关节滑块，不启动伺服；`ARGS="gui:=false"` 只看静态模型 |
| `make launch ARGS="visualize:=true"` | 可选 RViz、状态桥接与独立 TF；默认不启动硬件节点 |
| `make sim-check` | 检查 `SIM_ENV=towel-newton` 的 Newton/CUDA |
| `make sim-demo` | 官方 robot-cloth 示例 |
| `make sim` | 主项目双臂、毛巾和三路相机仿真 |
| `make sim-fold` | 自动抓角并完成一次对折 |
| `make sim-vr` | VR 控制 Newton；通过 `VENV` 指定 VR Python，不向头显传视频 |
| `make gesture-rviz` | 手势驱动隔离的 RViz 仿真 |

仿真参数通过 `ARGS="..."` 传递，场景和验收见[完整指南](../simulation/README.md)。
手势识别模块测试为 `make -C gesture_teleoperation test`，独立交付包的命令见其
[README](../newton_towel_sim/README.md)。

## 兼容与废弃入口

- `make record-opencv` 等价于 `make record`。
- `make vr-opencv` 仅转发到 `make vr`，相机窗口仍由 `make record` 启动。
- `make teleop-watch` 等价于 `make watch`。
- `make teleop`、`make experiment` 已移除，会直接报错；使用分终端启动流程。
- 旧部署 `--intervention`、`--intervention-dir` 和 `/towel/intervene` 已移除，改用显式 VR 接管与独立补采。
