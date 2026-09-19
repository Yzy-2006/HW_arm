# 擦毛巾：ENCOS 双臂 VR 遥操作与数据采集

本目录用于双臂叠毛巾的真机 VR 遥操作与数据采集：使用头部、左腕、右腕三路 RealSense，相机与机器人数据可导出为 LeRobot 数据集，供后续 π0.5 微调。

> **安全边界：** 软件退出、`Ctrl+C` 或关闭 Launch 都不是物理急停。进入任何会让机械臂运动的流程前，先确认现场急停、上电和 CAN/EtherCAT 操作符合机器人厂商规范。

## 从这里开始

首次配置机器人端环境、证书和 ROS 工作区，见 [环境与构建](environments/README.md)。完成后，在项目根目录执行：

```bash
make build       # 首次、更新源码或 ROS 工作区变化后执行
make check       # 只读检查控制、相机和 RViz 依赖
make test        # 不启动硬件的测试
```

所有机器人端 `make` 命令都会自动激活 `.venv-robot`、加载 ROS/OpenRobots 和工作区环境；不需要在每个终端重复 `source`。训练与导出仍应在独立的训练环境运行。
启动 `servo` 的命令另需 EtherCAT 原始网卡权限；`make build` 会在构建成功后自动提示
sudo 密码并配置 `servo_node`，具体见[环境与构建](environments/README.md#ethercat-网卡权限)。

## 文档导航

| 要做的事 | 使用的文档 | 常用命令 |
|---|---|---|
| 按正确顺序完成一次真机实验 | [真机实验流程](docs/REAL_ROBOT_EXPERIMENT.md) | `make experiment VISUALIZE=true` |
| 填写并核对三台 RealSense 的序列号和画面 | [相机序列号配置](docs/CAMERA_CONFIGURATION.md) | `make preview` |
| 记录单臂或双臂新的安全初始姿态 | [机械臂零位标定](docs/ARM_ZERO_CALIBRATION.md) | `make calibrate-arm-zero SIDE=both` |
| 采样左右夹爪开/闭端点 | [双夹爪标定](docs/GRIPPER_CALIBRATION.md) | `make servo`、`make calibrate ...` |
| 配置、检查和去皮两路六维力传感器 | [力传感器检查与去皮](docs/FORCE_SENSOR.md) | `make tare-force SIDE=left` |
| 连接 XR、检查按键与空载遥操 | [VR 遥操作检查](docs/VR_TELEOP_CHECK.md) | `make teleop` |
| 理解原始片段、字段、校验与 LeRobot 导出 | [数据集约定](docs/DATASET.md) | `make validate-data` |
| 训练 π0.5 | [π0.5 训练说明](training/README_PI05.md) | 在训练环境运行 |
| 排查环境或历史验证范围 | [依赖说明](docs/DEPENDENCIES.md)、[验证记录](docs/VALIDATION.md) | `make check` |

`REAL_ROBOT_EXPERIMENT.md` 是当天操作的总清单；表中每个专项步骤都跳转到唯一的模块文档。不要同时启动 `make teleop`、`make experiment` 或分节点命令，它们会发布同名控制话题。

## 常用启动命令

```bash
make preview                         # 仅采集器 + RViz，检查三路相机，不启动机器人控制
make teleop                          # servo → force → arms → robot → VR，不启动采集器
make teleop VISUALIZE=true           # 上述空载遥操 + RViz
make experiment VISUALIZE=true       # 完整实验：控制、三相机采集、VR 和 RViz
make watch                           # 观察 /towel/gripper_state；可通过 WATCH_TOPIC=... 改话题
make tare-force SIDE=left            # 力节点运行时，对左传感器去皮
make calibrate-arm-zero SIDE=both    # 双臂零电流的零位标定模式
make save-arm-zero SIDE=both         # 将稳定的双臂反馈写入源码零位配置
```

`make preview`、`make teleop` 和 `make experiment` 都是前台进程；按 `Ctrl+C` 只结束本次软件进程。完整实验开始前，必须先完成相机配置、夹爪标定和空载 VR 检查。

## 配置归属

| 文件 | 负责内容 | 对应说明 |
|---|---|---|
| `config/recording.yaml` | 任务文本、原始数据目录、三相机序列号、采集频率与时序阈值 | [相机序列号配置](docs/CAMERA_CONFIGURATION.md)、[数据集约定](docs/DATASET.md) |
| `teleop/VR_controller/config/grippers.yaml` | 左右夹爪端点、PD、力矩限制和标定状态 | [双夹爪标定](docs/GRIPPER_CALIBRATION.md) |
| `teleop/VR_controller/config/gripper_pd.yaml` | VR 位姿映射、缩放和右臂力前馈 | [VR 遥操作检查](docs/VR_TELEOP_CHECK.md) |
| `ros2_ws/src/ctl_msgs_hw/config/zeropos.yaml` | 双臂自动回零目标；专用标定命令可写入左臂或双臂初始姿态 | [机械臂零位标定](docs/ARM_ZERO_CALIBRATION.md) |
| `ros2_ws/src/encos_force_sensor/config/force_sensor.yaml` | 力传感器 CAN、坐标符号、话题名称和去皮采样数 | [力传感器检查与去皮](docs/FORCE_SENSOR.md) |

当前 `recording.yaml` 的 `right_wrist` 是占位值时，采集器会拒绝启动；请先完成相机模块。配置发生变化后，应使用新的原始数据目录，避免混合不兼容 episode。

## 数据与训练

采集器记录双臂关节反馈/目标、TCP 反馈/名义目标、夹爪二值状态、可选力话题和三路 RGB 图像。字段定义、时间对齐限制、数据质量规则和 LeRobot 映射都在 [数据集约定](docs/DATASET.md)。

当前默认原始目录与 `Makefile` 的 `DATASET_ROOT` 一致，为 `data/raw/fold_towel`：

```bash
make validate-data
make export-data OUTPUT=/absolute/path/to/fold_towel_lerobot
```

若修改了 `config/recording.yaml` 的 `dataset_root`，校验和导出时也显式传入相同目录：

```bash
make validate-data DATASET_ROOT=/absolute/path/to/raw_data
make export-data DATASET_ROOT=/absolute/path/to/raw_data OUTPUT=/absolute/path/to/lerobot_data
```

训练侧的动作/观测映射与 π0.5 示例见 [training/README_PI05.md](training/README_PI05.md)。
