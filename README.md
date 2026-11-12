# ENCOS 双臂叠毛巾

双臂叠毛巾实验项目，包含真机 VR/手势遥操作、三路 RealSense 数据采集、LeRobot 数据导出、PI05/ACT 策略部署，以及 Newton 布料仿真。

> 软件退出、`Ctrl+C` 或关闭 Launch 都不是物理急停。真机启动和标定前，必须确认现场急停、工作空间、供电及 CAN/EtherCAT 接线符合厂商规范。当前保护开关与限制见[控制安全说明](docs/CONTROL_SAFETY.md)。

## 从这里开始

完整资料见[文档总索引](docs/README.md)，命令见[命令速查](docs/COMMANDS.md)，代码关系见[项目架构](docs/ARCHITECTURE.md)。

| 使用场景 | 入口 |
|---|---|
| 首次安装、构建 ROS 工作区 | [环境与构建](environments/README.md) |
| 更换上位机或交接设备 | [迁移与交接](docs/MIGRATION_HANDOVER.md) |
| 真机遥操作与示教采集 | [真机数据采集流程](docs/DATA_COLLECTION_WORKFLOW.md) |
| 当天实验预检与验收 | [真机实验总清单](docs/REAL_ROBOT_EXPERIMENT.md) |
| 运行 PI05/ACT 模型 | [机器人部署指南](docs/DEPLOYMENT.md) |
| 数据集动作回放 | [数据集回放流程](docs/DATASET_REPLAY_WORKFLOW.md) |
| 纯仿真、自动对折或仿真 VR | [Newton 仿真指南](simulation/README.md) |
| 使用相机手势 | [RViz 仿真](docs/GESTURE_RVIZ.md)、[真机接入](docs/GESTURE_ROBOT.md) |
| 导出数据、筛选或训练 | [数据集约定](docs/DATASET.md)、[PI05 训练说明](training/README_PI05.md) |
| 独立交付仿真 | [Newton 交付包](newton_towel_sim/README.md) |

## 环境与检查

先按[环境说明](environments/README.md)创建 `.venv-robot`、配置系统依赖和 VR 证书，再在项目根目录运行：

```bash
make build       # 编译 ROS；成功后通过 sudo 配置 EtherCAT 网卡权限
make check       # 只读检查控制、相机和可视化依赖
make test        # 项目 tests/，不启动真机硬件
```

机器人端 Makefile 目标会自动加载 `.venv-robot`、ROS/OpenRobots 和工作区环境。
Newton 使用 `towel-newton`；数据导出/训练使用独立 LeRobot 环境；推理 worker 使用
`pi05-openpi` 或 `lerobot`。不要在 Conda 训练环境构建或运行 ROS 控制节点。
具体覆盖范围与测试前提见[验证说明](docs/VALIDATION.md)。

## 真机采集入口

完成相机配置、夹爪标定和空载 VR 检查后，按[正式采集流程](docs/DATA_COLLECTION_WORKFLOW.md)分五个终端依次启动：

| 终端 | 命令 | 职责 |
|---|---|---|
| 1 | `make servo` | 底层伺服与启动回零，会运动 |
| 2 | `make arms` | 双臂运动学与轨迹控制 |
| 3 | `make robot` | 上层控制与遥操作参考姿态 |
| 4 | `make record` | 三路相机、OpenCV 窗口和 episode 写入 |
| 5 | `make vr` | VR 输入；连接头显、进入 Virtual Reality 后按终端提示按 Enter |

每一步等待前一层初始化完成。左 B 开始/结束录制，左 A 丢弃；扳机控制对应臂，
squeeze 切换对应夹爪，右 A 回零，右 B 切换快速/精细模式。
完整按键、停止顺序和通过条件统一维护在[采集流程](docs/DATA_COLLECTION_WORKFLOW.md)。

`make preview`、`make record`、策略 bridge 都会占用相机，不能同时运行。
控制与 VR 节点也不要重复启动。`make vr-debug` 只诊断输入，不发送控制指令。
从部署切换到 VR 时，按[接管与补采](docs/POLICY_INTERVENTION.md)执行显式控制权交接。

## 数据出口

默认原始目录为 `data/raw/fold_towel`。采集结束后校验，再到训练环境导出：

```bash
make validate-data
make export-data OUTPUT=/absolute/path/to/fold_towel_lerobot
```

若修改 `config/recording.yaml` 的 `dataset_root`，两个命令都需传入相同的
`DATASET_ROOT=/absolute/path/to/raw_data`。字段、时间对齐、过滤及质量规则见[数据集约定](docs/DATASET.md)。

## 目录与资料维护

`ros2_ws/` 为真机控制，`teleop/` 为输入，`collection/` 为采集与导出，
`deployment/` 为策略与回放，`simulation/` 为当前仿真实现，`config/` 为任务配置。
完整目录和配置归属见[项目架构](docs/ARCHITECTURE.md)。

`newton_towel_sim/` 是独立交付快照；同名 `(2)` 目录及压缩包另行保留，
主项目开发使用 `simulation/`。事故资料和早期设计记录的入口、日期与适用范围
见[历史资料索引](docs/README.md#历史与交付资料)。
