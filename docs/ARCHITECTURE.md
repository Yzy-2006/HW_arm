# 项目架构与配置归属

[文档索引](README.md) · [命令速查](COMMANDS.md)

## 控制与数据链路

```text
VR / 相机手势 ───────────────┐
                           v
PI05 / ACT worker <-> policy bridge -> robot_control_hw（上层）
数据集 -> replay_dataset ──────────>          |
                                            v
                              dual_arm_control_hw（中层，IK/FK）
                                            |
                                            v
                              servo_control_hw -> EtherCAT/CAN -> 双臂

三路 RealSense + 关节/TCP 反馈与目标 + 夹爪状态
                         |
                         v
                 collection/record.py -> 原始 episode
                         -> 校验 / LeRobot 导出 -> 筛选 / 训练
```

夹爪由 VR、手势或 bridge 向 `/gripper/ctl` 发布指令，底层通过 `/gripper/fdb`
返回反馈。VR/手势链路提供采集器使用的 `/towel/gripper_state` 二值状态。
录制由 `/towel/record` 和 `/towel/discard` 服务控制，`/towel/recording` 发布录制状态。
具体字段与顺序见[数据集约定](DATASET.md)。

策略 worker 不加载 ROS 控制环境，通过 TCP 接收观测并返回动作；bridge 负责相机、
反馈、动作时序和 ROS 发布。PI05 与 ACT 共用 bridge，切换 worker 即可。
数据集回放复用桥接消息与运动规划，不调用推理 worker。

同一时间只运行一条有效控制输入链。部署与 VR 的切换按[控制权交接流程](POLICY_INTERVENTION.md)执行。
`preview`、`record` 和 policy bridge 都直接打开 RealSense，不能同时占用同一组设备。
录制器是被动采集进程，启动它不会自动创建 episode。

## 模块与目录

| 路径 | 职责 |
|---|---|
| `Makefile`、`scripts/` | 统一命令、环境加载、构建、预检、标定与交付工具 |
| `ros2_ws/src/ctl_msgs_hw/` | 控制消息及真机共享配置 |
| `ros2_ws/src/servo_control_hw/` | EtherCAT/CAN、电机协议、伺服、安全状态与回零 |
| `ros2_ws/src/dual_arm_control_hw/` | 双臂中层、IK/FK 与轨迹转发 |
| `ros2_ws/src/robot_control_hw/` | 上层遥操作、策略桥接与控制权处理 |
| `ros2_ws/src/dual_arm_hw/` | 真机控制模型资源 |
| `ros2_ws/src/chassis_control_hw/` | 上层仍依赖的底盘消息接口；本项目不提供底盘驱动节点 |
| `ros2_ws/src/towel_bringup/` | ROS Launch、模型展示、状态可视化和 RViz 配置 |
| `teleop/` | WebXR/Vuer 输入、VR 增量映射、夹爪和手势真机适配 |
| `gesture_teleoperation/` | 独立手势识别模块、模型、标定与环境 |
| `collection/` | 原始采集、质量校验、LeRobot 导出与离线筛选 |
| `deployment/` | PI05/ACT worker、policy bridge、数据回放、轨迹规划与保护 |
| `simulation/`、`dual_arm/` | 当前 Newton/RViz 手势仿真及新版双臂模型资产 |
| `training/`、`environments/` | 训练说明和各运行环境定义 |
| `config/` | 采集、导出、部署与手势任务配置 |
| `tests/` | Python、C++ 与运行时测试；执行前提见验证说明 |
| `data/`、`outputs/`、`logs/` | 本地数据、模型/运行输出和日志；不是环境重建所需源码 |
| `newton_towel_sim/` | 可脱离主项目运行的独立交付快照 |
| `docs/`、`HW_arm_replay_incident_20261104/` | 使用/设计资料与事故证据 |

`ros2_ws/build/`、`install/`、`log/` 是本机构建产物。迁移时按[交接指南](MIGRATION_HANDOVER.md)重建。
独立 Newton 包与同名副本、压缩产物的关系见[资料索引](README.md#历史与交付资料)。

## 配置归属

以下路径均相对于项目根目录。改配置后需重启对应进程；ROS 包配置还需确认安装目录已同步，
具体以[迁移指南](MIGRATION_HANDOVER.md)的配置同步规则为准。

| 配置 | 负责内容 | 使用说明 |
|---|---|---|
| `config/recording.yaml` | 三路相机序列号、原始目录、任务文本、采集频率、时序/过滤阈值 | [相机配置](CAMERA_CONFIGURATION.md)、[数据集](DATASET.md) |
| `config/export_lerobot.yaml` | 导出字段、图像、元数据和 sidecar 选择 | [数据集](DATASET.md) |
| `config/pi05_deployment.yaml` | worker、bridge、插值、回放接近段与保护开关 | [部署](DEPLOYMENT.md)、[PI05 参考](PI05_DEPLOYMENT.md) |
| `config/gesture_robot.yaml`、`config/gesture_rviz.yaml` | 手势真机和 RViz 模式 | [手势真机](GESTURE_ROBOT.md)、[RViz](GESTURE_RVIZ.md) |
| `teleop/VR_controller/config/grippers.yaml` | 夹爪开闭端点、标定状态与运动时间 | [夹爪标定](GRIPPER_CALIBRATION.md) |
| `teleop/VR_controller/config/gripper_pd.yaml` | VR 位姿映射、快速/精细模式倍率及限速 | [VR 检查](VR_TELEOP_CHECK.md) |
| `ros2_ws/src/ctl_msgs_hw/config/controllerConfig.yaml` | 控制周期、网卡、电机方向/零点、保护、夹爪与 VR 姿态控制 | [控制安全](CONTROL_SAFETY.md)、[迁移](MIGRATION_HANDOVER.md) |
| `ros2_ws/src/ctl_msgs_hw/config/initialPD.yaml` | 控制刚度/PD 预设 | [部署](DEPLOYMENT.md) |
| `ros2_ws/src/ctl_msgs_hw/config/zeropos.yaml` | 启动及 VR 右 A 的关节回零目标 | [软件零位](ARM_ZERO_CALIBRATION.md) |
| `ros2_ws/src/ctl_msgs_hw/config/desc_zeropos.yaml` | 遥操作初始末端位姿 | [软件零位](ARM_ZERO_CALIBRATION.md) |
| `simulation/scene.yaml` | 仿真初姿、桌面、毛巾、求解器和虚拟相机 | [仿真指南](../simulation/README.md) |

真机控制模型为 `ros2_ws/src/dual_arm_hw/` 中的资源，Newton 和模型展示使用 `dual_arm/`。
仿真初姿不修改真机零位。电机内部零点与 YAML 软件参考位姿是不同操作，分别见
[单电机内部零位](MOTOR_ZERO.md)和[软件零位标定](ARM_ZERO_CALIBRATION.md)。

`teleop/VR_controller/config/zero_return.yaml` 保留旧的笛卡尔回零时长参数，当前 VR 发布器
没有读取它；右 A 的关节回零以控制器实现及 `zeropos.yaml` 为准。

## 环境和验证边界

ROS 控制、VR、采集与 bridge 使用系统 Python 3.12 的 `.venv-robot`；
PI05/OpenPI、ACT/LeRobot、Newton 和手势识别各用独立环境。
环境文件和安装步骤见[环境说明](../environments/README.md)。

`make launch` 默认关闭全部硬件相关进程，正式采集使用分终端流程。
`make visualize` 为独立模型展示；`make sim*` 为 Newton 仿真，均不启动真机伺服。
离线测试不能证明物理回零、急停效果或真实采集吞吐；见[验证说明](VALIDATION.md)。
