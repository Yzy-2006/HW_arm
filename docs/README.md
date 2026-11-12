# 项目文档总索引

本索引覆盖主项目操作文档、模块说明、独立交付资料与历史记录。返回[项目首页](../README.md)。
整理日期：2026-11-12；具体运行参数以源码和 YAML 为准，历史测试结果不代表当前硬件验收。

## 推荐阅读顺序

- 首次接手真机：[环境与构建](../environments/README.md) → [项目架构](ARCHITECTURE.md) → [当天实验总清单](REAL_ROBOT_EXPERIMENT.md) → [正式采集流程](DATA_COLLECTION_WORKFLOW.md)。
- 部署模型：[部署指南](DEPLOYMENT.md) → [当前控制保护模式](CONTROL_SAFETY.md#current-configuration) → [PI05 细节](PI05_DEPLOYMENT.md) → [VR 接管与补采](POLICY_INTERVENTION.md)。
- 处理数据：[数据集约定](DATASET.md) → [训练说明](../training/README_PI05.md)；回放另见[回放流程](DATASET_REPLAY_WORKFLOW.md)。
- 只做仿真：[Newton 环境](../environments/NEWTON_SETUP.md) → [完整仿真指南](../simulation/README.md)；独立接收方从[交付包 README](../newton_towel_sim/README.md)开始。

## 环境与开发

| 文档 | 内容与适用范围 |
|---|---|
| [项目架构](ARCHITECTURE.md) | 模块边界、控制/数据链路、目录和配置归属 |
| [命令速查](COMMANDS.md) | 根 Makefile 命令、环境、参数和启动影响 |
| [环境与构建](../environments/README.md) | 系统 Python、ROS、Conda 分工、构建及 EtherCAT 权限 |
| [依赖与来源](DEPENDENCIES.md) | 提取来源、运行依赖、架构兼容性 |
| [迁移与交接](MIGRATION_HANDOVER.md) | 换上位机、文件交付、配置同步；参数表为交接快照 |
| [验证说明与历史记录](VALIDATION.md) | 当前检查入口、测试边界、2026-09-15 的历史结果 |

## 真机操作与标定

| 文档 | 内容与适用范围 |
|---|---|
| [当天实验总清单](REAL_ROBOT_EXPERIMENT.md) | 现场安全、预检顺序、试录与收尾 |
| [正式数据采集流程](DATA_COLLECTION_WORKFLOW.md) | 正式录制的唯一启动流程、VR 按键和停止顺序 |
| [相机配置](CAMERA_CONFIGURATION.md) | 三台 RealSense 序列号、预览和故障检查 |
| [VR 空载检查](VR_TELEOP_CHECK.md) | XR 连接、诊断、逐层控制和按键验证 |
| [夹爪端点标定](GRIPPER_CALIBRATION.md) | 左右开闭端点、反馈与编码 |
| [机械臂软件零位标定](ARM_ZERO_CALIBRATION.md) | 关节/末端参考位姿保存；零电流模式须先阅读风险说明 |
| [单电机内部零位](MOTOR_ZERO.md) | 写单个电机内部零点，与软件零位标定不同 |
| [控制安全](CONTROL_SAFETY.md) | 故障锁存、启动回零、接管、保护开关及离线验证边界 |
| [手势真机接入](GESTURE_ROBOT.md) | 张掌控制、握拳夹爪、OK 指令及手势输入配置 |

## 数据、训练与部署

| 文档 | 内容与适用范围 |
|---|---|
| [数据集约定](DATASET.md) | 原始格式、时间对齐、质量检查、LeRobot 导出和离线筛选 |
| [PI05 训练](../training/README_PI05.md) | 微调环境与示例；示例仍需真实数据和 GPU 验证 |
| [PI05/ACT 部署](DEPLOYMENT.md) | 共用 bridge、worker 启动、相机占用与停止 |
| [PI05 部署参考](PI05_DEPLOYMENT.md) | Checkpoint 格式、模型变体、动作与推理时序 |
| [部署转 VR 与补采](POLICY_INTERVENTION.md) | 停止 bridge、显式交接控制权、独立补采 |
| [数据集回放](DATASET_REPLAY_WORKFLOW.md) | dry-run、RViz、真机短片段、接近段和日志 |
| [Minimum Snap 插值](MINIMUM_SNAP.md) | 30 Hz 源动作到 50 Hz 指令、轨迹边界和保护模式 |
| [Thor 测速环境](../environments/PI05_THOR.md) | Thor 环境准备和模型加载验收 |
| [Thor PI05 测速](PI05_THOR_BENCHMARK.md) | 离线推理测速、持续负载和结果解释 |

## 仿真与手势模块

| 文档 | 内容与适用范围 |
|---|---|
| [Newton 环境](../environments/NEWTON_SETUP.md) | 主项目仿真 Conda 环境与 GPU 检查 |
| [Newton 完整指南](../simulation/README.md) | 当前主项目场景、模型、自动对折、VR、三路相机及验收 |
| [手势 RViz](GESTURE_RVIZ.md) | 隔离的 RViz 手势仿真与录制 |
| [手势识别模块](../gesture_teleoperation/README.md) | 识别模型、相机标定、UDP/ROS 接口、环境与模块测试 |

## 历史与交付资料

下列资料保留原路径，便于追溯。历史方案中的命令和未完成清单不能直接作为当前操作步骤。

| 资料 | 定位 |
|---|---|
| [独立 Newton 交付包](../newton_towel_sim/README.md) | 2026-11-05 交付快照；有独立 Makefile、文档、清单和验证流程 |
| [另一份 Newton 目录](<../newton_towel_sim (2)/README.md>) | 同名副本；主项目导出脚本的目标是无 `(2)` 的目录，副本不作为主项目更新入口 |
| [11 月 4 日回放事故资料](../HW_arm_replay_incident_20261104/00_先读说明.md) | 原始说明、证据索引、日志与离线复算；保留证据内容 |
| [10 月现场调试问题](调试问题.md) | 相机线、夹爪 CAN 和支架等现场问题原始记录，处理状态待现场核实 |
| [左臂零位设计](superpowers/specs/2026-09-19-left-arm-zero-calibration.md)、[实施计划](superpowers/plans/2026-09-19-left-arm-zero-calibration.md) | 2026-09-19 设计与计划，当前操作见机械臂软件零位标定 |
| [移除力控设计](superpowers/specs/2026-09-20-remove-force-control-design.md)、[实施计划](superpowers/plans/2026-09-20-remove-force-control.md) | 2026-09-20 历史设计与计划，包含已变更的启动方式 |

交付包内的环境、使用、架构、验证与第三方声明由其 README 导航。根目录的
`newton_towel_sim.tar.gz`、`.sha256` 是打包产物，不能假定与主项目源码自动同步。

## 文档维护约定

- 根 README 维护项目入口；本页维护完整索引；操作命令以各专项流程为准，速查页只作查找。
- 新增文档时在本页加入链接，说明适用场景；路径使用相对链接。保留既有路径，避免外部引用失效。
- 修改 CLI、Makefile 目标、按键、消息或 YAML 时，同步相应流程、命令速查与配置说明。当前默认值与示例值需明确区分。
- 验证记录写明日期、环境、执行命令和实际结果；区分离线测试、仿真验证和真机验收，不能把历史结果视为本次通过。
- 交付快照按其打包流程更新；事故日志、证据和历史计划保持可追溯。迁移文档的参数表按日期理解，以现场配置为准。
