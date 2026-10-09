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
| 把当前代码迁移到另一台电脑 | [新电脑迁移与交接](docs/MIGRATION_HANDOVER.md) | 环境重建、参数核对、分阶段验收 |
| 按正确顺序启动并采集真机数据 | [真机数据采集流程](docs/DATA_COLLECTION_WORKFLOW.md) | 分终端：`make servo` → `make arms` → `make robot` → `make record` → `make vr` |
| 填写并核对三台 RealSense 的序列号和画面 | [相机序列号配置](docs/CAMERA_CONFIGURATION.md) | `make preview` |
| 记录单臂或双臂新的安全初始姿态 | [机械臂零位标定](docs/ARM_ZERO_CALIBRATION.md) | `make calibrate-arm-zero SIDE=both` |
| 采样左右夹爪开/闭端点 | [双夹爪标定](docs/GRIPPER_CALIBRATION.md) | `make servo`、`make calibrate ...` |
| 连接 XR、检查按键与空载遥操 | [VR 遥操作检查](docs/VR_TELEOP_CHECK.md) | 分终端依次执行 `make servo`、`make arms`、`make robot`、`make vr` |
| 理解原始片段、字段、校验与 LeRobot 导出 | [数据集约定](docs/DATASET.md) | `make validate-data` |
| 训练 π0.5 | [π0.5 训练说明](training/README_PI05.md) | 在训练环境运行 |
| 排查环境或历史验证范围 | [依赖说明](docs/DEPENDENCIES.md)、[验证记录](docs/VALIDATION.md) | `make check` |

`DATA_COLLECTION_WORKFLOW.md` 是正式录制的唯一启动入口；`REAL_ROBOT_EXPERIMENT.md` 保留当天
预检和验收总清单。控制、采集和 VR 必须分终端运行，避免重复节点发布同名控制话题。

## 常用启动命令

正式采集使用五个独立终端。每个终端先进入项目根目录：

```bash
cd /home/yanziyu/projects/HW_arm/towel_task
```

先执行一次 `make check-control`，通过后按顺序启动，每一步等待前一个节点初始化完成：

| 顺序 | 命令 | 作用 |
|---|---|---|
| 终端 1 | `make servo` | 启动底层伺服，机械臂执行初始回零 |
| 终端 2 | `make arms` | 启动双臂运动学与轨迹控制 |
| 终端 3 | `make robot` | 启动上层控制，输出遥操作初始位姿 |
| 终端 4 | `make record` | 启动唯一的相机采集器；未按左 B 不会创建 episode |
| 终端 5 | `make vr` | 启动 VR 输入，接收手柄操作 |

头显连接页面并进入 **Virtual Reality** 后，回到终端 5 按 **Enter**。
出现“开始发布位姿增量（扣动扳机生效）”后，按住对应手柄的扳机、小幅移动手柄进行遥操作。
当前双臂开启工具轴自转锁定：过滤绕手腕到夹爪末端方向（工具局部 Z 轴）的 VR 旋转，
保留另外两个方向的倾转和平移；不锁定某个实体关节，也不限制右 A/B 的关节姿态运动。
恢复自转可将 `controllerConfig.yaml` 的 `vrOrientationControl.lock_tool_twist` 改为 `false`，
同步配置包后重启 `make robot`。
`make vr-debug` 只显示诊断数据，不发送控制指令，不能替代 `make vr`。

右手 A 键让双臂回到零位。右手 B 键在“快速移动”和“精细微调”两种 VR 响应模式之间切换；
当前模式只影响手柄位姿增量的倍率与限速，不会移动机械臂到预设姿态。
两套参数位于 `teleop/VR_controller/config/gripper_pd.yaml`，切换后立即作用于后续 VR 增量。

其他常用命令：

```bash
make preview                         # 三个 OpenCV 窗口检查相机，不启动 ROS 或录制
make record-opencv                   # 正式采集 + 三个 OpenCV 实时窗口；替代 make record
make record-rviz                     # 可选：向 ROS 发布 RViz 图像话题
make launch ARGS="visualize:=true"   # 可选：单独启动 RViz、状态桥接和独立 TF
make watch                           # 观察 /towel/gripper_state；可通过 WATCH_TOPIC=... 改话题
make calibrate-arm-zero SIDE=left    # 被选中左/右臂零电流；SIDE 可为 left、right、both
make save-arm-zero SIDE=left TARGET=both  # 同步所选侧关节与末端零位；也可选 joint / cartesian
```

上述命令均为前台进程；在一个终端按 `Ctrl+C` 不会停止其他终端的节点。开始正式采集前，必须先完成相机配置、夹爪标定和空载 VR 检查；完整步骤见[真机数据采集流程](docs/DATA_COLLECTION_WORKFLOW.md)。

## 配置归属

| 文件 | 负责内容 | 对应说明 |
|---|---|---|
| `config/recording.yaml` | 任务文本、原始数据目录、三相机序列号、采集频率与时序阈值 | [相机序列号配置](docs/CAMERA_CONFIGURATION.md)、[数据集约定](docs/DATASET.md) |
| `teleop/VR_controller/config/grippers.yaml` | 左右夹爪端点、运动时间和标定状态 | [双夹爪标定](docs/GRIPPER_CALIBRATION.md) |
| `ros2_ws/src/ctl_msgs_hw/config/controllerConfig.yaml` 的 `gripperControl` | 夹爪位控模式、电机 Kp/Kd、位置范围和机械零点 | [双夹爪标定](docs/GRIPPER_CALIBRATION.md) |
| `teleop/VR_controller/config/gripper_pd.yaml` | VR 位姿映射与缩放 | [VR 遥操作检查](docs/VR_TELEOP_CHECK.md) |
| `ros2_ws/src/ctl_msgs_hw/config/zeropos.yaml` | 伺服启动和 VR A 键关节回零目标；`TARGET=joint/both` 保存 | [机械臂零位标定](docs/ARM_ZERO_CALIBRATION.md) |
| `ros2_ws/src/ctl_msgs_hw/config/desc_zeropos.yaml` | 遥操作初始末端位姿；`TARGET=cartesian/both` 从关节反馈正运动学计算并保存 | [机械臂零位标定](docs/ARM_ZERO_CALIBRATION.md) |

当前 `recording.yaml` 的 `right_wrist` 是占位值时，采集器会拒绝启动；请先完成相机模块。配置发生变化后，应使用新的原始数据目录，避免混合不兼容 episode。

## 数据与训练

采集器记录双臂关节反馈/目标、TCP 反馈/名义目标、夹爪二值状态和三路 RGB 图像。字段定义、时间对齐限制、数据质量规则和 LeRobot 映射都在 [数据集约定](docs/DATASET.md)。

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
