# 真机实验流程：双臂 VR 叠毛巾与数据采集

本文件是一次实验当天的**总清单**。它只说明顺序、通过条件与下一份专项文档；相机、夹爪、VR 和数据格式的具体命令均在各自模块中维护，避免同一命令在多处失效。

适用目录：`/home/yanziyu/projects/HW_arm/towel_task`。软件停止不是物理急停；人员、机械臂或线缆存在风险时，先按现场规范使用物理急停。

## 当天流程表

| 阶段 | 目标 | 通过条件 | 进入的模块 |
|---|---|---|---|
| 0. 现场安全 | 清空工作空间，检查急停、供电、网线、CAN/EtherCAT、三台相机和毛巾 | 急停可触及，无线缆拉扯或运动干涉 | 本文件的“0” |
| 1. 软件就绪 | 构建并完成只读依赖检查 | `make build`、`make check`、`make test` 成功 | [环境与构建](../environments/README.md) |
| 2. 相机预检 | 确认三台相机序列号、物理位置和画面 | head/left_wrist/right_wrist 三路正确且稳定 | [相机序列号配置](CAMERA_CONFIGURATION.md) |
| 3. 夹爪标定 | 采样左右开/闭端点并检查反馈 | 两侧 `calibrated: true`；夹爪状态不为 `-1` | [双夹爪标定](GRIPPER_CALIBRATION.md) |
| 4. 空载 VR | 检查双臂位姿、夹爪和回零 | 所有方向、安全限幅和按键符合预期 | [VR 遥操作检查](VR_TELEOP_CHECK.md) |
| 5. 正式采集 | 分终端启动控制、采集器和 VR | 伺服、双臂、上层控制、采集器/可选相机窗口和 VR 均正常 | [真机数据采集流程](DATA_COLLECTION_WORKFLOW.md) |
| 6. 试录和正式采集 | 先保存短的无接触样本，再录制叠毛巾 | 三路图像和机器人数据完整，无意外 `.partial` | [数据集约定](DATASET.md) |
| 7. 验收与导出 | 校验完整 episode，并在训练环境导出 | 校验成功；训练/验证按 episode 划分 | [数据集约定](DATASET.md)、[π0.5 训练](../training/README_PI05.md) |
| 8. 安全停机 | 正常结束录制、松开操作输入并按现场规范停机 | 不把 `.partial` 或失败段用于训练 | 本文件的“8” |

## 0. 现场安全

在运行任何 ROS 节点前：

- 清空双臂可达空间，固定或铺平毛巾，确认无人员、工具或线缆会进入轨迹；
- 确认物理急停可触及，并按厂家规定完成上电、EtherCAT 和 CAN 配置；确认 EtherCAT
  网卡显示 `LOWER_UP`（没有 `NO-CARRIER`），并已按[环境与构建](../environments/README.md#ethercat-网卡权限)
  配置 `servo_node` 的网卡权限；
- 检查三台 RealSense 的供电和 USB 连接；
- 首次联调、重新标定或修改映射后，一律从空载、小幅、低速动作开始。

未通过任一项时不上电、不启动控制流程。

## 1. 软件就绪

在项目根目录执行：

```bash
make build
make check
make test
```

`make` 会自动加载机器人端环境。`make check` 是只读检查，不会打开 CAN、启动电机或占用相机。若失败，按 [环境与构建](../environments/README.md) 和 [依赖说明](DEPENDENCIES.md) 修复，不要跳过检查直接进入硬件阶段。
`make build` 的末尾会自动请求 sudo 并配置 EtherCAT 网卡权限；若该授权步骤失败，
不要进入涉及伺服的后续步骤。

## 2. 至 4. 完成三个独立预检

按下列顺序完成专项文档，完成前不要启动正式采集：

1. [相机序列号配置](CAMERA_CONFIGURATION.md)：填写三台非空且互不重复的序列号，并用 `make preview` 查看三路画面；
2. [双夹爪标定](GRIPPER_CALIBRATION.md)：只启动 `make servo`，写入四个端点并核对 `/gripper/fdb`；
3. [VR 遥操作检查](VR_TELEOP_CHECK.md)：在四个独立终端依次运行 `make servo`、`make arms`、`make robot`、`make vr`，每一步等待前一个节点初始化完成。连接头显并在 `make vr` 终端按 Enter 后，空载验证左右扳机、两侧 squeeze 和右 A。完成检查并按现场规范停止四个节点后，才能进入下一阶段。

进入正式采集前，停止相机预览和此前测试用的控制节点，避免相机占用或重复控制节点。

## 5. 进入正式采集

完成前置步骤、确认现场安全后，按[真机数据采集流程](DATA_COLLECTION_WORKFLOW.md)分终端
依次启动 `servo → arms → robot → record → vr`。需要在录制中查看相机时，按该文档以
`record-opencv` 替换 `record`；不要启动第二个采集器。

## 6. 试录与正式采集

分终端进程均稳定后，使用 VR 中左 B 开始/结束录制，左 A 丢弃当前片段。精确按键含义、XR
连接和安全验证顺序见 [VR 遥操作检查](VR_TELEOP_CHECK.md)。

先录制 3–5 段短、无接触示教，确认每个完整 episode 都有三路图像、双臂数据和有效夹爪状态，再批量录制叠毛巾。失败、碰撞、人工干预或不完整的片段应立即丢弃，不作为正样本。

原始字段、`complete`/`.partial` 生命周期和质量阈值见 [数据集约定](DATASET.md)。

## 7. 验收与导出

在采集结束且硬件已按现场规范停机后：

```bash
make validate-data
make export-data OUTPUT=/absolute/path/to/fold_towel_lerobot
```

若本次使用了非默认的 `dataset_root`，两个命令都增加 `DATASET_ROOT=/absolute/path/to/raw_data`。导出应在安装 LeRobot 的训练环境运行；不要把训练依赖安装到机器人控制环境。数据拆分与字段解释见 [数据集约定](DATASET.md)。

## 8. 收尾

先正常结束当前录制，再松开扳机和所有按键，按现场规范使机器人安全停机。发生异常优先物理急停。只把通过校验的完整 episode 交给训练；`.partial`、`too_short` 和已丢弃片段保留为排障证据或按数据管理规则清理，绝不伪造或补写原始帧。
