# 移除力传感器与力控链路

## 目标

本项目的数据采集和真机遥操只使用双臂位置、夹爪状态、VR 输入和相机数据。移除两路
六维力/力矩传感器的 SocketCAN 驱动、ROS 话题、数据集字段、接触检测、导纳控制以及
VR 人工力前馈。移除后，机器人应能在没有 `can0`、`can1` 的主机上完成位置遥操和夹爪
控制。

## 非目标与保留能力

- 不改变 EtherCAT 电机通信、自动回零、关节反馈、低层 PD 限幅或急停流程。
- 不改变双臂 IK/FK、位置轨迹、VR 左右扳机位置增量控制、双夹爪 squeeze 控制、录制
  状态切换或三路相机采集。
- 保留现有重力补偿和位置控制所需的非传感器动力学计算；不再基于外力修改位置目标或
  输出关节前馈力矩。
- 不提供运行时开关。力控将从本项目的构建、启动、控制和数据路径中移除；恢复需从
  Git 历史恢复相应实现。

## 运行时设计

启动顺序变为：

```text
servo_node -> dual_arm_control_node -> robot_control_node -> VR publisher
```

`make teleop` 启动上述空载遥操链，不再启动力节点；`make experiment` 在同一控制链基础
上增加采集器。Makefile 不再提供 `force` 或 `tare-force` 命令，Launch 文件也不再接受
`start_force` 参数。

## 控制链设计

### 双臂控制节点

移除 `/force_sensor/left/wrench` 与 `/force_sensor/right/wrench` 的订阅、六维传感器反馈
缓存、启动偏置估计、低通滤波、接触门控、外力坐标旋转、导纳位置修正和外力相关的关节
前馈力矩叠加。正常位置目标直接经过现有 IK/轨迹和低层控制链。

### VR 与上层控制节点

移除 `/vr/right_arm_force_feedforward` 发布、右手 B 键对应的人工力前馈配置，以及
`robot_control_node` 的订阅和转发。右手 B 键不再产生机器人控制效果；扳机和 squeeze
的既有位置/夹爪行为不变。

## 采集、构建与文档设计

- 采集器不再订阅 `WrenchStamped`，`recording.yaml` 不再声明 `force_topics`，导出样本不再
  含力字段。
- 删除 `encos_force_sensor` ROS 包及其 SocketCAN/ZLG 配置与专用力传感器文档。
- 清理 README、实验流程和 VR 操作说明中的力节点、去皮、力前馈与接触验证描述。

## 错误处理与安全边界

缺少 `can0` 或 `can1` 不得阻止 `make teleop` 或 `make experiment` 启动。移除力控不降低
既有的物理安全要求：运行前仍须清空工作区、保持急停可触及，并以小幅、空载动作检查
左右臂方向。接触操作不应依赖本项目的自动力控或接触检测。

## 验证

不依赖硬件的验证包括：

1. `make -n teleop` 和 `make -n experiment` 不含 `start_force`；Makefile 不再定义
   `force`、`tare-force` 目标。
2. 数据采集配置和 `record.py` 不含力话题或 `WrenchStamped` 订阅。
3. VR 发布器和上层控制节点不含人工力前馈话题或右手 B 键力控。
4. 双臂控制节点不订阅 `/force_sensor/*/wrench`，其位置控制路径不引用外力输入。
5. `make build` 与 `make test` 成功；由现场操作者依序启动 `servo`、`arms`、`robot`、`vr`
   进行空载、小幅人工验收。
