# 力传感器检查与去皮

本文件覆盖真机实验中的两路六维力/力矩传感器：配置、启动后的数据检查、空载去皮和右臂力前馈前的安全确认。底层驱动的实现和所有 CAN 配置字段见 [encos_force_sensor 使用说明](../ros2_ws/src/encos_force_sensor/docs/USAGE.md)。

> **安全边界：** 去皮只是消除当前零偏，不会限制机械臂、关闭导纳或替代物理急停。传感器、工具与毛巾无接触时才允许去皮；不要在接触、受力或夹持毛巾时调用去皮服务。

## 1. 配置归属

配置文件是 `ros2_ws/src/encos_force_sensor/config/force_sensor.yaml`。当前定义：

| 逻辑名 | CAN 设备 | 力话题 | 去皮服务 |
|---|---|---|---|
| `left` | `can0` | `/force_sensor/left/wrench` | `/force_sensor/left/tare` |
| `right` | `can1` | `/force_sensor/right/wrench` | `/force_sensor/right/tare` |

其中 `transport`、`device`、`can_id`、`is_extended` 和 `axis_flip` 必须与现场硬件一致。`axis_flip` 的六个元素依次控制 `Fx/Fy/Fz/Tx/Ty/Tz` 的正负号；不要只为让某个数值“看起来为正”就修改它，必须用小幅、已知方向的空载动作验证。

若使用 SocketCAN，接口的 CAN-FD 波特率与启用方式由厂商硬件决定；在启动本项目之前按 [驱动使用说明](../ros2_ws/src/encos_force_sensor/docs/USAGE.md#21-socketcan推荐) 配置实际 `can0`、`can1`。这些需要管理员权限的硬件命令不由 Makefile 自动执行，避免错误改写现场 CAN 接口。

## 2. 在最终实验进程中检查

相机、夹爪和 VR 空载预检已经完成后，用完整实验命令启动节点：

```bash
make experiment VISUALIZE=true
```

等待力节点启动后，在另一个终端分别观察两路话题。`make watch` 会自动加载机器人环境：

```bash
make watch WATCH_TOPIC=/force_sensor/left/wrench
make watch WATCH_TOPIC=/force_sensor/right/wrench
```

预期每条消息为 `geometry_msgs/WrenchStamped`，其中 `wrench.force.{x,y,z}` 单位为 N，`wrench.torque.{x,y,z}` 单位为 Nm。没有消息、持续断流或服务不存在时，不要进入接触动作；先检查 CAN 接口、供电、`force_sensor.yaml` 和启动日志。

## 3. 空载去皮

确认两侧工具无接触、夹爪没有夹住物体且传感器静止后，分别执行：

```bash
make tare-force SIDE=left
make tare-force SIDE=right
```

每次服务调用会对配置中的 `tare_samples` 帧求平均，并把该平均值作为当前力节点进程的零偏。服务返回成功后重新观察话题；静止空载下数值应接近零，并允许存在传感器噪声。

去皮偏置不写入 YAML，也不会跨进程保存。因此在以下任一种情况发生后都必须重新去皮：

- `make experiment` 或力节点被停止、崩溃或重启；
- 拆装传感器、工具、夹爪或改变预紧；
- 更换 CAN 配置、修改 `axis_flip` 或移动零位安装姿态。

## 4. 物理对应与符号确认

本项目保留历史控制链的原话题名；上层控制中曾存在左右力话题交叉订阅的历史情况。驱动的 `left`/`right` 只表示 `force_sensor.yaml` 中的配置键，**不自动证明它与左右机械臂的物理对应或控制器订阅关系一致**。

在不接触毛巾、低风险的条件下，对每只工具施加很小、已知方向的外力，逐路确认：

1. 只有预期的 `/force_sensor/<side>/wrench` 明显变化；
2. 变化的力轴和力矩轴与实际施力方向一致；
3. 左右工具不会被错误归属，松手后数值回到接近零；
4. 仅在以上三项成立后，才测试右 B 的力前馈或开始接触毛巾。

任何一项不成立时，停止接触实验；记录实际观测，不要在没有证据时交换话题名或反转 `axis_flip`。回到配置与控制链排查后，重新完成去皮和空载验证。

## 5. 常见问题

| 现象 | 优先检查 |
|---|---|
| 力话题不存在 | `make experiment` 是否仍在运行；`can0`/`can1` 是否已按实际硬件配置；启动日志是否报告打开传输失败。 |
| 有数据但数值长期不变 | 传感器供电、CAN ID、帧格式和 `can_id`/`is_extended` 配置；不要仅凭存在话题就判定真实反馈正常。 |
| 去皮服务调用失败 | 对应力节点未运行、`SIDE` 不是 `left` 或 `right`，或 ROS 域/环境不一致。 |
| 去皮后仍有明显偏置 | 去皮时存在接触或振动；等待静止后重试，并检查工具预紧和 `tare_samples`。 |
| 左右对应或正负方向异常 | 立即停止接触动作；分别确认 CAN 设备、配置键、`axis_flip` 及控制器历史订阅映射。 |

右臂前馈的按键、具体输出话题和安全测试顺序见 [VR 遥操作检查](VR_TELEOP_CHECK.md)。
