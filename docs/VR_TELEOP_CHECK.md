# VR 遥操作联调检查

本流程用于确认 XR 设备、VR 程序、ROS 话题、双臂控制链和新按键映射能够正确配合。
它包含会使真实机器人运动的步骤；软件退出或 `Ctrl+C` 不是物理急停。人员或设备出现风险时，先使用现场物理急停。

## 1. 检查范围和前置条件

| 级别 | 验证内容 | 是否会使机器人运动 |
|---|---|---|
| A：VR 连接 | XR 浏览器连接、按 Enter 后 VR 节点发布话题 | 不启动下游控制节点时不会。 |
| B：空载遥操作 | 位姿增量经上层、中层、伺服传到双臂 | 会，必须在清空工作空间后执行。 |
| C：夹爪、前馈、录制 | 夹爪开闭、右 B 前馈、录制/丢弃服务 | 会；前馈和夹爪需额外谨慎。 |

开始前必须完成：

- 急停可触及，双臂活动范围内无人，工具和线缆无干涉；
- EtherCAT、CAN、力传感器和双臂按厂家流程上电且无故障；力传感器配置、物理左右对应和去皮见 [力传感器检查与去皮](FORCE_SENSOR.md)；
- EtherCAT 网口显示 `LOWER_UP`，且已按[环境与构建](../environments/README.md#ethercat-网卡权限)为 `servo_node` 配置原始网卡权限；
- [夹爪标定](GRIPPER_CALIBRATION.md) 已完成，`/towel/gripper_state` 没有 `-1`；
- 若要检查录制，三台 RealSense 已按 [相机序列号配置](CAMERA_CONFIGURATION.md) 填写并用 `make preview` 验证；
- 终端均使用机器人 Python 环境，而非 Conda 训练环境。

## 2. 单命令启动空载遥操作

在完成现场硬件检查后，只需在项目根目录执行：

```bash
make teleop
```

该目标会自动激活 `.venv-robot`、加载 ROS/OpenRobots/工作区环境、先执行 `check-control`，然后按既有安全顺序启动：

```text
servo → force → arms → robot → VR
```

其中 `servo` 提供 EtherCAT 电机和夹爪反馈，`force` 提供力/力矩数据，`arms` 运行中层 IK/FK 与导纳控制，`robot` 处理 VR 位姿增量，最后才启动 VR Web 服务。
它**不会启动采集器**，因此不会打开 RealSense 或创建 episode。需要同时显示 RViz 时使用：

```bash
make teleop VISUALIZE=true
```

按 `Ctrl+C` 会停止本次 Launch 管理的节点，但不能替代物理急停。
如果只做级别 A 的 XR 连接检查，可使用 `make vr`；这同样自动初始化环境，但不启动下游控制节点，不能据此证明机器人会按预期运动。
若 `servo` 报 `No socket connection`，先检查网卡权限与 `LOWER_UP` 状态；不要用
`sudo -E make teleop` 或 `sudo -E make experiment` 作为替代，因为这会让 VR、RViz、采集器
和整个 ROS Launch 都以 root 身份运行。

### 可选：单命令观察 ROS 话题

另开一个终端，按需执行以下任一命令；每条命令都会自动初始化机器人环境：

```bash
make watch
make watch WATCH_TOPIC=/dexbotCtl/midMonitor_msg_send WATCH_ARGS=--once
make watch WATCH_TOPIC=/dexbotCtl/upperMonitor_msg_send WATCH_ARGS=--once
make watch WATCH_TOPIC=/vr/right_arm_force_feedforward
```

其中：

- `/dexbotCtl/midMonitor_msg_send`：双臂关节反馈；
- `/dexbotCtl/upperMonitor_msg_send`：上层发出的左右末端名义目标；
- `/towel/gripper_state`：`[左反馈, 右反馈, 左目标, 右目标]`；
- `/vr/right_arm_force_feedforward`：右 B 按住时的六维前馈力/力矩。

## 3. 连接 XR 设备

`make teleop` 启动 VR 后会等待提示。先取得机器人主机在 XR 设备所在局域网内的 IP：

```bash
hostname -I
```

在 XR 设备浏览器输入：

```text
https://<主机IP>:8012?ws=wss://<主机IP>:8012
```

例如主机地址为 `192.168.1.20`：

```text
https://192.168.1.20:8012?ws=wss://192.168.1.20:8012
```

浏览器如提示本地 HTTPS 证书，需要按现场安全策略信任本任务生成的本地证书；确认页面已连接后，点击 **Virtual Reality** 进入控制器模式。
然后回到运行 `make teleop` 的终端按 Enter。终端出现“开始发布位姿增量（扣动扳机生效）”才表示 VR 节点开始向 ROS 发布数据。

若页面无法打开，先确认 XR 设备与主机在同一网络、IP 选对、端口 `8012` 未被防火墙拦截，且 `make teleop` 输出中没有证书或 Vuer 错误。

## 4. 按键映射与空载验证顺序

当前映射以 `televuer` 库字段名为准；不同手柄上 A/B 的物理字样可能不同，首次务必空载核对。

| 输入 | 预期行为 | 安全检查方式 |
|---|---|---|
| 左扳机 | 左臂位置、姿态增量生效 | 从中立姿态开始，小幅移动左手；确认仅左臂按预期方向移动。 |
| 右扳机 | 右臂位置、姿态增量生效 | 小幅移动右手；确认仅右臂按预期方向移动。 |
| 左 squeeze | 左夹爪开/闭切换 | 短按一次，确认仅左夹爪动作；保持按住不应重复切换。 |
| 右 squeeze | 右夹爪开/闭切换 | 短按一次，确认仅右夹爪动作；保持按住不应重复切换。 |
| `right_aButton` | 双臂回零 | 仅在已确认回零目标和轨迹均安全时短按一次。 |
| `right_bButton` | 按住右臂力前馈，松开归零 | 仅在空载、前馈参数已审核、末端不接触任何物体时短暂测试。 |
| `left_bButton` | 开始/结束录制 | 仅在采集器已启动时测试。 |
| `left_aButton` | 丢弃当前录制片段 | 仅在录制进行中时测试。 |

按下面顺序操作，不要首次联调就同时按多个键：

1. 双手不扣扳机，确认机械臂静止。VR 程序会发布零增量；如果机械臂仍持续运动，立即松开手柄、使用急停或按现场规范停机。
2. 分别短暂扣住左、右扳机，以很小幅度测试每只手臂的平移和转动方向。方向不对时立即停止，不要靠大幅反向运动“试出来”。
3. 检查 `/towel/gripper_state` 没有 `-1`，再分别短按左右 squeeze。确认左右夹爪没有交叉控制、不会反复切换或撞击限位。
4. 仅在回零姿态和运动路径已经确认安全时，短按一次右 A；观察两臂是否按预期平滑回归参考姿态。
5. 查看 `teleop/VR_controller/config/gripper_pd.yaml` 的 `right_arm_feedforward_force`。其每个分量都已确认安全后，右 B 只短暂按住一次；观察 `/vr/right_arm_force_feedforward` 按住时为配置值、松开后全为零。不要在接触毛巾、桌面或人员时测试前馈。

## 5. 可选：检查录制和丢弃按键

只有三路相机和机器人反馈都已正常时，才在另一个终端启动采集器：

```bash
make record
```

观察 `/towel/recording` 后执行：

1. 按左 B 一次，话题应从 `false` 变为 `true`；
2. 做一小段安全、无接触动作；
3. 再按左 B 一次，话题应变回 `false`，并保存一个完整 episode；
4. 再次按左 B 开始，随后按左 A；当前 `.partial` 片段应被删除，而不会成为训练数据。

录制器未启动时，按左 B 或左 A 只会在 VR 终端给出服务不可用警告，不会自动启动采集器。

## 6. 常见问题

| 现象 | 优先检查 |
|---|---|
| XR 页面无法打开 | 主机 IP、XR 与主机是否同网、`8012` 端口、防火墙，以及运行 `make teleop` 的终端中的证书/Vuer 输出。 |
| 点击 Virtual Reality 后没有 ROS 输出 | 必须回到运行 `make teleop` 的终端按 Enter；检查控制器已连接而非仅打开网页。 |
| 扣扳机后机械臂不动 | `robot`、`arms`、`servo` 是否都运行；检查 `/vr/left_arm_delta`、`/vr/right_arm_delta` 和控制链话题。 |
| 机械臂方向错误或跳变 | 立即停止；核对手柄物理左右、坐标映射、零位与缩放配置，不要继续接触操作。 |
| 夹爪状态为 `-1` | 检查 `servo` 是否运行、`/gripper/fdb` 是否新鲜、端点是否已标定。 |
| 左 B / 左 A 无效 | 采集器可能未启动，或当前无完整新鲜的相机和机器人流，检查 `/towel/recording` 和采集器终端日志。 |
| 右 B 没有前馈或松开后仍非零 | 检查 `right_arm_feedforward_force`、`/vr/right_arm_force_feedforward` 以及手柄 `right_bButton` 字段；异常时停止接触动作。 |

## 7. 结束

先正常结束当前录制，再松开扳机和所有按键。按现场规范安全停机；发生异常时优先物理急停，不要依赖关闭 VR、Launch 或终端替代急停。
