# 双夹爪控制与端点标定

## 按键与编码

- 左侧 squeeze 上升沿切换左夹爪，保持按住不会重复切换。
- 右侧 squeeze 上升沿切换右夹爪。
- 左 B 录制、左 `left_aButton` 丢弃、右 A 回零。
- 0=打开，1=闭合。VR 发布 `[左反馈,右反馈,左目标,右目标]` 到 `/towel/gripper_state`。
- 观测是实际角度相对标定行程的二值分类，目标是按键切换的状态，二者分别采集。

## 端点文件

`teleop/VR_controller/config/grippers.yaml` 为每侧独立保存：
`open_deg`、`closed_deg`、`calibrated`、`motion_time_s`。
允许左/右电机方向相反，直接用实际开、闭角度计算，不需要猜测符号。

夹爪采用与 `src-4` 相同的电机 MIT 位控：VR 发送平滑位置目标，电机内部执行 PD。
`ros2_ws/src/ctl_msgs_hw/config/controllerConfig.yaml` 的 `gripperControl` 配置每侧
`mode: position`、`kp/kd`、`pos_min_deg/pos_max_deg` 和 `zero_pos_deg`。
当前 Kp/Kd 沿用参考实现：左 5.0/0.8，右 8.0/2.0，仍需空载实机验证。
位置边界采用本项目已标定端点，机械零点偏移均为 0。
`/gripper/ctl` 的 `l_arm_gripper_pos/r_arm_gripper_pos` 是相对机械零点的偏移（度）；
VR 从与伺服节点相同的已安装配置中读取零点，并从绝对编码器目标中减去一次零点。
速度、前馈力矩恒为 0。位置模式不使用旧软件 PD 的力矩限幅；超温时 Kp/Kd 清零，
降至温度阈值减 `temp_recover_hys` 后恢复。超温会释放夹持力。
更新消息接口或伺服配置后执行 `make build`，重启伺服与 VR 节点。

## 操作步骤

### 先理解：标定期间要启动什么

夹爪端点标定只需要**伺服节点**持续提供 `/gripper/fdb`，以及一个临时运行的标定脚本。
不要把标定误当成正式采集：`arms`、`robot`、`vr` 和 `record` 都不是标定的前置条件。

| 程序 | 标定时是否启动 | 原因 |
|---|---|---|
| `servo` | 必须 | 从 EtherCAT/CAN 电机反馈发布 `/gripper/fdb`。 |
| `calibrate_grippers.py` | 必须，且每个端点运行一次 | 被动采样反馈并写入端点配置；不发送夹爪力矩。 |
| `arms` / `robot` | 不要启动 | 标定不需要双臂中层或上层轨迹控制。 |
| `vr` / `record` | 必须停止 | 防止继续下发夹爪目标或把标定过程写入数据集。 |

在开始前，确认工作空间安全、急停可触及，并按机器人的既有硬件流程让待测夹爪处于可安全手动调整或释放力矩的状态。
**只停止 VR 会继续保持最后的位置目标。** 标定脚本不会清零或接管已有命令，不能代替硬件安全操作。
启动伺服但尚未收到有效位置目标时，夹爪输出为零力矩，不会自动回零；
端点标定应在这样的全新伺服会话中进行，并停止 VR。

### 终端 1：启动夹爪反馈发布者

完成机器人规定的上电、EtherCAT/CAN 和急停检查，并确认最后一次 `make build` 的 sudo
授权步骤成功后，在第一个终端执行：

```bash
make servo
```

这个终端应保持运行。它启动 `servo_node`，并从电机回包发布 `/gripper/fdb`。
此时**不要**启动 `make arms`、`make robot`、`make vr`、`make record` 或可视化 Launch。
不要为常规标定使用 `sudo -E make servo`；root 与普通用户 ROS 节点可能无法交换 DDS
共享内存消息。

### 终端 2：确认反馈真实、新鲜且左右对应

在第二个终端执行：

```bash
make watch WATCH_TOPIC=/gripper/fdb
```

预期有 `servo_node` 发布者，且每条消息都有 8 个数：

```text
[左角度_deg, 左速度_rad_s, 左电流_A, 左温度_C,
 右角度_deg, 右速度_rad_s, 右电流_A, 右温度_C]
```

观察输出应持续刷新，不能长期断流。保持安全的前提下，分别让左、右夹爪发生很小的实际位移，核对只有对应一侧的角度值变化且方向合理；不要仅因数组全为 `0` 就认为反馈正常。
确认后可让该终端继续用于观察，也可以按 `Ctrl+C` 停止 `make watch`；这不会停止伺服节点。

### 终端 3：逐个采集四个端点

每次只采一个端点。先让对应夹爪处于正常的打开或闭合位置并保持静止；不要把硬限位顶死作为标定目标。
然后在第三个终端执行对应命令：

```bash
# 1. 左夹爪处于正常打开位置且静止
make calibrate SIDE=left ENDPOINT=open

# 2. 左夹爪处于正常闭合位置且静止
make calibrate SIDE=left ENDPOINT=closed

# 3. 右夹爪处于正常打开位置且静止
make calibrate SIDE=right ENDPOINT=open

# 4. 右夹爪处于正常闭合位置且静止
make calibrate SIDE=right ENDPOINT=closed
```

每条命令结束后再移动夹爪至下一个端点。脚本会在最多 10 秒内等待完整反馈窗口；成功后会打印保存的角度和 `calibrated` 状态。失败或超时时不会修改配置文件。

一次有效采样要求至少 1 秒、至少 20 条连续反馈；角度极差不得大于 1 度，角速度绝对值不得超过 `0.05 rad/s`。端点之间还必须至少相差 5 度，否则脚本拒绝写入。

### 标定完成后的检查与恢复

四条命令完成后，在终端 3 检查配置：

```bash
sed -n '1,160p' teleop/VR_controller/config/grippers.yaml
```

只有左右两侧都包含有效的 `open_deg`、`closed_deg`，并显示 `calibrated: true`，才可以进入 VR 空载复核。
重新标定后还需核对 `gripperControl` 的位置边界覆盖两个端点；修改边界后重新构建并重启伺服。
VR 会拒绝启动范围不匹配的配置，避免端点被静默截断。
**保持终端 1 的 `servo` 持续运行**，否则 VR 收不到 `/gripper/fdb`，会把夹爪状态标为 `-1`。
在新的终端执行：

```bash
make vr
```

VR 从首次反馈角度保持（超出配置范围时受位置限幅约束），第一次按键才向对应端点运动；到达端点由电机 PD 保持。
反馈超时或该侧未标定时，VR 发送 NaN 位置表示不更新该侧目标，状态标为未知；伺服继续保持最后目标，
而非把位置 0 当成停止指令。反馈恢复后重新以当前位置保持。
在终端 2 观察：

```bash
make watch WATCH_TOPIC=/towel/gripper_state
```

它应为 `[左反馈, 右反馈, 左目标, 右目标]`，且没有 `-1`。分别短按左右 squeeze，确认左右夹爪独立切换、运动方向正确、不会撞击机械限位；确认无误后才按[真机数据采集流程](DATA_COLLECTION_WORKFLOW.md)启动采集器和其他控制层。
未知状态 -1 不会进入数据集。运动中滞回阈值为行程的 40%/60%，避免临界点抖动。
这种二值反馈不表示接触检测或夹持成功；毛巾较厚时，反馈状态与闭合目标可能不同。

标定过程记录角度用于控制；正式逐帧采集只保存开合状态，不保存夹爪角度、速度、电流、温度或力矩。
