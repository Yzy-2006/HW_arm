# 双臂数据集动作回放流程

本文档说明如何把 `/home/yanziyu/datasets/fold-towel-93` 中已经记录的双臂动作，
通过当前 ENCOS ROS2 控制链发送给机器人，用于检查控制部署链路。

这里回放的是数据集的 `action`，不是让 PI05/ACT 模型重新推理。因此回放时不需要启动
PI05 worker、ACT worker、RealSense 相机或 `pi05_bridge.py`。

## 回放链路

```text
LeRobot Parquet action
        |
        v
deployment/replay_dataset.py
        |
        +--> /dexbotCtl/bridge_ctl_msg  -> robot_control_hw -> arms -> servo -> 双臂
        |
        +--> /gripper/ctl               -> servo -> 左右夹爪
```

回放脚本默认是 dry-run，不发布运动命令。只有命令中显式加入 `--motion` 时才会向控制
话题发布动作。

## 数据格式

该数据集是 LeRobot v3.0 格式，当前数据包含 92 个 episode，采样率为 30 FPS。每一帧的
`action` 是 16 维绝对目标，顺序固定为：

```text
[左关节1..7, 左夹爪, 右关节1..7, 右夹爪]
```

- 关节目标单位是弧度。
- 夹爪值为二值状态：`0=open`，`1=closed`。
- 回放只读取 `data/**/*.parquet` 中的 `action`、`episode_index` 和 `frame_index`。
- 视频不会被读取，相机画面不会影响动作回放。

## 一次性准备

在项目根目录执行：

```bash
cd /home/yanziyu/projects/HW_arm/towel_task
```

回放需要在机器人 Python 环境中读取 Parquet。若尚未安装：

```bash
.venv-robot/bin/python -m pip install 'pyarrow>=14,<22' 'matplotlib>=3.7,<4'
```

确认工作区和控制依赖已经构建：

```bash
make build
make check-control
```

如果只是使用已经构建好的控制消息和节点，不需要为每次回放重新 `make build`。

现场开始前还需要确认：

- 双臂运动范围内没有人员、障碍物或容易被夹住的物体。
- 操作者能立即使用硬件急停。
- 左右夹爪已经完成开闭端点标定。
- `config/pi05_deployment.yaml` 中的 URDF、PD、控制器和夹爪配置指向当前机器人。

## 选择 episode

先列出所有 episode，不会启动 ROS，也不会发布动作：

```bash
make replay ARGS="--dataset /home/yanziyu/datasets/fold-towel-93 --list-episodes"
```

输出示例：

```text
episode=0 length=2935 frames=0:2935
episode=1 length=3802 frames=2935:6737
...
```

`episode` 是数据集的 episode 编号，`length` 是该 episode 的帧数，`frames` 是它在整个
Parquet 文件中的全局索引范围。实际回放使用 `episode` 编号，例如 `--episode 0`。

## 启动控制链

打开三个独立终端，每个终端都在项目根目录执行对应命令。

终端 1，启动底层伺服：

```bash
make servo
```

终端 2，启动双臂运动学和轨迹控制：

```bash
make arms
```

终端 3，启动上层机器人控制：

```bash
make robot
```

等待三个节点都没有启动错误后，在任意终端检查反馈是否持续：

```bash
ros2 topic hz /dexbotCtl/midMonitor_msg_send
ros2 topic hz /gripper/fdb
```

回放脚本在真机模式下需要这两个反馈流，以及 `/towel/safety/state` 的新鲜 `ready` 状态。
开始执行后，反馈超过 `state_timeout_s`（默认 0.25 秒）会锁存故障并中止回放，不会自动恢复。
启动前还没有反馈时仅等待，不发送动作。新的故障处理、补偿关闭状态及验证边界见
[控制安全说明](CONTROL_SAFETY.md)；短片段正常不代表可以恢复事故后的真机运行。

## 第一步：dry-run

dry-run 会加载动作、创建回放节点并按数据集帧率推进，但不会发布双臂或夹爪运动命令。
先使用一个短片段：

```bash
make replay ARGS="--dataset /home/yanziyu/datasets/fold-towel-93 --episode 0 --start-frame 0 --max-frames 90"
```

重点检查：

1. 日志显示 `dataset replay DRY-RUN`。
2. 日志中的 action 是 16 个数，左臂关节位于第 1 至第 7 位，右臂关节位于第 9 至第 15 位。
3. 日志显示 `Replay completed; holding position`。
4. 机器人没有因为这个 dry-run 产生运动。

### RViz 安全预览

可以在独立的 RViz 模型中预览同一段回放。该模式自动启动隔离的模型查看器，只发布
`/model_viewer/joint_states`，不会向真机控制话题发布命令，也不能与 `--motion` 同时使用：

```bash
make replay ARGS="--dataset /home/yanziyu/datasets/fold-towel-93 --episode 0 --start-frame 0 --max-frames 90 --rviz"
```

建议先用较短的 `--max-frames` 检查左右臂方向、关节限位和动作顺序，再进行真机回放。
脚本等待 RViz 和模型发布器就绪后开始回放。正常结束立即保存 CSV/PNG，窗口保持最后一帧，
关闭窗口或按 `Ctrl+C` 退出。若 RViz 启动失败则报错停止，不会继续进行不可见的回放。
关闭 RViz 或按 `Ctrl+C` 后，回放 CSV/PNG 仍会写入输出目录。

dry-run 只验证数据读取、episode 选择、帧率调度和消息构造，不验证电机实际响应。

## 第二步：短片段真机回放

首次真机测试建议只回放 30 至 90 帧，并把关节步长限制缩小到默认值的 10%。例如：

```bash
make replay ARGS="--dataset /home/yanziyu/datasets/fold-towel-93 --episode 0 --start-frame 0 --max-frames 30 --slow-factor 0.1 --motion"
```

确认动作方向、左右臂对应关系和夹爪开闭方向正确后，再扩大到 90 帧：

```bash
make replay ARGS="--dataset /home/yanziyu/datasets/fold-towel-93 --episode 0 --start-frame 0 --max-frames 90 --slow-factor 0.1 --motion"
```

`--motion` 模式必须显式指定 `--max-frames`，避免误把整个长 episode 直接发送给机器人。
需要从 episode 中间开始时，修改 `--start-frame`：

```bash
make replay ARGS="--dataset /home/yanziyu/datasets/fold-towel-93 --episode 0 --start-frame 300 --max-frames 60 --slow-factor 0.1 --motion"
```

## 独立的首帧接近段

当前 `bridge.motion_safety.protection_mode: speed_only` 下，
回放不等待安全心跳，不检查缓存反馈年龄或旧的反馈相对目标裁剪限制；
接近段结束后直接开始数据回放，不执行下文的首帧接管/反馈到位确认。
首次有效关节/夹爪反馈、有限数值与协议检查、显式停止和底层故障锁存仍保留。
接近轨迹及底层接管的平滑规划继续执行。下文的到位等待规则可通过将模式设为 `full` 恢复。

真机回放另有独立的即时跟踪误差保护：`bridge.replay_max_tracking_error_rad: 0.3`。
双臂全部 14 个关节均检查，不受后三关节不限速或 `speed_only` 模式影响。
接近、到位等待、数据回放期间，任何关节目标与反馈的绝对误差 **超过** 0.3 rad，
单次检查就停止发布轨迹和夹爪指令，并向 `/towel/safety/stop` 发送故障请求。
检查包括新反馈相对上次已发布目标，以及每次发布前的本次轨迹目标；不以尚未接近的首帧位置作为接近段目标。
日志报告关节名、目标、反馈、误差和阈值，回放 JSON 元数据记录失败原因，CSV 保留停止前已发布的采样。
该阈值必须为正的有限数值，只作用于回放，不改变模型部署或 VR 的检查策略。

底层接收停止请求后锁存故障，双臂输出 `Kp=0`、零目标速度/前馈力矩及配置的故障阻尼 `Kd`。
回放节点每 50 ms 重发停止请求，退出前最多等待 1 秒的底层 `fault:` 确认；确认超时会明确报告
`damping NOT confirmed`，不能将其视为底层已停止。这里的即时是首次检测即触发，仍有反馈、ROS 传输和伺服周期延迟。
故障不会因反馈重新接近目标而自动恢复；阻尼模式也不保证承重保持。RViz/dry-run 不触发实机停止请求。

真机回放先从新鲜的关节反馈位置接近所选片段的首帧，再开始数据回放计时。
接近段与数据段分别规划、检查，不再要求机械臂在一个数据帧间隔内赶到首帧。
接近段采用首尾速度、加速度、jerk 均为零的两点 minimum-snap 曲线；时长根据
14 个关节中的最大位移及速度、加速度限制计算，保留 5% 的时间余量。

`config/pi05_deployment.yaml` 中的默认参数为：

```yaml
bridge:
  replay_approach:
    max_velocity_rad_s: 5.0
    max_acceleration_rad_s2: 1.0
    min_duration_s: 1.0
    position_tolerance_rad: 0.1
    velocity_tolerance_rad_s: 0.05
    settle_time_s: 0.2
    settle_timeout_s: 10.0
```

实际接近限制还会取回放 `motion_safety` 和底层 `homingHandoff` 对应限制中的较小值。
当前 `bridge.motion_safety.speed_limit_enabled_per_arm` 和底层 `armSafety` 的同名配置
均为 `[true, true, true, true, false, false, false]`，分别应用于左右臂的 J1-J7。
只有每臂前四个关节启用软件限速；后三个腕关节不参与速度预检查、指令差分限速、
反馈超速保护和接近/接管速度上限。接近段的加速度平滑仍应用于全部关节。
首帧等待仍要求所有关节反馈速度不超过 0.05 rad/s；这是开始播放的稳定条件。
修改限速范围时需同步两处配置，重启 servo 和回放/部署进程。
当前 `bridge.motion_safety.max_acceleration_rad_s2: null` 关闭回放和模型部署的
加速度保护检查（轨迹预检查、相邻指令差分、解析加速度），速度检查仍启用。
该设置不取消接近段和底层衔接段用于平滑规划的加速度参数；其他保护保持原配置。
将 `null` 改成正数可恢复加速度检查，重启回放/部署进程生效，无需编译。
当前 `bridge.motion_safety.check_position_limits: false` 也关闭 Python 回放/部署的
绝对关节位置检查：接近段与数据段预检查、反馈越界检查，以及目标的 URDF 限位裁剪。
速度检查、有限数值检查、反馈时效、跟踪误差和反馈相对目标偏移限制仍保留。
设为 `true` 可恢复绝对位置检查；该项缺省为 `true`，不修改 URDF 或电机硬件限位。
例如最大位移为 0.2 rad 时，当前接近时长约 1.29 秒；0.5 rad 时约 2.04 秒。
接近和到位等待期间夹爪保持起始反馈对应的开闭状态，不提前执行数据集首帧的夹爪动作。

日志先输出 `Approach planned` 及计算时长，随后输出
`Approach trajectory finished; waiting for servo handoff and first-frame arrival`。
这时继续以原发布频率发送首帧静止目标（不切换 `hold_position` 或重置控制模式），
数据帧和回放时钟均暂停。只有以下条件持续满足 0.2 秒，才开始回放：

- 收到新鲜的 `/towel/control/handoff`，确认底层接管完成且输出已对齐首帧。
- 状态中的原始指令时间戳不早于本次等待阶段发出的首帧指令，防止误用上次接管结果。
- 14 个关节的新鲜反馈均距首帧不超过 0.1 rad，反馈速度绝对值不超过 0.05 rad/s。

随后打印 `First frame confirmed and servo handoff complete; starting dataset replay`。
等待超过 10 秒会锁存故障，日志说明是接管未完成、反馈未到位还是状态缺失；不会强行播放。
`check_position_limits: false` 关闭的是绝对位置限位，不关闭这项首帧相对到位条件。
`--speed` 只调整数据段帧间隔，接近时长独立计算。关闭数据段插值时也保留接近段；
dry-run/RViz 没有真机起始反馈，从数据集首帧直接开始预览。

接近段和 minimum-snap 数据段会在首次运动发布前完成预检查。数据段自身超限仍会
报错，不自动放宽限制或改变 `--speed`。两段以首帧静止边界衔接，可能改变数据段
开头的加速度峰值，因此需要重新检查所选速度。

CSV 增加 `phase` 列：接近段为 `approach`、`dataset_frame=-1`、`source_knot=0`；
等待到位为 `settling`，同样不占用数据帧、不标记原始点；
数据段为 `replay`，从实际选择的 `--start-frame` 开始标记原始点。接近期间的
`dataset_*` 列是待接近的首帧目标，不表示已播放该帧。

此流程需要新增的 `HandoffStatus` 消息和底层状态发布器，首次升级后须重建消息及三个
控制节点，并一起重启 `make servo`、`make arms`、`make robot`。旧版伺服没有该状态话题，
会导致等待超时；不要在缺少状态确认时绕过等待直接播放。

## 速度和安全限制

脚本使用 `config/pi05_deployment.yaml` 中的：

```yaml
bridge:
  state_timeout_s: 0.25
  max_joint_step_rad: 0.02
  slow_factor: 1.0
```

回放默认使用 YAML 中的 `bridge.slow_factor`；显式传入 `--slow-factor` 时，以命令行值为准。
YAML 未配置该字段时回退到 `1.0`，有效范围为 `0 < slow_factor <= 1`。
启动日志会显示实际使用的倍率。原始目标点频率由数据集 FPS 和 `--speed` 决定；
默认通过 [minimum snap 插值](MINIMUM_SNAP.md) 以 50 Hz 发布关节指令。

```text
目标相对当前反馈的最大偏差 = max_joint_step_rad * slow_factor
```

例如 `--slow-factor 0.1` 时，应用 URDF 限位前，目标相对当前反馈最多偏离 `0.002 rad`。
这不是相邻命令之间的变化量限制。脚本每一帧都会：

1. 检查最新双臂关节和夹爪反馈是否新鲜。
2. 按当前关节反馈限制本帧目标的偏差。
3. 按 URDF 关节上下限裁剪目标。
4. 将夹爪二值状态转换为已标定的开/闭位置。
5. 发布 `BridgeCtlMsg` 和 `GripperCtl`。

`--speed` 只改变数据帧之间的时间间隔，不会解除关节限位或单周期步长限制。例如：

```bash
make replay ARGS="--dataset /home/yanziyu/datasets/fold-towel-93 --episode 0 --max-frames 90 --speed 0.5 --slow-factor 0.1 --motion"
```

这会让原始目标点按 15 FPS 排列；启用插值时仍以 50 Hz 下发，
每个发布周期都执行当前配置的反馈偏差限制和 URDF 限位。

回放单独使用 `config/pi05_deployment.yaml` 中的 `bridge.replay_pd_level: mid`，
两臂 Kp/Kd 恢复为 `initialPD.yaml` 的原有 `mid` 预设，刚度模式为 `2`。
若以后选择 Kd 超过协议上限的预设，回放会将 Kd 截到 50，并通过自定义 PD 模式
`rigidity_mode=-1` 传递，避免中间控制器重新载入超限 Kd。
预设文件不变，模型部署仍使用 `bridge.pd_level: mid` 和 `bridge.rigidity_mode: 2`。
未配置 `replay_pd_level` 的旧配置继续沿用通用档位。0.3 rad 即时跟踪误差保护继续生效。

## 保存角度并生成对比图

每次回放都会记录四组 14 维机械臂关节角：

- `dataset_*`：Parquet 中的原始数据集动作，不经过 `slow_factor` 限幅。
- `command_*`：经过当前反馈限幅、实际放入 `BridgeCtlMsg` 的关节目标。
- `interpolated_*`：minimum-snap 插值后的目标；RViz 预览也显示这一组轨迹。
- `feedback_*`：回放节点在发送该帧时最近收到的 `/dexbotCtl/midMonitor_msg_send` 关节反馈。

`source_knot` 为 1 的行对应原始数据集锚点。PNG 中蓝色散点是原始轨迹，绿色曲线是
minimum-snap 插值，橙色曲线是电机反馈（如果有反馈）。

默认输出到：

```text
outputs/replay_logs/
├── YYYYMMDD_HHMMSS_microseconds.csv
├── YYYYMMDD_HHMMSS_microseconds.png
└── YYYYMMDD_HHMMSS_microseconds.json
```

同次回放的文件使用相同时间戳；episode 和起始帧信息保留在 JSON 内容中。

CSV 还会保存左右夹爪反馈的原始角度字段 `feedback_left_gripper_deg` 和
`feedback_right_gripper_deg`。PNG 包含 14 个子图，每个子图叠加原始锚点、插值目标和电机实际
反馈角度，单位为弧度。`command_*` 仍保留在 CSV 中，用于诊断限幅对原动作的影响。dry-run
也会保存数据集动作；如果没有收到反馈，对应反馈列会是空值，图中会标注 `no feedback`。

可以指定其他输出目录：

```bash
make replay ARGS="--dataset /home/yanziyu/datasets/fold-towel-93 --episode 0 --max-frames 90 --output-dir outputs/replay_logs/episode0"
```

如果只需要数据、不需要绘图：

```bash
make replay ARGS="--dataset /home/yanziyu/datasets/fold-towel-93 --episode 0 --max-frames 90 --no-plot"
```

PNG 对比 Parquet 原始 `action`、minimum-snap 插值和电机反馈，因此可以直接观察回放动作
相对于原始示教动作的变形。CSV 中的 `command_*` 列则是实际发送给控制器的限幅后目标。

## 停止回放

正常完成指定帧数后，脚本会停止定时器并发送保持当前位置的控制消息。

需要提前停止时，在回放终端按 `Ctrl+C`。脚本会尝试发送一次保持当前位置的消息，然后退出。
如果出现异常噪声、动作方向错误、机械干涉或反馈异常，先使用硬件急停，再处理软件进程。

建议停止顺序：

1. 停止回放终端。
2. 停止 `make robot`。
3. 停止 `make arms`。
4. 停止 `make servo`。

软件退出和 `Ctrl+C` 不是硬件急停的替代品。

## 常见问题

### `Dataset replay requires pyarrow`

说明当前机器人 Python 环境缺少 Parquet 读取库。安装后重试：

```bash
.venv-robot/bin/python -m pip install 'pyarrow>=14,<22' 'matplotlib>=3.7,<4'
```

### `No module named rclpy` 或消息类型不存在

不要在 Conda 环境中运行回放。使用 `make replay`，它会自动激活 `.venv-robot` 并加载
`scripts/env.sh`。如果工作区刚修改过消息定义，先运行：

```bash
make build
```

### `Unknown episode`

重新列出 episode，确认编号是 `--list-episodes` 输出的 `episode=` 值，而不是全局帧号：

```bash
make replay ARGS="--dataset /home/yanziyu/datasets/fold-towel-93 --list-episodes"
```

### `Waiting for fresh joint and gripper feedback`

检查控制链是否全部启动，并确认两个反馈话题有数据：

```bash
ros2 topic hz /dexbotCtl/midMonitor_msg_send
ros2 topic hz /gripper/fdb
```

如果频率中断或持续为 0，不要使用 `--motion` 强行跳过反馈检查，应先排查 `servo`、`arms`
和 `robot` 的启动日志。

### 动作很慢或看起来没有到达数据中的目标

先确认不是 `--slow-factor 0.1` 导致单周期步长很小。回放脚本始终会按当前反馈做限幅，
所以数据中的大幅目标可能需要多个控制周期才能接近。不要直接删除限幅；先用更长的短片段
观察，并确保机器人周围安全。

### dry-run 有日志但机器人不动

这是预期行为。dry-run 不发布运动命令。只有在控制链反馈正常、完成短片段检查并确认现场
安全后，才使用显式的 `--motion`。

## 与 PI05/ACT 部署的区别

数据集动作回放不启动 `make pi05`、`make act` 或 `make bridge`。如果要测试“相机观测经过
worker 推理后再控制机器人”的部署链路，应按照 [部署指南](DEPLOYMENT.md) 启动 worker 和
policy bridge；本手册只验证已记录动作经过 ROS 控制链的路径。
