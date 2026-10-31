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

回放脚本在真机模式下需要这两个反馈流。关节反馈或夹爪反馈超过配置中的
`state_timeout_s`（默认 0.25 秒）时，当前帧会暂停，不会推进到下一帧。

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
默认通过 [minimum snap 插值](MINIMUM_SNAP.md) 以 300 Hz 发布关节指令。

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

这会让原始目标点按 15 FPS 排列；启用插值时仍以 300 Hz 下发，
每个发布周期都执行当前配置的反馈偏差限制和 URDF 限位。

回放使用 `config/pi05_deployment.yaml` 中的 `bridge.pd_level` 和 `bridge.rigidity_mode`。
当前配置是 `pd_level: mid`、`rigidity_mode: 2`，因此左右两臂都会发送
`initialPD.yaml` 的 `mid` Kp/Kd，以及每个关节的刚度模式 `2`。

## 保存角度并生成对比图

每次回放都会记录三组 14 维机械臂关节角：

- `dataset_*`：Parquet 中的原始数据集动作，不经过 `slow_factor` 限幅。
- `command_*`：经过当前反馈限幅、实际放入 `BridgeCtlMsg` 的关节目标。
- `feedback_*`：回放节点在发送该帧时最近收到的 `/dexbotCtl/midMonitor_msg_send` 关节反馈。

默认输出到：

```text
outputs/replay_logs/
├── episode_0_start_0_YYYYMMDD_HHMMSS.csv
└── episode_0_start_0_YYYYMMDD_HHMMSS.png
```

CSV 还会保存左右夹爪反馈的原始角度字段 `feedback_left_gripper_deg` 和
`feedback_right_gripper_deg`。PNG 包含 14 个子图，每个子图叠加数据集原始动作和电机实际
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

注意：PNG 对比的是 Parquet 原始 `action` 和电机反馈，因此可以直接观察回放动作相对于原始
示教动作的变形。CSV 中的 `command_*` 列则是实际发送给控制器的限幅后目标。

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
