# PI05 与 ACT 部署

本文档是当前仓库的快速部署入口。两种策略共用同一个 ROS2 bridge，只更换推理 worker：

```text
RealSense + ROS2 feedback -> bridge -> TCP -> pi05 或 act worker
                                      -> BridgeCtlMsg / gripper command
```

## Makefile 命令

| 命令 | 作用 | 环境 |
| --- | --- | --- |
| `make pi05` | 启动 OpenPI/JAX PI05 worker | `pi05-openpi` |
| `make act` | 启动 LeRobot/PyTorch ACT worker | `lerobot` |
| `make bridge` | 启动通用 policy bridge，dry-run，不发机器人动作 | `.venv-robot` |
| `make motion` | 启动通用 policy bridge 并允许真实动作 | `.venv-robot` |

两个 worker 使用同一个默认端口 `127.0.0.1:8765`，同一时间只启动其中一个。

常用变量：

```bash
make pi05 PI05_MODEL=$HOME/models/pi05-encos-fold-towel-93
make pi05 PI05_CHECKPOINT=step-18000 PI05_PORT=8766
make act ACT_CHECKPOINT=/absolute/path/to/pretrained_model ACT_PORT=8766
make motion PI05_SLOW_FACTOR=0.1
```

`make bridge ARGS=...` 和 `make motion ARGS=...` 支持传递额外 bridge 参数。

## 环境分工

- `.venv-robot`：ROS2 Jazzy、RealSense、EtherCAT/CAN、机器人控制和 bridge。
- `pi05-openpi`：发布的 OpenPI/JAX/Orbax PI05 checkpoint。
- `lerobot`：本地 LeRobot `config.json` checkpoint，例如 ACT。

启动 worker 的终端不要执行 `.venv-robot/bin/activate` 或 `scripts/env.sh`。所有 Makefile
目标会在命令内部准备自己的环境；worker 启动器自动激活指定的 Conda 环境，机器人目标使用 `.venv-robot`
和 `scripts/env.sh`。

## 前置检查

```bash
cd /home/yanziyu/projects/HW_arm/towel_task
make build
make check-control
make camera-list
make preview
```

确认三台相机序列号与 `config/recording.yaml` 一致，退出 `make preview` 后再启动 bridge。
`make build` 在控制源码或消息定义变化后必须重新执行。

## 部署 PI05

默认模型目录是 `$HOME/models/pi05-encos-fold-towel-93`，必须包含 `inference.py` 和
`step-19999/params/`。首次安装环境：

```bash
MODEL=$HOME/models/pi05-encos-fold-towel-93
conda create -n pi05-openpi python=3.11 -y
conda run --no-capture-output -n pi05-openpi \
  python -m pip install -r "$MODEL/requirements-inference.txt"
conda run --no-capture-output -n pi05-openpi \
  python -m pip install chex==0.1.89
```

终端 1 启动 worker：

```bash
cd /home/yanziyu/projects/HW_arm/towel_task
make pi05
```

等待日志出现：

```text
PI05 worker listening on 127.0.0.1:8765
```

PI05 每次推理输出 **50 步动作**，推理积分步数默认为 **10 步**；两者含义不同。
参数写在 `config/pi05_deployment.yaml` 的 `worker.action_horizon` 和 `worker.num_steps`，
`make pi05` 会自动读取。`action_horizon` 用于校验模型原生输出，当前模型需保持 50。
完整参数表和执行时序见 [PI05 动作与时序](PI05_DEPLOYMENT.md#动作与时序)。

## 部署 ACT

ACT 模型应指向包含 `config.json`、`policy.safetensors` 或 `model.safetensors` 的
`pretrained_model` 目录。默认 checkpoint 是：

```text
outputs/act_fold_towel_b12/checkpoints/100000/pretrained_model
```

先确认 `lerobot` 环境能导入 PyTorch：

```bash
conda run --no-capture-output -n lerobot \
  python -c "import sys, torch; print(sys.executable); print(torch.__version__, torch.cuda.is_available())"
```

终端 1 启动 worker：

```bash
cd /home/yanziyu/projects/HW_arm/towel_task
make act
```

如果模型不在默认位置：

```bash
make act ACT_CHECKPOINT=/absolute/path/to/pretrained_model
```

ACT worker 会根据 `config.json` 自动识别模型类型，并读取 checkpoint 自带的图像预处理和
归一化统计。不要只把 `model.safetensors` 文件作为 `--model` 路径。

## 启动 bridge

启动现有控制链。通常分别在三个终端运行：

```bash
make servo
```

```bash
make arms
```

```bash
make robot
```

终端 4 先做 dry-run：

```bash
make bridge
```

dry-run 会读取三路相机、关节反馈和夹爪反馈，请求 worker 推理，但不会发送机械臂和夹爪
控制命令。确认日志持续出现 `action=[...]`，并且没有输入维度、相机键缺失或持续 stale
错误后，再开启动作：

```bash
make motion
```

首次观察动作方向时使用慢放：

```bash
make motion PI05_SLOW_FACTOR=0.1
```

这会把默认的 `0.02 rad` 单周期关节步长限制缩小到 `0.002 rad`。它只改变桥接层的安全
步长，不改变模型输出或关节顺序。

也可以保留旧写法：

```bash
make bridge ARGS="--motion --slow-factor 0.1"
```

## 安全与停止

- 默认 `make bridge` 是 dry-run；只有 `make motion` 或 `--motion` 才会发送动作。
- 第一次真机测试保持 `max_joint_step_rad: 0.02`，每次只运行几秒。
- 现场必须有人掌握硬件急停；软件退出和 `Ctrl+C` 不是物理急停。
- 停止时先按硬件急停或厂商规定的停机操作，再停止 bridge、`robot`、`arms`、`servo`，
  最后停止 worker。

## 回放已采集动作

完整的现场操作步骤见[双臂数据集动作回放流程](DATASET_REPLAY_WORKFLOW.md)。以下保留命令速查。

`fold-towel-93` 是 LeRobot v3 数据集，动作已经是 16 维的绝对关节目标和夹爪二值状态。
回放工具不读取相机，也不启动 PI05 worker；它直接复用同一 `BridgeCtlMsg` / `GripperCtl`
控制链。首次使用先在机器人环境安装 `pyarrow`，然后列出 episode：

```bash
.venv-robot/bin/python -m pip install 'pyarrow>=14,<22'
make replay ARGS="--dataset /home/yanziyu/datasets/fold-towel-93 --list-episodes"
```

先做不发布 ROS 动作的 dry-run，并限制帧数检查数据和关节顺序：

```bash
make replay ARGS="--dataset /home/yanziyu/datasets/fold-towel-93 --episode 0 --max-frames 90"
```

真机回放前必须已经启动 `make servo`、`make arms` 和 `make robot`，现场有人掌握硬件急停。
回放默认使用 `0.1` 的单周期步长倍率；真机模式必须显式给出 `--max-frames`，只有明确需要
下发时才增加 `--motion`，建议先从短片段、低速开始：

```bash
make replay ARGS="--dataset /home/yanziyu/datasets/fold-towel-93 --episode 0 --start-frame 0 --max-frames 90 --slow-factor 0.1 --motion"
```

回放期间如果关节反馈过期，当前帧会暂停；片段结束或按 `Ctrl+C` 时会发送保持当前位置的
控制消息。`--speed` 只改变帧间隔，不能绕过每周期关节步长和 URDF 限位。

动作顺序为：

```text
[left_joint_1..7, left_gripper, right_joint_1..7, right_gripper]
```

bridge 会使用 URDF 关节限位、每周期步长限制和已标定的夹爪 open/closed 端点。
当前部署默认使用 `initialPD.yaml` 的 `mid` 中刚度配置（`rigidity_mode: 2`），与 VR
遥操作默认等级一致；如需修改刚度，应同时修改 `pd_level` 和 `rigidity_mode`。

## 停止 worker 与释放显存

先停止 `make motion` / `make bridge`，再在 `make pi05` / `make act` 终端按
`Ctrl+C`。停止 bridge 仅断开推理连接，worker 会继续保留模型，便于重连。

worker 启动日志会显示 PID 和 Python 路径。启动器转发 `SIGINT` / `SIGTERM`；
若 worker 5 秒后仍未退出，发送 `SIGTERM`，再等 5 秒后强制终止。等待
`Worker process reaped` 日志后再启动下一个模型。端口会在加载模型前预留，避免
重复启动在占用显存后才发现端口冲突。

JAX 的显存池在 worker 存活期间可能保持高占用，进程退出后才会释放 CUDA
上下文。`torch.cuda.empty_cache()` 不会释放 JAX worker 的显存。

如果旧版启动方式留下后台进程，先查看：

```bash
nvidia-smi
pgrep -af '[p]ython.*deployment/pi05_worker.py'
ps -p <PID> -o pid,ppid,stat,args
```

确认 PID 是要停止的 worker 后，执行 `kill -TERM <PID>`，再用 `nvidia-smi` 检查。
如果仍存活且确认不需要该进程，执行 `kill -KILL <PID>`。不要批量终止所有 Python
进程，以免影响机器人控制节点。

## 常见日志

- `Waiting for all cameras`：相机刚启动，尚未收到三路第一帧。
- `Camera frame is stale`：最新帧超过 `config/recording.yaml` 的 `max_camera_age_s`，检查
  USB 带宽、相机占用和其他 RealSense 程序。
- `Waiting for fresh joint and gripper feedback`：反馈话题超过 bridge 的 `state_timeout_s`。
- `Waiting for a current action chunk`：worker 推理尚未返回，bridge 会保持位置；持续出现时
  检查 worker 延迟和 GPU。
- `ModuleNotFoundError: No module named 'torch'`：ACT worker 没有在 `lerobot` 环境中启动，
  执行 `conda run -n lerobot python -c "import torch"` 检查环境。

检查反馈频率：

```bash
ros2 topic hz /dexbotCtl/midMonitor_msg_send
ros2 topic hz /gripper/fdb
```

详细的 OpenPI checkpoint 格式、网络部署和动作时序说明见 [PI05 部署参考](PI05_DEPLOYMENT.md)。
