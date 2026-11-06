# PI05 部署参考

快速启动命令统一写在 [机器人部署指南](DEPLOYMENT.md)。本文只保留 PI05/OpenPI 格式、
worker 协议和动作时序的细节。

部署时暂停模型、切换 VR 并补采恢复数据，见 [VR 接管与补采](POLICY_INTERVENTION.md)。

控制链现已加入锁存故障保护，重力、惯性、科氏和摩擦补偿默认全部关闭。
运动需要底层安全心跳；故障状态禁止模型和 VR 恢复运动。构建范围、阈值和限制见
[控制安全说明](CONTROL_SAFETY.md)。这些修改尚未经过实机验证。

## Checkpoint 格式

Hugging Face 上的 `pi05-encos-fold-towel-93` 是 OpenPI/JAX/Orbax 格式，不是 LeRobot
PyTorch 格式。因此目录中没有 `config.json` 是正常的，不能使用：

```python
PI05Policy.from_pretrained(model_dir)
```

OpenPI checkpoint 应包含：

```text
pi05-encos-fold-towel-93/
├── inference.py
├── requirements-inference.txt
├── tokenizer/paligemma_tokenizer.model
└── step-19999/params/
    ├── manifest.ocdbt
    ├── _METADATA
    └── _sharding
```

检查路径：

```bash
MODEL=$HOME/models/pi05-encos-fold-towel-93
test -f "$MODEL/inference.py"
test -d "$MODEL/step-19999/params"
```

`step-18000` 和 `step-19999` 都可以作为 `PI05_CHECKPOINT`，默认使用后者。

## 更新到平铺起始 B 模型

2026-11-06 查询发布仓库，新增模型为
[`flat-towel-B-from-pi05-base-step-50000`](https://huggingface.co/hw-arm-towel/pi05-encos-fold-towel-93/tree/main/flat-towel-B-from-pi05-base-step-50000)。
它使用独立的 `inference_flat_towel.py` 和 `assets/encos_flat_train45/norm_stats.json`，
与旧 `step-49999` 的归一化统计不同。不能只更换权重目录后继续使用旧推理入口。
动作仍为 50 步、16 维绝对关节与夹爪目标，训练录制频率为 30 Hz。

先停止运动 bridge，再停止旧 worker，等待 `Worker process reaped`。
在联网的电脑上使用已有 `pi05-openpi` 环境，将新版本下载到独立目录，保留旧版本以便回退：

```bash
conda activate pi05-openpi
python - <<'PY'
from pathlib import Path
from huggingface_hub import snapshot_download

snapshot_download(
    repo_id="hw-arm-towel/pi05-encos-fold-towel-93",
    revision="08b4c7e98deca650c4080be15884ccb0062be5ef",
    local_dir=str(Path.home() / "models/pi05-flat-towel-B-50000"),
    allow_patterns=[
        "flat-towel-B-from-pi05-base-step-50000/**",
        "inference.py", "inference_flat_towel.py", "runtime/**", "tokenizer/**",
        "requirements-inference.txt", "README.md", "LICENSE", "LICENSE_GEMMA.txt", "NOTICE",
    ],
)
PY
```

固定 revision 保证权重和运行文件来自同一次提交，且只下载新 checkpoint 所需文件。
下载失败后重跑同一命令；无需先升级或重新安装现有 JAX 环境。
若目标电脑不联网，下载后通过 `rsync -avh --partial --info=progress2` 将整个
`pi05-flat-towel-B-50000/` 目录传到目标电脑的 `$HOME/models/pi05-flat-towel-B-50000/`。

在当前仓库根目录启动 worker：

```bash
make pi05 PI05_MODEL="$HOME/models/pi05-flat-towel-B-50000" \
  PI05_CHECKPOINT=flat-towel-B-from-pi05-base-step-50000 \
  PI05_ARGS="--inference-entry inference_flat_towel.py"
```

也可将 `config/pi05_deployment.yaml` 的 `worker.inference_entry` 改为
`inference_flat_towel.py`，之后省略该 CLI 参数；切换回旧模型时改回 `inference.py`。
`PI05_MODEL` 和 `PI05_CHECKPOINT` 仍通过 Makefile 变量指定。
启动日志应显示新 checkpoint、`OpenPI inference entry: inference_flat_towel.py`、GPU 设备和 listening。
在已有控制链正常运行的前提下，另一个终端先运行 `make bridge` 做 dry-run，稳定后停止 bridge，
再运行 `make motion PI05_SLOW_FACTOR=0.1` 观察真实动作。

仅测试推理速度时，停止 worker 后执行：

```bash
make pi05-bench PI05_MODEL="$HOME/models/pi05-flat-towel-B-50000" \
  PI05_CHECKPOINT=flat-towel-B-from-pi05-base-step-50000 \
  BENCH_ARGS="--inference-entry inference_flat_towel.py --output outputs/flat-towel-B-benchmark.json"
```

该入口选择已通过本地模拟加载测试；新权重的 GPU 推理和机器人任务效果仍需在目标机器验证。

## OpenPI 环境

worker 不依赖 ROS2，因此不要在机器人 `.venv-robot` 中安装 JAX 或启动 worker：

```bash
MODEL=$HOME/models/pi05-encos-fold-towel-93
conda create -n pi05-openpi python=3.11 -y
conda run --no-capture-output -n pi05-openpi \
  python -m pip install -r "$MODEL/requirements-inference.txt"
conda run --no-capture-output -n pi05-openpi \
  python -m pip install chex==0.1.89
conda run --no-capture-output -n pi05-openpi \
  python -c "import jax; print(jax.__version__, jax.devices())"
```

如果 `jax.devices()` 没有 GPU，先检查 NVIDIA 驱动和 JAX CUDA 版本。`--allow-cpu` 只用于
启动和接口烟测，不适合真机控制。

## Worker 启动

推荐使用 Makefile：

```bash
make pi05
```

等到以下日志出现：

```text
PI05 worker listening on 127.0.0.1:8765
```

等价的手动命令是：

```bash
python3 scripts/run_policy_worker.py --env pi05-openpi -- \
  --config config/pi05_deployment.yaml \
  --model "$HOME/models/pi05-encos-fold-towel-93" \
  --checkpoint step-19999 \
  --host 127.0.0.1 \
  --port 8765
```

worker 启动时不需要相机或机器人。首次 Orbax 恢复和 JAX 编译可能需要几分钟。
停止时按 `Ctrl+C`，等待 `Worker process reaped` 日志确认 worker 进程已退出。

## 相机名称与网络

bridge 发送的默认相机键是：

```text
head, left_wrist, right_wrist
```

如果 checkpoint 的 `inference.py` 使用其他名称，使用 JSON 映射：

```bash
make pi05 PI05_ARGS='--camera-map {"cam_high":"head","cam_left":"left_wrist","cam_right":"right_wrist"}'
```

跨机器部署时：

1. worker 使用推理机的局域网地址监听，例如 `PI05_HOST=192.168.1.20`。
2. bridge 的 `config/pi05_deployment.yaml` 中将 `worker.host` 改为该地址。
3. 两端配置相同的 token，并在 worker 侧传入 `--token`。
4. 只开放局域网 TCP 端口 `8765`，不要暴露到公网。

## 动作与时序

当前 PI05 推理与执行参数：

| 参数 | 当前值 | 含义与配置位置 |
| --- | --- | --- |
| 每次推理输出的动作步数 `action_horizon` | **50 步** | YAML 的 `worker.action_horizon`；用于校验模型原生 chunk 长度，必须与模型 `inference.py` 一致，单改 YAML 不会改变模型结构 |
| 每步动作维度 | 16 | 左臂 7 关节 + 左夹爪 + 右臂 7 关节 + 右夹爪 |
| 每次推理的积分步数 `num_steps` | **10 步** | YAML 的 `worker.num_steps`；Flow Matching 采样计算步数，可用 `--num-steps` 临时覆盖 |
| bridge 观测和规划频率 `rate_hz` | 30 Hz | YAML 的 `bridge.rate_hz`；检查观测、处理推理结果 |
| 插值后指令发布频率 | 300 Hz | `bridge.interpolation.publish_rate_hz`；minimum snap 关节插值 |
| 动作序列时间频率 `action_rate_hz` | 30 Hz | YAML 的 `bridge.action_rate_hz`；用于 chunk 动作的时间索引，默认匹配数据集的 30 FPS |
| 相邻预测动作的时间间隔 | 约 33.3 ms | `1 / action_rate_hz`；50 步对应约 1.67 秒的预测范围，不是每 33.3 ms 运行一次模型 |
| 重规划间隔 `replan_interval_s` | 0.5 秒 | 有效 chunk 存在时，推理请求的最小发起间隔；上一次请求完成后才能发起下一次 |
| 动作最大有效年龄 `max_action_age_s` | 1.0 秒 | 从该次观测请求的时刻计算；超时后保持位置 |
| 每段 chunk 实际执行步数 | 不固定 | 当前没有独立的 `n_action_steps` 参数；取决于推理耗时、重规划间隔和观测有效性 |

`make pi05` 自动读取 `config/pi05_deployment.yaml`（可用 `PI05_CONFIG=...` 指定其他文件）。
修改 `worker.num_steps` 后重启 worker 生效；修改 `bridge` 参数后重启 bridge 生效。
命令行参数优先于 YAML。例如，临时指定 10 步推理积分：

```bash
make pi05 PI05_CHECKPOINT=step-49999 PI05_ARGS="--num-steps 10"
```

该命令仍然每次输出 50 步动作。bridge 会跳过推理期间已经过去的动作时刻，
并在新的有效 chunk 返回后替换旧 chunk，因此不是按顺序执行完全部 50 步后才再推理。

OpenPI worker 返回完整的动作序列；bridge 按观测时间消费序列，而不是按推理完成时间
重新计时。以下为步长限制 0.02 rad 的配置示例，实际以现场 YAML 为准：

```yaml
worker:
  num_steps: 10
  action_horizon: 50
bridge:
  rate_hz: 30
  action_rate_hz: 30
  interpolation:
    enabled: true
    publish_rate_hz: 300
  replan_interval_s: 0.5
  max_action_age_s: 1.0
  max_joint_step_rad: 0.02
  pd_level: mid
  rigidity_mode: 2
```

动作顺序是：

```text
[left_joint_1..7, left_gripper, right_joint_1..7, right_gripper]
```

默认使用 [minimum snap 插值](MINIMUM_SNAP.md)：保持原始 30 Hz 动作时间轴，
将 14 个关节目标插值到 300 Hz；夹爪仍按原始帧的二值状态切换。
新 chunk 从最后一次成功发布的位置、速度、加速度和 jerk 衔接。
规划和预检查完成后，轨迹在首次发布时才开始计时；第一条指令始终取轨迹起点，
不会因为规划耗时跳过开头。夹爪动作使用相同的播放时间轴。
尚未发布的新规划不会替代已发送状态；保持/暂停后重新从反馈构造静止起点。
首次启动、超时恢复或安全裁剪后的重规划从反馈/最后实际指令重新锚定。
插值不延长动作有效期，也不会补发推理期间已过期的动作点。

当前 `bridge.motion_safety.protection_mode: speed_only` 关闭非限速保护。
模型部署设置 `bridge.motion_safety.policy_disable_all_speed_checks: true`，关闭
模型轨迹的速度预检查、相邻指令差分速度和解析速度检查，并随模型指令关闭
servo 的指令速度、位置步进及实际反馈超速检查，双臂全部关节均适用。
暂停保持、切换 VR/回放或模型源指令超过 0.25 秒未更新后，底层恢复原限速配置；
重复转发旧指令不会延长此期限。启动回零仍按原配置限速。
数值有效性、协议范围、显式故障和停止仍保留，开关不会清除已经锁存的故障。
要恢复部署限速，将上述开关设为 `false`，同时将
`bridge.motion_safety.policy_command_speed_check` 设为 `true`，再重启 `make motion`。
若只将后者设为 `false`，仅关闭 Python 模型指令速度检查。
本次 ROS 消息增加了字段，需重新编译并重启 servo、arms、robot 和 motion。
关闭检查不会消除指令发布抖动或目标跳变，也不会使电机超速触发上述保护。
`speed_only` 关闭心跳就绪/超时、反馈和相机时效/时间偏差、位置与加速度、跟踪误差和反馈相对目标偏移检查。
仍需有初始有效反馈和图像；模型结果完成规划后从首次发布开始播放，不再因原 `max_action_age_s`
限制被丢弃。无动作、动作序列结束或网络请求失败时仍等待/保持。
显式停止、底层上报故障、有限数值和协议格式检查保留。

以下启动稳定性和心跳超时规则仅在 `protection_mode: full` 下启用：
`make motion` 启动时先等待安全状态 `ready`、新鲜关节/夹爪反馈和有效相机观测。
就绪前只记录 `Waiting for startup ...`，不发送动作，也不把首次观测尚未检查
误报为插值失败。出现 `Startup inputs ready ...` 后才请求模型动作；从 VR 恢复
模型控制时会重新等待这些输入。
安全心跳接收使用独立回调组，避免与控制计算互斥。首次发布控制指令前，需收到
至少三条 `ready` 心跳并覆盖 0.2 秒，期间消息间隔不能超过 0.2 秒；启动抖动
会重新等待稳定，不会因只收到过一次 `ready` 就切换到运行态超时处理。
启动的初始等待期限为 5 秒，显式底层故障始终立即锁存。
首次控制指令发布后，心跳超时阈值仍是 0.2 秒，暂停/VR 切换不会重置这一保护。
故障日志区分从未收到心跳、心跳超时及非 ready 状态，并附带
`servo_state`、`heartbeat_age_s`、`joint_age_s`、`gripper_age_s` 和
`observation_check_age_s`，便于定位具体延迟来源。
`safety_phase=startup/armed` 可区分启动等待与运行监测。
Ctrl+C/SIGTERM 会先停止并排空回调，最后关闭 ROS 上下文，避免退出时的
`cannot use Destroyable because destruction was requested` 异常。

14 个关节动作是绝对弧度目标。bridge 在发布前使用最新关节反馈执行 URDF 限位和每周期
步长限制；夹爪动作经过二值化后映射到 `grippers.yaml` 的开闭端点。
默认 `pd_level: mid` 与 `rigidity_mode: 2` 使用 VR 遥操作的中刚度配置。

相机或反馈过期、worker 超时或动作序列耗尽时，bridge 会清空旧序列并保持当前位置。恢复
后会用新观测重新推理。默认策略不发布动作，必须显式执行 `make motion`。

## PI05 与 ACT 的区别

| 项目 | PI05 | ACT |
| --- | --- | --- |
| checkpoint | `inference.py` + Orbax `params` | `config.json` + safetensors |
| worker 环境 | `pi05-openpi` | `lerobot` |
| Makefile | `make pi05` | `make act` |
| 图像输入 | OpenPI 原生 HWC `uint8` | LeRobot CHW `float32 [0,1]` |

两者都使用同一个 `bridge` / `motion`，不能同时占用端口 `8765`。
