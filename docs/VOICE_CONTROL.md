# 语音控制叠毛巾

`make voice` 持续监听麦克风，无需唤醒词。识别文本包含“毛巾”就启动
`make motion`；包含“停止”就先结束它启动的 motion 进程组，再请求
`/dexbotCtl/keyboard_home`（`std_srvs/srv/Trigger`）。同一句同时包含两词时优先停止。
运行中重复“毛巾”不会重复启动。

回零服务与 **make robot 终端的 H 键**共用 `request_keyboard_home(false)`：
使用现有 `zeropos.yaml` 和平滑轨迹，并保留夹爪回零流程，完成后策略保持锁定，
直到下一次“毛巾”启动的 `make motion` 成功请求控制权。
它与 **make motion 终端的 H 键自动恢复策略**不同。
回零服务成功仅表示轨迹请求已接受，不表示实际已经到位。
回零期间的开始指令会被机器人拒绝，需等回零结束后再说“毛巾”；不排队自动开始。

## 安装

使用与 ROS Jazzy 相同的系统 Python 3.12，单独创建语音环境，避免变更机器人环境：

```bash
cd /home/yanziyu/projects/HW_arm/towel_task
sudo apt-get install python3.12-venv python3.12-dev portaudio19-dev build-essential
/usr/bin/python3.12 -m venv --system-site-packages .venv-voice
.venv-voice/bin/python -m pip install -r requirements-voice.txt
```

使用 CPU int8 的 faster-whisper small，不需要安装 PyTorch。
首次运行会下载模型；离线使用时将 `config/voice_control.yaml` 的
`audio.model_name` 改为已下载的 faster-whisper 模型目录。
原语音项目中的 `/home/lhy/...` 模型目录不适用于本机。

本次新增了 C++ 服务，需重新编译并重启 `make robot`：

```bash
make build
```

## 启动

按现有流程在不同终端启动 `make servo`、`make arms`、`make robot`，等待初始化结束，
启动项目需要的相机/采集与推理 worker（`make record`、`make pi05` 或 `make pi05-thor`）。
随后在新终端启动语音控制，motion 由语音进程负责启动和停止：

```bash
make voice
# 若推理在 Thor 上，选择下面这一条：
make voice ARGS="--worker-host 192.168.3.251"
```

不要同时手动启动另一份 `make motion`、`make motion-thor` 或 VR 控制程序。
语音监督器只负责自己启动的进程，不会结束其他终端的进程。
`PI05_CONFIG`、`PI05_SLOW_FACTOR`、`VENV` 和 `ARGS` 会传给 `make motion`；
语音专用参数通过 `VOICE_ARGS` 传递。

列出麦克风，或先测试识别而不控制硬件：

```bash
make voice VOICE_ARGS="--list-devices"
make voice VOICE_ARGS="--dry-run"
# 无需麦克风、模型或 ROS 的文本流程测试：
.venv-robot/bin/python scripts/run_voice_control.py --dry-run --stdin
```

默认选择系统默认麦克风。可在 YAML 中指定 `input_device_name: UACDemoV1.0`，
或设置 `input_device_index`；数字编号优先。音量阈值可调整 `rms_threshold`。

回零服务失败或超时后保持禁止启动，修复原因后再次说“停止”以重试。
超时前的开始口令会丢弃。Ctrl-C 或语音线程出错会结束语音启动的 motion，
但不会额外触发回零；需要回零时先说“停止”并查看成功日志。

识别复用自相邻 `robot_voice_control` 项目的 `audio.py`、`commands.py`，
副本在 `deployment/voice/`，无需运行原项目或安装其 Humble ROS 包。
保留 VAD、静音分段、置信度和重复幻觉过滤。录音在模型推理期间暂停，
“停止”有语音分段和识别延迟，不是硬件急停；使用现有硬件急停应对紧急情况。
