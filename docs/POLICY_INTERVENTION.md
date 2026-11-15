# 部署转 VR 遥操作与补采

在另一个终端用 VR 丢弃键结束模型部署并接管，随后专门进行遥操作和补采。
PI05 和 ACT 共用这个流程：接管时自动停止 GPU worker 和管理进程，销毁部署 bridge，
复用相机和采集器运行单线程录制会话。补采结束后手动重启部署，不自动加载模型。

## 1. 正常部署

保持 `make servo`、`make arms`、`make robot` 及夹爪所需节点运行。
分别在两个终端运行：

```bash
make pi05
```

```bash
make motion
```

## 2. 启动待命 VR

另开一个终端，在项目目录运行：

```bash
make vr-intervene
```

按终端提示连接 XR 浏览器并按 Enter 开始。此入口仅在确认 VR 控制权后发布机械臂、夹爪、
VR 回零指令；待命及 motion 回零期间不发布这些指令。重新部署时此终端可以继续待命。

不要同时启动普通 `make vr` 或其他夹爪发布器。这里也不需要运行 `make vr-takeover`。

## 3. 丢弃键打断部署

部署运行时，按左手柄的丢弃键：程序字段为 `left_aButton`，部分手柄标为物理 X 键。

1. motion 停止发布模型的双臂和夹爪指令、停止发起新推理，并使旧 chunk 和在途推理结果失效。
2. 底层控制节点锁定接管时的双臂关节反馈位置，拒绝后续模型动作，保留当前夹爪位置。
3. 收到 `VR 接管完成` 后，松开双手扳机、squeeze 和右 A 键，再重新扣动扳机开始遥操作。
4. 同时请求 GPU worker 和管理进程退出并回收进程，随后销毁部署 bridge 和三线程执行器。
5. 原相机与采集器转交单线程执行器；等待 `VR recording-only session ready` 后再按 B 录制。
   切换完成前不允许开始补采，但不阻塞已经取得控制权的 VR 遥操作。

关闭的部署资源包括模型通信连接、推理与管理线程池、bridge 的关节/夹爪订阅、
30 Hz 控制定时器、50 Hz 插值发布定时器、部署 H 键监听和自动恢复逻辑。
继续运行的只有相机接收、采集与写入、VR 模式心跳、必要的伺服安全状态监听；
`make servo`、`make arms`、`make robot`、夹爪节点及 `make vr-intervene` 继续工作。

在新的非零 VR 位移/姿态增量到来前，双臂保持捕获的关节目标；不会先回零。
首次遥操作从接管时的 FK 反馈重新锚定，过滤与手柄位姿基准会重置，
夹爪从最新反馈重新初始化，不沿用部署前的 VR 夹爪目标。

接管通过 ROS 服务执行，生效时间受 ROS 调度和控制周期影响，不是硬实时急停。
伺服故障、关节或 FK 反馈过期、VR 姿态越界会拒绝接管；motion 保持暂停并显示原因，
不会因失败或超时自动恢复部署。回零及控制权切换进行中不重复接受丢弃键接管请求。

## 4. 补采与保存

接管本身不自动开始录制。左 B 开始采集，再按左 B 保存结束；左 A 丢弃当前采集。
处于 VR 模式时，左 A 沿用丢弃录制的含义，丢弃后仍保持 VR 控制。
保存结束也不会自动启动模型。

`make motion` 内置的采集节点复用现有相机对象，保存格式与独立采集器一致，
保留原始数据流、标定配置及 JPEG 质量 90；metadata 的 `source` 为 `vr_policy_intervention`。
不用另开 `make record`、`make preview` 或 `make bridge`，它们会争用同一组相机或录制服务。

默认目录为 `data/raw/interventions`。配置位于 `config/pi05_deployment.yaml`：

```yaml
bridge:
  intervention:
    enabled: true
    dataset_root: data/raw/interventions
    takeover_timeout_s: 5.0
    stop_worker: true
    worker_stop_timeout_s: 15.0
```

管理端口默认是推理端口加 1，即 `8766`；可在同一 YAML 的 `worker.control_port` 指定。
管理连接使用与 worker 相同的 host 和 token。跨机器推理时，两台机器之间须能连接推理和管理两个端口。
若停止失败，录制保持禁用；检查 worker 终端后，再按 B 或丢弃键可重试停止请求。

录制仍使用 `config/recording.yaml` 的采样、时效、丢帧检查与静止帧过滤设置；
采集错误会保留 `.partial`，不会把不完整数据当作已完成 episode。

例如 `Capture frame gap exceeds limit (gap=133.9 ms, limit=100.0 ms)` 表示相邻有效采集帧间隔超限。
诊断中的保存 FPS 会受静止帧过滤影响，不能直接当作相机采集 FPS。
若 writer 队列没有积压，而回调耗时或相机接收间隔升高，需要继续检查 CPU 调度、USB 和相机接收；
退出 worker 能减少推理资源竞争，但不能保证解决所有丢帧原因。此流程不放宽 100 ms 的采集检查。

采集与 bridge 默认通过 `config/recording.yaml` 的 `feedback_rate_hz: 120` 合并高频 ROS 反馈：
每个话题的 Python 接收回调最多 120 Hz、队列只保留最新一条，采集仍为 30 Hz，底层控制频率不变。
快照仅在复制缓存时持锁，完整校验在锁外完成。
`diagnostics_interval_s: 0.0` 默认关闭周期诊断输出，避免慢终端同步日志阻塞采集；
保存和错误时仍打印诊断。需要周期观察时可设为 `2.0`，但它会在采集回调内同步输出。
`capture_stage_ms(last/max)` 分别记录机器人快照、相机快照、入队及诊断日志的耗时。
RealSense 接收线程把 SDK 帧复制为独立只读数组，入队复用其引用，避免采集回调重复复制三路图像。
队列继续持有旧帧引用，新的曝光不会覆盖它；其他提供可变图像的相机适配器仍在入队前复制图像，
数据复制在队列锁外完成，提交时重新检查关闭、取消与容量状态。
如果 `enqueue=170` 而下一次采样 `gap=174`，说明上一帧入队阶段的延迟拉长了采样间隔；
新增 `enqueue_check_lock`、`enqueue_sample_copy`、`enqueue_image_prepare`、`enqueue_commit_lock`
可以继续区分入队锁等待、标签复制、图像准备及提交阶段的延迟。
`receiver_step` 是相邻两次相机接收的帧号差；`device_interval_ms=100`、`receiver_step=3`
表示跨过了约三个 30 Hz 曝光周期，不能单凭 100 ms 就认定相机工作在 10 Hz。
`frame_gap_ms` 从最后保存帧计算，最大值也可能包含主动过滤的静止时段；
`receiver_frame_gaps`、接收间隔最大值则从相机启动开始累计，并不只统计当前 episode。

更新后保存或丢弃当前录制，退出旧 motion 和旧 worker 管理终端，再执行 `make pi05` / `make act`
及 `make motion`；独立采集则重启 `make record`。
无需重新编译底层，也无需重新下载模型。真机仍需验证相机接收间隔和是否继续触发帧间隔检查。

```bash
make validate-data DATASET_ROOT="$PWD/data/raw/interventions"
make export-data DATASET_ROOT="$PWD/data/raw/interventions" OUTPUT="$PWD/data/lerobot/interventions"
```

## 5. 开始下一轮部署

1. 左 B 保存，等待 `Saved ...` 日志；或左 A 丢弃。可以继续补采多段。
2. 在原 `make motion` 终端按 Ctrl+C，退出录制会话并释放三台相机。
3. 在 worker 终端重新执行 `make pi05`（ACT 使用 `make act`），等待模型 listening。
4. 在原 motion 终端执行 `make motion`，重新申请模型控制权并启动部署。

VR 接管后 H 不再回零或恢复模型。需要回零时可在非录制状态使用 VR 右 A 或 robot 终端 H，
等待回零结束后再手动启动 motion。保持 `make vr-intervene` 终端运行即可；
录制会话退出会发布 `stopped`，重新部署发布 `policy`，两种状态下 VR 均停止发布控制指令。

此流程需要更新并重新编译 `robot_control_hw`，然后重启 `make robot`、`make motion`
和 `make vr-intervene`。不要在有未保存录制时重启 motion；退出会中止未保存 episode。
若底层已支持上述接管功能，本次专用补采会话更新不需要重新编译底层。
停止旧 motion 和旧 worker 后，按原模型参数重新执行 `make pi05` / `make act`，再执行 `make motion`；
旧的、没有 shutdown 管理接口的 worker 必须先退出再重启，才能自动关闭。

## 接口与手动流程

`/towel/intervene`（Trigger）立即确认请求是否已受理；只有 `/towel/policy_mode` 的 `vr`
状态才表示底层接管成功。该状态以 0.2 秒间隔发布；VR 超过 0.6 秒未收到状态时停止发布控制。
状态包括 `policy`、`requesting_vr`、`vr`、`home`、`failed`、`stopped`。
底层 `/dexbotCtl/policy_intervene` 负责关节保持与 VR 锚定，`/dexbotCtl/policy_home`
和 `/dexbotCtl/vr_override` 负责回零与恢复。`/towel/record`、`/towel/discard` 沿用采集服务。

独立 VR/独立采集仍可使用：先完全退出 motion 释放相机，运行 `make vr`、`make vr-takeover`，
再启动 `make record`。这条手动流程与上面的持续接管流程不能同时运行。
