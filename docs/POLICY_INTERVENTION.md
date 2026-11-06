# 部署时 VR 接管与补采

适用于当前 `pi05_bridge.py` 控制路径，包括通过该桥部署的 PI05/ACT worker。
正常接管时模型和 VR 不必重启。左 B 键暂停模型、切换 VR 并开始补采；再次按左 B 键停止 VR、
保存 episode，再以最新观测恢复模型。普通部署和普通 VR 的行为保持原有入口。

## 启动

控制消息和三层控制器均已修改，需要一起构建并重启。不要在机器人正在运动时重启。
底层故障会进入 `fault`，禁止接管和恢复；复位条件、阻尼输出的限制见
[控制安全说明](CONTROL_SAFETY.md)。以下仅是构建命令，不启动硬件：

```bash
source scripts/env.sh
colcon build --base-paths ros2_ws/src --build-base ros2_ws/build --install-base ros2_ws/install --packages-select ctl_msgs_hw dual_arm_control_hw robot_control_hw servo_control_hw
```

按现有流程启动 `make servo`、`make arms`、`make robot` 和模型 worker。在两个独立终端执行：

```bash
# 终端 1：先启动 VR 并连接头显；在提示处按 Enter。
make vr ARGS="--intervention"

# 终端 2：部署桥自动启动独立补采进程。
make motion ARGS="--intervention --intervention-dir data/raw/interventions"
```

不要同时运行普通 `make vr`、`make record`、回放节点或其他动作发布器。
独立补采进程提供原有 `/towel/record`、`/towel/discard` 服务；单独启动采集器会争抢相机和服务。
该进程独占三路相机，负责 ROS 反馈订阅、定时采样及后台写盘；模型桥通过有界共享内存
读取最新图像和原始采集时间戳，不重复打开相机，也不累积图像消息队列。
开始、停止录制通过进程间请求确认，保存完成后才恢复模型。进程退出或录制失败会使干预流程暂停。
退出部署桥时会一并关闭补采进程；未经显式保存的录制保留为 `.partial`。

干预部署初始状态为 `paused`，不会自动运行模型。确认反馈和相机就绪后，在已加载 ROS 环境的终端执行：

```bash
ros2 service call /towel/intervene std_srvs/srv/SetBool '{data: false}'
ros2 topic echo /towel/intervention/state
```

服务返回的是“已接受切换”，以状态话题进入 `policy` 或 `vr` 为完成标志。
请求补采后会先准备目录和元数据，随后等待初始化完成后新收到的机器人反馈及
三路图像；期间状态保持 `arming_vr`，VR 不发布操作指令。
首帧成功入队后才输出 `Capture started: first fresh frame queued` 和
`VR intervention recording started`，进入 `vr`。首帧 `timestamp_s` 为 0，
episode 的 `start_monotonic_s` / `start_wall_time_ns` 对应该首帧；
`requested_*` 单独保存初始化前的请求时间。
`config/recording.yaml` 中 `startup_timeout_s: 3.0` 是初始化完成后的首帧等待期限。
超时保留 `.partial` 并进入暂停，日志指出缺失、过期或尚未更新的数据流。
机器人反馈使用独立回调组和加锁缓存，补采进程自己的多线程执行器允许其在初始化期间继续更新。
首帧之后仍使用原有的数据时效和帧间隔检查。

## 延迟诊断

干预部署自动启动低频资源诊断进程，每秒写入
`outputs/runtime_diagnostics/intervention_<时间戳>_<桥PID>.jsonl`。
启动日志 `Runtime diagnostics:` 显示完整路径和桥/补采 PID；每个 episode 的
`episode.json` 也记录 `runtime_diagnostics` 路径。退出部署时自动结束诊断进程。
不需要新增终端，诊断进程不发布机器人指令，也不修改采样和运动阈值。

文件包含同一单调时钟下的补采状态、采样耗时/相机信息、模型桥和补采线程的 CPU 时间与
运行队列等待，以及当前用户占用 CPU 或读写较多的进程 PID、名称、RSS 和读写速率。
`cpu_percent` 以一个 CPU 核心为 100%，允许大于 100%；`runqueue_wait_ms` 是线程
已经可以运行却等待 CPU 的累计时间增量；线程的 `io_delay_ms` 是块 I/O 等待增量，
进程级 `leader_io_delay_ms` 仅代表其主线程。系统字段包括 CPU 忙碌/I/O 等待占比、可用内存及
CPU、内存、I/O 压力。诊断自身 PID 也会列出，便于评估监测开销。

`sched_schedstats` / `task_delayacct` 表示内核是否启用对应统计；未启用时零等待值不能
作为没有资源竞争的证据。无法读取的字段会缺失或为 `null`，不自动改内核设置。
`interval_s` 是资源采样的实际间隔，若诊断进程也被延迟，不能把这份记录当成精确的 1 Hz 测量。
相机接收间隔和采样耗时的字段解释见 [数据采集说明](DATASET.md)。

## 现场操作

| 操作 | 结果 |
| --- | --- |
| 模型运行时按左 B | 停止模型输出、作废缓存轨迹；控制器从当前末端反馈重新锚定 VR；补采成功启动后进入 `vr` |
| VR 中扣动左右扳机 | 沿用现有增量遥操作 |
| VR 中按左右 squeeze | 沿用现有夹爪开闭控制 |
| VR 中再次按左 B | 控制器先保持当前位置，保存本段补采，随后模型从新观测重新推理 |
| 干预模式的左 A / 右 A | 禁用丢弃和回零快捷键，避免误删除补采或离开失败现场 |
| 右 B | 沿用快/精细灵敏度切换 |

也可用服务替代左 B：`data: true` 请求接管补采，`data: false` 请求保存并恢复模型。
补采失败后状态停在 `paused`，先查看部署日志；修复后显式重新请求接管或恢复。
超时请求未收到最终应答前不接受另一项切换，避免迟到的服务改变控制权。

VR 接管时会清除旧的手柄位姿基准和关节插值缓存。控制器拒绝过期、非有限或超出 VR
工作空间限位的末端反馈，不会为了接管直接回零或将姿态裁到边界。
接管拒绝的详细原因同时输出到 `make robot` 和 `make motion`：
`fk_feedback_missing` 表示尚未收到六维末端反馈，`fk_feedback_stale` 表示反馈超过
0.25 秒未更新，`fk_feedback_nonfinite` 表示坐标含 NaN/Inf，
`vr_position_limit` / `vr_orientation_limit` 表示位置/姿态越界。
日志包含反馈年龄及话题；数值异常还包含 `arm`、`axis`、`value` 和单位，
越界包含 `bounds`。接管使用 `make arms` 持续发布的
`/dexbotCtl/DescPub/Fk_ret_data_pub`（`ctl_msgs_hw/msg/FkDataPub`），与补采 TCP
反馈来源一致，按 `[x, y, z, roll, pitch, yaw]` 映射双臂位姿。
旧绘图话题 `/feedback_get/end_desc_traj_pub` 不再作为接管反馈来源。
VR 权限心跳超过 0.3 秒未刷新时，手柄不再发布增量和夹爪目标。软件切换不是硬件急停，
也不承诺硬实时停机延迟；首次验证应在受控环境中低速进行。

## 数据与导出

默认补采目录为 `data/raw/interventions/`，每次成功接管生成一个独立 episode：

- 三路相机图像、14 维关节反馈及控制目标、TCP、夹爪状态继续使用原始采集格式。
- `episode.json` 标记 `source: policy_intervention`、`control_source: vr`，保留部署配置和标定。
- 关闭相机时效检查时，配置记录 `camera_timing_checks_enabled: false`，原阈值保留；不使用 JSON 不支持的 `inf`。
- 补采固定保留静止帧，便于学习等待、夹爪动作和恢复动作的时序。
- 保存过短的片段、相机/反馈/写盘异常或直接退出时保留 `.partial`，不会被正常导出。
- `Ctrl+C` 不等同于保存；正常结束补采使用左 B 或 `data: false`。

采样回调只复制带时间戳的数据并入队，三路 JPEG 和 `frames.jsonl` 由单个后台线程
按采样顺序写入。`config/recording.yaml` 的 `writer_queue_frames: 16` 限制等待写入的
帧数（每帧含三路图像，另有一帧正在写入），短暂写盘卡顿不会直接阻塞采样。
队列满或写盘失败会中止补采并保留 `.partial`，不会静默丢帧。
保存时停止接收新帧，等待所有已入队帧落盘后才完成 episode 并恢复模型。
`writer_finish_timeout_s: 10.0` 控制等待写盘完成的期限，超时会取消待写帧并保留
`.partial`；旧写盘线程尚未退出时，禁止开始下一段补采。
`max_frame_gap_s` 仍检查实际采样间隔。日志中的 `frames` 是已入队帧数，
`written_frames` 是已完成写入帧数，`writer_queue` 和 `writer_queue_high_water`
表示当前积压及本段录制的最大积压。持续写盘不足时需降低采集负载或改善存储性能。

```bash
make validate-data DATASET_ROOT="$PWD/data/raw/interventions"
make export-data DATASET_ROOT="$PWD/data/raw/interventions" OUTPUT="$PWD/data/lerobot/interventions"
```

导出前查看日志中的保存路径、episode 帧数和采样诊断。成功接管仅说明切换和采集已就绪，
恢复动作是否适合加入训练集仍需按原有数据检查流程筛选。
