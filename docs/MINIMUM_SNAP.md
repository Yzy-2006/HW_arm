# Minimum Snap 关节插值

[文档索引](README.md) · [控制保护配置](CONTROL_SAFETY.md#current-configuration)

模型部署和数据集回放默认保留每秒 30 个源目标点，并以 50 Hz 发布插值后的
14 关节位置指令。数据集去重后的相邻行仍按 `1 / fps` 排列，不恢复已删除帧的时间。
夹爪不做连续插值，按对应源帧的开/闭状态切换。

## 配置与启动

`config/pi05_deployment.yaml`：

```yaml
bridge:
  rate_hz: 30
  action_rate_hz: 30
  interpolation:
    enabled: true
    publish_rate_hz: 50
  replan_interval_s: 0.5
  max_action_age_s: 1.0
```

`requirements-record.txt` 新增 SciPy。更新环境并构建后，重新启动控制节点：

```bash
.venv-robot/bin/python -m pip install -r requirements-record.txt
make build
```

原有 `make bridge`、`make motion` 和 `make replay` 命令自动使用上述配置。
例如仅检查回放，不发送机器人指令：

```bash
make replay ARGS="--episode 0 --max-frames 90"
```

`controllerConfig.yaml` 的上层 `robot_ctlNode_loop_ctl.TempTime` 和中层
`dual_arm_ctl_node_loop_ctl.TempTime` 均为 `0.02` 秒（50 Hz），底层仍为 `0.001` 秒。
这是共享控制配置，也影响遥操作时的控制循环频率；加载该配置的节点必须重启。
确认安装配置已同步后再启动，以免仍使用旧的 300 Hz 配置。
Python 启动时检查配置中的各层频率是否足以承载发布频率，但不能检测运行中节点
是否已经重新加载配置。50 Hz 是定时器目标值，实际吞吐仍取决于主机和 ROS 调度。

修改 `publish_rate_hz` 时应同时匹配上层、中层的控制频率，且不超过底层频率。
设 `enabled: false` 可以恢复直接发布原始点；缺少整个 `interpolation` 配置时也
沿用旧逻辑。回放 `--speed 0.5` 将源点间隔改为 `1/15` 秒，发布频率仍为 50 Hz。
`--slow-factor` 只缩放目标相对反馈的最大偏差；当前 `speed_only` 关闭这项裁剪，
因此该参数不会降低轨迹速度或步长。

## 轨迹定义

对每个关节求解 `min integral (d^4 q / dt^4)^2 dt`，约束轨迹通过全部给定点，
并固定两端的速度、加速度和 jerk。使用 SciPy 的七次 B 样条带状方程求解，
内部结点为单重结点，满足 C6 连续性。这些条件对应固定端点导数下的
minimum snap 插值，并非每两个点独立停稳的逐段插值。
实现位于 `deployment/minimum_snap.py`；SciPy 接口见
[make_interp_spline](https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.make_interp_spline.html)。

回放先通过独立接近段从反馈接入第一帧，再对所选帧规划；接近段参数与到位确认规则见
[回放流程](DATASET_REPLAY_WORKFLOW.md#独立的首帧接近段)。末尾速度、加速度和 jerk 为零。
正常回放以单调时钟采样，不按回调次数推进源帧；调度延迟不会导致积压后突发补发。
反馈超时检查取决于保护模式，当前 `speed_only` 可以继续使用缓存反馈。
CSV 每个发布周期记录一行，同一个 `dataset_frame` 可以出现多次；`dataset_*`
为该源帧原始值，`command_*` 为插值并经过安全限制的实际指令。

模型新 chunk 从最后成功发布的 q/v/a/jerk 接入，规划和预检查不消耗播放时间，
首次发布时才启动新轨迹的时间轴。首次接入及 hold 后重建从反馈锚定，末端前三阶导数为零。
高频发布独立于 30 Hz 观测定时器，推理在工作线程运行。
`full` 模式仍按观测的实际年龄检查有效性；当前 `speed_only` 关闭观测时效和结果年龄拒绝，
但网络失败、序列耗尽、显式故障仍等待/保持。详见[PI05 时序](PI05_DEPLOYMENT.md#动作与时序)。

## 边界

URDF 限位和相对反馈偏差裁剪由保护模式及各自开关决定，当前 `speed_only` 不执行这些裁剪。
生成轨迹满足上述连续性；启用的安全裁剪或紧急 hold 会打断数学上的导数连续性。
minimum snap 可能在源点之间过冲，规划极值和发布检查的生效范围以[控制安全说明](CONTROL_SAFETY.md)为准。
当前优化不包含关节速度、加速度、力矩的硬约束；在固定的 `1/30` 秒内移动更远，
不能仅靠插值保证动力学可行。需要降低运动速度时，回放使用 `--speed` 延长源点间隔。
末帧的规划速度为零，但结束时仍按现有逻辑 hold 当前反馈，不等待电机追到被裁剪的目标。
