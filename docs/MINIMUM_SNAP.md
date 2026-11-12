# Minimum Snap 关节插值

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
bash scripts/build.sh --packages-select ctl_msgs_hw robot_control_hw dual_arm_control_hw
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
`--slow-factor` 仍只缩放目标相对反馈的最大偏差。

## 轨迹定义

对每个关节求解 `min integral (d^4 q / dt^4)^2 dt`，约束轨迹通过全部给定点，
并固定两端的速度、加速度和 jerk。使用 SciPy 的七次 B 样条带状方程求解，
内部结点为单重结点，满足 C6 连续性。这些条件对应固定端点导数下的
minimum snap 插值，并非每两个点独立停稳的逐段插值。
实现位于 `deployment/minimum_snap.py`；SciPy 接口见
[make_interp_spline](https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.make_interp_spline.html)。

回放对所选帧整体求解，开始时额外用一个源帧周期从当前反馈接入第一帧，末尾速度、
加速度和 jerk 为零。正常回放以单调时钟采样，不按回调次数推进源帧；调度延迟
不会导致积压后突发补发。反馈超时时发布 hold 并暂停，恢复后从反馈重建剩余轨迹。
CSV 每个发布周期记录一行，同一个 `dataset_frame` 可以出现多次；`dataset_*`
为该源帧原始值，`command_*` 为插值并经过安全限制的实际指令。

模型部署保持观测时间对齐。接收新 chunk 时，从旧曲线在当前时刻的位置及前三阶
导数接入尚未过期的后续点；距离当前时刻不足半个源周期的点被跳过，避免极短接入段。
单点返回的模型用一个源周期接入该点。首次接入从反馈开始，末端前三阶导数为零。
高频发布独立于 30 Hz 观测定时器，每次检查反馈、最近观测检查时间和 chunk 有效期。
推理仍在工作线程运行；相机异常、推理失败或过期动作均进入原有 hold 保护。

## 边界

插值保留 URDF 限位及相对反馈的偏差裁剪。没有触发这些限制时，生成轨迹满足上述
连续性；安全裁剪或紧急 hold 会打断数学上的导数连续性，安全限制优先。
minimum snap 可能在源点之间过冲，因此每个高频样本都重新执行安全检查。
当前优化不包含关节速度、加速度、力矩的硬约束；在固定的 `1/30` 秒内移动更远，
不能仅靠插值保证动力学可行。需要降低运动速度时，回放使用 `--speed` 延长源点间隔。
末帧的规划速度为零，但结束时仍按现有逻辑 hold 当前反馈，不等待电机追到被裁剪的目标。
