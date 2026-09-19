# 采集数据约定 v2

本文件只定义原始数据、时序、质量规则和 LeRobot 映射。真机当天的操作顺序见[真机实验流程](REAL_ROBOT_EXPERIMENT.md)，三相机和夹爪的专项准备分别见[相机序列号配置](CAMERA_CONFIGURATION.md)和[双夹爪标定](GRIPPER_CALIBRATION.md)。

## 原始片段

```text
<dataset_root>/episode_<time>_<id>/
├── episode.json             任务、相机、配置内容和校验值、状态、帧数
├── frames.jsonl             每行一个采样记录
└── images/{head,left_wrist,right_wrist}/000000.png
```

未完成片段目录以 `.partial` 结尾。丢弃操作只删除当前未提交片段。
采集和保存使用同一 ROS 单线程执行器，没有后台编码线程争抢下一段缓冲区。
图像先落盘、JSONL 后追加；写入失败的片段不会被提交为 complete。

## 原始帧字段

| 字段 | 顺序/单位 | 来源 |
|---|---|---|
| `q` | 左 7 + 右 7；rad | `/dexbotCtl/midMonitor_msg_send` |
| `dq` | 左 7 + 右 7；rad/s | 同上 |
| `joint_target` | 左 7 + 右 7；rad | `/dexbotCtl/ctl_data_pub` 的 `l_position/r_position` |
| `tcp` | 左 XYZ/RPY + 右 XYZ/RPY；m/rad | `/dexbotCtl/DescPub/Fk_ret_data_pub` |
| `tcp_target` | 左 XYZ/RPY + 右 XYZ/RPY；m/rad | `/dexbotCtl/upperMonitor_msg_send` |
| `gripper_state` | [左反馈、右反馈、左目标、右目标]；0=开，1=闭 | `/towel/gripper_state` |
| `force_topic_left/right` | Fx/Fy/Fz/Tx/Ty/Tz；N/Nm；可选 | 同名原力话题 |
| `timestamp_s` | 自片段开始的单调时钟秒数 | 采样端 |
| `stream_timestamps` | 各话题到达时间、存在时的 ROS 消息时间 | 回调端 |
| `camera_timestamps` | 相机帧号、设备时间戳和时钟域、到达时间 | RealSense 接收线程 |

`tcp_target` 是导纳修正前的上层名义目标；`joint_target` 是中层发布的关节目标，
已经过逆解/当前中层控制逻辑，随后仍由伺服层插补。二者不能混为同一个动作标签。
`tcp` 和 `q` 是观测，不能拿它们冒充已下发动作。

## 时间对齐

ROS 各话题和三台 USB 相机没有共享硬件触发。本采集器进行最新样本对齐并检查接收时间，
**不是严格硬件同步**。没有 header 的 FK/夹爪消息只能记录到达时间。
采样目标 30 Hz，但重复相机帧不会作为新帧写入；间隙超限会中止片段并保留原始证据。

导出时按真实采样时间建立固定 FPS 网格，并使用此前最近样本保持，不将缺帧时间压缩。
因此导出视频允许重复图像；`auxiliary.capture_time_s` 保留所选原始帧的时间。
完整的原始消息和相机时钟信息仍保存在 JSONL 中，便于以后重新对齐/导出。

## LeRobot 默认表示

- `observation.state`：16 维，左 7 关节角 + 左夹爪反馈状态 + 右 7 关节角 + 右夹爪反馈状态。
- `action`：16 维，左 7 关节目标角 + 左夹爪目标状态 + 右 7 关节目标角 + 右夹爪目标状态。
- `observation.images.head/left_wrist/right_wrist`：三路 RGB 图像；由 PNG 转成 LeRobot 视频流。
- `task`：每段的自然语言任务描述。
- 额外保留关节速度、实测 TCP、名义 TCP 目标、原始采样时间；没有原始夹爪角度/力矩数组。

关节量单位 rad；第 7、15 维夹爪量为二值 0/1，不是连续开度。
反馈状态按标定后的行程比例判断，使用 0.4/0.6 阈值滞回；初次反馈使用 0.5 阈值。
所以“反馈闭合”表示行程分类结果，并不证明已夹紧毛巾；动作状态表示操作者的开/闭意图。
控制中仍使用角度反馈和 PD 力矩，但这些数值不进入逐帧采集数据。
标定端点保存在独立配置和片段配置快照里，便于复现状态判定。

未标定、未收到反馈或反馈超时时，VR 发布 -1，采集器拒绝开始/中止当前片段。
已经切换闭合目标但夹爪尚未移动完成时，观测可以为 0，动作是 1，不能用同一个按键位替代两者。

此版 schema_version=2，不兼容旧版力矩标签。导出器拒绝 v1；旧数据不能只改维度名当作开合状态。
旧片段如需迁移，必须有对应的端点标定与真实开合目标来源，当前没有自动迁移。

## 数据质量与拆分

每段只包含一次完整示教，任务文本描述实际操作。失败/误操作片段用丢弃服务剔除。
训练/验证按整段示教划分，避免同一段相邻帧同时进入训练和验证。
相机序列号、分辨率或机器人配置变化时分数据集保存，并保留原始文件和配置。

## 校验与导出入口

当前默认 `dataset_root` 是 `data/raw/fold_towel`，对应以下命令：

```bash
make validate-data
make export-data OUTPUT=/absolute/path/to/fold_towel_lerobot
```

如果修改了 `config/recording.yaml` 的 `dataset_root`，两个命令均增加 `DATASET_ROOT=/absolute/path/to/raw_data`。导出需要在安装 LeRobot 的训练环境执行；训练侧的后续步骤见[π0.5 训练说明](../training/README_PI05.md)。
