# 真机实验流程：双臂 VR 叠毛巾与数据采集

适用目录：`/home/yanziyu/projects/HW_arm/towel_task`。
本流程假设操作者已了解本机物理急停、上电和 CAN/EtherCAT 的厂家操作要求。
**软件停止或 Ctrl+C 不是物理急停。** 人或设备出现风险时先使用物理急停，确认机械臂和环境安全后再排查。

## 明日流程表

| 阶段 | 要做什么 | 通过条件 | 未通过时 |
|---|---|---|---|
| 0. 准备 | 清理工作台；固定/铺平毛巾；检查两臂、工具、相机、网线、CAN、急停和供电 | 运动空间无人、线缆无拉扯、急停可触及 | 不上电，不启动软件 |
| 1. 软件 | 退出 Conda；如已创建则激活 `.venv-robot`；source 环境；构建并执行只读检查 | `make build`、`make check-control`、`make check-record`、`make check-visualize` 通过 | 修依赖后重试；不要绕过检查 |
| 2. 相机预检 | 在不启动机械臂的情况下运行 `make preview` | RViz 有 head/wrist 实时图像；相机画面方向、曝光、帧率正常 | 停止占用相机的软件，检查序列号和 USB |
| 3. 夹爪标定 | 停止 VR；按 [夹爪标定说明](GRIPPER_CALIBRATION.md) 采集左右开/闭端点 | 两侧 `calibrated: true`，单独按键方向正确 | 保持未标定侧禁用；不要采集双手抓取示教 |
| 4. 硬件上电 | 按机器人厂家规定上电、配置网络/EtherCAT/CAN；工具未接触毛巾 | 无硬件故障；力传感器持续输出 | 急停/断电后检查接线和总线 |
| 5. 力与反馈 | 无载荷时对每路力传感器去皮；检查关节反馈、夹爪状态和 VR 状态 | 数据新鲜、方向符合预期、无异常力 | 不做接触动作；检查传感器话题映射 |
| 6. 空载联调 | 启动全套节点，但先不触碰毛巾；低幅度、低速度分别测试左右臂和夹爪 | 左右触发器、姿态轴、夹爪开闭、回零均正确 | 立即停住；修正标定/映射后重新验证 |
| 7. 试录 | 用短、无接触动作录制并保存一段 | 有完整 episode，无 `.partial`；两路视频和关节数据齐全 | 丢弃片段，先处理时序或相机问题 |
| 8. 正式采集 | 每段只完成一次完整叠毛巾；包含不同毛巾初始姿态和抓取方式 | 完成段数持续增加，失败段不混入数据 | 用左 A 丢弃失败片段 |
| 9. 收尾 | 正常结束当前段；松开触发器；按现场规范停机 | 数据目录只保留准备导出的完整段 | 不把 `.partial` 当训练数据 |
| 10. 数据验收 | 在训练机或导出环境校验数据 | `make validate-data` 成功，再导出 LeRobot | 修复问题后重新采，不修改原始帧伪造数据 |

## 0. 当天开始前的命令

在每个机器人端终端执行：

```bash
cd /home/yanziyu/projects/HW_arm/towel_task
conda deactivate 2>/dev/null || true
if [[ -f .venv-robot/bin/activate ]]; then source .venv-robot/bin/activate; fi
source scripts/env.sh
```

首次或代码变更后：

```bash
make build
make check-control
make check-record
make check-visualize
make test
```

`check-control` 失败通常代表 ROS、Pinocchio/CasADi 或厂商 SDK 未就绪；
`check-record` 失败通常代表 RealSense Python 包或相机运行环境未就绪；
`check-visualize` 失败代表 `towel_bringup` 尚未构建或 `rviz2` 未安装。

## 1. 相机和 RViz 预检

在机械臂未启动的状态下运行：

```bash
make preview
```

这会启动采集器和 RViz；采集器会将其正在读取的图像发布到：

```text
/towel/camera/head/image_raw
/towel/camera/wrist/image_raw
```

不按右 B 或调用 `/towel/record`，不会创建 episode。此模式没有关节反馈，所以 RViz 中机器人模型不动是正常的；重点检查两路图像。

## 2. 夹爪和力传感器

完成 [双夹爪控制与端点标定](GRIPPER_CALIBRATION.md) 后，重新启动 VR。
双夹爪状态话题为：

```bash
ros2 topic echo /towel/gripper_state
```

格式是 `[左反馈, 右反馈, 左目标, 右目标]`，`0` 为开、`1` 为闭、`-1` 为无有效反馈。
正式录制前必须没有 `-1`。

工具不接触任何物体时，对实际存在的每个传感器去皮。例如当前配置含 left/right 时：

```bash
ros2 service call /force_sensor/left/tare std_srvs/srv/Trigger '{}'
ros2 service call /force_sensor/right/tare std_srvs/srv/Trigger '{}'
```

当前历史控制代码的左右力话题接线映射尚未由本目录擅自修正。去皮后先以小幅空载动作核对
`/force_sensor/left/wrench`、`/force_sensor/right/wrench` 的实际工具对应关系和正负方向。

## 3. 启动全套实验

完成物理检查、力传感器去皮和夹爪标定后：

```bash
make experiment VISUALIZE=true
```

启动顺序为：伺服与力传感器 → 2 秒后双臂控制 → 4 秒后上层控制 → 5 秒后采集/RViz → 6 秒后 VR。
RViz 使用独立的 `rviz_` TF 前缀，不会修改或覆盖原控制节点的 TF。

启动后，在另一个已 source 环境的终端做只读观察：

```bash
ros2 topic echo --once /dexbotCtl/midMonitor_msg_send
ros2 topic echo --once /towel/gripper_state
ros2 topic hz /dexbotCtl/midMonitor_msg_send
ros2 topic hz /towel/camera/head/image_raw
```

确认反馈、图像和夹爪状态正常后，XR 设备浏览器打开：

```text
https://<主机IP>:8012?ws=wss://<主机IP>:8012
```

进入 Virtual Reality，回到 VR 终端按 Enter。先空载分别测试两侧手臂与夹爪，确认方向后才接触毛巾。

## 4. 采集单段示教

| 操作 | 结果 |
|---|---|
| 右 B 或 `/towel/record true` | 开始一段新示教 |
| 完成一次连续叠毛巾后再按右 B | 结束并保存完整段 |
| 左 `left_aButton` 或 `/towel/discard` | 丢弃当前段 |
| 控制/相机/文件写入异常 | 片段保留为 `.partial`，不会导出 |

建议先采 3–5 段短演示，检查图像、双臂轨迹和夹爪状态，再开始批量采集。
每段保持单一任务文本；使用多种毛巾位置、褶皱、朝向和抓取起点，但不要把明显失败、被干预或碰撞的过程保存为正样本。

## 5. 结束和数据验收

先结束当前录制，再松开所有触发器，按现场规定安全停机。发生异常时优先物理急停；
不要依赖 Launch 退出过程代替安全机制。

采集结束后：

```bash
make validate-data
```

导出命令应在 `towel-pi05` 训练环境中执行：

```bash
make export-data OUTPUT=/absolute/path/to/towel_wipe_lerobot
```

导出前请按完整 episode 划分训练/验证集；`.partial`、`too_short`、丢弃的片段不参与导出。
