# 真机数据采集：分终端启动流程

本文件是开始录制毛巾任务数据时的唯一操作入口。控制、采集和 VR 必须在独立终端
启动，便于逐层确认、排障和单独停止。软件退出或 `Ctrl+C` 不是物理急停；发生碰撞、
失控、线缆拉扯或人员风险时，优先按现场规范使用物理急停。

## 0. 开始前

完成以下事项后才可上电并启动控制：

1. 清空双臂可达空间，确认急停可触及；检查双臂、EtherCAT 网关、夹爪和三台相机均已
   按厂家流程上电、接线且无报警。
2. EtherCAT 网卡必须是 `LOWER_UP`，没有 `NO-CARRIER`；最近一次 `make build` 必须成功，
   并已完成其末尾请求的 `sudo` 网卡权限配置。
3. 完成[相机序列号配置](CAMERA_CONFIGURATION.md)，三路画面曾在 `make preview` 中正确显示；
   此时必须已经退出 `make preview`，否则它会占用相机。
4. 完成[双夹爪标定](GRIPPER_CALIBRATION.md)，确认 `/towel/gripper_state` 没有 `-1`。
5. 完成一次[空载 VR 遥操作检查](VR_TELEOP_CHECK.md)，确认手柄左右、关节方向和右 A 回零
   姿态都安全。

在项目根目录运行只读检查；它不会上电、打开 CAN 或占用相机：

```bash
cd /home/yanziyu/projects/HW_arm/towel_task
make check
```

任一项未通过时，不要启动下一层。

## 1. 标准采集：五个终端

打开五个终端，都进入项目根目录。每条 `make` 命令会自行加载机器人环境，不需要重复
`source`。严格按下表顺序启动，并等待前一层的初始化日志稳定后再进行下一层。

| 终端 | 命令 | 通过后再进入下一步的条件 |
|---|---|---|
| 1 | `make servo` | EtherCAT 找到从站并进入 OPERATIONAL；不能出现 `slave count = 0`、`No socket connection` 或“调试模式”。机械臂只可向已确认安全的零位运动。 |
| 2 | `make arms` | 双臂中层节点持续运行，没有初始化或运动学错误。 |
| 3 | `make robot` | 上层节点持续运行；确认启动后的参考姿态和周围空间安全。 |
| 4 | `make record` | 日志显示已就绪并提供 `/towel/record`、`/towel/discard`，同时打开三个 OpenCV 窗口；此时尚未开始 episode。 |
| 5 | `make vr` | 头显连接后进入 **Virtual Reality**，再回终端按 **Enter**；看到“开始发布位姿增量”后才可操作。 |

使用相机手势时，终端 5 改为 `make gesture`，不要同时运行 `make vr`。
先用 `make gesture-check` 检查识别和控制器接口，再退出检查并启动实机入口。
完全张掌驱动对应机械臂，离开张掌手势停止，握拳切换夹爪，左 OK 启停采集，右 OK 回零。
接管完成后先收拢手指至少 0.2 秒再张掌；详见[手势真机接入](GESTURE_ROBOT.md)。

开始 VR 前，双手保持不扣扳机。先做一次最小幅度的左右单臂动作，确认控制链仍然正确，
再开始正式示教。

`make record` 只负责相机和数据写入；它不会自行开始录制。按左 B 开始/结束当前 episode，
按左 A 丢弃当前未完成片段。若采集器日志报告相机或机器人流不新鲜，当前片段会拒绝开始
或中止；修复问题后再录制，不要把失败片段当作训练数据。

## 2. 采集时的 VR 按键

当前映射以 `televuer` 的字段名为准；不同头显手柄上的物理字样可能不同。第一次使用时，
先按[VR 遥操作检查](VR_TELEOP_CHECK.md)完成空载验证，再在正式采集时使用下表。

| 输入 | 采集时的行为 | 使用限制 |
|---|---|---|
| 左扳机 | 只有按住时左臂位置和姿态增量生效。 | 从小幅动作开始；松开后应为零增量。 |
| 右扳机 | 只有按住时右臂位置和姿态增量生效。 | 从小幅动作开始；松开后应为零增量。 |
| 左 squeeze | 切换左夹爪开/闭目标。 | 短按一次；保持按住不会重复切换。 |
| 右 squeeze | 切换右夹爪开/闭目标。 | 短按一次；保持按住不会重复切换。 |
| 左 B（`left_bButton`） | 开始或结束当前 episode。 | 只在终端 4 的采集器已就绪时使用；结束后等待提交完成。 |
| 左 A / X（`left_aButton`） | 丢弃当前未完成 episode。 | 仅在录制中使用；用于误操作、碰撞、断流或人工干预的片段。 |
| 右 A（`right_aButton`） | 触发双臂平滑回到 `zeropos.yaml` 的关节回零目标。 | 正式示教期间不要按；只在已确认回零姿态和路径安全、且不在录制时短按。 |
| 右 B | 未绑定控制或采集功能。 | 不要把它当作操作命令。 |

录制开始前双手都不要扣扳机；先确认两臂静止，再按左 B。夹爪状态为 `-1`、相机断流或
采集器报错时，先停止或丢弃当前片段、修复问题，不要继续示教。

## 3. 相机窗口与正式采集

`make record` 是唯一的相机采集器，同时打开 `Head camera`、`Left wrist camera`、
`Right wrist camera` 三个 OpenCV 窗口；不需要额外终端。按 `q` 或 `Esc` 只关闭窗口，采集器
和正在进行的 episode 会继续运行。`make record-opencv` 只是兼容别名。

`make preview` 也会占用三台相机，只用于开始前检查。它不能和 `make record` 或
`make record-opencv` 同时运行。

## 4. 录制一段有效示教

1. 所有五个终端都稳定后，确认终端 4 的采集器和可选 OpenCV 窗口没有错误，且机器人
   周围没有人或障碍物。
2. 左 B 按一次；确认采集器显示录制开始，再执行一段完整、平稳的叠毛巾动作。
3. 左 B 再按一次；等待采集器确认 episode 已完整提交。不要在写入结束前关闭任一终端。
4. 对每次碰撞、误操作、相机断流或人工干预的片段，按左 A 丢弃；只保留完整、无异常的
   episode。
5. 先录 3–5 段短的无接触动作。每段均有三路图像、双臂数据和有效夹爪状态后，再批量采集。

## 5. 监看与停止

需要额外观察状态时，另开终端运行一个 `make watch` 命令，例如：

```bash
make watch WATCH_TOPIC=/towel/recording
make watch WATCH_TOPIC=/towel/gripper_state
```

正常结束时，先确保当前 episode 已由左 B 结束，然后按反向顺序在各自终端 `Ctrl+C`：

1. `make vr`；
2. `make record` 或 `make record-opencv`；
3. `make robot`；
4. `make arms`；
5. `make servo`，随后按机器人厂家流程断电。

任何时候 `Ctrl+C` 都只会停止当前终端的进程，不能代替物理急停。

硬件按规范停机后，校验本次数据：

```bash
make validate-data
```

使用非默认原始数据目录时，追加相同的 `DATASET_ROOT=/absolute/path/to/raw_data`。仅导出
校验通过的完整 episode；LeRobot 导出和训练环境见[数据集约定](DATASET.md)。
