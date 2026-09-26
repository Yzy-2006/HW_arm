# RealSense 相机序列号配置

本任务采集三路 RGB 图像，固定使用以下逻辑名称：

| 配置键 | 物理安装位置 | ROS 图像话题 | LeRobot 图像字段 |
|---|---|---|---|
| `head` | 头部/全局视角 | `/towel/camera/head/image_raw` | `observation.images.head` |
| `left_wrist` | 左臂腕部 | `/towel/camera/left_wrist/image_raw` | `observation.images.left_wrist` |
| `right_wrist` | 右臂腕部 | `/towel/camera/right_wrist/image_raw` | `observation.images.right_wrist` |

逻辑名称决定 ROS 话题、原始图像目录和导出数据集字段；不要仅因相机安装方向变化而改名。
如果物理相机换了位置，应交换对应的**序列号**。

## 1. 准备

这一步只读取 USB 设备信息，不启动机器人、电机、VR 或录制器。将三台 RealSense 接到主机后，在项目根目录执行以下 Make 命令即可；它会自动加载机器人环境。

为避免把序列号和物理位置配错，建议一次只连接一台相机：记录它的序列号和安装位置，再连接下一台。
给线缆或相机贴上 `head`、`left_wrist`、`right_wrist` 标签，后续更换 USB 口也不会影响配置。

## 2. 读取序列号

使用下面的命令列出当前连接的相机：

```bash
make camera-list
```

预期在三台相机都连接时看到三个不同的序列号。若输出为 `发现 0 台`，先不要修改配置；检查供电、USB 线、USB 口、系统 udev 权限以及是否有其他程序独占相机。

`rs-enumerate-devices` 不是本项目的必需工具，当前机器未安装它时使用上面的 Python 命令即可。

## 3. 写入 `recording.yaml`

打开 [../config/recording.yaml](../config/recording.yaml)，在 `cameras` 下填入三台设备的真实序列号：

```yaml
cameras:
  head: "HEAD_CAMERA_SERIAL"
  left_wrist: "LEFT_WRIST_CAMERA_SERIAL"
  right_wrist: "RIGHT_WRIST_CAMERA_SERIAL"
```

所有序列号必须：

- 使用引号包裹，防止 YAML 把纯数字错误解释为数值；
- 非空且两两不同；
- 与实际连接的 RealSense 设备完全一致；
- 不保留 `REPLACE_WITH_RIGHT_WRIST_SERIAL` 之类占位文本。

当前配置中已有头部和一个腕部序列号，但旧腕部相机实际安装在哪一侧尚未确认。先按“读取序列号”步骤核实：若 `260422274993` 在右臂，将它填到 `right_wrist`，并把左臂真实序列号填到 `left_wrist`。

采集器启动时会检查序列号非空且不重复；随后 RealSense 还会要求每一个序列号对应实际可打开的设备。配置中写入不存在的序列号、拔掉任一相机或两台相机使用同一序列号，都会导致采集器启动失败，而不会只采集部分图像。

## 4. 不启动机器人地验证三路画面

修改配置后，先执行依赖检查：

```bash
make check-record
```

然后在机械臂未启动的状态下运行：

```bash
make preview
```

该命令只打开 `Head camera`、`Left wrist camera`、`Right wrist camera` 三个 OpenCV 窗口，
不启动 ROS、伺服、电机、双臂控制、VR 或录制器。按 `q` 或 `Esc` 关闭全部窗口。

逐路确认：画面来自正确位置、方向正确、没有长期断流、分辨率符合 `width`/`height`，且头部、左右腕的物理对应关系正确。
确认完毕后按 `Ctrl+C` 停止 `make preview`；没有按录制键或调用 `/towel/record` 不会创建 episode。

## 5. 更换相机后的数据管理

序列号、分辨率、相机数量或逻辑位置发生变化时，不要将新旧 episode 一起导出。每个完整 episode 都保存当时的相机配置；导出器会拒绝混合不同相机配置的片段。

建议为变化后的采集使用新的 `dataset_root`，例如：

```yaml
dataset_root: data/raw/fold_towel_three_cameras_v1
```

这样可以保留旧数据、简化追溯，也能避免把双相机和三相机样本混入同一个训练数据集。

## 常见问题

| 现象 | 优先检查 |
|---|---|
| 发现 0 台 RealSense | USB 供电/线缆、系统权限、设备是否被其他程序占用。 |
| 只发现 1 或 2 台 | 每台相机是否独立供电；避免三个高带宽设备共用不稳定 USB Hub。 |
| `make preview` 启动即失败 | `recording.yaml` 的序列号是否有占位值、拼写错误、重复，或设备未连接。 |
| 图像话题存在但内容不对 | 按物理安装位置交换 `left_wrist` 与 `right_wrist` 的序列号，不要修改话题名称。 |
| 导出器拒绝混合 episode | 相机配置变化是预期原因；使用新的原始数据目录和新的输出数据集。 |
