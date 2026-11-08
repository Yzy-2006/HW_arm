# 部署转 VR 遥操作与补采

使用手动切换：退出模型桥接进程，启动普通 VR，再请求控制权交接。
原来的 `--intervention`、`--intervention-dir` 和 `/towel/intervene` 已移除。

## 1. 正常部署

保持 `make servo`、`make arms`、`make robot` 及夹爪所需节点运行。
分别在两个终端运行：

```bash
make pi05
```

```bash
make motion
```

## 2. 停止部署

在 `make motion` 终端按 `Ctrl+C`，等待程序完全退出并返回命令提示符。
正常退出会请求保持位置、停止推理线程并释放相机。
底层控制权仍保持在桥接侧，直到显式交接到 VR。

不再需要模型时，在 `make pi05` 终端也按 `Ctrl+C`，等待 worker 退出释放资源。
保持伺服、双臂控制、上层控制和夹爪节点运行，不通过重启这些节点交接控制权。

## 3. 启动 VR 并交接

在项目目录启动：

```bash
make vr
```

按终端提示连接 XR 设备并开始发送数据。先松开扳机，在另一个终端运行：

```bash
make vr-takeover
```

确认返回 `success=True`、`VR anchored to feedback` 后，再扣动扳机进行遥操作。
接管以当前反馈的末端姿态为起点，不保证与模型最后一帧目标完全相同。
伺服故障、FK 反馈缺失或过期、姿态超出 VR 配置范围时会拒绝交接；
此时根据服务返回的原因排查，不重复重启底层尝试绕过。

## 4. 独立补采

确认 `make motion` 已完全退出、释放相机，再在另一个终端运行：

```bash
make record ARGS="--dataset-root data/raw/interventions"
```

采集进程不会自动开始录制。左手柄 B 键开始录制，再按 B 保存结束；
左手柄 A 键丢弃当前录制。保存后继续保持 VR 控制，不自动恢复模型。
数据仍使用独立采集器的原始 episode 格式，不再自动附加部署配置和模型接管元数据。

```bash
make validate-data DATASET_ROOT="$PWD/data/raw/interventions"
make export-data DATASET_ROOT="$PWD/data/raw/interventions" OUTPUT="$PWD/data/lerobot/interventions"
```

## 5. 需要重新部署时

先保存采集，退出 `make record` 和 `make vr`，避免相机占用及夹爪命令冲突。
在项目目录的另一终端恢复桥接控制权：

```bash
source scripts/env.sh
ros2 service call /dexbotCtl/vr_override std_srvs/srv/SetBool "{data: false}"
```

确认成功后，按需重启 `make pi05`，再启动 `make motion`。
