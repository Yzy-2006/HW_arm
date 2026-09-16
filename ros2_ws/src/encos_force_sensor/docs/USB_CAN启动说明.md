# USB 转 CAN-FD 模块启动说明

本包配套一个启动脚本 `start_usbcan.sh`，用于加载达妙 **DM-USB2FDCAN**
（gs_usb 类 USB 转 CAN-FD 模块）的内核驱动并配置 CAN-FD 接口。

## 1. 背景

Jetson Thor 的官方内核（`6.8.12-1021-tegra`）**没有编译 gs_usb 驱动**
（`CONFIG_CAN_GS_USB is not set`），因此达妙 USB 转 CAN-FD 模块插入后，
USB 层虽能识别设备（`1d50:606f`），却不会生成 `canX` 接口，导致
`encos_force_sensor` 节点收不到数据。

`start_usbcan.sh` 解决两个问题：

1. 加载自行编译的 `gs_usb.ko` 内核模块；
2. 把 gs_usb 对应的 CAN 接口配置成 CAN-FD（默认 1M 仲裁域 / 5M 数据域）。

> 注意：板载的 `can0~can3` 是 mttcan 控制器，与本模块无关；USB 模块会
> 被命名为 `can4`、`can5`（双通道）。脚本会自动区分二者，不硬编码接口名。

## 2. 一次性准备（编译安装 gs_usb 驱动）

只需执行一次。内核升级后需用新版本内核源码重做本步骤。

```bash
# 2.1 下载与当前内核版本匹配的 gs_usb.c 源码
#     （内核 6.8.12 对应 v6.8.12；升级内核后请改成对应 tag）
mkdir -p ~/gs_usb_build && cd ~/gs_usb_build
curl -sL https://raw.githubusercontent.com/gregkh/linux/v6.8.12/drivers/net/can/usb/gs_usb.c -o gs_usb.c

# 2.2 编写外部模块 Makefile
cat > Makefile <<'EOF'
obj-m := gs_usb.o
KDIR := /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
EOF

# 2.3 编译（需先安装内核头文件；Jetson 镜像通常已自带）
make

# 2.4 安装到系统模块目录并建立依赖索引
sudo mkdir -p /lib/modules/$(uname -r)/extra
sudo cp gs_usb.ko /lib/modules/$(uname -r)/extra/
sudo depmod -a

# 2.5 验证可被 modprobe 找到
modinfo gs_usb | grep -E "filename|vermagic"
```

## 3. 每次启动步骤

```bash
cd /home/nico/catkin_ws_ENCOS/src/encos_force_sensor

# 启动 USB 模块并配置 CAN-FD（默认 1M/5M）
sudo ./start_usbcan.sh

# 如需自定义波特率：sudo ./start_usbcan.sh <仲裁域bps> <数据域bps>
```

脚本会依次：加载 gs_usb 模块 → 识别 gs_usb 接口 → 配置 CAN-FD，并打印结果。

## 4. 验证

```bash
# 4.1 确认接口已 up 且为 CAN-FD（应显示 mtu 72、bitrate 1000000、dbitrate 5000000）
ip -details link show can4

# 4.2 确认能收到传感器帧（left 传感器 = 0x101 的 32 字节 FD 帧）
candump can4

# 4.3 运行传感器节点并查看数据
cd /home/nico/catkin_ws_ENCOS
source install/setup.bash
ros2 launch encos_force_sensor force_sensor.launch.py
ros2 topic echo /force_sensor/left/wrench
```

## 5. 接口与传感器对应关系

| gs_usb 接口 | 传感器 | CAN ID | 话题 |
|---|---|---|---|
| can4（通道1） | left | 0x101 | `/force_sensor/left/wrench` |
| can5（通道2） | right | 0x102 | `/force_sensor/right/wrench` |

对应关系已在 `config/force_sensor.yaml` 中配置（`left→can4`、`right→can5`）。

## 6. 故障排查

- **`modprobe: FATAL: Module gs_usb not found`**：gs_usb.ko 未安装或内核已升级，
  重新执行第 2 步。
- **`错误：未找到 gs_usb 接口`**：USB 模块未插入 / 未被内核识别，用 `lsusb`
  确认存在 `1d50:606f` 设备，用 `dmesg | grep gs_usb` 查看驱动绑定日志。
- **`candump` 收不到帧**：核对传感器供电与接线、终端电阻（120Ω），并确认
  传感器波特率为 1M/5M；若传感器默认其他波特率，用 `./start_usbcan.sh <arb> <data>`
  匹配。
- **重启后失效**：本脚本未设置开机自启，重启后需重新执行第 3 步。
