# 使用方法

## 1. 前置条件

- ROS2 Humble（Ubuntu 22.04）或 ROS2 Jazzy（Ubuntu 24.04 / Jetson AGX Thor）。
- 已安装 `can-utils`（SocketCAN 后端调试用，可选）：
  `sudo apt install -y can-utils`。
- 一块 CAN-FD 转 USB 适配器：达妙 DM-USB2CANFD（SocketCAN 固件）或 ZLG USBCANFD-200U。

## 2. 配置硬件接口

### 2.1 SocketCAN（推荐，x86 与 Thor 通用）

```bash
# 插入达妙 DM-USB2CANFD，确认被内核识别
dmesg | tail -20
ip link show type can          # 应出现 can0

# 配置 CAN-FD：仲裁域 1M，数据域 5M，开启 FD
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
ip -details link show can0     # 应显示 CAN-FD 与两档波特率

# 自测：candump 应看到传感器 1kHz 推送的 32 字节帧
candump can0
```

> 注意：双通道的 DM-USB2CANFD *Dual* 是否被主线 gs_usb 驱动识别为两个 `canX` 接口
> 取决于固件实现，请以 `ip link show type can` 实际输出为准。

### 2.2 ZLG USBCANFD-200U（仅 x86_64）

无需手动配置接口，节点会通过官方库 `libcontrolcanfd.so` 完成
波特率/终端电阻/通道初始化。

## 3. 编辑配置文件

`config/force_sensor.yaml`（构建后位于 `install/encos_force_sensor/share/encos_force_sensor/config/`）。

**传感器数量** = `sensors:` 下的条目数。示例见文件内注释。关键字段：

| 字段 | 含义 | 示例 |
|------|------|------|
| `name`（Map 键） | 传感器名，决定 topic 名 | `left` |
| `transport` | 后端 `socketcan` / `zlg` | `socketcan` |
| `device` | SocketCAN 接口名 / ZLG 通道号 | `can0` 或 `"0"` |
| `frame_id` | 发布消息的 frame_id | `left_gripper_force` |
| `can_id` | 传感器 CAN ID（`0`=不过滤） | `0x101` |
| `id_mask` | ID 过滤掩码 | `0x1FFFFFFF` |
| `is_extended` | 是否扩展帧 | `false` |
| `axis_flip` | 六维符号翻转 `[Fx,Fy,Fz,Mx,My,Mz]` | `[-1,1,1,-1,1,1]` |
| `publish_rate` | 发布频率 Hz（`<=0` 每帧都发） | `1000.0` |
| `tare_samples` | 去皮采样帧数 | `10` |

## 4. 构建

```bash
cd /home/user/catkin_ws_ENCOS
colcon build --packages-select encos_force_sensor
source install/setup.bash
```

## 5. 运行

```bash
ros2 launch encos_force_sensor force_sensor.launch.py
# 或
ros2 run encos_force_sensor force_sensor_node
```

## 6. 查看数据与去皮

```bash
# 查看六维力（force.x/y/z 为力，torque.x/y/z 为力矩）
ros2 topic echo /force_sensor/left/wrench

# 去皮（夹爪空载、无接触时调用）
ros2 service call /force_sensor/left/tare std_srvs/srv/Trigger

# 查看话题/服务列表
ros2 topic list | grep force_sensor
ros2 service list | grep force_sensor
```

## 7. 增加一个传感器

在 `sensors:` 下新增一条即可，例如第三路手腕传感器：

```yaml
      wrist:
        transport: "socketcan"
        device: "can2"
        frame_id: "wrist_force"
        can_id: 0x103
        axis_flip: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
```

重新构建后运行，即可自动出现 `/force_sensor/wrist/wrench` 与 `/force_sensor/wrist/tare`。

## 8. 在 Jetson Thor (jazzy / aarch64) 上运行

- 使用 `transport: "socketcan"`（`zlg` 后端在 aarch64 上不可用，官方库无 aarch64 版本）。
- 其余步骤与 x86 完全一致，构建与运行无额外改动。

## 9. 常见问题

- **`打开传输失败`**：SocketCAN 后端检查接口是否 `up`、名称是否与 `device` 一致；
  ZLG 后端检查设备是否插入、`ZLG_LIBRARY_PATH`（可选）是否指向库。
- **收不到数据但 `candump` 有输出**：核对 `can_id` / `is_extended` 是否与传感器实际 ID 一致。
- **收到"长度不足的帧"告警**：确认传感器按 32 字节数据域发送；检查是否误把经典 CAN 帧当 FD 帧。
- **去皮后数值不为零**：去皮时传感器须空载、无接触；可调大 `tare_samples`。
