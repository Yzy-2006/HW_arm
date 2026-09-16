# encos_force_sensor

HKVL-56 六维力/力矩传感器 CAN-FD 驱动节点（ROS2 Humble / Jazzy）。

通过 **CAN-FD** 接收六维力传感器反馈值（6×float32），转换为
`geometry_msgs/WrenchStamped` 发布，并支持配置**任意数量**的传感器。

## 特性

- **双后端传输**：SocketCAN（内核主线，跨 x86_64 / aarch64，推荐）与 ZLG USBCANFD-200U（官方驱动，仅 x86_64）。
- **多传感器**：一个节点管理 N 路传感器，每路独立线程 / topic / 去皮服务。
- **去皮服务**：`std_srvs/Trigger`，采样平均消除安装预紧与零点。
- **坐标变换**：每路可配 `axis_flip`（力/力矩符号翻转）。
- **跨发行版 / 跨架构**：仅依赖标准 ROS2 包，无硬编码路径、无 Python 版本依赖；ZLG 后端按架构条件编译。

## 快速开始

```bash
# 1. 配置 SocketCAN 接口（以 can0 为例）
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on

# 2. 构建
cd /home/user/catkin_ws_ENCOS
colcon build --packages-select encos_force_sensor
source install/setup.bash

# 3. 运行
ros2 launch encos_force_sensor force_sensor.launch.py
# 或
ros2 run encos_force_sensor force_sensor_node

# 4. 查看数据
ros2 topic echo /force_sensor/left/wrench
```

## 文档

- [代码架构描述](docs/ARCHITECTURE.md)
- [使用方法](docs/USAGE.md)

## 目录结构

```
encos_force_sensor/
├── CMakeLists.txt
├── package.xml
├── config/force_sensor.yaml       # 传感器数量与信息配置
├── include/encos_force_sensor/    # 公共头（传输抽象/解码/去皮）
│   └── zlg/                       # ZLG 官方头文件（原样保留）
├── lib/libcontrolcanfd.so         # ZLG 官方库（x86_64）
├── src/                           # 后端实现 + 主节点
├── launch/force_sensor.launch.py
└── docs/                          # 架构与使用文档
```
