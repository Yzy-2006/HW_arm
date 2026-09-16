# 代码架构描述

`encos_force_sensor` 采用「**传输抽象层（HAL）+ 多传感器实例**」的分层设计，
核心目标是：**同一套解码与发布逻辑，在 SocketCAN 与 ZLG 官方驱动两条传输路径上复用**，
从而同时满足 humble(x86_64) 与 jazzy(aarch64/Jetson Thor) 两套平台。

## 1. 总体分层

```
┌──────────────────────────────────────────────────────┐
│  应用层（本包不含，可另建节点订阅 wrench topic）         │
├──────────────────────────────────────────────────────┤
│  force_sensor_node.cpp     主节点                     │
│    ├─ load_config()        读 YAML → N 个 SensorConfig │
│    ├─ spawn_sensors()      建传输/发布者/服务/线程      │
│    └─ receive_loop()       每传感器独立接收线程         │
│        收帧 → ID过滤 → 解码 → axis_flip → 去皮 → 发布  │
├──────────────────────────────────────────────────────┤
│  FtDecoder       六维力解码（传输无关）                  │
│  TareManager     去皮重（N 路，线程安全）               │
├──────────────────────────────────────────────────────┤
│  ICanTransport   抽象接口                              │
│    ├─ SocketCanTransport   内核 SocketCAN（跨平台）      │
│    └─ ZlgTransport         官方 libcontrolcanfd.so（x86）│
├──────────────────────────────────────────────────────┤
│  硬件：达妙 DM-USB2CANFD / ZLG USBCANFD-200U          │
└──────────────────────────────────────────────────────┘
```

## 2. 关键抽象：`ICanTransport`

定义在 `include/encos_force_sensor/can_transport.hpp`：

- `CanFrame`：传输无关的统一帧结构（id / len / flags / data[64] / timestamp_us）。
- `TransportOpenOptions`：打开参数（device、波特率、fd 开关、ZLG 设备类型/索引）。
- `ICanTransport`：`open() / receive() / close() / backend_name()` 四个纯虚接口。
- `create_transport(backend)`：工厂函数，按后端名构造实例；aarch64 上请求 `"zlg"`
  时因宏 `ENCOS_FORCE_SENSOR_HAS_ZLG` 未定义而返回 `nullptr`。

**设计要点**：ZLG 的 `canfd_frame` 与 Linux `struct canfd_frame` 内存布局一致
（`can_id + len + flags + 2×res + data[64]`），CAN ID 标志位编码也一致
（`EFF=bit31 / RTR=bit30 / ERR=bit29`）。因此 `FtDecoder` 与节点主逻辑完全不感知后端差异。

## 3. 两个后端

### 3.1 `SocketCanTransport`（跨平台，默认推荐）

- `socket(AF_CAN, SOCK_RAW, CAN_RAW)` + `setsockopt(CAN_RAW_FD_FRAMES)` 使能 FD。
- `ioctl(SIOCGIFINDEX)` + `bind(sockaddr_can)` 绑定接口名（如 `can0`）。
- `receive()`：`poll()` 带超时等待，`read()` 按 `CAN_MTU/CANFD_MTU` 区分经典帧与 FD 帧，
  `ioctl(SIOCGSTAMPNS)` 取接收时间戳。
- **波特率不在此设置**，由外部 `ip link set ... type can bitrate ... dbitrate ... fd on` 配置
  （需要 root，见 USAGE.md）。这是 SocketCAN 的标准做法。

### 3.2 `ZlgTransport`（x86_64 专用）

- 采用 `dlopen/dlsym` 动态加载官方 `libcontrolcanfd.so`，不硬链接。
- `ZlgApi` 单例：`dlopen` 一次，缓存全部 `ZCAN_*` 函数指针，供所有实例共享。
- 设备句柄共享：`std::map<uint64_t, SharedDevice>` 以 `(device_type<<32|device_index)`
  为键做引用计数，使一块 USBCANFD-200U 的两个通道可分别给两个传感器使用。
- 调用序列（官方驱动标准流程）：
  `OpenDevice → SetAbitBaud → SetDbitBaud → SetCANFDStandard → SetResistanceEnable → InitCAN → StartCAN → ReceiveFD`。
- **pimpl 隔离**：`ZlgTransport::Impl` 定义在 `.cpp`，避免官方头文件 `controlcanfd.h` 里
  全局 `using namespace std` 污染本包其余代码。
- **平台限制**：官方库为 x86_64 only。CMake 仅当 `CMAKE_SYSTEM_PROCESSOR == x86_64`
  时编译该后端并 `install` 库文件；aarch64 上该后端自动缺席。

## 4. 解码与去皮

### `FtDecoder`（`ft_decoder.hpp/.cpp`）

32 字节数据域布局（小端）：

| 偏移 | 字段 | 类型 |
|------|------|------|
| 0..3   | FX | float32 |
| 4..7   | FY | float32 |
| 8..11  | FZ | float32 |
| 12..15 | MX | float32 |
| 16..19 | MY | float32 |
| 20..23 | MZ | float32 |
| 24..27 | 保留 | — |
| 28..31 | 帧序号 | uint32 |

`decode()` 用 `memcpy` 读小端 float，`static_assert` 校验平台为小端；长度 < 32 返回失败。

### `TareManager`（`tare_manager.hpp/.cpp`）

- `begin_tare(samples)` 清空缓冲、进入去皮态。
- `add_sample(value)` 累积采样，满 `samples` 帧取平均作为零点偏移并退出去皮态。
- 内部 `std::mutex` 保证服务回调线程（写 begin）与接收线程（写 sample）安全。

## 5. 主节点 `force_sensor_node.cpp`

- `load_config()`：从 `share/encos_force_sensor/config/force_sensor.yaml` 读取
  全局默认参数与 `sensors` Map，构造 `std::vector<SensorConfig>`。
- `spawn_sensors()`：为每个 `SensorConfig` 创建传输、`WrenchStamped` 发布者、
  `Trigger` 去皮服务与接收线程；任一步失败则记录错误并跳过该传感器（不拖垮整节点）。
- `receive_loop(rt)`：每传感器独立线程
  1. `receive()` 收帧；
  2. **CAN ID 过滤**（`can_id != 0` 时按 `id_mask` 比对 `expected_raw_id`）；
  3. `FtDecoder::decode()` 解码；
  4. **帧序丢帧检测**（`seq` 跳变计数告警）；
  5. `axis_flip` 坐标变换；
  6. 去皮：`is_taring()` 期间只采样不发布，完成后减去 `offset`；
  7. 按 `publish_rate` 节流发布 `WrenchStamped`。
- 生命周期：析构时置 `running_=false` → 逐线程 `join` → 清空实例触发传输 `close()`。

## 6. 多传感器与话题约定

- 每路发布 `/force_sensor/<name>/wrench`，`<name>` 即 YAML 中 Map 的键。
- 每路提供去皮服务 `/force_sensor/<name>/tare`。
- 坐标系语义：`+X` 夹爪闭合方向、`+Y` 向上、`+Z` 向前（与参考节点一致），
  可用 `axis_flip` 适配不同安装姿态。

## 7. 可移植性设计（humble ↔ jazzy / x86 ↔ aarch64）

| 关注点 | 做法 |
|--------|------|
| 后端选择 | `transport` 参数二选一；Thor 用 `socketcan` |
| ZLG 库 | 仅 x86_64 编译/安装，aarch64 自动编译为 SocketCAN-only |
| 头文件 | SocketCAN 用 `<linux/can.h>`（内核头，两平台通用） |
| 依赖 | 仅 `rclcpp/geometry_msgs/std_srvs/yaml-cpp/ament_index_cpp`，无发行版差异 |
| 字节序 | `static_assert` 小端；x86 与 aarch64 均小端 |
| 构建 | C++17 + ament_cmake，`ament_target_dependencies` 声明依赖，无裸 `link_directories` |
