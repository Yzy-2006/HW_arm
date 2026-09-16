// Copyright 2026 encos
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ENCOS_FORCE_SENSOR__CAN_TRANSPORT_HPP_
#define ENCOS_FORCE_SENSOR__CAN_TRANSPORT_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace encos_force_sensor
{

// CAN ID 位标志（ZLG 与 Linux SocketCAN 采用相同编码：bit31=EFF, bit30=RTR, bit29=ERR）
constexpr uint32_t kCanEffFlag = 0x80000000U;  // 扩展帧标志
constexpr uint32_t kCanRtrFlag = 0x40000000U;  // 远程帧标志
constexpr uint32_t kCanErrFlag = 0x20000000U;  // 错误帧标志
constexpr uint32_t kCanIdMask  = 0x1FFFFFFFU;  // 纯 ID 掩码

/// 从带标志位的原始 can_id 中提取纯 ID
inline uint32_t can_frame_id(uint32_t raw_id) { return raw_id & kCanIdMask; }
/// 是否为扩展帧
inline bool can_frame_is_extended(uint32_t raw_id) { return (raw_id & kCanEffFlag) != 0; }
/// 构造带标志位的 can_id
inline uint32_t make_can_id(uint32_t id, bool extended)
{
  return (id & kCanIdMask) | (extended ? kCanEffFlag : 0);
}

/// 一帧 CAN/CAN-FD 数据（传输无关的统一表示）
struct CanFrame
{
  uint32_t id = 0;               // 原始 can_id（含 EFF/RTR/ERR 标志位）
  uint8_t len = 0;               // 数据域长度（字节）
  uint8_t flags = 0;             // CAN-FD 标志（BRS/ESI 等）
  uint8_t data[64] = {0};        // 数据域
  uint64_t timestamp_us = 0;     // 接收时间戳（微秒，0 表示后端未提供）
};

/// 打开传输所需的参数
struct TransportOpenOptions
{
  std::string device;            // SocketCAN: 网络接口名(如 "can0")；ZLG: 通道号字符串(如 "0")
  uint32_t arbitration_baud = 1000000;  // 仲裁域波特率
  uint32_t data_baud = 5000000;         // 数据域波特率（CAN-FD）
  bool fd_enabled = true;               // 是否启用 CAN-FD
  uint32_t zlg_device_type = 41;        // ZLG: 设备类型（USBCANFD_200U=41）
  uint32_t zlg_device_index = 0;        // ZLG: 设备索引
};

/// CAN 传输抽象接口。
/// 上层节点只依赖该接口，具体后端（SocketCAN / ZLG）可替换。
class ICanTransport
{
public:
  virtual ~ICanTransport() = default;

  /// 打开并配置传输通道
  /// @return true 成功，false 失败（节点应记录错误并跳过该传感器）
  virtual bool open(const TransportOpenOptions & opts) = 0;

  /// 接收若干帧 CAN 数据
  /// @param[out] out        接收到的帧
  /// @param[in]  max_frames 单次最多接收帧数
  /// @param[in]  timeout_ms 等待超时（毫秒，<=0 表示非阻塞）
  /// @return 实际接收帧数；0 表示超时无数据；<0 表示错误
  virtual int receive(std::vector<CanFrame> & out, int max_frames, int timeout_ms) = 0;

  /// 关闭传输通道并释放资源
  virtual void close() = 0;

  /// 后端名称（用于日志/诊断）
  virtual const char * backend_name() const = 0;
};

/// 工厂函数：根据后端名称创建传输实例。
/// 未识别的后端返回 nullptr。
std::unique_ptr<ICanTransport> create_transport(const std::string & backend);

}  // namespace encos_force_sensor

#endif  // ENCOS_FORCE_SENSOR__CAN_TRANSPORT_HPP_
