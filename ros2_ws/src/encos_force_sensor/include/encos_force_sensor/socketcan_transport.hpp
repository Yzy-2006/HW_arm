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

#ifndef ENCOS_FORCE_SENSOR__SOCKETCAN_TRANSPORT_HPP_
#define ENCOS_FORCE_SENSOR__SOCKETCAN_TRANSPORT_HPP_

#include "encos_force_sensor/can_transport.hpp"

namespace encos_force_sensor
{

/// SocketCAN 后端：基于 Linux 内核 SocketCAN（AF_CAN / CAN_RAW）。
/// 优点：内核主线驱动，天然跨 x86_64 与 aarch64（Jetson Thor），无需第三方 .so。
/// 波特率由外部 `ip link set ... type can bitrate ... dbitrate ... fd on` 配置，
/// 本后端只负责打开 socket、绑定接口并收发帧。
class SocketCanTransport : public ICanTransport
{
public:
  SocketCanTransport() = default;
  ~SocketCanTransport() override;

  bool open(const TransportOpenOptions & opts) override;
  int receive(std::vector<CanFrame> & out, int max_frames, int timeout_ms) override;
  void close() override;
  const char * backend_name() const override { return "socketcan"; }

private:
  int sock_ = -1;       // AF_CAN 原始套接字
  bool fd_enabled_ = true;
};

}  // namespace encos_force_sensor

#endif  // ENCOS_FORCE_SENSOR__SOCKETCAN_TRANSPORT_HPP_
