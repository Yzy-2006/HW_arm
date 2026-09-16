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

#ifndef ENCOS_FORCE_SENSOR__ZLG_TRANSPORT_HPP_
#define ENCOS_FORCE_SENSOR__ZLG_TRANSPORT_HPP_

#include "encos_force_sensor/can_transport.hpp"

#include <memory>

namespace encos_force_sensor
{

/// ZLG USBCANFD-200U 后端（官方驱动 libcontrolcanfd.so）。
///
/// 实现要点：
///   - 采用 dlopen/dlsym 动态加载官方库，避免编译期硬链接 .so；
///   - 官方库为 x86_64 only，本类仅在 x86_64 构建（见 CMakeLists 的 ENCOS_FORCE_SENSOR_HAS_ZLG）；
///   - 同一块设备可开多个通道（CAN1/CAN2），设备句柄在实例间共享、引用计数管理。
///
/// 使用 pimpl 将官方头文件（controlcanfd.h）隔离在 .cpp 中，避免其全局 `using namespace std`
/// 污染本包其余代码。
class ZlgTransport : public ICanTransport
{
public:
  ZlgTransport();
  ~ZlgTransport() override;

  bool open(const TransportOpenOptions & opts) override;
  int receive(std::vector<CanFrame> & out, int max_frames, int timeout_ms) override;
  void close() override;
  const char * backend_name() const override { return "zlg"; }

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace encos_force_sensor

#endif  // ENCOS_FORCE_SENSOR__ZLG_TRANSPORT_HPP_
