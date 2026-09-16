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

#include "encos_force_sensor/can_transport.hpp"

#include "encos_force_sensor/socketcan_transport.hpp"

// ZLG 后端仅在 x86_64 编译（官方 libcontrolcanfd.so 无 aarch64 版本）
#ifdef ENCOS_FORCE_SENSOR_HAS_ZLG
#include "encos_force_sensor/zlg_transport.hpp"
#endif

namespace encos_force_sensor
{

std::unique_ptr<ICanTransport> create_transport(const std::string & backend)
{
  if (backend == "socketcan") {
    return std::make_unique<SocketCanTransport>();
  }

#ifdef ENCOS_FORCE_SENSOR_HAS_ZLG
  if (backend == "zlg") {
    return std::make_unique<ZlgTransport>();
  }
#endif

  // 未识别的后端（例如在 aarch64 上请求 "zlg"）
  return nullptr;
}

}  // namespace encos_force_sensor
