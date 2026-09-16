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

#include "encos_force_sensor/ft_decoder.hpp"

#include <cstring>

namespace encos_force_sensor
{

namespace
{
// 传感器以 float32 小端编码，编译期校验当前平台为小端
static_assert(
  (static_cast<uint16_t>(0x1234) & 0xFF) == 0x34,
  "FtDecoder 假定小端平台；若为大端需在此处做字节交换");

// 从 data+offset 读取一个小端 float32
double read_float32_le(const uint8_t * data, size_t offset)
{
  float v = 0.0f;
  std::memcpy(&v, data + offset, sizeof(float));
  return static_cast<double>(v);
}
}  // namespace

bool FtDecoder::decode(const uint8_t * data, uint8_t len, Wrench & out)
{
  // 数据域至少需要 32 字节：6×float32 + 4 字节保留 + uint32 帧序
  if (len < 32) {
    return false;
  }

  out.fx = read_float32_le(data, 0);
  out.fy = read_float32_le(data, 4);
  out.fz = read_float32_le(data, 8);
  out.mx = read_float32_le(data, 12);
  out.my = read_float32_le(data, 16);
  out.mz = read_float32_le(data, 20);
  std::memcpy(&out.seq, data + 28, sizeof(uint32_t));
  return true;
}

}  // namespace encos_force_sensor
