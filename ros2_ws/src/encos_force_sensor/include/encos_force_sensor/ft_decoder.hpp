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

#ifndef ENCOS_FORCE_SENSOR__FT_DECODER_HPP_
#define ENCOS_FORCE_SENSOR__FT_DECODER_HPP_

#include <cstdint>

namespace encos_force_sensor
{

/// 六维力/力矩解算结果（SI 单位：N 与 N·m）
struct Wrench
{
  double fx = 0.0;
  double fy = 0.0;
  double fz = 0.0;
  double mx = 0.0;
  double my = 0.0;
  double mz = 0.0;
  uint32_t seq = 0;   // 传感器帧序号（用于丢帧检测）
};

/// HKVL-56 六维力传感器 CAN-FD 帧解码器（传输无关）。
///
/// 数据域布局（32 字节，小端）：
///   [0..3]   FX (float32)
///   [4..7]   FY (float32)
///   [8..11]  FZ (float32)
///   [12..15] MX (float32)
///   [16..19] MY (float32)
///   [20..23] MZ (float32)
///   [24..27] 保留
///   [28..31] 帧序号 (uint32)
class FtDecoder
{
public:
  /// 解析一帧数据
  /// @return true 成功；false 数据长度不足（<32 字节）
  static bool decode(const uint8_t * data, uint8_t len, Wrench & out);
};

}  // namespace encos_force_sensor

#endif  // ENCOS_FORCE_SENSOR__FT_DECODER_HPP_
