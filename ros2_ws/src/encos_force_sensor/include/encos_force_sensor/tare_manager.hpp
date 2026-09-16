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

#ifndef ENCOS_FORCE_SENSOR__TARE_MANAGER_HPP_
#define ENCOS_FORCE_SENSOR__TARE_MANAGER_HPP_

#include <array>
#include <mutex>
#include <vector>

namespace encos_force_sensor
{

/// 单路传感器的去皮重（零点标定）管理。
///
/// 调用 begin_tare() 后开始采样，add_sample() 喂入连续采样的六维力，
/// 累计满 samples 帧后取平均作为零点偏移并结束去皮。
/// 线程安全：begin_tare() 通常由 ROS 服务回调线程调用，add_sample() 由接收线程调用。
class TareManager
{
public:
  /// 开始去皮：清空采样缓冲，设置采样帧数
  void begin_tare(int samples);

  /// 喂入一帧采样。若处于去皮状态则累积并判断是否完成。
  /// @return true 表示本次调用恰好完成去皮（偏移已更新）
  bool add_sample(const std::array<double, 6> & value);

  /// 当前零点偏移
  std::array<double, 6> offset() const;

  /// 是否正在去皮
  bool is_taring() const;

  /// 直接设置零点偏移（用于外部加载/恢复标定值）
  void set_offset(const std::array<double, 6> & value);

private:
  mutable std::mutex mutex_;
  bool taring_ = false;
  int target_samples_ = 10;
  std::vector<std::array<double, 6>> buffer_;
  std::array<double, 6> offset_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

}  // namespace encos_force_sensor

#endif  // ENCOS_FORCE_SENSOR__TARE_MANAGER_HPP_
