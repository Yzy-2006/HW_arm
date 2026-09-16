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

#include "encos_force_sensor/tare_manager.hpp"

namespace encos_force_sensor
{

void TareManager::begin_tare(int samples)
{
  std::lock_guard<std::mutex> lock(mutex_);
  buffer_.clear();
  target_samples_ = samples > 0 ? samples : 1;
  taring_ = true;
}

bool TareManager::add_sample(const std::array<double, 6> & value)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!taring_) {
    return false;
  }

  buffer_.push_back(value);
  if (static_cast<int>(buffer_.size()) < target_samples_) {
    return false;
  }

  // 累计满目标帧数，取平均作为零点偏移
  std::array<double, 6> sum{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  for (const auto & s : buffer_) {
    for (size_t i = 0; i < 6; ++i) {
      sum[i] += s[i];
    }
  }
  const double n = static_cast<double>(buffer_.size());
  for (size_t i = 0; i < 6; ++i) {
    offset_[i] = sum[i] / n;
  }
  buffer_.clear();
  taring_ = false;
  return true;
}

std::array<double, 6> TareManager::offset() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return offset_;
}

bool TareManager::is_taring() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return taring_;
}

void TareManager::set_offset(const std::array<double, 6> & value)
{
  std::lock_guard<std::mutex> lock(mutex_);
  offset_ = value;
  taring_ = false;
  buffer_.clear();
}

}  // namespace encos_force_sensor
