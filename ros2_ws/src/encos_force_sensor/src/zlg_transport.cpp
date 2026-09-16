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

#include "encos_force_sensor/zlg_transport.hpp"

// 官方头文件（内部 `#include "config.h"` 会相对定位到同目录下的 config.h）
#include "encos_force_sensor/zlg/controlcanfd.h"

#include <ament_index_cpp/get_package_prefix.hpp>

#include <dlfcn.h>

#include <cstdlib>
#include <cstring>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace encos_force_sensor
{

// 与 libcontrolcanfd.so 导出函数严格对应的函数指针类型
using ZCAN_OpenDevice_t = DEVICE_HANDLE (*)(UINT, UINT, UINT);
using ZCAN_CloseDevice_t = UINT (*)(DEVICE_HANDLE);
using ZCAN_SetAbitBaud_t = UINT (*)(DEVICE_HANDLE, UINT, UINT);
using ZCAN_SetDbitBaud_t = UINT (*)(DEVICE_HANDLE, UINT, UINT);
using ZCAN_SetResistanceEnable_t = UINT (*)(DEVICE_HANDLE, UINT, UINT);
using ZCAN_SetCANFDStandard_t = UINT (*)(DEVICE_HANDLE, UINT, UINT);
using ZCAN_InitCAN_t = CHANNEL_HANDLE (*)(DEVICE_HANDLE, UINT, ZCAN_CHANNEL_INIT_CONFIG *);
using ZCAN_StartCAN_t = UINT (*)(CHANNEL_HANDLE);
using ZCAN_ResetCAN_t = UINT (*)(CHANNEL_HANDLE);
using ZCAN_ReceiveFD_t = UINT (*)(CHANNEL_HANDLE, ZCAN_ReceiveFD_Data *, UINT, int);

namespace
{

// 全局唯一的官方库 API 指针集合（dlopen 一次，所有 ZlgTransport 实例共享）
struct ZlgApi
{
  void * lib = nullptr;
  ZCAN_OpenDevice_t open_device = nullptr;
  ZCAN_CloseDevice_t close_device = nullptr;
  ZCAN_SetAbitBaud_t set_abit_baud = nullptr;
  ZCAN_SetDbitBaud_t set_dbit_baud = nullptr;
  ZCAN_SetResistanceEnable_t set_resistance = nullptr;
  ZCAN_SetCANFDStandard_t set_canfd_std = nullptr;
  ZCAN_InitCAN_t init_can = nullptr;
  ZCAN_StartCAN_t start_can = nullptr;
  ZCAN_ResetCAN_t reset_can = nullptr;
  ZCAN_ReceiveFD_t receive_fd = nullptr;

  bool loaded = false;

  bool load()
  {
    if (loaded) {
      return lib != nullptr;
    }
    loaded = true;

    // 候选库路径：环境变量 > 安装前缀 > 系统搜索路径
    std::vector<std::string> paths;
    if (const char * env = std::getenv("ZLG_LIBRARY_PATH")) {
      paths.emplace_back(env);
    }
    try {
      const std::string prefix = ament_index_cpp::get_package_prefix("encos_force_sensor");
      paths.push_back(prefix + "/lib/encos_force_sensor/libcontrolcanfd.so");
    } catch (...) {
      // 安装前缀不存在时忽略，继续尝试其它路径
    }
    paths.emplace_back("libcontrolcanfd.so");

    for (const auto & p : paths) {
      lib = dlopen(p.c_str(), RTLD_LAZY);
      if (lib) {
        break;
      }
    }
    if (!lib) {
      return false;
    }

    open_device = reinterpret_cast<ZCAN_OpenDevice_t>(dlsym(lib, "ZCAN_OpenDevice"));
    close_device = reinterpret_cast<ZCAN_CloseDevice_t>(dlsym(lib, "ZCAN_CloseDevice"));
    set_abit_baud = reinterpret_cast<ZCAN_SetAbitBaud_t>(dlsym(lib, "ZCAN_SetAbitBaud"));
    set_dbit_baud = reinterpret_cast<ZCAN_SetDbitBaud_t>(dlsym(lib, "ZCAN_SetDbitBaud"));
    set_resistance = reinterpret_cast<ZCAN_SetResistanceEnable_t>(dlsym(lib, "ZCAN_SetResistanceEnable"));
    set_canfd_std = reinterpret_cast<ZCAN_SetCANFDStandard_t>(dlsym(lib, "ZCAN_SetCANFDStandard"));
    init_can = reinterpret_cast<ZCAN_InitCAN_t>(dlsym(lib, "ZCAN_InitCAN"));
    start_can = reinterpret_cast<ZCAN_StartCAN_t>(dlsym(lib, "ZCAN_StartCAN"));
    reset_can = reinterpret_cast<ZCAN_ResetCAN_t>(dlsym(lib, "ZCAN_ResetCAN"));
    receive_fd = reinterpret_cast<ZCAN_ReceiveFD_t>(dlsym(lib, "ZCAN_ReceiveFD"));

    return open_device && close_device && set_abit_baud && set_dbit_baud &&
           set_resistance && set_canfd_std && init_can && start_can && reset_can && receive_fd;
  }
};

ZlgApi & api()
{
  static ZlgApi a;
  return a;
}

// 设备句柄共享注册表：同一块 USBCANFD-200U 可开多通道，句柄引用计数
struct SharedDevice
{
  DEVICE_HANDLE dev = INVALID_DEVICE_HANDLE;
  int refcount = 0;
};

std::mutex g_dev_mutex;
std::map<uint64_t, SharedDevice> g_devices;

}  // namespace

struct ZlgTransport::Impl
{
  DEVICE_HANDLE dev = INVALID_DEVICE_HANDLE;
  CHANNEL_HANDLE ch = INVALID_CHANNEL_HANDLE;
  uint64_t device_key = 0;
  bool dev_acquired = false;
};

ZlgTransport::ZlgTransport()
: impl_(std::make_unique<Impl>())
{
}

ZlgTransport::~ZlgTransport()
{
  close();
}

bool ZlgTransport::open(const TransportOpenOptions & opts)
{
  close();

  ZlgApi & a = api();
  if (!a.load()) {
    return false;
  }

  // 通道号（"0"/"1"）
  uint32_t channel = 0;
  try {
    channel = static_cast<uint32_t>(std::stoul(opts.device));
  } catch (...) {
    return false;
  }

  const uint64_t key =
    (static_cast<uint64_t>(opts.zlg_device_type) << 32) | opts.zlg_device_index;

  // 打开/共享设备句柄
  {
    std::lock_guard<std::mutex> lock(g_dev_mutex);
    auto it = g_devices.find(key);
    if (it == g_devices.end()) {
      DEVICE_HANDLE dev = a.open_device(opts.zlg_device_type, opts.zlg_device_index, 0);
      if (dev == INVALID_DEVICE_HANDLE) {
        return false;
      }
      SharedDevice sd;
      sd.dev = dev;
      sd.refcount = 1;
      g_devices[key] = sd;
      impl_->dev = dev;
    } else {
      it->second.refcount++;
      impl_->dev = it->second.dev;
    }
    impl_->device_key = key;
    impl_->dev_acquired = true;
  }

  // 配置通道：仲裁域/数据域波特率、CANFD 标准、终端电阻
  if (a.set_abit_baud(impl_->dev, channel, opts.arbitration_baud) != STATUS_OK) {
    close();
    return false;
  }
  if (a.set_dbit_baud(impl_->dev, channel, opts.data_baud) != STATUS_OK) {
    close();
    return false;
  }
  a.set_canfd_std(impl_->dev, channel, 0);   // 0 = ISO CAN-FD
  a.set_resistance(impl_->dev, channel, 1);  // 使能 120Ω 终端电阻

  // 初始化通道（接收所有帧）
  ZCAN_CHANNEL_INIT_CONFIG cfg;
  std::memset(&cfg, 0, sizeof(cfg));
  cfg.can_type = TYPE_CANFD;
  cfg.canfd.acc_code = 0;
  cfg.canfd.acc_mask = 0xFFFFFFFF;
  cfg.canfd.filter = 0;  // 接收所有帧，上层软件按 ID 过滤
  cfg.canfd.mode = 0;    // 正常模式

  impl_->ch = a.init_can(impl_->dev, channel, &cfg);
  if (impl_->ch == INVALID_CHANNEL_HANDLE) {
    close();
    return false;
  }
  if (a.start_can(impl_->ch) != STATUS_OK) {
    close();
    return false;
  }

  return true;
}

int ZlgTransport::receive(std::vector<CanFrame> & out, int max_frames, int timeout_ms)
{
  if (!impl_ || impl_->ch == INVALID_CHANNEL_HANDLE) {
    return -1;
  }
  if (max_frames <= 0) {
    return 0;
  }

  ZlgApi & a = api();
  std::vector<ZCAN_ReceiveFD_Data> buf(static_cast<size_t>(max_frames));

  const UINT n = a.receive_fd(impl_->ch, buf.data(), static_cast<UINT>(max_frames), timeout_ms);
  if (n == 0 || n == static_cast<UINT>(-1)) {
    return 0;  // 超时或错误
  }

  out.resize(max_frames);
  int got = 0;
  for (UINT i = 0; i < n && got < max_frames; ++i) {
    const canfd_frame & f = buf[i].frame;
    CanFrame & c = out[static_cast<size_t>(got)];
    c.id = f.can_id;
    c.len = f.len;
    c.flags = f.flags;
    const uint8_t copy_len = (f.len <= 64) ? f.len : 64;
    std::memcpy(c.data, f.data, copy_len);
    c.timestamp_us = buf[i].timestamp;
    ++got;
  }
  return got;
}

void ZlgTransport::close()
{
  if (!impl_) {
    return;
  }

  ZlgApi & a = api();
  if (impl_->ch != INVALID_CHANNEL_HANDLE && a.reset_can) {
    a.reset_can(impl_->ch);
  }
  impl_->ch = INVALID_CHANNEL_HANDLE;

  if (impl_->dev_acquired) {
    std::lock_guard<std::mutex> lock(g_dev_mutex);
    auto it = g_devices.find(impl_->device_key);
    if (it != g_devices.end()) {
      it->second.refcount--;
      if (it->second.refcount <= 0) {
        if (a.close_device) {
          a.close_device(it->second.dev);
        }
        g_devices.erase(it);
      }
    }
    impl_->dev_acquired = false;
  }
  impl_->dev = INVALID_DEVICE_HANDLE;
}

}  // namespace encos_force_sensor
