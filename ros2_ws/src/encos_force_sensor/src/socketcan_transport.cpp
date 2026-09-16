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

#include "encos_force_sensor/socketcan_transport.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/sockios.h>

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <poll.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace encos_force_sensor
{

SocketCanTransport::~SocketCanTransport()
{
  close();
}

bool SocketCanTransport::open(const TransportOpenOptions & opts)
{
  close();

  fd_enabled_ = opts.fd_enabled;

  // 1) 创建 CAN_RAW 原始套接字
  sock_ = ::socket(AF_CAN, SOCK_RAW, CAN_RAW);
  if (sock_ < 0) {
    return false;
  }

  // 2) 若启用 CAN-FD，使能 FD 帧接收（否则 8 字节以上帧会被丢弃）
  if (fd_enabled_) {
    const int enable_fd = 1;
    if (::setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd)) < 0) {
      close();
      return false;
    }
  }

  // 3) 根据接口名解析接口索引并绑定
  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", opts.device.c_str());
  if (::ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
    close();
    return false;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(sock_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    close();
    return false;
  }

  return true;
}

int SocketCanTransport::receive(std::vector<CanFrame> & out, int max_frames, int timeout_ms)
{
  if (sock_ < 0) {
    return -1;
  }
  if (max_frames <= 0) {
    return 0;
  }
  if (static_cast<int>(out.size()) < max_frames) {
    out.resize(max_frames);
  }

  // 等待可读（带超时）
  struct pollfd pfd;
  pfd.fd = sock_;
  pfd.events = POLLIN;
  pfd.revents = 0;
  const int poll_rc = ::poll(&pfd, 1, timeout_ms);
  if (poll_rc < 0) {
    return -1;
  }
  if (poll_rc == 0) {
    return 0;  // 超时，无数据
  }
  if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
    return -1;
  }

  int got = 0;
  for (int i = 0; i < max_frames; ++i) {
    struct canfd_frame frame;
    const ssize_t n = ::read(sock_, &frame, sizeof(frame));
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }
      return -1;
    }

    CanFrame & c = out[got];

    // 经典 CAN 帧（CAN_MTU=16）与 CAN-FD 帧（CANFD_MTU=72）返回长度不同
    if (n == CAN_MTU) {
      const struct can_frame * cf = reinterpret_cast<const struct can_frame *>(&frame);
      c.id = cf->can_id;
      c.len = cf->can_dlc;
      c.flags = 0;
      std::memcpy(c.data, cf->data, cf->can_dlc);
    } else if (n == CANFD_MTU) {
      c.id = frame.can_id;
      c.len = frame.len;
      c.flags = frame.flags;
      std::memcpy(c.data, frame.data, frame.len);
    } else {
      continue;  // 未知长度，跳过
    }

    // 接收时间戳（内核收包时刻，微秒）
    struct timespec ts;
    if (::ioctl(sock_, SIOCGSTAMPNS, &ts) == 0) {
      c.timestamp_us = static_cast<uint64_t>(ts.tv_sec) * 1000000ULL +
                       static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
    } else {
      c.timestamp_us = 0;
    }

    ++got;

    // 非阻塞探测：缓冲已空则提前退出，避免下一轮 poll 再空转
    if (::poll(&pfd, 1, 0) <= 0) {
      break;
    }
  }

  return got;
}

void SocketCanTransport::close()
{
  if (sock_ >= 0) {
    ::close(sock_);
    sock_ = -1;
  }
}

}  // namespace encos_force_sensor
