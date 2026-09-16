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

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include "encos_force_sensor/can_transport.hpp"
#include "encos_force_sensor/ft_decoder.hpp"
#include "encos_force_sensor/tare_manager.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace encos_force_sensor
{

namespace
{

/// 从 YAML 节点安全解析 uint32（支持整数与 "0x.."/十进制字符串）
uint32_t parse_u32(const YAML::Node & n, uint32_t dflt)
{
  if (!n || !n.IsScalar()) {
    return dflt;
  }
  try {
    return n.as<uint32_t>();  // 处理整数（含 0x 十六进制字面量）
  } catch (const YAML::Exception &) {
    try {
      return static_cast<uint32_t>(std::stoul(n.as<std::string>(), nullptr, 0));
    } catch (...) {
      return dflt;
    }
  }
}

/// 从 YAML 节点安全解析 double
double parse_double(const YAML::Node & n, double dflt)
{
  if (!n || !n.IsScalar()) {
    return dflt;
  }
  try {
    return n.as<double>();
  } catch (const YAML::Exception &) {
    return dflt;
  }
}

}  // namespace

class ForceSensorNode : public rclcpp::Node
{
public:
  ForceSensorNode()
  : Node("encos_force_sensor_node")
  {
    if (!load_config()) {
      RCLCPP_WARN(get_logger(), "未加载到任何有效传感器配置，节点将空闲运行");
    }
    spawn_sensors();
  }

  ~ForceSensorNode() override
  {
    shutdown();
  }

private:
  using WrenchStamped = geometry_msgs::msg::WrenchStamped;
  using Trigger = std_srvs::srv::Trigger;

  /// 单个传感器的静态配置（来自 YAML）
  struct SensorConfig
  {
    std::string name;
    std::string backend = "socketcan";     // "socketcan" 或 "zlg"
    std::string device;                    // SocketCAN 接口名 / ZLG 通道号
    std::string frame_id;
    uint32_t can_id = 0;                   // 0 = 不过滤
    uint32_t id_mask = kCanIdMask;
    bool is_extended = false;
    std::array<double, 6> axis_flip{1, 1, 1, 1, 1, 1};
    double publish_rate = 1000.0;          // Hz，<=0 表示每帧都发
    int tare_samples = 10;
  };

  /// 单个传感器的运行期实例
  struct SensorRuntime
  {
    SensorConfig cfg;
    std::unique_ptr<ICanTransport> transport;
    rclcpp::Publisher<WrenchStamped>::SharedPtr pub;
    rclcpp::Service<Trigger>::SharedPtr tare_srv;
    TareManager tare;
    std::thread thread;
  };

  // ===== 全局默认参数 =====
  std::string default_backend_ = "socketcan";
  uint32_t arbitration_baud_ = 1000000;
  uint32_t data_baud_ = 5000000;
  bool fd_enabled_ = true;
  uint32_t zlg_device_type_ = 41;
  uint32_t zlg_device_index_ = 0;
  double default_publish_rate_ = 1000.0;
  int default_tare_samples_ = 10;

  std::vector<SensorConfig> sensor_configs_;
  std::vector<std::shared_ptr<SensorRuntime>> sensors_;
  std::atomic<bool> running_{false};

  /// 加载 config/force_sensor.yaml
  bool load_config()
  {
    std::string yaml_path;
    try {
      yaml_path =
        ament_index_cpp::get_package_share_directory("encos_force_sensor") +
        "/config/force_sensor.yaml";
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "定位包 share 目录失败: %s", e.what());
      return false;
    }

    YAML::Node root;
    try {
      root = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception & e) {
      RCLCPP_ERROR(get_logger(), "解析 %s 失败: %s", yaml_path.c_str(), e.what());
      return false;
    }

    const YAML::Node params = root["encos_force_sensor_node"]["ros__parameters"];
    if (!params) {
      RCLCPP_WARN(get_logger(), "配置缺少 encos_force_sensor_node/ros__parameters 段");
      return false;
    }

    // ---- 全局默认 ----
    default_backend_ = params["transport"] ? params["transport"].as<std::string>() : "socketcan";
    arbitration_baud_ = parse_u32(params["arbitration_baud"], 1000000);
    data_baud_ = parse_u32(params["data_baud"], 5000000);
    fd_enabled_ = params["fd_enabled"] ? params["fd_enabled"].as<bool>() : true;
    zlg_device_type_ = parse_u32(params["zlg_device_type"], 41);
    zlg_device_index_ = parse_u32(params["zlg_device_index"], 0);
    default_publish_rate_ = parse_double(params["publish_rate"], 1000.0);
    default_tare_samples_ = static_cast<int>(parse_u32(params["tare_samples"], 10));

    // ---- 传感器列表 ----
    const YAML::Node sensors_node = params["sensors"];
    if (!sensors_node || !sensors_node.IsMap()) {
      RCLCPP_WARN(get_logger(), "配置缺少 sensors 列表（Map）");
      return false;
    }

    for (YAML::const_iterator it = sensors_node.begin(); it != sensors_node.end(); ++it) {
      SensorConfig c;
      c.name = it->first.as<std::string>();
      const YAML::Node n = it->second;

      c.backend = n["transport"] ? n["transport"].as<std::string>() : default_backend_;
      c.device = n["device"] ? n["device"].as<std::string>() : "";
      c.frame_id = n["frame_id"] ? n["frame_id"].as<std::string>() : c.name;
      c.can_id = parse_u32(n["can_id"], 0);
      c.id_mask = parse_u32(n["id_mask"], kCanIdMask);
      c.is_extended = n["is_extended"] ? n["is_extended"].as<bool>() : false;
      c.publish_rate = parse_double(n["publish_rate"], default_publish_rate_);
      c.tare_samples = static_cast<int>(parse_u32(n["tare_samples"], default_tare_samples_));

      if (n["axis_flip"] && n["axis_flip"].IsSequence()) {
        for (size_t j = 0; j < 6 && j < n["axis_flip"].size(); ++j) {
          c.axis_flip[j] = n["axis_flip"][j].as<double>();
        }
      }

      if (c.device.empty()) {
        RCLCPP_ERROR(get_logger(), "传感器 '%s' 缺少 device 配置，已跳过", c.name.c_str());
        continue;
      }

      sensor_configs_.push_back(c);
    }

    RCLCPP_INFO(get_logger(), "已加载 %zu 个传感器配置", sensor_configs_.size());
    return !sensor_configs_.empty();
  }

  /// 为每个传感器创建传输、发布者、去皮服务与接收线程
  void spawn_sensors()
  {
    for (const auto & cfg : sensor_configs_) {
      auto rt = std::make_shared<SensorRuntime>();
      rt->cfg = cfg;

      rt->transport = create_transport(cfg.backend);
      if (!rt->transport) {
        RCLCPP_ERROR(
          get_logger(), "未知传输后端 '%s'（或当前平台不支持），跳过传感器 '%s'",
          cfg.backend.c_str(), cfg.name.c_str());
        continue;
      }

      TransportOpenOptions opts;
      opts.device = cfg.device;
      opts.arbitration_baud = arbitration_baud_;
      opts.data_baud = data_baud_;
      opts.fd_enabled = fd_enabled_;
      opts.zlg_device_type = zlg_device_type_;
      opts.zlg_device_index = zlg_device_index_;

      if (!rt->transport->open(opts)) {
        RCLCPP_ERROR(
          get_logger(), "传感器 '%s' 打开传输失败（backend=%s, device=%s），跳过",
          cfg.name.c_str(), cfg.backend.c_str(), cfg.device.c_str());
        continue;
      }

      rt->pub = create_publisher<WrenchStamped>(
        "/force_sensor/" + cfg.name + "/wrench", 10);

      // 去皮服务（弱引用打破 service→runtime→service 的循环）
      std::weak_ptr<SensorRuntime> weak_rt = rt;
      rt->tare_srv = create_service<Trigger>(
        "/force_sensor/" + cfg.name + "/tare",
        [this, weak_rt](
          const Trigger::Request::SharedPtr,
          Trigger::Response::SharedPtr res)
        {
          auto locked = weak_rt.lock();
          if (!locked) {
            res->success = false;
            res->message = "sensor 实例已销毁";
            return;
          }
          locked->tare.begin_tare(locked->cfg.tare_samples);
          res->success = true;
          res->message = locked->cfg.name + ": 开始去皮，采样 " +
            std::to_string(locked->cfg.tare_samples) + " 帧";
          RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
        });

      sensors_.push_back(rt);
      RCLCPP_INFO(
        get_logger(), "传感器 '%s' 就绪：backend=%s, device=%s, can_id=0x%X, frame_id=%s",
        cfg.name.c_str(), cfg.backend.c_str(), cfg.device.c_str(), cfg.can_id, cfg.frame_id.c_str());
    }

    running_ = true;
    for (auto & rt : sensors_) {
      rt->thread = std::thread([this, rt]() {receive_loop(rt);});
    }
  }

  /// 单个传感器接收循环：收帧 → 过滤 → 解码 → 坐标变换 → 去皮 → 发布
  void receive_loop(const std::shared_ptr<SensorRuntime> & rt)
  {
    const int max_frames = 64;
    std::vector<CanFrame> frames(static_cast<size_t>(max_frames));

    auto last_pub = std::chrono::steady_clock::now();
    std::chrono::microseconds pub_period(0);
    if (rt->cfg.publish_rate > 0.0) {
      pub_period = std::chrono::microseconds(
        static_cast<long long>(1e6 / rt->cfg.publish_rate));
    }

    const uint32_t expected_raw_id = make_can_id(rt->cfg.can_id, rt->cfg.is_extended);
    uint32_t last_seq = 0;
    bool have_seq = false;
    uint64_t dropped_total = 0;

    while (running_) {
      const int n = rt->transport->receive(frames, max_frames, 50);
      if (n <= 0) {
        continue;
      }

      for (int i = 0; i < n; ++i) {
        const CanFrame & f = frames[static_cast<size_t>(i)];

        // ---- CAN ID 过滤 ----
        if (rt->cfg.can_id != 0) {
          if ((f.id & rt->cfg.id_mask) != (expected_raw_id & rt->cfg.id_mask)) {
            continue;
          }
        }

        // ---- 解码 ----
        Wrench w;
        if (!FtDecoder::decode(f.data, f.len, w)) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "[%s] 收到长度不足的帧 len=%u，已忽略", rt->cfg.name.c_str(), f.len);
          continue;
        }

        // ---- 帧序丢帧检测 ----
        if (have_seq) {
          if (w.seq > last_seq) {
            const uint32_t gap = w.seq - last_seq - 1;
            if (gap > 0) {
              dropped_total += gap;
              RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "[%s] 检测到丢帧 %u 帧，累计 %llu 帧",
                rt->cfg.name.c_str(), gap,
                static_cast<unsigned long long>(dropped_total));
            }
          }
        }
        last_seq = w.seq;
        have_seq = true;

        // ---- 坐标变换（axis_flip）----
        const std::array<double, 6> value = {
          w.fx * rt->cfg.axis_flip[0],
          w.fy * rt->cfg.axis_flip[1],
          w.fz * rt->cfg.axis_flip[2],
          w.mx * rt->cfg.axis_flip[3],
          w.my * rt->cfg.axis_flip[4],
          w.mz * rt->cfg.axis_flip[5]};

        // ---- 去皮 ----
        if (rt->tare.is_taring()) {
          if (rt->tare.add_sample(value)) {
            const auto off = rt->tare.offset();
            RCLCPP_INFO(
              get_logger(), "[%s] 去皮完成 offset=[%.3f %.3f %.3f %.3f %.3f %.3f]",
              rt->cfg.name.c_str(), off[0], off[1], off[2], off[3], off[4], off[5]);
          }
          continue;  // 去皮采样期间不发布
        }
        const std::array<double, 6> off = rt->tare.offset();
        const std::array<double, 6> final = {
          value[0] - off[0], value[1] - off[1], value[2] - off[2],
          value[3] - off[3], value[4] - off[4], value[5] - off[5]};

        // ---- 发布节流 ----
        if (rt->cfg.publish_rate > 0.0) {
          const auto now = std::chrono::steady_clock::now();
          if (now - last_pub < pub_period) {
            continue;
          }
          last_pub = now;
        }

        WrenchStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = rt->cfg.frame_id;
        msg.wrench.force.x = final[0];
        msg.wrench.force.y = final[1];
        msg.wrench.force.z = final[2];
        msg.wrench.torque.x = final[3];
        msg.wrench.torque.y = final[4];
        msg.wrench.torque.z = final[5];
        rt->pub->publish(msg);
      }
    }
  }

  void shutdown()
  {
    running_ = false;
    for (auto & rt : sensors_) {
      if (rt->thread.joinable()) {
        rt->thread.join();
      }
    }
    sensors_.clear();  // 释放传输资源，触发 close()
  }
};

}  // namespace encos_force_sensor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<encos_force_sensor::ForceSensorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
