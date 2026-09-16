#ifndef CAN_FRAME_FEEDBACK_FRAME_362_H
#define CAN_FRAME_FEEDBACK_FRAME_362_H

#include "can/frame/feedback/base_frame.h"
#include "chassis_control_hw/msg/bms_alarm_warning.hpp"
#include <memory>

namespace agx::nav {

class Frame362 : public BaseFrame
{
public:
  Frame362();
  ~Frame362() = default;
  virtual void parse(const struct can_frame& frame);

private:
  void parse_alarm_warning(uint8_t alarm1, uint8_t alarm2, uint8_t warn1, uint8_t warn2, 
                          chassis_control_hw::msg::BMSAlarmWarning& msg);
  
  rclcpp::Publisher<chassis_control_hw::msg::BMSAlarmWarning>::SharedPtr m_alarm_pub;
  int m_max_count;
  bool m_publisher_initialized{false};
};

} // namespace agx::nav

#endif