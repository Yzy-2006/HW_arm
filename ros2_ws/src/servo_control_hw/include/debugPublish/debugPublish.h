#ifndef SERVO_CONTROL_HW__DEBUGPUBLISH__DEBUGPUBLISH_H_
#define SERVO_CONTROL_HW__DEBUGPUBLISH__DEBUGPUBLISH_H_

#include <iostream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ctl_msgs_hw/msg/dbg_information.hpp"

class debugMsgPub{
    private:
        rclcpp::Node* node_ptr_;

        //初始化发送的消息类型
        rclcpp::Publisher<ctl_msgs_hw::msg::DbgInformation>::SharedPtr debug_msg_send;
        rclcpp::Publisher<ctl_msgs_hw::msg::DbgInformation>::SharedPtr debug_msg_send_2;
        rclcpp::Publisher<ctl_msgs_hw::msg::DbgInformation>::SharedPtr debug_msg_send_3;
    public:

        debugMsgPub(rclcpp::Node* node);

        void debug_msg_publish_1(std::vector<double> l_reserve, std::vector<double> r_reserve);
        void debug_msg_publish_2(std::vector<double> l_reserve, std::vector<double> r_reserve);
        void debug_msg_publish_3(std::vector<double> l_reserve, std::vector<double> r_reserve);
};

//-----------------------------------------------------------------------------------

extern debugMsgPub* global_debugPub;

#endif  // SERVO_CONTROL_HW__DEBUGPUBLISH__DEBUGPUBLISH_H_