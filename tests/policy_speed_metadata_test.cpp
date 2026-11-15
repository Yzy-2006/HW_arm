#include <cassert>
#include "rclcpp/rclcpp.hpp"
#include "ctl_msgs_hw/msg/bridge_ctl_msg.hpp"
#include "ctl_msgs_hw/msg/mid_node_ctl_msg_send.hpp"
#include "robot_control/msgType.h"
#include "messageType/upper_msgType.h"
#include "messageType/chassis_msgType.h"
#include "debugPublish/debugPublish.h"
#include "controller/jointStreamGuard.h"
#define private public
#include "controller/ctlTransmit.h"
#undef private

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    {
        auto node = std::make_shared<rclcpp::Node>("policy_speed_metadata_test");
        CtlTransmit bridge;
        bridge.node_ptr_ = node.get();
        bridge.bridge_active_ = bridge.joint_stream_mode_ = true;
        auto sample = std::make_shared<ctl_msgs_hw::msg::BridgeCtlMsg>();
        bridge.stream_message_ = sample;
        sample->header.stamp = node->now();
        sample->header.frame_id = "pi05_policy";
        sample->policy_speed_checks_disabled = true;
        sample->target_period_s = .02;
        sample->derivatives_valid = true;
        ctl_msgs_hw::msg::MidNodeCtlMsgSend packet;
        bridge.fill_stream_metadata(packet);
        assert(packet.bridge_joint_stream && packet.policy_speed_checks_disabled);

        // Holding stops trajectory streaming but preserves the deployment mode.
        sample->hold_position = true;
        packet = ctl_msgs_hw::msg::MidNodeCtlMsgSend();
        bridge.fill_stream_metadata(packet);
        assert(!packet.bridge_joint_stream && !packet.derivatives_valid);
        assert(packet.policy_speed_checks_disabled && packet.trajectory_header == sample->header);
        sample->hold_position = false;
        sample->header.stamp = node->now() - rclcpp::Duration::from_seconds(2);
        packet = ctl_msgs_hw::msg::MidNodeCtlMsgSend();
        bridge.fill_stream_metadata(packet);
        assert(!packet.bridge_joint_stream && packet.policy_speed_checks_disabled);

        sample->header.frame_id = "replay";
        packet = ctl_msgs_hw::msg::MidNodeCtlMsgSend();
        bridge.fill_stream_metadata(packet);
        assert(!packet.policy_speed_checks_disabled);
        sample->header.frame_id = "pi05_policy";
        bridge.set_vr_override(true);
        packet = ctl_msgs_hw::msg::MidNodeCtlMsgSend();
        bridge.fill_stream_metadata(packet);
        assert(!packet.bridge_joint_stream && !packet.policy_speed_checks_disabled);
    }
    rclcpp::shutdown();
}
