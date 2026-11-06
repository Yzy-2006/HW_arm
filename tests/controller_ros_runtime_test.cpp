#include <cassert>
#include <stdexcept>
#include "servo_control/shutdownAwareRate.h"
#include "ctl_msgs_hw/msg/ctl_information.hpp"
#include "ctl_msgs_hw/msg/handoff_status.hpp"
#include "ctl_msgs_hw/msg/bridge_ctl_msg.hpp"
#include "ctl_msgs_hw/msg/mid_node_ctl_msg_send.hpp"
#include "rclcpp/serialization.hpp"

class FailingRate : public rclcpp::Rate {
public:
    FailingRate() : rclcpp::Rate(1000) {}
    bool shutdown_first = false;
    int calls = 0;
    bool sleep() override {
        ++calls;
        if (shutdown_first) rclcpp::shutdown();
        throw std::runtime_error("sleep failure");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // Exercise the generated ROS wire type without a node, executor or hardware.
    ctl_msgs_hw::msg::CtlInformation sent, received;
    sent.handoff_epoch = 123;
    sent.trajectory_sample = true;
    sent.policy_speed_checks_disabled = true;
    sent.trajectory_header.frame_id = "pi05_policy";
    sent.trajectory_header.stamp.sec = 42;
    sent.trajectory_header.stamp.nanosec = 123;
    sent.l_position = {.1, .2, .3, .4, .5, .6, .7};
    rclcpp::Serialization<ctl_msgs_hw::msg::CtlInformation> serializer;
    rclcpp::SerializedMessage wire;
    serializer.serialize_message(&sent, &wire);
    serializer.deserialize_message(&wire, &received);
    assert(received.handoff_epoch == 123 && received.trajectory_sample);
    assert(received.policy_speed_checks_disabled);
    ctl_msgs_hw::msg::BridgeCtlMsg bridge, bridge_received;
    assert(!bridge.policy_speed_checks_disabled);
    bridge.policy_speed_checks_disabled = true;
    bridge.header = sent.trajectory_header;
    rclcpp::Serialization<ctl_msgs_hw::msg::BridgeCtlMsg> bridge_serializer;
    rclcpp::SerializedMessage bridge_wire;
    bridge_serializer.serialize_message(&bridge, &bridge_wire);
    bridge_serializer.deserialize_message(&bridge_wire, &bridge_received);
    assert(bridge_received.policy_speed_checks_disabled && bridge_received.header == bridge.header);
    ctl_msgs_hw::msg::MidNodeCtlMsgSend mid, mid_received;
    assert(!mid.policy_speed_checks_disabled);
    mid.policy_speed_checks_disabled = true;
    mid.trajectory_header = bridge.header;
    rclcpp::Serialization<ctl_msgs_hw::msg::MidNodeCtlMsgSend> mid_serializer;
    rclcpp::SerializedMessage mid_wire;
    mid_serializer.serialize_message(&mid, &mid_wire);
    mid_serializer.deserialize_message(&mid_wire, &mid_received);
    assert(mid_received.policy_speed_checks_disabled && mid_received.trajectory_header == bridge.header);
    assert(received.l_position == sent.l_position);
    assert(received.trajectory_header == sent.trajectory_header);
    ctl_msgs_hw::msg::HandoffStatus status, status_received;
    status.source_header = received.trajectory_header;
    status.handoff_epoch = received.handoff_epoch;
    status.complete = true;
    status.commanded_position.assign(14, .1);
    rclcpp::Serialization<ctl_msgs_hw::msg::HandoffStatus> status_serializer;
    rclcpp::SerializedMessage status_wire;
    status_serializer.serialize_message(&status, &status_wire);
    status_serializer.deserialize_message(&status_wire, &status_received);
    assert(status_received.source_header == sent.trajectory_header);
    assert(status_received.complete && status_received.handoff_epoch == 123);
    assert(status_received.commanded_position == status.commanded_position);

    rclcpp::Rate normal(1000);
    sleep_while_running(normal);
    FailingRate rate;
    bool threw = false;
    try { sleep_while_running(rate); }
    catch (const std::runtime_error&) { threw = true; }
    assert(threw && rate.calls == 1); // Runtime errors while active remain visible.
    rate.shutdown_first = true;
    sleep_while_running(rate); // Shutdown between ok() and sleep() is a normal exit.
    assert(!rclcpp::ok() && rate.calls == 2);
    sleep_while_running(rate); // Already shut down: do not call sleep at all.
    assert(rate.calls == 2);
}
