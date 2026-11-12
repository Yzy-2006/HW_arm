#include <cassert>
#include <chrono>
#include <memory>

#include "ros_controller/rosLoop.h"

// Exercise the real control-side drain without opening EtherCAT or running homing.
struct ServoRosIoTest {
    static auto reset_request(ServoNode& node) {
        auto request = std::make_shared<ServoNode::ResetRequest>();
        auto result = request->result.get_future();
        node.pending_reset_.store(request);
        return result;
    }

    static void run(ServoNode& node) {
        node.startup_homing_ = false;
        node.servo_desMsg_orig.resize(14);
        node.ifGetGripperMsg = false;
        arm_safety = ArmSafetyGuard{};
        arm_safety.limits.lower.fill(-3);
        arm_safety.limits.upper.fill(3);
        for (auto& feedback : arm_safety.feedback)
            feedback = {0, 0, arm_safety_now(), 0};

        auto gripper = std::make_shared<ctl_msgs_hw::msg::GripperCtl>();
        gripper->l_arm_gripper_tor = 1;
        node.pending_command_.store(std::make_shared<ctl_msgs_hw::msg::CtlInformation>());
        node.pending_gripper_.store(gripper);
        node.pending_stop_.store(std::make_shared<std::string>("test_stop"), true);
        node.stop_requested_ = true;
        node.ctlmsg_get();
        assert(arm_safety.fault == "test_stop");
        assert(!node.ifGetGripperMsg);

        // Reception inhibits reset even when the command mailbox has not been drained.
        node.last_command_received_at_ = arm_safety_now();
        auto rejected = reset_request(node);
        node.ctlmsg_get();
        assert(!rejected.get()->success);
        assert(arm_safety.fault == "test_stop");

        // Reset discards queued arm/gripper commands and does not revive their timestamps.
        node.last_command_received_at_ = arm_safety_now() - 1.1;
        node.pending_command_.store(std::make_shared<ctl_msgs_hw::msg::CtlInformation>());
        node.pending_gripper_.store(gripper);
        auto accepted = reset_request(node);
        node.ctlmsg_get();
        assert(accepted.get()->success);
        assert(arm_safety.fault.empty());
        assert(!node.ifGetGripperMsg);
        for (const auto& command : node.servo_desMsg_orig) {
            assert(command.joint_Kp == 0);
            assert(command.joint_spd == 0);
            assert(command.joint_tor == 0);
            assert(command.joint_Kd == arm_safety.limits.damping);
        }
        node.ctlmsg_get();
        arm_safety.check_feedback(arm_safety_now());
        assert(arm_safety.fault.empty());

        // A concurrent stop must survive a successful reset in the same cycle.
        node.pending_stop_.store(std::make_shared<std::string>("stop_during_reset"), true);
        node.stop_requested_ = true;
        auto concurrent = reset_request(node);
        node.ctlmsg_get();
        (void) concurrent.get();
        assert(arm_safety.fault == "stop_during_reset");

        node.ros_io_failed_ = true;
        auto failed = reset_request(node);
        node.ctlmsg_get();
        assert(!failed.get()->success);
        assert(!arm_safety.fault.empty());
        node.ros_io_failed_ = false;

        // Cancellation immediately after startup must not leave an executor spinning.
        node.start_ros_io();
        node.stop_ros_io();
        assert(!node.ros_receiver_.joinable());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    {
        ServoNode node;
        ServoRosIoTest::run(node);
    }
    rclcpp::shutdown();
}
