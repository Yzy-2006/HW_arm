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
        node.lArm_transMod.assign(7, 1);
        node.rArm_transMod.assign(7, 1);
        for (auto* values : {&node.l_posReceive_atRuninSt, &node.r_posReceive_atRuninSt,
                &node.lArm_pos_receive, &node.rArm_pos_receive,
                &node.lArm_vel_receive, &node.rArm_vel_receive,
                &node.lArm_torComp_grav_receive, &node.rArm_torComp_grav_receive,
                &node.lArm_torComp_interia_receive, &node.rArm_torComp_interia_receive,
                &node.lArm_torComp_feedforward_receive, &node.rArm_torComp_feedforward_receive,
                &node.lArm_Kp_receive, &node.rArm_Kp_receive,
                &node.lArm_Kd_receive, &node.rArm_Kd_receive})
            values->assign(7, 0);

        // An old model sample may be held while the controller keeps publishing.
        // The deployment setting follows controller ownership, not sample age.
        auto policy = std::make_shared<ctl_msgs_hw::msg::CtlInformation>();
        policy->temp_time = 1.0 / 300;
        policy->base_rigidity_ctl_mode_l.assign(7, 0);
        policy->base_rigidity_ctl_mode_r.assign(7, 0);
        for (auto* values : {&policy->l_position_at_runin_st, &policy->r_position_at_runin_st,
                &policy->l_position, &policy->r_position, &policy->l_speed, &policy->r_speed,
                &policy->l_torque_grav_comp, &policy->r_torque_grav_comp,
                &policy->l_torque_interia_comp, &policy->r_torque_interia_comp,
                &policy->l_torque_feedforward, &policy->r_torque_feedforward,
                &policy->l_kp, &policy->r_kp, &policy->l_kd, &policy->r_kd})
            values->assign(7, 0);
        policy->trajectory_header.frame_id = "pi05_policy";
        policy->trajectory_header.stamp = node.now() - rclcpp::Duration::from_seconds(2);
        policy->policy_speed_checks_disabled = true;
        policy->header.stamp = node.now();
        node.message_callback(policy);
        assert(arm_safety.policy_speed_override_active());
        arm_safety.check_command(10, {0, -5.347407295, 0, 100, 30}, arm_safety_now());
        assert(arm_safety.fault.empty());
        // A fresh controller packet cannot grant this override to VR/replay.
        policy->trajectory_header.frame_id = "replay";
        node.message_callback(policy);
        assert(!arm_safety.policy_speed_override_active());
        policy->trajectory_header.frame_id = "pi05_policy";
        policy->policy_speed_checks_disabled = false;
        node.message_callback(policy);
        assert(!arm_safety.policy_speed_override_active());
        // Loss of the controller transport still expires the override.
        policy->policy_speed_checks_disabled = true;
        policy->header.stamp = node.now() - rclcpp::Duration::from_seconds(.3);
        arm_safety.limits.speed_only = true;
        node.message_callback(policy);
        assert(!arm_safety.policy_speed_override_active());
        arm_safety.limits.speed_only = false;
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
