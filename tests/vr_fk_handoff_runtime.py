"""Exercise the built upper controller with isolated, remapped mock feedback."""

import os
from pathlib import Path
import pty
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from ctl_msgs_hw.msg import BridgeCtlMsg, FkDataPub, GripperCtl, MidNodeCtlMsgSend
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


def main():
    binary, log_path = sys.argv[1:]
    prefix = f"/vr_fk_test_{os.getpid()}"
    fk_topic = "/dexbotCtl/DescPub/Fk_ret_data_pub"
    service = "/dexbotCtl/vr_override"
    topics = [fk_topic, service, "/dexbotCtl/policy_home", "/dexbotCtl/policy_intervene", "/towel/safety/state",
              "/dexbotCtl/upperMonitor_msg_send", "/dexbotCtl/midMonitor_msg_send",
              "/dexbotCtl/bridge_ctl_msg", "/dexbotCtl/pingpong_simu_start",
              "/vr/left_arm_delta", "/vr/right_arm_delta",
              "/vr/left_arm_euler_delta", "/vr/right_arm_euler_delta", "/vr/zero_return_cmd",
              "/gripper/ctl"]
    args = [binary, "--ros-args"]
    for topic in topics:
        args += ["-r", f"{topic}:={prefix}{topic}"]
    rclpy.init()
    node = rclpy.create_node("vr_fk_test_driver")
    safety = node.create_publisher(String, prefix + "/towel/safety/state",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    fk = node.create_publisher(FkDataPub, prefix + fk_topic, 10)
    client = node.create_client(SetBool, prefix + service)
    home_client = node.create_client(Trigger, prefix + "/dexbotCtl/policy_home")
    intervene_client = node.create_client(Trigger, prefix + "/dexbotCtl/policy_intervene")
    grippers = []
    gripper_sub = node.create_subscription(GripperCtl, prefix + "/gripper/ctl",
        lambda msg: grippers.append((time.monotonic(), msg)), 10)
    import yaml
    root = Path(os.environ["TOWEL_ROOT"])
    calibration = yaml.safe_load((root / "teleop/VR_controller/config/grippers.yaml").read_text())
    controls = yaml.safe_load((root / "ros2_ws/src/ctl_msgs_hw/config/controllerConfig.yaml").read_text())["gripperControl"]

    def assert_closed(msg):
        for side, key, field in (("left", "l_gripper_ctl", "l_arm_gripper_pos"),
                                 ("right", "r_gripper_ctl", "r_arm_gripper_pos")):
            expected = calibration[side]["closed_deg"] - controls[key].get("zero_pos_deg", 0.)
            assert abs(getattr(msg, field) - expected) < 1e-9
        assert msg.l_arm_gripper_tor == msg.r_arm_gripper_tor == 0.
    pose = FkDataPub(x_l=.21, y_l=.12, z_l=-.31, roll_l=.11, pitch_l=-.22, yaw_l=.33,
                     x_r=.24, y_r=-.15, z_r=-.36, roll_r=-.17, pitch_r=.28, yaw_r=-.39)
    publish_fk = False

    def inputs():
        safety.publish(String(data="ready"))
        if publish_fk:
            fk.publish(pose)

    timer = node.create_timer(.02, inputs)

    def spin_for(duration):
        until = time.monotonic() + duration
        while time.monotonic() < until:
            assert process.poll() is None, Path(log_path).read_text()
            rclpy.spin_once(node, timeout_sec=.02)

    def request(manual):
        future = client.call_async(SetBool.Request(data=manual))
        until = time.monotonic() + 3
        while not future.done() and time.monotonic() < until:
            spin_for(.02)
        assert future.done(), "VR service did not respond"
        return future.result()

    with open(log_path, "w") as log:
        master, slave = pty.openpty()
        process = subprocess.Popen(args, stdin=slave, stdout=log, stderr=log)
        os.close(slave)
        try:
            # Pause the production output loop; it still spins its real services/subscriptions.
            deadline = time.monotonic() + 10
            while not client.service_is_ready() and time.monotonic() < deadline:
                spin_for(.05)
            assert client.service_is_ready(), Path(log_path).read_text()
            os.write(master, b"r")
            spin_for(.4)
            response = request(True)
            assert not response.success and "fk_feedback_missing" in response.message, response
            assert fk_topic in response.message
            publish_fk = True
            spin_for(.3)
            assert fk.get_subscription_count() == 1, "Upper controller is not subscribed to live FK"
            response = request(True)
            assert response.success and response.message == "VR anchored to feedback", response

            # Resume output in the isolated graph and verify the anchor's field ordering.
            outputs = []
            subscription = node.create_subscription(MidNodeCtlMsgSend,
                prefix + "/dexbotCtl/upperMonitor_msg_send", outputs.append, 10)
            from ctl_msgs_hw.msg import MidNodeCtlMsgRet
            joints = node.create_publisher(MidNodeCtlMsgRet, prefix + "/dexbotCtl/midMonitor_msg_send", 10)
            feedback = MidNodeCtlMsgRet()
            for field, kind in feedback.get_fields_and_field_types().items():
                if kind == "sequence<double>":
                    setattr(feedback, field, [0.] * 7)
                elif kind == "sequence<boolean>":
                    setattr(feedback, field, [True] * 7)
            feedback_timer = node.create_timer(.02, lambda: joints.publish(feedback))
            os.write(master, b"e")
            spin_for(.5)
            assert outputs, "Controller produced no remapped test output"
            latest = outputs[-1]
            assert latest.ctl_trans_mod == 4
            for side, values in (("l", latest.l_arm_des_pos_descartes), ("r", latest.r_arm_des_pos_descartes)):
                expected = [getattr(pose, f"{axis}_{side}") for axis in ("x", "y", "z", "roll", "pitch", "yaw")]
                assert all(abs(a - b) < 1e-9 for a, b in zip(values, expected)), (values, expected)

            assert request(False).success
            publish_fk = False
            spin_for(.4)
            response = request(True)
            assert not response.success and "fk_feedback_stale" in response.message, response
            pose.pitch_r = float("nan")
            publish_fk = True
            spin_for(.3)
            response = request(True)
            assert not response.success and "fk_feedback_nonfinite arm=right axis=pitch" in response.message, response
            pose.pitch_r = .28
            spin_for(.3)
            assert request(True).success

            # Use the real home service and keep injecting model commands. They
            # must not displace the native home/hold, even after its trajectory ends.
            policy_pub = node.create_publisher(BridgeCtlMsg, prefix + "/dexbotCtl/bridge_ctl_msg", 1)
            policy = BridgeCtlMsg()
            for field, kind in policy.get_fields_and_field_types().items():
                if kind.startswith("sequence<double>"):
                    setattr(policy, field, [0.] * 7)
                elif kind.startswith("sequence<int32>"):
                    setattr(policy, field, [1] * 7)
            policy.header.frame_id = "pi05_policy"
            policy.ctl_trans_mod = 3
            policy.target_period_s = .02
            policy.policy_speed_checks_disabled = True
            policy.l_arm_des_msg = [.4] * 7
            policy.r_arm_des_msg = [-.4] * 7

            def publish_policy():
                policy.header.stamp = node.get_clock().now().to_msg()
                policy_pub.publish(policy)

            assert request(False).success
            policy_timer = node.create_timer(.02, publish_policy)
            spin_for(.3)
            assert outputs[-1].ctl_trans_mod == 3
            future = intervene_client.call_async(Trigger.Request())
            deadline = time.monotonic() + 3
            while not future.done() and time.monotonic() < deadline:
                spin_for(.02)
            assert future.done() and future.result().success, future.result()
            spin_for(.4)
            assert outputs[-1].ctl_trans_mod == 1
            assert outputs[-1].l_arm_des_pos_jntspace == feedback.l_arm_pos_fdback
            assert outputs[-1].r_arm_des_pos_jntspace == feedback.r_arm_pos_fdback
            assert not outputs[-1].policy_speed_checks_disabled
            # Zero VR frames cannot release the captured joint hold.
            from geometry_msgs.msg import Vector3
            vr_delta = node.create_publisher(Vector3, prefix + "/vr/left_arm_delta", 1)
            spin_for(.2)
            vr_delta.publish(Vector3())
            spin_for(.1)
            assert outputs[-1].ctl_trans_mod == 1
            vr_delta.publish(Vector3(x=.01))
            spin_for(.2)
            assert outputs[-1].ctl_trans_mod == 4
            assert abs(outputs[-1].l_arm_des_pos_descartes[0] - (pose.x_l + .01)) < 1e-9

            assert home_client.service_is_ready()
            assert not grippers, "Grippers closed before an explicit home request"
            future = home_client.call_async(Trigger.Request())
            deadline = time.monotonic() + 3
            while not future.done() and time.monotonic() < deadline:
                spin_for(.02)
            assert future.done() and future.result().success, future.result()
            started_home_at = time.monotonic()
            spin_for(.3)
            assert not grippers, "Grippers closed before the home trajectory ended"
            assert outputs[-1].ctl_trans_mod == 1
            assert not outputs[-1].policy_speed_checks_disabled
            assert outputs[-1].l_arm_des_pos_jntspace != policy.l_arm_des_msg
            assert outputs[-1].l_arm_des_pos_jntspace != feedback.l_arm_pos_fdback
            assert not request(False).success, "Policy resumed before home completed"
            deadline = time.monotonic() + 18
            while time.monotonic() < deadline:
                spin_for(.25)
                assert outputs[-1].ctl_trans_mod == 1, "Model displaced the home trajectory/hold"
                home_output = outputs[-1]
                response = request(False)
                if response.success:
                    break
            assert response.success, response
            assert home_output.l_arm_des_pos_jntspace != feedback.l_arm_pos_fdback, "Test must resume without arriving at home"
            assert time.monotonic() - started_home_at >= 9.0, "Home trajectory or 3 s delay was skipped"
            assert len(grippers) == 1, "Policy home must close both grippers once"
            closed_at, closed_msg = grippers[0]
            assert_closed(closed_msg)
            assert closed_at - started_home_at >= 6., "Grippers closed during the trajectory"
            assert time.monotonic() - closed_at >= 3., "Model resumed before the 3 s delay after closing"
            spin_for(.2)
            assert outputs[-1].ctl_trans_mod == 3
            assert outputs[-1].l_arm_des_pos_jntspace == policy.l_arm_des_msg
            policy_timer.cancel()

            # Exercise the other entry point through the real robot terminal H.
            assert request(True).success
            grippers.clear()
            os.write(master, b"h")
            spin_for(.3)
            assert outputs[-1].ctl_trans_mod == 1 and not grippers
            manual_started_at = time.monotonic()
            deadline = time.monotonic() + 10
            while not grippers and time.monotonic() < deadline:
                spin_for(.05)
            assert len(grippers) == 1, "Robot terminal home did not close both grippers"
            assert time.monotonic() - manual_started_at >= 6.
            assert_closed(grippers[0][1])
            spin_for(.3)
            assert len(grippers) == 1 and outputs[-1].ctl_trans_mod == 4
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            os.close(master)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
