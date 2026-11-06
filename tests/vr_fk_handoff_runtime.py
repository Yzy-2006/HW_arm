"""Exercise the built upper controller with isolated, remapped mock feedback."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from ctl_msgs_hw.msg import FkDataPub, MidNodeCtlMsgSend
from std_msgs.msg import String
from std_srvs.srv import SetBool


def main():
    binary, log_path = sys.argv[1:]
    prefix = f"/vr_fk_test_{os.getpid()}"
    fk_topic = "/dexbotCtl/DescPub/Fk_ret_data_pub"
    service = "/dexbotCtl/vr_override"
    topics = [fk_topic, service, "/towel/safety/state",
              "/dexbotCtl/upperMonitor_msg_send", "/dexbotCtl/midMonitor_msg_send",
              "/dexbotCtl/bridge_ctl_msg", "/dexbotCtl/pingpong_simu_start",
              "/vr/left_arm_delta", "/vr/right_arm_delta",
              "/vr/left_arm_euler_delta", "/vr/right_arm_euler_delta", "/vr/zero_return_cmd"]
    args = [binary, "--ros-args"]
    for topic in topics:
        args += ["-r", f"{topic}:={prefix}{topic}"]
    rclpy.init()
    node = rclpy.create_node("vr_fk_test_driver")
    safety = node.create_publisher(String, prefix + "/towel/safety/state",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    fk = node.create_publisher(FkDataPub, prefix + fk_topic, 10)
    client = node.create_client(SetBool, prefix + service)
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
        process = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=log, stderr=log)
        try:
            # Pause the production output loop; it still spins its real services/subscriptions.
            process.stdin.write(b"r\n")
            process.stdin.flush()
            deadline = time.monotonic() + 10
            while not client.service_is_ready() and time.monotonic() < deadline:
                spin_for(.05)
            assert client.service_is_ready(), Path(log_path).read_text()
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
            process.stdin.write(b"e\n")
            process.stdin.flush()
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
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            process.stdin.close()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
