"""Exercise real ROS interfaces with mock hardware in isolated domain 87.

Run in the robot environment: python tests/gesture_robot_runtime.py
Never starts controllers, opens cameras, or connects to the hardware ROS domain.
"""

import json
import os
from pathlib import Path
import socket
import sys
import time
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import SetBool
from ctl_msgs_hw.msg import FkDataPub, GripperCtl, MidNodeCtlMsgSend, RetInformation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "teleop"))
from gesture_control import FIST, OPEN_PALM
from gesture_robot import GestureHardware, DELTA_TOPICS, rotation_delta_zyx
from test_gesture_teleop import packet


def main():
    os.environ["ROS_DOMAIN_ID"] = "87"
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ.pop("ROS_STATIC_PEERS", None)
    os.environ.pop("ROS_DISCOVERY_SERVER", None)
    for rotvec in ([0, 0, 0], [0.1, -0.2, 0.3], [-0.01, 0.03, -0.02]):
        np.testing.assert_allclose(Rotation.from_euler("ZYX", rotation_delta_zyx(rotvec)).as_matrix(),
                                   Rotation.from_rotvec(rotvec).as_matrix(), atol=1e-12)
    rclpy.init()
    mock = rclpy.create_node("gesture_mock_hardware")
    executor = SingleThreadedExecutor()
    executor.add_node(mock)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    args = SimpleNamespace(config=ROOT / "config/gesture_robot.yaml", port=port, rgb_port=None, dry_run=True)
    hardware = None
    state = {"feedback": True, "mode": 4, "recording": False, "takeover": 0,
             "record_requests": [], "home": 0, "accept": True, "old_stamp": False}
    samples = {topic: [] for topics in DELTA_TOPICS.values() for topic in topics}
    gripper_samples = []
    status = {}
    pubs = {
        "safety": mock.create_publisher(String, "/towel/safety/state", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        "joints": mock.create_publisher(RetInformation, "/dexbotCtl/dualArmCtl_fdback", qos_profile_sensor_data),
        "fk": mock.create_publisher(FkDataPub, "/dexbotCtl/DescPub/Fk_ret_data_pub", 1),
        "upper": mock.create_publisher(MidNodeCtlMsgSend, "/dexbotCtl/upperMonitor_msg_send", 1),
        "gripper": mock.create_publisher(Float64MultiArray, "/gripper/fdb", 1),
        "record": mock.create_publisher(Bool, "/towel/recording", 10),
    }
    for topic in samples:
        mock.create_subscription(Vector3, topic, lambda msg, t=topic: samples[t].append([msg.x, msg.y, msg.z]), qos_profile_sensor_data)
    mock.create_subscription(GripperCtl, "/gripper/ctl", lambda msg: gripper_samples.append(msg), 10)
    mock.create_subscription(Bool, "/vr/zero_return_cmd", lambda msg: state.update(home=state["home"] + int(msg.data)), 10)
    mock.create_subscription(String, "/gesture/status", lambda msg: status.update(json.loads(msg.data)), 10)

    def takeover(request, response):
        assert request.data
        state["takeover"] += 1
        response.success = state["accept"]
        response.message = "Mock feedback anchor" if response.success else "Mock rejection"
        return response

    def record(request, response):
        state["record_requests"].append(request.data)
        state["recording"] = request.data
        response.success, response.message = True, "Mock recorder"
        return response

    mock.create_service(SetBool, "/dexbotCtl/vr_override", takeover)
    mock.create_service(SetBool, "/towel/record", record)

    def feedback():
        if hardware is None:
            return
        pubs["record"].publish(Bool(data=state["recording"]))
        if not state["feedback"]:
            return
        stamp = mock.get_clock().now().to_msg()
        if state["old_stamp"]:
            stamp.sec -= 2
        joints = RetInformation(l_position=hardware.home_target[:7].tolist(), r_position=hardware.home_target[7:].tolist(),
                                l_speed=[0.0] * 7, r_speed=[0.0] * 7)
        joints.header.stamp = stamp
        upper = MidNodeCtlMsgSend(ctl_trans_mod=state["mode"])
        upper.header.stamp = stamp
        pubs["safety"].publish(String(data="ready"))
        pubs["joints"].publish(joints)
        pubs["upper"].publish(upper)
        pubs["fk"].publish(FkDataPub(x_l=0.3, y_l=0.2, z_l=0.2, x_r=0.3, y_r=-0.2, z_r=0.2))
        config = hardware.grippers.sides
        pubs["gripper"].publish(Float64MultiArray(data=[config["left"].config["open_deg"], 0.0, 0.0, 0.0,
                                                       config["right"].config["open_deg"], 0.0, 0.0, 0.0]))

    mock.create_timer(0.02, feedback)
    sequence = 0

    def drive(duration, left=0, right=0, speed=0.0, send=True, angular_speed=0.0):
        nonlocal sequence
        start = next_send = time.monotonic()
        while time.monotonic() - start < duration:
            now = time.monotonic()
            if send and now >= next_send:
                elapsed = now - start
                message = packet(now, left, right, position=(0, 0, 0.5 - elapsed * speed),
                                 rotation=Rotation.from_rotvec(np.array([1.0, 2.0, -1.0]) * angular_speed * elapsed))
                message["sequence"] = sequence
                sequence += 1
                sender.sendto(json.dumps(message).encode(), ("127.0.0.1", port))
                next_send = now + 1 / 30
            executor.spin_once(timeout_sec=0.003)

    def clear():
        for values in samples.values():
            values.clear()
        gripper_samples.clear()

    def assert_stopped():
        assert all(not values or np.max(np.abs(values)) == 0 for values in samples.values()), samples

    try:
        hardware = GestureHardware(args)
        executor.add_node(hardware)
        drive(1.0)
        drive(0.4, OPEN_PALM, OPEN_PALM, speed=0.08)
        drive(0.4, 2, 2)
        drive(0.4, FIST, FIST)
        assert not state["takeover"] and not state["record_requests"] and not state["home"]
        assert not gripper_samples and all(not values for values in samples.values())
        assert status["dry_run"] and not status["ready"]
        executor.remove_node(hardware)
        hardware.close()
        hardware = None

        args.dry_run = False
        state["feedback"] = False
        hardware = GestureHardware(args)
        executor.add_node(hardware)
        drive(0.6, OPEN_PALM, OPEN_PALM, speed=0.08)
        assert not state["takeover"] and not gripper_samples
        state["feedback"], state["accept"] = True, False
        drive(0.8)
        assert state["takeover"] == 1 and not hardware.owned and not gripper_samples
        state["accept"] = True
        drive(2.4, OPEN_PALM, OPEN_PALM, speed=0.01)
        assert hardware.owned and status["ready"], status
        assert_stopped()  # An already-open palm cannot engage on takeover.
        drive(0.4)
        clear()
        drive(0.8, OPEN_PALM, OPEN_PALM, speed=0.06, angular_speed=0.1)
        for side in ("left", "right"):
            pos, rot = (np.array(samples[topic]) for topic in DELTA_TOPICS[side])
            assert pos[:, 0].sum() > 0.008, pos
            np.testing.assert_allclose(pos[:, 1:], 0, atol=1e-10)
            assert np.max(np.abs(rot)) > 0.002
            assert np.max(np.linalg.norm(pos, axis=1)) < 0.004
        drive(0.1, send=False)
        clear()
        drive(0.4, send=False)
        assert_stopped()  # No new UDP packet must never replay an increment.
        drive(0.4)
        clear()
        drive(1.0, OPEN_PALM, OPEN_PALM, speed=0.008)
        for side in ("left", "right"):
            assert np.array(samples[DELTA_TOPICS[side][0]])[:, 0].sum() > 0.0012, "Slow increments lost between camera frames"
        drive(0.4)
        clear()
        drive(0.9, FIST, FIST)
        assert gripper_samples
        for side, field in (("left", "l_arm_gripper_pos"), ("right", "r_arm_gripper_pos")):
            cfg = hardware.grippers.sides[side].config
            np.testing.assert_allclose(getattr(gripper_samples[-1], field), cfg["closed_deg"] - cfg["zero_pos_deg"], atol=1e-6)
        drive(0.4)
        drive(0.5, left=2)
        assert state["record_requests"] == [True]
        drive(0.4)
        drive(0.5, right=2)
        assert not state["home"], "Homing must not interrupt an episode"
        drive(0.4)
        drive(0.5, left=2)
        assert state["record_requests"] == [True, False]
        drive(0.4)
        drive(0.5, right=2)
        assert state["home"] == 1 and hardware.home_started is not None
        state["mode"] = 1
        clear()
        drive(0.4, OPEN_PALM, OPEN_PALM, speed=0.08)
        assert_stopped()
        state["mode"] = 4
        drive(0.4, OPEN_PALM, OPEN_PALM, speed=0.08)
        assert hardware.home_started is None and hardware.owned
        assert_stopped()
        state["old_stamp"] = True
        drive(0.35, OPEN_PALM, OPEN_PALM)
        assert not hardware.owned and not status["ready"]
        state["old_stamp"] = False
        drive(0.8, OPEN_PALM, OPEN_PALM, speed=0.08)
        assert hardware.owned
        assert_stopped()
        drive(0.4)
        drive(0.4, OPEN_PALM, OPEN_PALM, speed=0.08)
        assert any(np.max(np.abs(values)) > 0 for values in samples.values())
        competitor = mock.create_publisher(Vector3, DELTA_TOPICS["left"][0], 1)
        drive(0.5)
        assert not hardware.owned and "competing publisher" in status["blocked_reason"]
        clear()
        drive(0.4, OPEN_PALM, OPEN_PALM, speed=0.08)
        assert_stopped()
        assert not gripper_samples
        mock.destroy_publisher(competitor)
        drive(0.9)
        assert hardware.owned
        drive(0.5, right=2)
        assert hardware.home_started is not None
        state["feedback"] = False
        drive(0.4)
        assert hardware.fault and not hardware.owned
        state["feedback"] = True
        drive(0.4)
        assert not hardware.owned and hardware.fault
        print("PASS: dry-run isolation, rejected/accepted takeover, palm translation/rotation, no replay, grippers, recording, home, stale stamps, re-arm, competing input and home feedback fault", flush=True)
    finally:
        if hardware is not None:
            executor.remove_node(hardware)
            hardware.close()
        sender.close()
        executor.shutdown()
        mock.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
