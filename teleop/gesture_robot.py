"""Open-palm teleoperation through the existing hardware VR command interface."""

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool, Float64MultiArray, Int8MultiArray, String
from std_srvs.srv import SetBool
from ctl_msgs_hw.msg import FkDataPub, GripperCtl, MidNodeCtlMsgSend, RetInformation
from ament_index_python.packages import get_package_share_directory

from gesture_control import GestureMapper, SIDES, load_config
from gesture_input import HandPacketReceiver

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "teleop/VR_controller"))
sys.path.insert(0, str(ROOT / "scripts"))
from grippers import DualGrippers, load_config as gripper_config
from pose_mapping import _conservative_deadband, quaternion_to_euler_zyx
from calibrate_arm_zero import ARM_JOINT_KEYS

DELTA_TOPICS = {side: (f"/vr/{side}_arm_delta", f"/vr/{side}_arm_euler_delta") for side in SIDES}
COMMAND_TOPICS = [topic for pair in DELTA_TOPICS.values() for topic in pair] + ["/gripper/ctl", "/vr/zero_return_cmd"]


def rotation_delta_zyx(rotvec):
    quaternion = Rotation.from_rotvec(rotvec).as_quat()
    return quaternion_to_euler_zyx(quaternion[[3, 0, 1, 2]])


class GestureHardware(Node):
    def __init__(self, args):
        super().__init__("gesture_hardware")
        self.dry_run = args.dry_run
        self.config = load_config(args.config)
        for key in ("feedback_timeout_s", "home_timeout_s", "home_position_tolerance_rad", "home_speed_tolerance_radps"):
            if not np.isfinite(self.config[key]) or self.config[key] <= 0:
                raise ValueError(f"{key} must be finite and positive")
        share = Path(get_package_share_directory("ctl_msgs_hw")) / "config"
        self.grippers = DualGrippers(gripper_config(controller_path=share / "controllerConfig.yaml"))
        zero = yaml.safe_load((share / "zeropos.yaml").read_text())
        self.home_target = np.array([zero[f"{side}_arm"][key] for side in SIDES for key in ARM_JOINT_KEYS[side]])
        if not np.isfinite(self.home_target).all():
            raise ValueError("Hardware joint zero is non-finite")
        self.mapper = GestureMapper(self.config)
        self.receiver = HandPacketReceiver(args.port, args.rgb_port, self.config)
        self.received = {}
        self.safety = "missing"
        self.fault = ""
        self.joint_q = self.joint_v = None
        self.upper_mode = None
        self.bridge_active = False
        self.owned = False
        self.takeover_future = None
        self.takeover_started = 0.0
        self.retry_at = 0.0
        self.home_started = None
        self.home_seen_joint = False
        self.conflict = "checking publishers"
        self.last_graph_check = -float("inf")
        self.last_status_log = None
        self.last_packet = None
        self.recording = False
        self.record_received = None
        self.record_future = None
        self.record_started = 0.0
        self.residual = {side: np.zeros((2, 3)) for side in SIDES}
        self.status_pub = self.create_publisher(String, "/gesture/status", 10)
        self.delta_pubs = {}
        if not self.dry_run:
            qos = QoSProfile(depth=1, reliability=qos_profile_sensor_data.reliability)
            self.delta_pubs = {side: [self.create_publisher(Vector3, topic, qos) for topic in DELTA_TOPICS[side]] for side in SIDES}
            self.home_pub = self.create_publisher(Bool, "/vr/zero_return_cmd", 1)
            self.gripper_pub = self.create_publisher(GripperCtl, "/gripper/ctl", 1)
            self.gripper_state_pub = self.create_publisher(Int8MultiArray, "/towel/gripper_state", 10)
        self.takeover_client = self.create_client(SetBool, "/dexbotCtl/vr_override")
        self.record_client = self.create_client(SetBool, "/towel/record")
        self.create_subscription(String, "/towel/safety/state", self.on_safety,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(RetInformation, "/dexbotCtl/dualArmCtl_fdback", self.on_joints, qos_profile_sensor_data)
        self.create_subscription(FkDataPub, "/dexbotCtl/DescPub/Fk_ret_data_pub", self.on_fk, 1)
        self.create_subscription(MidNodeCtlMsgSend, "/dexbotCtl/upperMonitor_msg_send", self.on_upper, 1)
        self.create_subscription(Float64MultiArray, "/gripper/fdb", lambda m: self.grippers.feedback(m.data, time.monotonic()), 1)
        self.create_subscription(Bool, "/towel/recording", self.on_recording, 10)
        self.create_timer(0.02, self.tick)
        self.get_logger().info("Gesture hardware input: " + ("DRY RUN (no command publishers or service calls)" if self.dry_run else "LIVE, waiting for feedback and takeover"))

    def on_safety(self, msg):
        self.safety = msg.data
        self.received["safety"] = time.monotonic()
        if msg.data.startswith("fault:"):
            self.fault = msg.data

    def stamp_valid(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        return -0.05 <= age <= self.config["feedback_timeout_s"]

    def on_joints(self, msg):
        values = [msg.l_position, msg.r_position, msg.l_speed, msg.r_speed]
        if any(len(value) != 7 for value in values) or not np.isfinite(values).all() or not self.stamp_valid(msg):
            self.received.pop("joints", None)
            return
        self.joint_q = np.r_[msg.l_position, msg.r_position]
        self.joint_v = np.r_[msg.l_speed, msg.r_speed]
        self.received["joints"] = time.monotonic()

    def on_fk(self, msg):
        values = [getattr(msg, f"{axis}_{side}") for side in ("l", "r") for axis in ("x", "y", "z", "roll", "pitch", "yaw")]
        if np.isfinite(values).all():
            self.received["fk"] = time.monotonic()
        else:
            self.received.pop("fk", None)

    def on_upper(self, msg):
        if not self.stamp_valid(msg):
            self.received.pop("upper", None)
            return
        self.received["upper"] = time.monotonic()
        self.upper_mode = msg.ctl_trans_mod
        self.bridge_active = msg.bridge_joint_stream
        if self.home_started is not None and msg.ctl_trans_mod == 1:
            self.home_seen_joint = True

    def on_recording(self, msg):
        self.record_received = time.monotonic()
        if self.record_future is None:
            self.recording = msg.data

    def blocked_reason(self, now):
        if self.fault:
            return self.fault
        if self.conflict:
            return self.conflict
        for name in ("safety", "joints", "fk", "upper"):
            if now - self.received.get(name, -float("inf")) > self.config["feedback_timeout_s"]:
                return f"waiting for fresh {name} feedback"
        if self.safety != "ready":
            return "servo " + self.safety
        if self.bridge_active:
            return "policy bridge is active; stop it before gesture control"
        if self.home_started is None and self.upper_mode != 4:
            return "upper controller is not in Cartesian teleoperation mode"
        return ""

    def lose_ownership(self):
        self.owned = False
        if self.takeover_future is not None:
            self.takeover_future.cancel()
            self.takeover_future = None
        self.mapper.reset("feedback_lost")
        for residual in self.residual.values():
            residual.fill(0)
        for gripper in self.grippers.sides.values():
            gripper.invalidate()

    def ensure_takeover(self, now):
        if self.owned or self.dry_run or self.home_started is not None:
            return
        if self.takeover_future is not None:
            if self.takeover_future.done():
                try:
                    response = self.takeover_future.result()
                    self.owned = response.success
                    self.get_logger().info(f"Gesture takeover: {response.message}")
                except Exception as error:
                    self.get_logger().error(f"Takeover failed: {error}")
                self.takeover_future = None
                self.mapper.reset("takeover")
                self.retry_at = now + 2.0
            elif now - self.takeover_started > 2.0:
                self.takeover_future.cancel()
                self.takeover_future = None
                # A late successful service may already have changed ownership.
                self.fault = "takeover response timeout; restart gesture input"
        elif now >= self.retry_at and self.takeover_client.service_is_ready():
            self.takeover_started = now
            self.takeover_future = self.takeover_client.call_async(SetBool.Request(data=True))

    def toggle_recording(self, now):
        if self.record_future is not None:
            return
        if (not self.record_client.service_is_ready() or self.record_received is None
                or now - self.record_received > 1.0):
            self.get_logger().warning("Recording unavailable: start make record and wait for /towel/recording")
            return
        request = SetBool.Request(data=not self.recording)
        self.record_started = now
        self.record_future = self.record_client.call_async(request)
        future = self.record_future

        def done(completed):
            if self.record_future is not completed:
                return
            self.record_future = None
            try:
                response = completed.result()
                if response.success:
                    self.recording = request.data
                self.get_logger().info(f"Recording: {response.message}")
            except Exception as error:
                self.get_logger().error(f"Recording service failed: {error}")
        future.add_done_callback(done)

    def publish_deltas(self, deltas):
        for side, pair in self.delta_pubs.items():
            for pub, values in zip(pair, deltas[side]):
                pub.publish(Vector3(x=float(values[0]), y=float(values[1]), z=float(values[2])))

    def tick(self):
        now = time.monotonic()
        if now - self.last_graph_check > 0.25:
            expected = 0 if self.dry_run else 1
            conflicts = [topic for topic in COMMAND_TOPICS if self.count_publishers(topic) > expected]
            self.conflict = "competing publisher: " + ", ".join(conflicts) if conflicts else ""
            self.last_graph_check = now
        reason = self.blocked_reason(now)
        packet = self.receiver.poll(now)
        if packet is not None:
            self.last_packet = packet
        input_fresh = self.last_packet is not None and now - self.last_packet["capture_monotonic_ns"] / 1e9 <= self.config["timeout_s"]
        if reason and not self.dry_run:
            if self.home_started is not None:
                self.fault = "feedback or ownership lost during home; inspect controller before restarting gesture input"
            self.lose_ownership()
        elif input_fresh:
            self.ensure_takeover(now)
        result = self.mapper.consume(packet) if packet is not None else None
        stale = self.mapper.expired(now)
        if not self.dry_run and not self.owned:
            self.mapper.reset("takeover")
            result = None
        ready = self.owned and not reason and not self.dry_run
        if self.home_started is not None:
            self.mapper.suspend_motion("home")
            if (ready and self.home_seen_joint and self.upper_mode == 4
                    and np.max(np.abs(self.joint_q - self.home_target)) <= self.config["home_position_tolerance_rad"]
                    and np.max(np.abs(self.joint_v)) <= self.config["home_speed_tolerance_radps"]):
                self.home_started = None
                self.get_logger().info("Hardware home reached; curl fingers before reopening palms")
            elif now - self.home_started > self.config["home_timeout_s"]:
                self.fault = "home timeout; inspect controller feedback before restarting gesture input"
            result = None
        events = result["events"] if result is not None and not stale else []
        if ready and "record" in events:
            self.toggle_recording(now)
        if ready and "home" in events:
            if self.recording or self.record_future is not None or (
                    self.record_received is not None and now - self.record_received > 1.0):
                self.get_logger().warning("Finish recording before requesting home")
            else:
                self.home_started, self.home_seen_joint = now, False
                self.home_pub.publish(Bool(data=True))
                self.mapper.suspend_motion("home")
                result = None
                self.get_logger().info("Right OK: hardware joint return requested")
        deltas = {side: [np.zeros(3), np.zeros(3)] for side in SIDES}
        for side in SIDES:
            hand = result["hands"][side] if result is not None else None
            if hand is None:
                if stale or reason or self.home_started is not None or (not self.dry_run and not self.owned):
                    self.residual[side].fill(0)
                continue
            if not hand["active"] or hand["motion_state"] == "ANCHOR" or stale:
                self.residual[side].fill(0)
                continue
            deltas[side][0] = _conservative_deadband(hand["position_delta"], self.residual[side][0], 0.0004, True)
            deltas[side][1] = _conservative_deadband(rotation_delta_zyx(hand["rotation_delta"]), self.residual[side][1], 0.002, True)
        if ready:
            self.publish_deltas(deltas)
            positions, states = self.grippers.step(now, {side: f"gripper_{side}" in events and self.home_started is None for side in SIDES})
            if np.isfinite(positions).all():
                self.gripper_pub.publish(GripperCtl(ctl_time=0.02, l_arm_gripper_pos=float(positions[0]),
                                                   r_arm_gripper_pos=float(positions[1]), l_arm_gripper_tor=0.0, r_arm_gripper_tor=0.0))
            self.gripper_state_pub.publish(Int8MultiArray(data=states))
        if self.record_future is not None and now - self.record_started > 5:
            self.record_future.cancel()
            self.record_future = None
            self.record_received = None
            self.get_logger().warning("Recording request timed out; waiting for authoritative recorder state")
        status = {"dry_run": self.dry_run, "ready": ready, "blocked_reason": reason,
                  "owned": self.owned, "tracking_stale": stale, "homing": self.home_started is not None,
                  "takeover_available": self.takeover_client.service_is_ready(),
                  "recording_available": self.record_client.service_is_ready(),
                  "recording": self.recording, "events": events,
                  "hands": {} if result is None else {side: {k: v for k, v in hand.items() if not k.endswith("delta")} for side, hand in result["hands"].items()},
                  "deltas": {side: [v.tolist() for v in pair] for side, pair in deltas.items()}}
        self.status_pub.publish(String(data=json.dumps(status)))
        waiting = ("waiting for fresh gesture input" if not input_fresh else
                   "waiting for /dexbotCtl/vr_override service" if not self.takeover_client.service_is_ready()
                   else "waiting for takeover")
        summary = reason or ("dry run" if self.dry_run else "ready" if ready else waiting)
        if summary != self.last_status_log:
            self.get_logger().info(summary)
            self.last_status_log = summary

    def close(self):
        if rclpy.ok() and self.owned and not self.conflict and not self.dry_run:
            self.publish_deltas({side: [np.zeros(3), np.zeros(3)] for side in SIDES})
        self.receiver.close()
        self.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/gesture_robot.yaml")
    parser.add_argument("--port", type=int, default=9874)
    parser.add_argument("--rgb-port", type=int)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    rclpy.init()
    node = None
    try:
        node = GestureHardware(args)
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
