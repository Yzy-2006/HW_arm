"""Receive local hand tracking and publish isolated RViz simulation state."""

import argparse
import json
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import Marker

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "teleop"))
from gesture_control import GESTURE_NAMES, GestureMapper, SIDES, load_config
from gesture_input import HandPacketReceiver
from gesture_kinematics import GestureRobot
from gesture_recording import SimulationRecorder


class GestureSimulation(Node):
    def __init__(self, args):
        super().__init__("gesture_simulation", namespace="gesture_sim")
        config = load_config(args.config)
        self.mapper = GestureMapper(config)
        self.robot = GestureRobot(config["joint_speed_radps"])
        root = args.recording_root or Path(config["recording_root"])
        self.recorder = SimulationRecorder(root if root.is_absolute() else ROOT / root, config)
        self.receiver = HandPacketReceiver(args.port, args.rgb_port, config)
        self.joints = self.create_publisher(JointState, "/model_viewer/joint_states", 10)
        self.status_pub = self.create_publisher(String, "status", 10)
        self.marker_pub = self.create_publisher(Marker, "status_marker", 10)
        self.packet = None
        self.result = None
        self.release_notices = {}
        self.last_tick = time.monotonic()
        self.create_timer(1 / 30, self.tick)
        self.get_logger().info(f"RViz simulation only; UDP 127.0.0.1:{args.port}. Curl fingers to arm, then open palm to move.")

    def poll(self, now):
        return self.receiver.poll(now)

    def tick(self):
        now = time.monotonic()
        dt = min(now - self.last_tick, 0.1)
        self.last_tick = now
        packet = self.poll(now)
        events = []
        if packet is not None:
            previous_stamp = self.mapper.last_stamp
            self.packet = packet
            self.result = self.mapper.consume(packet)
            if self.result is not None:
                events = self.result["events"]
                if "record" in events:
                    if self.recorder.active:
                        path = self.recorder.stop("left_ok")
                        self.get_logger().info(f"Recording saved: {path}")
                    else:
                        self.recorder.start()
                        self.get_logger().info(f"Recording started: {self.recorder.path}")
                if "home" in events:
                    self.robot.start_home()
                    self.get_logger().info("Returning both simulated arms to the hardware joint zero loaded at startup")
                for side in SIDES:
                    if f"gripper_{side}" in events:
                        self.robot.toggle_gripper(side)
                    hand = self.result["hands"][side]
                    if hand["active"] and not self.robot.homing:
                        frame_dt = 0 if previous_stamp is None else self.mapper.last_stamp - previous_stamp
                        if not self.robot.move(side, hand["position_delta"], hand["rotation_delta"], frame_dt):
                            self.mapper.suspend_motion("ik_failed")
                            self.get_logger().warning("IK rejected a target; curl fingers, then open palm to resume")
        stale = self.mapper.expired(now)
        if stale:
            self.result = None
        was_homing = self.robot.homing
        self.robot.tick(dt)
        if was_homing:
            self.mapper.suspend_motion()
        snapshot = self.robot.snapshot()
        stamp = self.get_clock().now().to_msg()
        self.joints.publish(JointState(header=self._header(stamp), name=snapshot["joint_names"],
                                       position=snapshot["joint_positions"]))
        hands = {} if self.result is None else {
            side: {key: value for key, value in hand.items() if not key.endswith("delta")}
            for side, hand in self.result["hands"].items()
        }
        for side in hands:
            hands[side]["active"] = bool(hands[side]["active"] and not was_homing and not self.robot.homing)
            if was_homing or self.robot.homing:
                hands[side]["motion_state"] = "HOME"
                hands[side]["release_reason"] = "home"
            reason = hands[side].get("release_reason") if hands[side]["motion_state"] == "RELEASE" else None
            if reason is not None and self.release_notices.get(side) != reason:
                self.get_logger().info(f"{side}: RELEASE required ({reason})")
            self.release_notices[side] = reason
        status = {"simulation": True, "tracking_stale": stale, "recording": self.recorder.active,
                  "homing": self.robot.homing, "hands": hands, "gripper_closed": self.robot.closed}
        self.status_pub.publish(String(data=json.dumps(status)))
        marker = Marker(header=self._header(stamp), ns="gesture_status", id=0,
                        type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
        marker.pose.position.z = 1.55
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.045
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.9, 0.9, 0.9, 1.0)
        marker.text = "SIM:" + ("REC" if self.recorder.active else "IDLE")
        marker.text += ":" + ("HOME" if self.robot.homing else "WAITING" if stale else "TRACKING")
        for side in SIDES:
            hand = hands.get(side, {})
            state = hand.get("motion_state", "STALE")
            marker.text += f"\n{side.upper()}:{GESTURE_NAMES[hand.get('gesture', -1)]}/{state}/"
            marker.text += "CLOSED" if self.robot.closed[side] else "OPEN"
        self.marker_pub.publish(marker)
        if self.recorder.active:
            self.recorder.write({"type": "sample", "time_ns": time.time_ns(), "monotonic_ns": time.monotonic_ns(),
                                 "state": snapshot, "status": status, "events": events,
                                 "input": self.packet if not stale else None})

    @staticmethod
    def _header(stamp):
        from std_msgs.msg import Header
        return Header(stamp=stamp, frame_id="world")

    def close(self):
        path = self.recorder.stop("shutdown")
        if path:
            self.get_logger().info(f"Recording saved: {path}")
        self.receiver.close()
        self.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=9872)
    parser.add_argument("--rgb-port", type=int, help="Calibrated PnP stream from the detector's compare mode")
    parser.add_argument("--config", type=Path, default=ROOT / "config/gesture_rviz.yaml")
    parser.add_argument("--recording-root", type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = None
    try:
        node = GestureSimulation(args)
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
