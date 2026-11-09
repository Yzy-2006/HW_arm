"""Run with system Python after sourcing /opt/ros/jazzy/setup.bash."""

import json
from pathlib import Path
import socket
import sys
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from pose_protocol import FRAME_ID
from ros_hand_bridge import HandBridge


class BridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.bridge = HandBridge(port=0, timeout=0.2)
        self.client = Node("test_hand_pose_subscriber")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.bridge)
        self.executor.add_node(self.client)
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.destination = self.bridge.socket.getsockname()
        self.poses, self.valid = {}, {}
        self.marker_messages = []
        self.subscriptions = []
        for side in ("left", "right"):
            self.subscriptions.extend((
                self.client.create_subscription(
                    PoseStamped, f"/hands/{side}/pose",
                    lambda message, side=side: self.poses.__setitem__(side, message), 1,
                ),
                self.client.create_subscription(
                    Bool, f"/hands/{side}/valid",
                    lambda message, side=side: self.valid.__setitem__(side, message.data), 1,
                ),
            ))
        self.subscriptions.append(self.client.create_subscription(
            MarkerArray, "/hands/markers", self.marker_messages.append, 10,
        ))
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.client)

    def tearDown(self):
        self.sender.close()
        self.bridge.destroy_node()
        self.client.destroy_node()
        self.executor.shutdown()

    def packet(self, hands=True):
        return {
            "version": 1, "frame_id": FRAME_ID, "sequence": 1,
            "capture_monotonic_ns": time.monotonic_ns(), "capture_time_ns": time.time_ns(),
            "hands": [
                {
                    "label": label, "position": [x, 0.0, 0.6],
                    "orientation": [0.0, 0.0, 0.0, 1.0],
                    "joints": [[x, 0.0, 0.6] for _ in range(21)], "quality": 0.8,
                }
                for label, x in (("Left", -0.1), ("Right", 0.1))
            ] if hands else [],
        }

    def spin(self, duration, packet_factory=None):
        deadline = time.monotonic() + duration
        next_send = 0
        while time.monotonic() < deadline:
            if packet_factory and time.monotonic() >= next_send:
                self.sender.sendto(json.dumps(packet_factory()).encode(), self.destination)
                next_send = time.monotonic() + 0.03
            self.executor.spin_once(timeout_sec=0.01)

    def test_two_hand_poses_tf_and_expiration(self):
        self.spin(1.2, self.packet)
        self.assertEqual(self.valid, {"left": True, "right": True})
        self.assertEqual(set(self.poses), {"left", "right"})
        self.assertAlmostEqual(self.poses["left"].pose.position.x, -0.1)
        self.assertEqual(self.poses["left"].header.frame_id, FRAME_ID)
        transform = self.buffer.lookup_transform("camera_link", "left_hand_palm", rclpy.time.Time())
        self.assertAlmostEqual(transform.transform.translation.x, 0.6)
        self.assertAlmostEqual(transform.transform.translation.y, 0.1)
        self.assertAlmostEqual(transform.transform.translation.z, 0.0)
        self.assertTrue(any(len(message.markers) == 14 for message in self.marker_messages))
        self.spin(0.45)
        self.assertEqual(self.valid, {"left": False, "right": False})
        self.assertFalse(self.bridge.active)
        self.assertTrue(any(
            len(message.markers) == 14 and all(marker.action == Marker.DELETE for marker in message.markers)
            for message in self.marker_messages
        ))

    def test_empty_packet_clears_both_hands(self):
        self.spin(1.0, self.packet)
        self.assertTrue(self.bridge.active)
        self.spin(0.12, lambda: self.packet(hands=False))
        self.assertEqual(self.valid, {"left": False, "right": False})
        self.assertFalse(self.bridge.active)


class ComparisonBridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_two_methods_have_separate_topics_tf_and_expiring_differences(self):
        depth = HandBridge(port=0, timeout=0.2, namespace="/hands/rgbd", tf_prefix="rgbd_", style="rgbd")
        rgb = HandBridge(port=0, timeout=0.2, namespace="/hands/rgb", tf_prefix="rgb_", style="rgb")
        client = Node("test_comparison_subscriber")
        executor = SingleThreadedExecutor()
        for node in (depth, rgb, client):
            executor.add_node(node)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        poses, valid, markers, differences = {}, {}, {}, {}
        subscriptions = []
        for method in ("rgbd", "rgb"):
            for side in ("left", "right"):
                key = method, side
                subscriptions.append(client.create_subscription(
                    PoseStamped, f"/hands/{method}/{side}/pose",
                    lambda message, key=key: poses.__setitem__(key, message), 1,
                ))
                subscriptions.append(client.create_subscription(
                    Bool, f"/hands/{method}/{side}/valid",
                    lambda message, key=key: valid.__setitem__(key, message.data), 1,
                ))
            subscriptions.append(client.create_subscription(
                MarkerArray, f"/hands/{method}/markers",
                lambda message, method=method: markers.__setitem__(method, message), 1,
            ))
        subscriptions.extend((
            client.create_subscription(Bool, "/hands/comparison/left/valid",
                                       lambda msg: differences.__setitem__("valid", msg.data), 1),
            client.create_subscription(Float32, "/hands/comparison/left/position_difference_m",
                                       lambda msg: differences.__setitem__("position", msg.data), 1),
            client.create_subscription(Float32, "/hands/comparison/left/orientation_difference_deg",
                                       lambda msg: differences.__setitem__("orientation", msg.data), 1),
        ))
        buffer = Buffer()
        listener = TransformListener(buffer, client)

        def spin(duration, include_rgb=True, transmit=True):
            deadline = time.monotonic() + duration
            next_send = 0
            while time.monotonic() < deadline:
                if transmit and time.monotonic() >= next_send:
                    packet = {
                        "version": 1, "frame_id": FRAME_ID, "sequence": 1,
                        "capture_monotonic_ns": time.monotonic_ns(), "capture_time_ns": time.time_ns(),
                        "hands": [{
                            "label": side, "position": [x, 0, 0.6], "orientation": [0, 0, 0, 1],
                            "joints": [[x, 0, 0.6] for _ in range(21)], "quality": 0.9,
                            "pose_source": "rgbd",
                        } for side, x in (("Left", -0.1), ("Right", 0.1))],
                        "comparison": {"Left": {
                            "position_difference_m": 0.01, "orientation_difference_deg": 0,
                            "delta_m": [0.01, 0, 0], "rgb_position": [-0.09, 0, 0.6],
                        } if include_rgb else None, "Right": None},
                    }
                    sender.sendto(json.dumps(packet).encode(), depth.socket.getsockname())
                    other = dict(packet)
                    other.pop("comparison")
                    other["hands"] = [dict(hand, position=[hand["position"][0] + 0.01, 0, 0.6],
                                           pose_source="rgb_pnp") for hand in packet["hands"]] if include_rgb else []
                    sender.sendto(json.dumps(other).encode(), rgb.socket.getsockname())
                    next_send = time.monotonic() + 0.03
                executor.spin_once(timeout_sec=0.01)

        try:
            spin(1.2)
            self.assertEqual(len(poses), 4)
            self.assertTrue(all(valid.values()))
            self.assertTrue(differences["valid"])
            self.assertAlmostEqual(differences["position"], 0.01, places=6)
            self.assertEqual(differences["orientation"], 0)
            depth_transform = buffer.lookup_transform(FRAME_ID, "rgbd_left_hand_palm", rclpy.time.Time())
            rgb_transform = buffer.lookup_transform(FRAME_ID, "rgb_left_hand_palm", rclpy.time.Time())
            self.assertAlmostEqual(rgb_transform.transform.translation.x - depth_transform.transform.translation.x, 0.01)
            self.assertEqual(poses["rgbd", "left"].header.stamp, poses["rgb", "left"].header.stamp)
            depth_dots = next(item for item in markers["rgbd"].markers if item.id == 4 and item.ns == "left")
            rgb_dots = next(item for item in markers["rgb"].markers if item.id == 4 and item.ns == "left")
            self.assertNotEqual(depth_dots.color, rgb_dots.color)
            self.assertTrue(any(item.id == 8 and "10.0 mm" in item.text for item in markers["rgbd"].markers))
            spin(0.3, include_rgb=False)
            self.assertTrue(valid["rgbd", "left"])
            self.assertFalse(valid["rgb", "left"])
            self.assertFalse(differences["valid"])
            self.assertTrue(any(item.id == 8 and item.action == Marker.DELETE for item in markers["rgbd"].markers))
            spin(0.35, transmit=False)
            self.assertFalse(any(valid.values()))
        finally:
            sender.close()
            for node in (client, rgb, depth):
                executor.remove_node(node)
                node.destroy_node()
            executor.shutdown()


if __name__ == "__main__":
    unittest.main()
