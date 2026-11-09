"""Publish local RGB-D hand packets as ROS 2 poses, TF, and RViz markers."""

import argparse
import json
import math
import socket
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float32
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from pose_protocol import DEFAULT_PORT, FRAME_ID, STALE_SECONDS, validate_packet


CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12), (9, 13), (13, 14), (14, 15),
    (15, 16), (13, 17), (0, 17), (17, 18), (18, 19), (19, 20),
)


def set_pose(pose, position, quaternion):
    pose.position.x, pose.position.y, pose.position.z = map(float, position)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, quaternion)


def point(xyz):
    return Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))


class HandBridge(Node):
    def __init__(self, port=DEFAULT_PORT, timeout=STALE_SECONDS, namespace="/hands", tf_prefix="", style="default"):
        super().__init__("rgbd_hand_pose_bridge" if style == "default" else f"{style}_hand_pose_bridge")
        self.namespace = namespace.rstrip("/")
        self.tf_prefix = tf_prefix
        self.style = style
        self.timeout = timeout
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.bind(("127.0.0.1", port))
        except OSError:
            self.socket.close()
            raise
        self.socket.setblocking(False)
        self.markers = self.create_publisher(MarkerArray, f"{self.namespace}/markers", 1)
        self.poses = {
            side: self.create_publisher(PoseStamped, f"{self.namespace}/{side}/pose", 1)
            for side in ("left", "right")
        }
        self.valid = {
            side: self.create_publisher(Bool, f"{self.namespace}/{side}/valid", 1)
            for side in self.poses
        }
        self.quality = {
            side: self.create_publisher(Float32, f"{self.namespace}/{side}/quality", 1)
            for side in self.poses
        }
        self.comparison_publishers = {
            side: {
                "valid": self.create_publisher(Bool, f"/hands/comparison/{side}/valid", 1),
                "position": self.create_publisher(Float32, f"/hands/comparison/{side}/position_difference_m", 1),
                "orientation": self.create_publisher(Float32, f"/hands/comparison/{side}/orientation_difference_deg", 1),
            } for side in ("left", "right")
        } if style == "rgbd" else {}
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "camera_link"
        transform.child_frame_id = FRAME_ID
        # Optical X right / Y down / Z forward -> camera X forward / Y left / Z up.
        transform.transform.rotation.x = -0.5
        transform.transform.rotation.y = 0.5
        transform.transform.rotation.z = -0.5
        transform.transform.rotation.w = 0.5
        if style != "rgb":
            self.static_tf.sendTransform(transform)
        self.last_capture_ns = None
        self.active = set()
        self.received_frames = 0
        self.pose_frames = 0
        self.dual_pose_frames = 0
        self.timer = self.create_timer(1.0 / 60.0, self.receive)
        self.expire()
        self.get_logger().info(f"Listening on 127.0.0.1:{port}; topics {self.namespace}; fixed frame camera_link")

    def marker(self, side, marker_id, marker_type, stamp):
        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.header.stamp = stamp
        marker.ns = side
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        marker.lifetime.sec = int(self.timeout)
        marker.lifetime.nanosec = int((self.timeout % 1.0) * 1e9)
        return marker

    def hand_markers(self, hand, stamp, comparison=None):
        side = hand["label"].lower()
        position, quaternion = hand["position"], hand["orientation"]
        color = (0.1, 0.75, 1.0) if side == "left" else (1.0, 0.55, 0.15)
        if self.style in ("rgbd", "rgb"):
            color = (0.05, 0.85, 1.0) if self.style == "rgbd" else (1.0, 0.2, 0.7)
        output = []
        for axis in range(3):
            marker = self.marker(side, axis, Marker.ARROW, stamp)
            set_pose(marker.pose, position, quaternion)
            endpoint = [0.0, 0.0, 0.0]
            endpoint[axis] = 0.075 if self.style == "rgb" else 0.10
            marker.points = [point((0, 0, 0)), point(endpoint)]
            marker.scale.x, marker.scale.y, marker.scale.z = (
                (0.003, 0.008, 0.012) if self.style == "rgb" else (0.005, 0.012, 0.018)
            )
            marker.color.r, marker.color.g, marker.color.b = (
                (1.0, 0.15, 0.15) if axis == 0
                else (0.15, 1.0, 0.15) if axis == 1 else (0.2, 0.4, 1.0)
            )
            output.append(marker)
        joints = hand["joints"]
        bones = self.marker(side, 3, Marker.LINE_LIST, stamp)
        bones.scale.x = 0.004 if self.style == "rgbd" else 0.002 if self.style == "rgb" else 0.003
        bones.color.r, bones.color.g, bones.color.b = color
        for start, end in CONNECTIONS:
            if joints[start] is not None and joints[end] is not None:
                bones.points.extend((point(joints[start]), point(joints[end])))
        output.append(bones)
        dots = self.marker(side, 4, Marker.SPHERE_LIST, stamp)
        dots.scale.x = dots.scale.y = dots.scale.z = 0.006 if self.style == "rgb" else 0.008
        dots.color.r, dots.color.g, dots.color.b = color
        dots.points = [point(joint) for joint in joints if joint is not None]
        output.append(dots)
        center = self.marker(side, 5, Marker.SPHERE, stamp)
        set_pose(center.pose, position, quaternion)
        center.scale.x = center.scale.y = center.scale.z = 0.015
        center.color.r, center.color.g, center.color.b = color
        output.append(center)
        label = self.marker(side, 6, Marker.TEXT_VIEW_FACING, stamp)
        label.pose.position = point((position[0], position[1] + (0.10 if self.style == "rgb" else -0.10), position[2]))
        label.scale.z = 0.025
        label.color.r = label.color.g = label.color.b = 1.0
        label.text = (
            f"{hand['label']}  {position[0]:+.2f}, {position[1]:+.2f}, {position[2]:+.2f} m"
        )
        if hand.get("pose_source") == "rgb_pnp":
            label.text += " [RGB PnP]"
        elif self.style == "rgbd":
            label.text += " [RGB-D]"
        output.append(label)
        if comparison is not None:
            link = self.marker(side, 7, Marker.LINE_LIST, stamp)
            link.scale.x = 0.003
            link.color.r, link.color.g, link.color.b = (1.0, 0.9, 0.2)
            link.points = [point(position), point(comparison["rgb_position"])]
            output.append(link)
            difference = self.marker(side, 8, Marker.TEXT_VIEW_FACING, stamp)
            difference.pose.position = point((position[0], position[1] - 0.14, position[2]))
            difference.scale.z = 0.018
            difference.color.r, difference.color.g, difference.color.b = (1.0, 0.9, 0.2)
            difference.text = (
                f"{hand['label']} delta: {comparison['position_difference_m'] * 1000:.1f} mm / "
                f"{comparison['orientation_difference_deg']:.1f} deg"
            )
            output.append(difference)
        elif self.style == "rgbd":
            for marker_id in (7, 8):
                marker = self.marker(side, marker_id, Marker.LINE_LIST, stamp)
                marker.action = Marker.DELETE
                output.append(marker)
        return output

    def remove_markers(self, sides):
        output = []
        for side in sides:
            for marker_id in range(9 if self.style == "rgbd" else 7):
                marker = Marker()
                marker.header.frame_id = FRAME_ID
                marker.ns, marker.id, marker.action = side, marker_id, Marker.DELETE
                output.append(marker)
        return output

    def publish_packet(self, packet):
        ns = packet["capture_time_ns"]
        stamp = self.get_clock().now().to_msg()
        stamp.sec, stamp.nanosec = divmod(ns, 1_000_000_000)
        hands = packet["hands"]
        active = {hand["label"].lower() for hand in hands}
        markers = self.remove_markers(self.active - active)
        transforms = []
        for hand in hands:
            side = hand["label"].lower()
            pose = PoseStamped()
            pose.header.frame_id, pose.header.stamp = FRAME_ID, stamp
            set_pose(pose.pose, hand["position"], hand["orientation"])
            self.poses[side].publish(pose)
            quality = float(hand.get("quality", 0.0))
            self.quality[side].publish(Float32(data=quality if math.isfinite(quality) else 0.0))
            transform = TransformStamped()
            transform.header = pose.header
            transform.child_frame_id = f"{self.tf_prefix}{side}_hand_palm"
            transform.transform.translation.x = pose.pose.position.x
            transform.transform.translation.y = pose.pose.position.y
            transform.transform.translation.z = pose.pose.position.z
            transform.transform.rotation = pose.pose.orientation
            transforms.append(transform)
            comparison = packet.get("comparison", {}).get(hand["label"])
            markers.extend(self.hand_markers(hand, stamp, comparison))
        if transforms:
            self.tf.sendTransform(transforms)
        for side in self.valid:
            self.valid[side].publish(Bool(data=side in active))
            if side not in active:
                self.quality[side].publish(Float32(data=0.0))
        for side, publishers in self.comparison_publishers.items():
            difference = packet.get("comparison", {}).get(side.title())
            publishers["valid"].publish(Bool(data=difference is not None))
            if difference is not None:
                publishers["position"].publish(Float32(data=float(difference["position_difference_m"])))
                publishers["orientation"].publish(Float32(data=float(difference["orientation_difference_deg"])))
        self.markers.publish(MarkerArray(markers=markers))
        self.active = active
        self.last_capture_ns = packet["capture_monotonic_ns"]
        self.received_frames += 1
        self.pose_frames += bool(hands)
        self.dual_pose_frames += len(hands) == 2

    def expire(self):
        self.markers.publish(MarkerArray(markers=self.remove_markers(self.active)))
        for side in self.valid:
            self.valid[side].publish(Bool(data=False))
            self.quality[side].publish(Float32(data=0.0))
        for publishers in self.comparison_publishers.values():
            publishers["valid"].publish(Bool(data=False))
        self.active.clear()
        self.last_capture_ns = None

    def receive(self):
        latest = None
        for _ in range(100):
            try:
                data, _ = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            try:
                candidate = json.loads(data)
                validate_packet(candidate, time.monotonic_ns(), self.timeout)
                latest = candidate
            except (ValueError, TypeError, AttributeError, KeyError):
                continue
        if latest is not None:
            self.publish_packet(latest)
        if (
            self.last_capture_ns is not None
            and (time.monotonic_ns() - self.last_capture_ns) / 1e9 > self.timeout
        ):
            self.expire()

    def destroy_node(self):
        self.expire()
        self.socket.close()
        return super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--timeout", type=float, default=STALE_SECONDS)
    parser.add_argument("--namespace", default="/hands")
    parser.add_argument("--tf-prefix", default="")
    parser.add_argument("--style", choices=("default", "rgbd", "rgb"), default="default")
    args = parser.parse_args()
    if not (1 <= args.port <= 65535 and 0.05 <= args.timeout <= 2.0):
        parser.error("Invalid UDP port or timeout")
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = HandBridge(args.port, args.timeout, args.namespace, args.tf_prefix, args.style)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            print(
                f"ROS bridge stopped: {node.received_frames} packets, "
                f"{node.pose_frames} with poses, {node.dual_pose_frames} with both hands.",
                flush=True,
            )
            if rclpy.ok():
                node.destroy_node()
            else:
                node.socket.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
