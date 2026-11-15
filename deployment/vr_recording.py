"""Minimal ROS runtime retained after policy deployment is handed to VR."""

import time


def make_recording_session(recorder, cameras):
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String

    class VRRecordingSession(Node):
        def __init__(self):
            super().__init__("vr_recording_session")
            self.recorder, self.cameras = recorder, cameras
            self.companion_nodes = [recorder]
            self.safety_state, self.safety_received_at = "waiting", None
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.mode_pub = self.create_publisher(String, "/towel/policy_mode", qos)
            self.create_subscription(String, "/towel/safety/state", self.receive_safety, qos)
            self.create_timer(.2, self.publish_mode)
            self.recorder.start_guard = self.recording_guard
            self.publish_mode()
            self.get_logger().warning(
                "VR recording-only session ready; policy bridge and inference threads are closed. "
                "B starts/saves, A discards. Save/discard then Ctrl+C; restart make pi05 and make motion manually")

        def publish_mode(self):
            self.mode_pub.publish(String(data="vr"))

        def receive_safety(self, message):
            self.safety_state, self.safety_received_at = message.data, time.monotonic()
            if message.data.startswith("fault:") and self.recorder.episode is not None:
                self.recorder.abort_capture(f"Servo safety fault during VR recording: {message.data}")

        def recording_guard(self):
            if self.safety_state != "ready" or self.safety_received_at is None:
                return f"Waiting for servo safety ready: {self.safety_state}"
            if time.monotonic() - self.safety_received_at > .25:
                return "Waiting for fresh servo safety state"
            return None

        def close(self):
            try:
                self.mode_pub.publish(String(data="stopped"))
            finally:
                try:
                    self.recorder.close()
                finally:
                    self.companion_nodes.clear()
                    self.cameras.close()

    return VRRecordingSession()


def switch_to_recording_only(executor, bridge):
    from rclpy.executors import SingleThreadedExecutor
    from ros_shutdown import drain_executor

    nodes = executor.get_nodes()
    drain_executor(executor)
    for node in nodes:
        executor.remove_node(node)
    session = bridge.release_to_recording_only()
    bridge.destroy_node()
    single = SingleThreadedExecutor()
    single.add_node(session)
    for node in nodes:
        if node is not bridge:
            single.add_node(node)
    return single, session
