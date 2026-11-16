"""Minimal ROS runtime retained after policy deployment is handed to VR."""

import time


def make_recording_session(recorder, cameras, *, resume_factory=None, check_worker=None):
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    class VRRecordingSession(Node):
        def __init__(self):
            super().__init__("vr_recording_session")
            self.recorder, self.cameras = recorder, cameras
            self.resume_factory = resume_factory
            self.resume_policy_requested = False
            self.resume_started_at = None
            self.mode = "vr"
            self.companion_nodes = [recorder]
            self.safety_state, self.safety_received_at = "waiting", None
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.mode_pub = self.create_publisher(String, "/towel/policy_mode", qos)
            self.create_subscription(String, "/towel/safety/state", self.receive_safety, qos)
            self.create_timer(.2, self.publish_mode)
            self.resume_timer = self.create_timer(.05, self.poll_resume)
            self.resume_timer.cancel()
            self.create_service(Trigger, "/towel/intervene", self.intervene)
            self.recorder.start_guard = self.recording_guard
            self.publish_mode()
            self.get_logger().warning(
                "VR recording-only session ready; policy bridge and inference threads are closed. " +
                ("B starts/saves; A discards while recording, resumes policy while idle; no homing"
                 if resume_factory else "B starts/saves, A discards. Save/discard then Ctrl+C; restart deployment manually"))

        def publish_mode(self):
            self.mode_pub.publish(String(data=self.mode))

        def intervene(self, request, response):
            if self.mode != "vr":
                response.success, response.message = False, "Policy resume is already pending"
            elif self.recorder.episode is not None:
                return self.recorder.discard(request, response)
            elif self.recorder.busy():
                response.success, response.message = False, "Episode writer is still stopping; press discard again when idle"
            elif self.resume_factory is None:
                response.success, response.message = False, "Worker was stopped; restart deployment manually"
            elif self.recording_guard():
                response.success, response.message = False, self.recording_guard()
            else:
                self.mode = "resuming_policy"
                self.resume_started_at = time.monotonic()
                self.resume_timer.reset()
                self.publish_mode()
                response.success, response.message = True, "Pausing VR; resuming policy without homing"
            return response

        def poll_resume(self):
            if self.resume_started_at is None or self.resume_policy_requested:
                return
            # Let the VR mode heartbeat gate all arm and gripper publishers
            # before requesting native policy ownership on a new bridge.
            if time.monotonic() - self.resume_started_at < .7:
                return
            try:
                reason = self.safety_guard()
                if reason:
                    raise RuntimeError(reason)
                check_worker()
                self.resume_policy_requested = True
                self.resume_timer.cancel()
            except Exception as exc:
                self.resume_timer.cancel()
                self.resume_started_at = None
                self.mode = "vr"
                self.publish_mode()
                self.get_logger().error(f"Policy resume blocked; VR retained: {exc}")

        def receive_safety(self, message):
            self.safety_state, self.safety_received_at = message.data, time.monotonic()
            if message.data.startswith("fault:") and self.recorder.episode is not None:
                self.recorder.abort_capture(f"Servo safety fault during VR recording: {message.data}")

        def recording_guard(self):
            if self.mode != "vr":
                return "Recording blocked while resuming policy"
            return self.safety_guard()

        def safety_guard(self):
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


def switch_to_policy(executor, session):
    from rclpy.executors import MultiThreadedExecutor
    from ros_shutdown import drain_executor

    nodes = executor.get_nodes()
    drain_executor(executor)
    for node in nodes:
        executor.remove_node(node)
    try:
        bridge = session.resume_factory(session.cameras, session.recorder)
    except Exception as exc:
        from rclpy.executors import SingleThreadedExecutor

        session.resume_policy_requested = False
        session.resume_started_at = None
        session.mode = "vr"
        session.recorder.start_guard = session.recording_guard
        session.publish_mode()
        session.get_logger().error(f"Cannot rebuild deployment; VR retained: {exc}")
        single = SingleThreadedExecutor()
        for node in nodes:
            single.add_node(node)
        return single, session
    bridge.resume_from_vr()
    session.destroy_node()
    multi = MultiThreadedExecutor(num_threads=3)
    multi.add_node(bridge)
    for node in nodes:
        if node is not session:
            multi.add_node(node)
    return multi, bridge
