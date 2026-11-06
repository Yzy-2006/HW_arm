"""Acknowledged policy/VR handoff; all methods run in the bridge callback group."""

import time


class Intervention:
    def __init__(self, pause, resume, controller, recorder, report, *, now=time.monotonic,
                 save_timeout_s=11.0, recorder_ready=None, startup_timeout_s=4.0):
        self.pause = pause
        self.resume = resume
        self.controller = controller
        self.recorder = recorder
        self.report = report
        self.now = now
        self.save_timeout_s = save_timeout_s
        self.recorder_ready = recorder_ready
        self.startup_timeout_s = startup_timeout_s
        self.mode = "paused"
        self.pending = None
        self.deadline = 0.0
        self.recording = False
        self.error = ""

    def request(self, manual):
        if self.mode == "fault":
            return False, "Motion fault latched; reset servo fault and restart deployment"
        if self.pending is not None or self.mode in {"arming_vr", "saving", "resuming"}:
            return False, "Handoff in progress; inspect /towel/intervention/state"
        if manual and self.mode == "vr":
            return True, "Already in VR"
        if not manual and self.mode == "policy":
            return True, "Already in policy"
        self.mode = "paused"
        self.error = ""
        try:
            self.pause()
            if manual:
                self.mode = "arming_vr"
                self._call(self.controller, True, "controller_vr")
            else:
                # Freeze VR at the controller before finalizing the episode.
                self.mode = "saving"
                self._call(self.controller, False, "controller_hold")
            return True, "Handoff accepted; completion is reported on /towel/intervention/state"
        except Exception as exc:
            self.fail(exc)
            return False, str(exc)

    def _call(self, callback, value, stage):
        self.pending = callback(value)
        self.stage = stage
        self.deadline = self.now() + (self.save_timeout_s if stage in {"saved", "fault_saved"} else 5.0)

    def fail(self, reason):
        if self.mode == "fault":
            return
        self.mode = "paused"
        self.error = str(reason)
        self.report(f"Intervention paused: {reason}")
        # Do not clear a timed-out request: its server may still execute it.
        # Consume its eventual response without enabling either control source.

    def latch_fault(self, reason):
        self.mode = "fault"
        self.error = str(reason)
        self.report(f"Motion fault latched: {reason}")

    def tick(self):
        if self.mode == "fault":
            if self.pending is not None:
                if not self.pending.done():
                    return
                stage = self.stage
                self.pending = None
                if stage == "recording":
                    self.recording = True
                elif stage in {"saved", "fault_saved"}:
                    self.recording = False
            if self.recording:
                try:
                    self._call(self.recorder, False, "fault_saved")
                except Exception as exc:
                    self.report(f"Fault recording finalization failed: {exc}")
                    self.recording = False
            return
        if self.pending is None:
            if self.mode == "arming_vr" and self.stage == "first_frame":
                try:
                    if self.recorder_ready():
                        self.mode = "vr"
                        self.report("VR intervention recording started")
                    elif self.now() > self.deadline:
                        self.fail("Recorder first-frame acknowledgement timed out")
                except Exception as exc:
                    self.fail(exc)
            return
        if not self.pending.done():
            if self.now() > self.deadline and self.mode != "paused":
                self.fail("Service acknowledgement timed out")
            return
        future, self.pending = self.pending, None
        if self.mode == "paused":
            return
        try:
            result = future.result()
            if not result.success:
                raise RuntimeError(result.message)
            if self.stage == "controller_vr":
                self._call(self.recorder, True, "recording")
            elif self.stage == "recording":
                self.recording = True
                if self.recorder_ready is None:
                    self.mode = "vr"
                    self.report("VR intervention recording started")
                else:
                    self.stage = "first_frame"
                    self.deadline = self.now() + self.startup_timeout_s
            elif self.stage == "controller_hold":
                self._call(self.recorder, False, "saved")
            elif self.stage == "saved":
                self.recording = False
                self.report(result.message)
                self.resume()
                self.mode = "policy"
                self.report("Policy resumed from fresh observations")
        except Exception as exc:
            self.fail(exc)


def attach_intervention(node):
    import numpy as np
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from ctl_msgs_hw.msg import GripperCtl

    controller = node.create_client(SetBool, "/dexbotCtl/vr_override")

    def call(client, value):
        if not client.service_is_ready():
            raise RuntimeError(f"Service unavailable: {client.srv_name}")
        request = SetBool.Request()
        request.data = value
        return client.call_async(request)

    manager = Intervention(node.pause_policy, node.resume_policy,
                           lambda value: call(controller, value),
                           node.recorder.request, node.get_logger().info,
                           save_timeout_s=node.recorder.writer_finish_timeout_s + 1.0,
                           recorder_ready=node.recorder.ready_for_capture,
                           startup_timeout_s=node.recorder.startup_timeout_s + 1.0)
    node.intervention = manager
    publisher = node.create_publisher(String, "/towel/intervention/state", 1)

    def request(req, response):
        if node.safety is not None and not node.safety.ready():
            response.success = False
            response.message = "Handoff requires healthy servo safety state"
            return response
        response.success, response.message = manager.request(req.data)
        return response

    node.create_service(SetBool, "/towel/intervene", request)

    def tick():
        try:
            node.recorder.check_health()
        except Exception as exc:
            if manager.mode not in {"paused", "fault"}:
                node.pause_policy()
                manager.fail(exc)
        manager.tick()
        if manager.mode == "vr":
            try:
                if not node.recorder.ready_for_capture():
                    raise RuntimeError("Correction recorder is no longer ready")
            except Exception as exc:
                node.pause_policy()
                manager.fail(exc)
        publisher.publish(String(data=manager.mode))

    node.create_timer(0.05, tick)

    def gripper(msg):
        if manager.mode != "vr" or not node.motion or not node.safety.ready():
            return
        values = [msg.l_arm_gripper_pos, msg.r_arm_gripper_pos, msg.ctl_time]
        if np.isfinite(values).all() and 0 < msg.ctl_time <= 1:
            node.safety.publish_if_ready(lambda: node.gripper_pub.publish(msg))

    # Policy and VR grippers pass through the same serialized publisher.
    node.create_subscription(GripperCtl, "/towel/intervention/gripper_command", gripper, 1)
    node.get_logger().warning(
        "Intervention enabled; starts PAUSED. Start managed VR, then call "
        "/towel/intervene with data=false to start policy or data=true to record VR."
    )
