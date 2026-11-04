"""Latched safety heartbeat and checks shared by replay and policy execution."""

import time
import numpy as np

JOINTS = np.r_[0:7, 8:15]


class MotionFault(ValueError):
    pass


class SafetyState:
    def __init__(self, stop, *, now=time.monotonic):
        self.now, self.stop = now, stop
        self.started = now()
        self.received = None
        self.state = "waiting"
        self.fault = ""
        self.was_ready = False

    def trip(self, reason):
        if not self.fault:
            self.fault = str(reason)
            self.stop(self.fault)

    def receive(self, state):
        self.received, self.state = self.now(), state
        if state.startswith("fault:"):
            self.trip(state)

    def ready(self):
        if self.fault:
            return False
        now = self.now()
        healthy = self.received is not None and now - self.received <= 0.2 and self.state == "ready"
        if healthy:
            self.was_ready = True
        elif self.was_ready or now - self.started > 5:
            self.trip("Servo safety heartbeat missing or feedback invalid")
        return healthy and not self.fault


def attach_safety(node, on_fault):
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, DurabilityPolicy

    publisher = node.create_publisher(String, "/towel/safety/stop", 1)

    def stop(reason):
        try:
            publisher.publish(String(data=reason))
        finally:
            on_fault(reason)

    state = SafetyState(stop)
    node.create_subscription(String, "/towel/safety/state", lambda msg: state.receive(msg.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_timer(0.05, state.ready)
    return state


class CommandSafety:
    """Check emitted commands separately from target-to-feedback allowance."""

    def __init__(self, config, lower, upper, *, now=time.monotonic):
        settings = config.get("motion_safety", {})
        self.max_velocity = float(settings.get("max_velocity_rad_s", 1.0))
        self.max_acceleration = float(settings.get("max_acceleration_rad_s2", 5.0))
        self.max_tracking_error = float(settings.get("max_tracking_error_rad", 0.3))
        self.tracking_timeout = float(settings.get("tracking_timeout_s", 0.1))
        values = [self.max_velocity, self.max_acceleration, self.max_tracking_error, self.tracking_timeout]
        if not np.isfinite(values).all() or min(values) <= 0:
            raise ValueError("Invalid motion_safety limits")
        self.lower, self.upper = np.asarray(lower), np.asarray(upper)
        self.now = now
        self.reset()

    def reset(self):
        self.position = self.velocity = self.last_time = self.tracking_since = None

    def check(self, action, feedback, period, velocity=None, acceleration=None):
        q = np.asarray(action)[JOINTS]
        f = np.asarray(feedback)[JOINTS]
        if not np.isfinite(q).all() or not np.isfinite(f).all():
            raise MotionFault("Non-finite joint command or feedback")
        if np.any(f < self.lower - 0.02) or np.any(f > self.upper + 0.02):
            raise MotionFault("Joint feedback outside absolute limits")
        if np.any(q < self.lower) or np.any(q > self.upper):
            raise MotionFault("Joint command outside absolute limits")
        now = self.now()
        if self.position is None:
            self.position, self.velocity, self.last_time = f.copy(), np.zeros(14), now - period
        dt = now - self.last_time
        if dt == 0 and np.array_equal(q, self.position):
            return action
        if dt <= 0 or dt > 0.25:
            raise MotionFault("Command interval invalid or expired")
        v = (q - self.position) / dt
        a = (v - self.velocity) / dt
        if np.any(np.abs(v) > self.max_velocity + 1e-6) or np.any(np.abs(a) > self.max_acceleration + 1e-6):
            raise MotionFault("Successive command velocity or acceleration exceeds limits")
        for sample, limit in ((velocity, self.max_velocity), (acceleration, self.max_acceleration)):
            if sample is not None and (not np.isfinite(sample).all() or np.any(np.abs(sample) > limit + 1e-6)):
                raise MotionFault("Trajectory derivative exceeds limits")
        if np.any(np.abs(self.position - f) > self.max_tracking_error):
            if self.tracking_since is None:
                self.tracking_since = now
            if now - self.tracking_since >= self.tracking_timeout:
                raise MotionFault("Persistent tracking error")
        else:
            self.tracking_since = None
        self.position, self.velocity, self.last_time = q.copy(), v, now
        return action
