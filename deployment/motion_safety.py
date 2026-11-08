"""Latched safety heartbeat and checks shared by replay and policy execution."""

import time
import threading
import numpy as np

JOINTS = np.r_[0:7, 8:15]


class MotionFault(ValueError):
    pass


def speed_only_mode(config):
    mode = config.get("motion_safety", {}).get("protection_mode", "full")
    if mode not in ("full", "speed_only"):
        raise ValueError("motion_safety.protection_mode must be full or speed_only")
    return mode == "speed_only"


def speed_limit_mask(settings):
    enabled = settings.get("speed_limit_enabled_per_arm", [True] * 7)
    if not isinstance(enabled, list) or len(enabled) != 7 or any(type(v) is not bool for v in enabled):
        raise ValueError("speed_limit_enabled_per_arm must contain 7 booleans")
    return np.tile(enabled, 2)


def configured_position_limits(config, lower, upper):
    """Apply one absolute-position switch to planning, clipping and feedback checks."""
    enabled = config.get("motion_safety", {}).get("check_position_limits", True)
    if not isinstance(enabled, bool):
        raise ValueError("motion_safety.check_position_limits must be a boolean")
    lower, upper = np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)
    if enabled and not speed_only_mode(config):
        return lower, upper
    return np.full_like(lower, -np.inf), np.full_like(upper, np.inf)


class SafetyState:
    def __init__(self, stop, *, now=time.monotonic, defer_receive_fault=False,
                 arm_on_publish=False, startup_settle_s=0.0, monitor_heartbeat=True):
        self.now, self.stop = now, stop
        self._lock = threading.RLock()
        self._notified = False
        self.defer_receive_fault = defer_receive_fault
        self.arm_on_publish = arm_on_publish
        self.monitor_heartbeat = monitor_heartbeat
        self.startup_settle_s = float(startup_settle_s)
        if not np.isfinite(self.startup_settle_s) or not 0 <= self.startup_settle_s < 5:
            raise ValueError("Invalid safety startup settling time")
        self.ready_since = None
        self.ready_samples = 0
        self.started = now()
        self.received = None
        self.state = "waiting"
        self.fault = ""
        self.was_ready = False

    def trip(self, reason):
        with self._lock:
            if not self.fault:
                self.fault = str(reason)
        self._notify_fault()

    def _notify_fault(self):
        with self._lock:
            if not self.fault or self._notified:
                return
            self._notified = True
            reason = self.fault
        self.stop(reason)

    def receive(self, state):
        with self._lock:
            now = self.now()
            continuous = (self.state == "ready" and self.received is not None and
                          0 <= now - self.received <= 0.2)
            if state == "ready":
                if not continuous:
                    self.ready_since = now
                    self.ready_samples = 0
                self.ready_samples += 1
            else:
                self.ready_since = None
                self.ready_samples = 0
            self.received, self.state = now, state
            if state.startswith("fault:") and not self.fault:
                self.fault = state
        # An independent receiver latches immediately, but controller/VR cleanup
        # must run in the control callback group on the next ready() check.
        if not self.defer_receive_fault:
            self._notify_fault()

    def diagnostics(self):
        with self._lock:
            age = "missing" if self.received is None else f"{self.now() - self.received:.3f}"
            return (f"servo_state={self.state!r} heartbeat_age_s={age} heartbeat_timeout_s=0.200 "
                    f"safety_phase={'armed' if self.was_ready else 'startup'} ready_samples={self.ready_samples}")

    def ready(self):
        with self._lock:
            now = self.now()
            healthy = self.received is not None and 0 <= now - self.received <= 0.2 and self.state == "ready"
            settled = (self.startup_settle_s == 0 or self.was_ready or
                       (self.ready_since is not None and self.ready_samples >= 3 and
                        self.received - self.ready_since >= self.startup_settle_s))
            available = healthy and settled
            if not self.monitor_heartbeat:
                available = True
            if not self.fault:
                if available and not self.arm_on_publish:
                    self.was_ready = True
                elif not available and (self.was_ready or now - self.started > 5):
                    if self.received is None:
                        reason = "Servo safety heartbeat never received"
                    elif not 0 <= now - self.received <= 0.2:
                        reason = "Servo safety heartbeat expired"
                    elif self.state != "ready":
                        reason = "Servo safety state is not ready"
                    else:
                        reason = "Servo safety heartbeat did not stabilize during startup"
                    self.fault = f"{reason}; {self.diagnostics()}"
            result = available and not self.fault
        self._notify_fault()
        return result

    def publish_if_ready(self, publish):
        # Serialize the final publication with fault reception, not trajectory computation.
        with self._lock:
            if not self.ready():
                return False
            self.was_ready = True
            publish()
            return True


def attach_safety(node, on_fault, *, receive_callback_group=None, diagnostics=None,
                  arm_on_publish=False, startup_settle_s=0.0, monitor_heartbeat=True,
                  repeat_stop_until_ack=False):
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, DurabilityPolicy

    publisher = node.create_publisher(String, "/towel/safety/stop", 1)

    def stop(reason):
        if diagnostics is not None:
            reason = f"{reason}; {diagnostics()}"
        try:
            publisher.publish(String(data=reason))
        finally:
            on_fault(reason)

    state = SafetyState(stop, defer_receive_fault=receive_callback_group is not None,
                        arm_on_publish=arm_on_publish, startup_settle_s=startup_settle_s,
                        monitor_heartbeat=monitor_heartbeat)
    options = {} if receive_callback_group is None else {"callback_group": receive_callback_group}
    node.create_subscription(String, "/towel/safety/state", lambda msg: state.receive(msg.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL), **options)
    def update():
        state.ready()
        if repeat_stop_until_ack and state.fault and not state.state.startswith("fault:"):
            publisher.publish(String(data=state.fault))

    node.create_timer(0.05, update)
    return state


class CommandSafety:
    """Check emitted commands separately from target-to-feedback allowance."""

    def __init__(self, config, lower, upper, *, now=time.monotonic):
        settings = config.get("motion_safety", {})
        self.speed_only = speed_only_mode(config)
        self.max_velocity = float(settings.get("max_velocity_rad_s", 1.0))
        acceleration_limit = settings.get("max_acceleration_rad_s2", 5.0)
        self.max_acceleration = None if acceleration_limit is None else float(acceleration_limit)
        self.max_tracking_error = float(settings.get("max_tracking_error_rad", 0.3))
        self.tracking_timeout = float(settings.get("tracking_timeout_s", 0.1))
        values = [self.max_velocity, self.max_tracking_error, self.tracking_timeout]
        if self.max_acceleration is not None:
            values.append(self.max_acceleration)
        if not np.isfinite(values).all() or min(values) <= 0:
            raise ValueError("Invalid motion_safety limits")
        self.velocity_limits = np.where(speed_limit_mask(settings), self.max_velocity, np.inf)
        if self.speed_only:
            self.max_acceleration = None
        self.lower, self.upper = configured_position_limits(config, lower, upper)
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
        if dt <= 0 or (not self.speed_only and dt > 0.25):
            raise MotionFault("Command interval invalid or expired")
        v = (q - self.position) / dt
        a = (v - self.velocity) / dt
        if not np.isfinite(v).all() or not np.isfinite(a).all():
            raise MotionFault(f"Non-finite successive command derivative; dt_s={dt:.9g}")
        for kind, values, limit in (("velocity", v, self.velocity_limits),
                                    ("acceleration", a, self.max_acceleration)):
            if limit is None:
                continue
            exceeded = np.flatnonzero(np.abs(values) > limit + 1e-6)
            if len(exceeded):
                joint = int(exceeded[np.argmax(np.abs(values[exceeded]))])
                bound = float(np.broadcast_to(limit, (14,))[joint])
                side = "left" if joint < 7 else "right"
                raise MotionFault(
                    f"Successive command {kind} exceeds limits: {side}_joint_{joint % 7 + 1} "
                    f"previous_q={self.position[joint]:.9g} target_q={q[joint]:.9g} "
                    f"delta_q={q[joint] - self.position[joint]:.9g}rad dt_s={dt:.9g} "
                    f"velocity={v[joint]:.9g}rad/s acceleration={a[joint]:.9g}rad/s^2 "
                    f"max_abs_{kind}={bound:.9g}")
        for sample, limit in ((velocity, self.velocity_limits), (acceleration, self.max_acceleration)):
            if sample is not None and (not np.isfinite(sample).all() or
                                      (limit is not None and np.any(np.abs(sample) > limit + 1e-6))):
                raise MotionFault("Trajectory derivative exceeds limits")
        if not self.speed_only and np.any(np.abs(self.position - f) > self.max_tracking_error):
            if self.tracking_since is None:
                self.tracking_since = now
            if now - self.tracking_since >= self.tracking_timeout:
                raise MotionFault("Persistent tracking error")
        else:
            self.tracking_since = None
        self.position, self.velocity, self.last_time = q.copy(), v, now
        return action
