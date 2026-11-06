"""Wait for this replay's servo handoff and sustained physical arrival."""

import numpy as np

from motion_safety import MotionFault


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class ReplayStartGate:
    def __init__(self, settings, feedback_timeout):
        self.position_tolerance = float(settings.get("position_tolerance_rad", .05))
        self.velocity_tolerance = float(settings.get("velocity_tolerance_rad_s", .05))
        self.settle_time = float(settings.get("settle_time_s", .2))
        self.timeout = float(settings.get("settle_timeout_s", 10.))
        self.feedback_timeout = float(feedback_timeout)
        values = [self.position_tolerance, self.velocity_tolerance, self.settle_time,
                  self.timeout, self.feedback_timeout]
        if not np.isfinite(values).all() or min(values) <= 0 or self.timeout <= self.settle_time:
            raise ValueError("Invalid replay first-frame settling settings")
        self.started = self.settled_since = self.required_stamp = None
        self.status_at = None
        self.source_stamp = 0
        self.complete = False
        self.command = None
        self.reason = "waiting for first-frame command acknowledgement"

    def observe(self, source_stamp, complete, command, now):
        command = np.asarray(command, dtype=float)
        if command.shape != (14,) or not np.isfinite(command).all():
            self.status_at = None
            return
        self.source_stamp, self.complete = source_stamp, bool(complete)
        self.command, self.status_at = command.copy(), now

    def command_sent(self, stamp):
        if self.required_stamp is None:
            self.required_stamp = stamp

    def update(self, now, target, feedback, velocity, feedback_at):
        if self.started is None:
            self.started = now
        feedback = np.asarray(feedback, dtype=float)
        velocity = np.asarray(velocity, dtype=float)
        valid_feedback = (feedback.shape == (14,) and velocity.shape == (14,) and
                          np.isfinite(feedback).all() and np.isfinite(velocity).all() and
                          0 <= now - feedback_at <= self.feedback_timeout)
        if self.status_at is None or not 0 <= now - self.status_at <= self.feedback_timeout:
            self.reason = "servo handoff status missing or stale"
        elif self.required_stamp is None or self.source_stamp < self.required_stamp:
            self.reason = "servo has not acknowledged this first-frame command"
        elif not self.complete:
            self.reason = "servo handoff still active"
        elif np.max(np.abs(self.command - target)) > 1e-4:
            self.reason = "servo command has not reached the first frame"
        elif not valid_feedback:
            self.reason = "joint position/velocity feedback missing or stale"
        else:
            error = np.abs(feedback - target)
            speed = np.abs(velocity)
            if np.max(error) > self.position_tolerance:
                self.reason = f"joint {np.argmax(error) + 1} position error={np.max(error):.6f}rad"
            elif np.max(speed) > self.velocity_tolerance:
                self.reason = f"joint {np.argmax(speed) + 1} feedback speed={np.max(speed):.6f}rad/s"
            else:
                self.reason = "settling at the first frame"
                if self.settled_since is None:
                    self.settled_since = now
                if now - self.settled_since >= self.settle_time:
                    return True
                if now - self.started >= self.timeout:
                    raise MotionFault(f"First-frame settle timeout: {self.reason}")
                return False
        self.settled_since = None
        if now - self.started >= self.timeout:
            raise MotionFault(f"First-frame settle timeout: {self.reason}")
        return False
