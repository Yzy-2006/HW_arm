"""Coalesce high-rate ROS telemetry before dispatching Python callbacks."""

import copy
import math
import threading
import time
import weakref


def feedback_qos():
    from rclpy.qos import qos_profile_sensor_data

    qos = copy.copy(qos_profile_sensor_data)
    qos.depth = 1
    return qos


def feedback_event_callbacks():
    from rclpy.event_handler import SubscriptionEventCallbacks

    # Jazzy/RMW default QoS event waitables can spin under high-rate telemetry.
    # Feedback validity/age and explicit servo faults are checked separately.
    return SubscriptionEventCallbacks(use_default_callbacks=False)


def make_feedback_group(node, rate_hz):
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

    if isinstance(rate_hz, bool) or not isinstance(rate_hz, (int, float)) or not math.isfinite(rate_hz) or rate_hz <= 0:
        raise ValueError("feedback_rate_hz must be finite and positive")
    period = 1.0 / rate_hz

    class LatestFeedbackGroup(MutuallyExclusiveCallbackGroup):
        def __init__(self):
            super().__init__()
            self.schedule_lock = threading.Lock()
            self.next_due = weakref.WeakKeyDictionary()

        def can_execute(self, entity):
            with self.schedule_lock:
                return time.monotonic() >= self.next_due.get(entity, -float("inf")) and super().can_execute(entity)

        def beginning_execution(self, entity):
            with self.schedule_lock:
                now = time.monotonic()
                if now < self.next_due.get(entity, -float("inf")) or not super().beginning_execution(entity):
                    return False
                self.next_due[entity] = now + period
                return True

    # A separate, unrestricted timer wakes the executor when throttled readers
    # are excluded from its wait set. DDS keeps only their newest messages.
    node.create_timer(period, lambda: None, callback_group=MutuallyExclusiveCallbackGroup())
    return LatestFeedbackGroup()
