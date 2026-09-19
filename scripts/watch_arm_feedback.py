#!/usr/bin/env python3
"""Print one arm's seven joint-position feedback values at a controlled rate."""

import argparse
import math
import sys
import time


JOINT_COUNT = 7


class FeedbackThrottle:
    """Allow one print at most every 1 / rate_hz seconds."""

    def __init__(self, rate_hz: float) -> None:
        if not math.isfinite(rate_hz) or rate_hz <= 0.0:
            raise ValueError("rate_hz must be finite and positive")
        self.interval_seconds = 1.0 / rate_hz
        self.last_print_time: float | None = None

    def ready(self, timestamp: float) -> bool:
        if not math.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        return (
            self.last_print_time is None
            or timestamp - self.last_print_time >= self.interval_seconds
        )

    def record(self, timestamp: float) -> None:
        self.last_print_time = timestamp


def format_joint_feedback(side: str, positions) -> str:
    """Render exactly seven finite positions, with no non-position feedback."""
    if side not in ("left", "right"):
        raise ValueError("side must be left or right")
    if len(positions) != JOINT_COUNT:
        raise ValueError(f"Expected {JOINT_COUNT} joint positions")
    try:
        values = [float(position) for position in positions]
    except (TypeError, ValueError) as exc:
        raise ValueError("Joint position is not numeric") from exc
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Joint position is not finite")
    return f"{side}: " + " ".join(f"{value:+.4f}" for value in values) + " rad"


def format_both_arm_feedback(left_positions, right_positions) -> str:
    """Render both seven-joint feedback vectors on one line."""
    return (
        f"{format_joint_feedback('left', left_positions)} | "
        f"{format_joint_feedback('right', right_positions)}"
    )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right", "both"), required=True)
    parser.add_argument("--rate", type=float, default=3.0)
    args = parser.parse_args()
    if not math.isfinite(args.rate) or args.rate <= 0.0:
        parser.error("--rate must be finite and positive")
    return args


def main() -> None:
    args = parse_arguments()

    import rclpy
    from ctl_msgs_hw.msg import RetInformation
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = rclpy.create_node("towel_arm_feedback_watch")
    throttle = FeedbackThrottle(args.rate)

    def feedback(message: RetInformation) -> None:
        now = time.monotonic()
        if not throttle.ready(now):
            return
        try:
            if args.side == "both":
                rendered = format_both_arm_feedback(
                    message.l_position, message.r_position
                )
            else:
                positions = (
                    message.l_position if args.side == "left" else message.r_position
                )
                rendered = format_joint_feedback(args.side, positions)
            print(rendered, flush=True)
        except ValueError as error:
            print(f"Invalid {args.side}-arm feedback: {error}", file=sys.stderr, flush=True)
        finally:
            throttle.record(now)

    node.create_subscription(
        RetInformation,
        "/dexbotCtl/dualArmCtl_fdback",
        feedback,
        qos_profile_sensor_data,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
