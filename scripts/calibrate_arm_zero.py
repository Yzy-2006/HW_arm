#!/usr/bin/env python3
"""Capture a stable arm feedback pose and save it as source zero positions.

This tool is intentionally passive: it only subscribes to feedback and edits
the source YAML after a complete, stable sample window has been validated.
Run it only while the matching arm is in its dedicated passive calibration
mode; it never enables motors or publishes a control command.
"""

import argparse
import math
import os
from pathlib import Path
import re
import statistics
import tempfile
import time

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "ros2_ws/src/ctl_msgs_hw/config/zeropos.yaml"
ARM_JOINT_KEYS = {
    "left": (
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
    ),
    "right": (
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    ),
}
MIN_SAMPLES = 20
MAX_POSITION_SPREAD_RAD = 0.01
NONZERO_POSITION_EPSILON_RAD = 1e-6


class FeedbackWindow:
    """Collect one contiguous feedback window without ROS runtime dependencies."""

    def __init__(self, max_gap_seconds: float = 0.2) -> None:
        if not math.isfinite(max_gap_seconds) or max_gap_seconds <= 0.0:
            raise ValueError("max_gap_seconds must be finite and positive")
        self.max_gap_seconds = max_gap_seconds
        self.samples: list[tuple[float, ...]] = []
        self.first_sample_time: float | None = None
        self.last_sample_time: float | None = None

    def reset(self) -> None:
        self.samples.clear()
        self.first_sample_time = None
        self.last_sample_time = None

    def add(self, timestamp: float, raw_positions) -> None:
        if not math.isfinite(timestamp):
            raise ValueError("Feedback timestamp is not finite")
        if (
            self.last_sample_time is not None
            and timestamp - self.last_sample_time > self.max_gap_seconds
        ):
            self.reset()
        if len(raw_positions) != 7:
            self.reset()
            return
        self.samples.append(tuple(raw_positions))
        self.first_sample_time = (
            timestamp if self.first_sample_time is None else self.first_sample_time
        )
        self.last_sample_time = timestamp

    def is_ready(self, duration_seconds: float) -> bool:
        if not math.isfinite(duration_seconds) or duration_seconds <= 0.0:
            raise ValueError("duration_seconds must be finite and positive")
        return (
            len(self.samples) >= MIN_SAMPLES
            and self.first_sample_time is not None
            and self.last_sample_time is not None
            and self.last_sample_time - self.first_sample_time >= duration_seconds
        )


def validate_pose_samples(samples: list[tuple[float, ...]]) -> list[float]:
    """Return a median seven-joint pose or reject unsafe feedback samples."""
    if len(samples) < MIN_SAMPLES:
        raise ValueError(f"Need at least {MIN_SAMPLES} feedback samples")
    if any(len(sample) != 7 for sample in samples):
        raise ValueError("Expected exactly seven joint positions per feedback sample")

    values = []
    for sample in samples:
        try:
            pose = tuple(float(value) for value in sample)
        except (TypeError, ValueError) as exc:
            raise ValueError("Feedback contains a non-numeric position") from exc
        if not all(math.isfinite(value) for value in pose):
            raise ValueError("Feedback contains a non-finite position")
        values.append(pose)

    for index in range(7):
        joint_values = [sample[index] for sample in values]
        if max(joint_values) - min(joint_values) > MAX_POSITION_SPREAD_RAD:
            raise ValueError("Arm is moving; hold the calibration pose still and retry")

    median_pose = [statistics.median(sample[index] for sample in values) for index in range(7)]
    if all(abs(value) <= NONZERO_POSITION_EPSILON_RAD for value in median_pose):
        raise ValueError(
            "Feedback is all zero; check left-arm power, green motor LEDs, and EtherCAT"
        )
    return median_pose


def replace_arm_positions_in_yaml(
    original: str, arm_key: str, updated_pose: dict[str, float]
) -> str:
    """Replace only existing arm-position scalars while preserving all other text."""
    in_target_arm = False
    seen_keys: set[str] = set()
    rendered_lines = []
    for line in original.splitlines(keepends=True):
        section = re.match(r"^([^\s#][^:]*):(?:\s*(?:#.*)?)?(?:\r?\n)?$", line)
        if section:
            in_target_arm = section.group(1).strip() == arm_key
            rendered_lines.append(line)
            continue

        if not in_target_arm:
            rendered_lines.append(line)
            continue

        rendered_line = line
        for joint_key, value in updated_pose.items():
            match = re.match(
                rf"^(\s*{re.escape(joint_key)}\s*:\s*)(.*?)(\r?\n)?$", line
            )
            if not match:
                continue
            value_and_comment = match.group(2)
            newline = match.group(3) or ""
            comment_start = value_and_comment.find("#")
            value_field = (
                value_and_comment
                if comment_start == -1
                else value_and_comment[:comment_start]
            )
            trailing_space = value_field[len(value_field.rstrip()):]
            comment = "" if comment_start == -1 else value_and_comment[comment_start:]
            rendered_line = (
                f"{match.group(1)}{value:.10f}{trailing_space}{comment}{newline}"
            )
            seen_keys.add(joint_key)
            break
        rendered_lines.append(rendered_line)

    if seen_keys != set(updated_pose):
        raise ValueError("Could not locate every zero-position key in the source YAML")
    return "".join(rendered_lines)


def save_zero_poses(
    path: Path, arm_samples: dict[str, list[tuple[float, ...]]]
) -> dict[str, dict[str, float]]:
    """Atomically replace one or both arms after every requested side validates."""
    if not arm_samples or any(side not in ARM_JOINT_KEYS for side in arm_samples):
        raise ValueError("Unknown or missing arm side")

    updated_poses = {}
    for side, samples in arm_samples.items():
        pose = validate_pose_samples(samples)
        updated_poses[side] = dict(zip(ARM_JOINT_KEYS[side], pose, strict=True))

    original = path.read_text()
    config = yaml.safe_load(original)
    updated_text = original
    for side, updated_pose in updated_poses.items():
        arm_key = f"{side}_arm"
        joint_keys = ARM_JOINT_KEYS[side]
        if not isinstance(config, dict) or not isinstance(config.get(arm_key), dict):
            raise ValueError(f"Missing {arm_key} zero-position mapping")
        if set(config[arm_key]) != set(joint_keys):
            raise ValueError("Zero-position configuration keys do not match the arm schema")
        updated_text = replace_arm_positions_in_yaml(
            updated_text, arm_key, updated_pose
        )

    backup = path.with_name(path.name + f".{time.time_ns()}.backup")
    backup.write_text(original)
    file_descriptor, temporary = tempfile.mkstemp(
        prefix=path.name + ".", dir=path.parent, text=True
    )
    try:
        with os.fdopen(file_descriptor, "w") as file:
            file.write(updated_text)
            file.flush()
            os.fsync(file.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    return updated_poses


def save_zero_pose(
    path: Path, side: str, samples: list[tuple[float, ...]]
) -> dict[str, float]:
    """Compatibility wrapper for a single-arm zero-pose capture."""
    return save_zero_poses(path, {side: samples})[side]


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", required=True, choices=("left", "right", "both"))
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()
    if (
        not math.isfinite(args.duration)
        or not math.isfinite(args.timeout)
        or args.duration < 1.0
        or args.timeout <= args.duration
    ):
        parser.error("Use finite duration >= 1.0 and timeout > duration")
    return args


def main() -> None:
    args = parse_arguments()

    # Keep pure YAML validation importable by pytest without a ROS installation.
    import rclpy
    from ctl_msgs_hw.msg import RetInformation
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = rclpy.create_node("towel_arm_zero_calibration")
    selected_sides = ("left", "right") if args.side == "both" else (args.side,)
    windows = {side: FeedbackWindow() for side in selected_sides}

    def feedback(message: RetInformation) -> None:
        now = time.monotonic()
        for side, window in windows.items():
            raw_positions = message.l_position if side == "left" else message.r_position
            window.add(now, raw_positions)

    node.create_subscription(
        RetInformation,
        "/dexbotCtl/dualArmCtl_fdback",
        feedback,
        qos_profile_sensor_data,
    )
    print(
        f"Capturing {args.side} arm zero pose. Keep the arm stationary; "
        "this tool will not send motor commands."
    )
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if all(window.is_ready(args.duration) for window in windows.values()):
                before_backups = set(args.config.parent.glob(args.config.name + ".*.backup"))
                poses = save_zero_poses(
                    args.config,
                    {side: window.samples for side, window in windows.items()},
                )
                created_backups = set(args.config.parent.glob(args.config.name + ".*.backup")) - before_backups
                backup = next(iter(created_backups), None)
                print("Saved source zero pose:")
                for side, pose in poses.items():
                    print(f"{side} arm:")
                    for key, value in pose.items():
                        print(f"  {key}: {value:+.6f} rad")
                print(f"Backup: {backup}" if backup else "Backup: created")
                return
        raise RuntimeError("Timed out waiting for a complete, contiguous feedback window; config unchanged")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
