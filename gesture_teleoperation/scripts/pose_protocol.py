"""Small local JSON protocol shared by Python 3.11 perception and ROS Python."""

import math


FRAME_ID = "camera_color_optical_frame"
DEFAULT_PORT = 9870
STALE_SECONDS = 0.35


def validate_packet(packet, now_ns, timeout=STALE_SECONDS):
    if not isinstance(packet, dict) or packet.get("version") != 1 or packet.get("frame_id") != FRAME_ID:
        raise ValueError("Unsupported hand pose packet")
    capture_ns = packet.get("capture_monotonic_ns")
    stamp_ns = packet.get("capture_time_ns")
    if not isinstance(capture_ns, int) or not isinstance(stamp_ns, int) or stamp_ns <= 0:
        raise ValueError("Invalid capture timestamps")
    age = (now_ns - capture_ns) / 1e9
    if age < -0.05 or age > timeout:
        raise ValueError("Stale hand pose packet")
    hands = packet.get("hands")
    if not isinstance(hands, list) or len(hands) > 2:
        raise ValueError("Expected at most two hands")
    labels = set()

    def vector(value, length, bound):
        return (
            isinstance(value, list) and len(value) == length
            and all(isinstance(x, (int, float)) and math.isfinite(x) and abs(x) <= bound for x in value)
        )

    for hand in hands:
        if not isinstance(hand, dict):
            raise ValueError("Invalid hand record")
        label = hand.get("label")
        if label not in ("Left", "Right") or label in labels:
            raise ValueError("Invalid or duplicate hand identity")
        labels.add(label)
        if not vector(hand.get("position"), 3, 10.0):
            raise ValueError("Invalid position")
        quaternion = hand.get("orientation")
        if not vector(quaternion, 4, 1.01):
            raise ValueError("Invalid quaternion")
        norm = math.sqrt(sum(x * x for x in quaternion))
        if abs(norm - 1.0) > 0.01:
            raise ValueError("Quaternion is not normalized")
        joints = hand.get("joints")
        if not isinstance(joints, list) or len(joints) != 21:
            raise ValueError("Expected 21 joint entries")
        if any(point is not None and not vector(point, 3, 10.0) for point in joints):
            raise ValueError("Invalid joint position")
        quality = hand.get("quality")
        if not isinstance(quality, (float, int)) or not math.isfinite(quality) or not 0 <= quality <= 1:
            raise ValueError("Invalid quality")
    if "comparison" in packet:
        comparison = packet["comparison"]
        if not isinstance(comparison, dict) or set(comparison) != {"Left", "Right"}:
            raise ValueError("Invalid pose comparison")
        for difference in comparison.values():
            if difference is None:
                continue
            if not isinstance(difference, dict):
                raise ValueError("Invalid pose difference")
            for name, bound in (("position_difference_m", 20), ("orientation_difference_deg", 180)):
                value = difference.get(name)
                if not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= bound:
                    raise ValueError("Invalid pose difference magnitude")
            if not vector(difference.get("rgb_position"), 3, 10) or not vector(difference.get("delta_m"), 3, 20):
                raise ValueError("Invalid pose difference vector")
    return packet
