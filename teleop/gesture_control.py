"""Frame-synchronous hand gestures and incremental poses, independent of ROS."""

import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


ROOT = Path(__file__).resolve().parents[1]
SIDES = ("left", "right")
FIST, OK, PINCH, OPEN_PALM = 1, 2, 3, 4
GESTURE_NAMES = {-1: "UNKNOWN", 0: "OTHER", FIST: "FIST", OK: "OK", PINCH: "PINCH", OPEN_PALM: "OPEN_PALM"}


def load_config(path=ROOT / "config/gesture_rviz.yaml"):
    config = yaml.safe_load(Path(path).read_text())
    config.setdefault("resume_grace_s", 1.0)
    config.setdefault("recovery_hold_s", 0.15)
    rotation = np.asarray(config["camera_to_robot_rotation"], dtype=float)
    if (rotation.shape != (3, 3) or not np.isfinite(rotation).all()
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-6)):
        raise ValueError("camera_to_robot_rotation must be a proper rotation (det = +1)")
    for name in ("position_scale", "orientation_scale", "position_speed_mps",
                 "orientation_speed_radps", "joint_speed_radps", "max_position_jump_m",
                 "max_orientation_jump_rad", "max_frame_gap_s", "timeout_s",
                 "release_hold_s", "event_hold_s", "resume_grace_s", "recovery_hold_s"):
        value = config[name]
        if value is None and name in ("position_speed_mps", "orientation_speed_radps"):
            continue
        if isinstance(value, bool) or not np.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    if not 0 <= config["min_quality"] <= 1 or type(config["swap_hands"]) is not bool:
        raise ValueError("Invalid min_quality or swap_hands")
    if config["max_frame_gap_s"] > config["timeout_s"]:
        raise ValueError("max_frame_gap_s must not exceed timeout_s")
    if not config["recovery_hold_s"] < config["resume_grace_s"] or config["timeout_s"] > config["resume_grace_s"]:
        raise ValueError("resume_grace_s must cover timeout_s and exceed recovery_hold_s")
    return config


def decode_packet(raw, now, timeout):
    """Consume the existing perception UDP schema, rejecting stale/invalid data."""
    try:
        packet = json.loads(raw)
        if (not isinstance(packet, dict) or packet.get("version") != 1
                or packet.get("frame_id") != "camera_color_optical_frame"):
            return None
        for key in ("sequence", "capture_monotonic_ns", "capture_time_ns"):
            if type(packet.get(key)) is not int or packet[key] < 0:
                return None
        stamp = packet["capture_monotonic_ns"] / 1e9
        if not -0.02 <= now - stamp <= timeout:
            return None
        gestures = packet["gestures"]
        if (not isinstance(gestures, dict) or set(gestures) != {"Left", "Right"}
                or any(type(g) is not int or g not in GESTURE_NAMES for g in gestures.values())):
            return None
        hands = packet["hands"]
        if not isinstance(hands, list) or len(hands) > 2:
            return None
        seen = set()
        for hand in hands:
            label = hand["label"]
            if label not in gestures or label in seen:
                return None
            seen.add(label)
            position = np.asarray(hand["position"], dtype=float)
            quaternion = np.asarray(hand["orientation"], dtype=float)
            quality = float(hand["quality"])
            if (position.shape != (3,) or quaternion.shape != (4,)
                    or not np.isfinite(position).all() or np.max(np.abs(position)) > 10
                    or not np.isfinite(quaternion).all()
                    or abs(np.linalg.norm(quaternion) - 1) > 0.01
                    or not np.isfinite(quality) or not 0 <= quality <= 1):
                return None
        return packet
    except (ValueError, TypeError, KeyError, OverflowError):
        return None


class GestureEdge:
    """Require a stable known release; UNKNOWN cannot re-arm a held gesture."""

    def __init__(self, gesture, release_s, hold_s):
        self.gesture, self.release_s, self.hold_s = gesture, release_s, hold_s
        self.reset()

    def reset(self):
        self.armed = False
        self.release_since = self.held_since = None

    def update(self, gesture, stamp):
        if gesture < 0:
            self.release_since = self.held_since = None
        elif gesture != self.gesture:
            self.held_since = None
            if self.release_since is None:
                self.release_since = stamp
            if stamp - self.release_since >= self.release_s:
                self.armed = True
        else:
            self.release_since = None
            if self.held_since is None:
                self.held_since = stamp
            if self.armed and stamp - self.held_since >= self.hold_s:
                self.armed = False
                return True
        return False


def limit_norm(vector, maximum):
    length = np.linalg.norm(vector)
    return vector * min(1.0, maximum / max(length, 1e-12))


class GestureMapper:
    def __init__(self, config):
        self.config = config
        self.rotation = np.asarray(config["camera_to_robot_rotation"], dtype=float)
        self.edges = {
            (side, gesture): GestureEdge(gesture, config["release_hold_s"], config["event_hold_s"])
            for side in SIDES for gesture in (OK, FIST)
        }
        self.last_stamp = None
        self.reset()

    def reset(self, reason="startup"):
        self.previous = {side: None for side in SIDES}
        self.armed = dict.fromkeys(SIDES, False)
        self.release_since = dict.fromkeys(SIDES, None)
        self.unknown_since = dict.fromkeys(SIDES, None)
        self.recovery = dict.fromkeys(SIDES, None)
        self.release_reason = dict.fromkeys(SIDES, reason)
        for edge in self.edges.values():
            edge.reset()

    def suspend_motion(self, reason="home"):
        self.previous = dict.fromkeys(SIDES, None)
        self.armed = dict.fromkeys(SIDES, False)
        self.release_since = dict.fromkeys(SIDES, None)
        self.unknown_since = dict.fromkeys(SIDES, None)
        self.recovery = dict.fromkeys(SIDES, None)
        self.release_reason = dict.fromkeys(SIDES, reason)

    def pause(self):
        """Discard pending deltas and button edges without forgetting clutch intent."""
        self.previous = dict.fromkeys(SIDES, None)
        self.release_since = dict.fromkeys(SIDES, None)
        self.recovery = dict.fromkeys(SIDES, None)
        for edge in self.edges.values():
            edge.reset()

    def expired(self, now):
        if self.last_stamp is None:
            return True
        age = now - self.last_stamp
        if age > self.config["timeout_s"]:
            if age > self.config["resume_grace_s"]:
                self.reset("stream_lost")
            else:
                self.pause()
            return True
        return False

    def consume(self, packet):
        stamp = packet["capture_monotonic_ns"] / 1e9
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return None
        dt = 0 if self.last_stamp is None else stamp - self.last_stamp
        if self.last_stamp is None:
            self.reset()
        elif dt > self.config["resume_grace_s"]:
            self.reset("stream_lost")
        elif dt > self.config["max_frame_gap_s"]:
            self.pause()
        self.last_stamp = stamp
        labels = ("Right", "Left") if self.config["swap_hands"] else ("Left", "Right")
        hands = {hand["label"]: hand for hand in packet["hands"]}
        result = {"events": [], "hands": {}}
        for side, label in zip(SIDES, labels):
            gesture = packet["gestures"][label]
            hand = hands.get(label)
            valid = hand is not None and hand["quality"] >= self.config["min_quality"]
            source = hand.get("pose_source", "rgbd") if valid else None
            dp, dr = np.zeros(3), np.zeros(3)
            active = False
            motion_state = "HOLD"
            for event_gesture in (OK, FIST):
                if self.edges[side, event_gesture].update(gesture, stamp):
                    event = "gripper_" + side if event_gesture == FIST else (
                        "record" if side == "left" else "home")
                    result["events"].append(event)
            if gesture >= 0:
                self.unknown_since[side] = None
            if gesture != OPEN_PALM or not valid:
                self.recovery[side] = None
            if gesture < 0:
                # The upstream recognizer emits UNKNOWN during every transition.
                # Hold immediately, preserving the last confirmed clutch release.
                self.release_since[side] = None
                self.previous[side] = None
                if self.unknown_since[side] is None:
                    self.unknown_since[side] = stamp
                if stamp - self.unknown_since[side] >= self.config["resume_grace_s"]:
                    self.armed[side] = False
                    self.release_reason[side] = "gesture_lost"
                motion_state = "UNKNOWN"
            elif gesture != OPEN_PALM:
                self.previous[side] = None
                if self.release_since[side] is None:
                    self.release_since[side] = stamp
                if stamp - self.release_since[side] >= self.config["release_hold_s"]:
                    self.armed[side] = True
            elif not valid:
                # A recognized open palm retains intent through depth dropouts. Never
                # integrate across a missing pose; recovery starts a new anchor.
                self.release_since[side] = None
                self.previous[side] = None
                motion_state = "NO_POSE"
            elif self.armed[side]:
                self.release_since[side] = None
                current_pose = (np.asarray(hand["position"]), Rotation.from_quat(hand["orientation"]).as_matrix(), source)
                old = self.previous[side]
                active = True
                motion_state = "ANCHOR"
                recovery = self.recovery[side]
                if recovery is not None:
                    active = False
                    motion_state = "RECOVER"
                    candidate = recovery["pose"]
                    if stamp - recovery["started"] >= self.config["resume_grace_s"]:
                        self.armed[side] = False
                        self.release_reason[side] = "pose_unstable"
                        self.recovery[side] = None
                        motion_state = "RELEASE"
                    else:
                        if (source != candidate[2]
                                or np.linalg.norm(current_pose[0] - candidate[0]) > self.config["max_position_jump_m"]
                                or Rotation.from_matrix(current_pose[1] @ candidate[1].T).magnitude() > self.config["max_orientation_jump_rad"]):
                            recovery["stable_since"] = stamp
                        if stamp - recovery["stable_since"] >= self.config["recovery_hold_s"]:
                            self.recovery[side] = None
                            active = True
                            motion_state = "ANCHOR"
                        recovery["pose"] = current_pose
                elif old is not None and old[2] == source:
                    motion_state = "MOVE"
                    dp = self.rotation @ (np.asarray(hand["position"]) - old[0])
                    current = Rotation.from_quat(hand["orientation"]).as_matrix()
                    delta = self.rotation @ current @ old[1].T @ self.rotation.T
                    dr = Rotation.from_matrix(delta).as_rotvec()
                    if (np.linalg.norm(dp) > self.config["max_position_jump_m"]
                            or np.linalg.norm(dr) > self.config["max_orientation_jump_rad"]):
                        active = False
                        self.recovery[side] = {"started": stamp, "stable_since": stamp, "pose": current_pose}
                        motion_state = "RECOVER"
                        dp, dr = np.zeros(3), np.zeros(3)
                    else:
                        dp *= self.config["position_scale"]
                        dr *= self.config["orientation_scale"]
                        if self.config["position_speed_mps"] is not None:
                            dp = limit_norm(dp, self.config["position_speed_mps"] * dt)
                        if self.config["orientation_speed_radps"] is not None:
                            dr = limit_norm(dr, self.config["orientation_speed_radps"] * dt)
                self.previous[side] = current_pose if active else None
            else:
                self.release_since[side] = None
                motion_state = "RELEASE"
            result["hands"][side] = {"active": active, "pose_valid": valid, "gesture": gesture,
                                     "pose_source": source, "armed": self.armed[side],
                                     "release_reason": None if self.armed[side] else self.release_reason[side],
                                     "motion_state": motion_state,
                                     "position_delta": dp, "rotation_delta": dr}
        if "home" in result["events"]:
            self.suspend_motion()
            for hand in result["hands"].values():
                hand["active"] = False
                hand["armed"] = False
                hand["release_reason"] = "home"
                hand["motion_state"] = "HOME"
                hand["position_delta"].fill(0)
                hand["rotation_delta"].fill(0)
        return result
