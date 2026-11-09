"""Depth-based palm frames and temporal tracking, independent of ROS."""

from itertools import permutations
import math

import cv2
import numpy as np
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation, Slerp


PALM_IDS = (0, 5, 9, 13, 17)
REQUIRED_IDS = (0, 5, 9, 17)


def estimate_palm(landmarks, depth_m, intrinsics, min_depth=0.2, max_depth=1.5):
    """Return a measured palm pose, or a reason why it cannot be measured.

    Axes: +X little-finger MCP toward index MCP; +Y wrist toward middle MCP;
    +Z = X cross Y. Coordinates remain in the unmirrored color optical frame.
    """
    height, width = depth_m.shape
    pixels = np.asarray([(p.x * width, p.y * height) for p in landmarks])
    if len(pixels) != 21 or not np.isfinite(pixels).all():
        return None, "invalid landmarks"
    palm_pixels = pixels[list(PALM_IDS)]
    if np.any(palm_pixels < 0) or np.any(palm_pixels >= [width, height]):
        return None, "palm outside image"
    hull = cv2.convexHull(np.rint(palm_pixels).astype(np.int32))
    if cv2.contourArea(hull) < 80:
        return None, "palm too small or edge-on"
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillConvexPoly(mask, hull, 1)
    mask = cv2.erode(mask, np.ones((3, 3), dtype=np.uint8))
    valid = np.isfinite(depth_m) & (depth_m >= min_depth) & (depth_m <= max_depth)
    interior = depth_m[(mask != 0) & valid]
    if interior.size < 30 or interior.size < 0.35 * np.count_nonzero(mask):
        return None, "insufficient palm depth"
    reference = float(np.median(interior))
    # Restrict every sample to the palm's depth neighborhood to exclude background.
    depth_band = 0.12
    joints = np.full((21, 3), np.nan)
    supports = np.zeros(21)
    for index, (u, v) in enumerate(pixels):
        if not (0 <= u < width and 0 <= v < height):
            continue
        x, y = int(round(u)), int(round(v))
        x0, x1 = max(0, x - 3), min(width, x + 4)
        y0, y1 = max(0, y - 3), min(height, y + 4)
        patch = depth_m[y0:y1, x0:x1]
        accepted = valid[y0:y1, x0:x1] & (np.abs(patch - reference) < depth_band)
        values = patch[accepted]
        if values.size < 8:
            continue
        median = float(np.median(values))
        mad = float(np.median(np.abs(values - median)))
        values = values[np.abs(values - median) <= max(0.004, 3.0 * 1.4826 * mad)]
        if values.size < 8 or mad > 0.015:
            continue
        supports[index] = values.size / patch.size
        joints[index] = rs.rs2_deproject_pixel_to_point(
            intrinsics, [float(u), float(v)], float(np.median(values)),
        )
    if not np.isfinite(joints[list(REQUIRED_IDS)]).all():
        return None, "missing wrist or palm anchor depth"
    palm_valid = [index for index in PALM_IDS if np.isfinite(joints[index]).all()]
    palm = joints[palm_valid]
    origin = np.mean(palm, axis=0)
    lateral = joints[5] - joints[17]
    longitudinal = joints[9] - joints[0]
    palm_width, palm_length = np.linalg.norm(lateral), np.linalg.norm(longitudinal)
    if not (0.025 < palm_width < 0.14 and 0.025 < palm_length < 0.18):
        return None, "implausible palm dimensions"
    normal_hint = np.cross(lateral, longitudinal)
    if np.linalg.norm(normal_hint) / (palm_width * palm_length) < 0.3:
        return None, "degenerate palm axes"
    _, singular, vectors = np.linalg.svd(palm - origin, full_matrices=False)
    if singular[1] < 0.008:
        return None, "degenerate palm plane"
    normal = vectors[-1]
    if np.dot(normal, normal_hint) < 0:
        normal = -normal
    residual = float(np.sqrt(np.mean(((palm - origin) @ normal) ** 2)))
    if residual > 0.012:
        return None, "inconsistent palm depth"
    y_axis = longitudinal - np.dot(longitudinal, normal) * normal
    y_axis /= np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, normal)
    x_axis /= np.linalg.norm(x_axis)
    matrix = np.column_stack((x_axis, y_axis, np.cross(x_axis, y_axis)))
    quality = float(
        len(palm_valid) / len(PALM_IDS)
        * np.mean(supports[palm_valid])
        * math.exp(-residual / 0.012)
    )
    if quality < 0.2:
        return None, "low depth support"
    return {
        "position": origin,
        "orientation": Rotation.from_matrix(matrix).as_quat(),
        "joints": joints,
        "quality": quality,
        "plane_rms_m": residual,
    }, None


class PalmTracker:
    """Associate up to two palms and smooth poses without holding invalid output."""

    def __init__(self, position_tau=0.06, rotation_tau=0.10, reset_after=0.4):
        self.position_tau = position_tau
        self.rotation_tau = rotation_tau
        self.reset_after = reset_after
        self.previous = {}

    def update(self, observations, now):
        self.previous = {
            label: state for label, state in self.previous.items()
            if 0 <= now - state["time"] < self.reset_after
        }
        if not observations:
            return []
        # An ambiguous duplicate label cannot initialize a second anatomical identity.
        if not self.previous:
            best = {}
            for observation in observations:
                label = observation["detected_label"]
                if label not in best or observation["score"] > best[label]["score"]:
                    best[label] = observation
            observations = list(best.values())

        def cost(assignment):
            total = 0.0
            for observation, label in zip(observations, assignment):
                same = label == observation["detected_label"]
                probability = observation["score"] if same else 1.0 - observation["score"]
                total -= math.log(max(probability, 1e-4))
                prior = self.previous.get(label)
                if prior:
                    distance = np.linalg.norm(observation["position"] - prior["position"])
                    total += 4.0 * min(distance / 0.15, 5.0)
                elif not same:
                    total += 20.0
            return total

        assignments = permutations(("Left", "Right"), len(observations))
        labels = min(assignments, key=cost)
        output = []
        for observation, label in zip(observations, labels):
            if label != observation["detected_label"] and label not in self.previous:
                continue
            raw_position = observation["position"]
            raw_rotation = Rotation.from_quat(observation["orientation"])
            position, rotation = raw_position.copy(), raw_rotation
            prior = self.previous.get(label)
            if prior:
                dt = now - prior["time"]
                if np.linalg.norm(position - prior["position"]) > 0.20:
                    continue
                alpha = 1.0 - math.exp(-dt / self.position_tau)
                position = prior["position"] + alpha * (position - prior["position"])
                alpha_rotation = 1.0 - math.exp(-dt / self.rotation_tau)
                rotation = Slerp(
                    [0.0, 1.0],
                    Rotation.from_quat([prior["orientation"], raw_rotation.as_quat()]),
                )([alpha_rotation])[0]
            quaternion = rotation.as_quat()
            if prior and np.dot(quaternion, prior["orientation"]) < 0:
                quaternion = -quaternion
            self.previous[label] = {
                "position": position, "orientation": quaternion, "time": now,
            }
            joints = rotation.apply(raw_rotation.inv().apply(observation["joints"] - raw_position))
            joints += position
            tracked = {
                "label": label,
                "position": position.tolist(),
                "orientation": quaternion.tolist(),
                "joints": [point.tolist() if np.isfinite(point).all() else None for point in joints],
                "quality": observation["quality"],
                "detected_label": observation["detected_label"],
                "handedness_score": (
                    observation["score"] if label == observation["detected_label"]
                    else 1.0 - observation["score"]
                ),
                "plane_rms_m": observation.get("plane_rms_m"),
            }
            for field in ("pose_source", "reprojection_rms_px", "pnp_orientation_hint_used",
                          "pnp_hint_angle_deg", "pnp_candidate_count"):
                if field in observation:
                    tracked[field] = observation[field]
            if "detection_index" in observation:
                tracked["detection_index"] = observation["detection_index"]
            output.append(tracked)
        return output
