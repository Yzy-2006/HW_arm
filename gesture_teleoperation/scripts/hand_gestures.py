"""Scale- and rotation-invariant gestures from MediaPipe's relative 3D skeleton."""

import numpy as np


GESTURE_UNKNOWN, OTHER, FIST, OK, FIVE_TIP_PINCH, OPEN_PALM = -1, 0, 1, 2, 3, 4
GESTURE_NAMES = {
    GESTURE_UNKNOWN: "UNKNOWN", OTHER: "OTHER", FIST: "FIST",
    OK: "OK", FIVE_TIP_PINCH: "FIVE_TIP_PINCH", OPEN_PALM: "OPEN_PALM",
}
FINGER_BASES = (5, 9, 13, 17)
TIPS = (4, 8, 12, 16, 20)
DEFAULT_OK_DISTANCE = 0.42
DEFAULT_PINCH_DISTANCE = 1.0


def fingertip_distances(points):
    """Distances in palm-size units, for 3D landmarks or unmirrored 2D pixels."""
    points = np.asarray(points, dtype=float)
    if points.shape not in ((21, 2), (21, 3)) or not np.isfinite(points).all():
        return None
    lateral = points[5] - points[17]
    longitudinal = points[9] - points[0]
    scale = 0.5 * (np.linalg.norm(lateral) + np.linalg.norm(longitudinal))
    if scale < 1e-6:
        return None
    distances = np.linalg.norm(points[[8, 12, 16, 20]] - points[4], axis=1) / scale
    result = {
        "thumb_index": float(distances[0]),
        "thumb_distances": distances,
        "thumb_max": float(np.max(distances)),
        "thumb_mean": float(np.mean(distances)),
        "other_tip_reach": np.linalg.norm(points[[12, 16, 20]] - points[[9, 13, 17]], axis=1) / scale,
    }
    if points.shape[1] == 2:
        area = abs(lateral[0] * longitudinal[1] - lateral[1] * longitudinal[0])
        result["palm_area"] = float(area / scale ** 2)
    return result


def gesture_features(points):
    points = np.asarray(points, dtype=float)
    if points.shape != (21, 3) or not np.isfinite(points).all():
        return None
    scale = 0.5 * (np.linalg.norm(points[5] - points[17]) + np.linalg.norm(points[9] - points[0]))
    if scale < 1e-6:
        return None
    pip_angles, dip_angles, extension = [], [], []
    for base in FINGER_BASES:
        chain = points[base:base + 4]
        lengths = np.linalg.norm(np.diff(chain, axis=0), axis=1)
        if np.any(lengths < 0.025 * scale) or np.any(lengths > 1.3 * scale):
            return None
        before, after = chain[0] - chain[1], chain[2] - chain[1]
        cosine = np.dot(before, after) / (np.linalg.norm(before) * np.linalg.norm(after))
        pip_angles.append(float(np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0)))))
        before, after = chain[1] - chain[2], chain[3] - chain[2]
        cosine = np.dot(before, after) / (lengths[1] * lengths[2])
        dip_angles.append(float(np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0)))))
        extension.append(float(np.linalg.norm(chain[-1] - chain[0]) / np.sum(lengths)))
    pip_angles, extension = np.array(pip_angles), np.array(extension)
    tips = points[list(TIPS)]
    palm_center = points[[0, 5, 9, 13, 17]].mean(axis=0)
    contact_center = 0.5 * (points[4] + points[8])
    return {
        "palm_scale": float(scale),
        "pip_angles": pip_angles,
        "dip_angles": np.asarray(dip_angles),
        "extension": extension,
        "thumb_index": float(np.linalg.norm(points[4] - points[8]) / scale),
        "tip_spread": float(np.max(np.linalg.norm(tips[:, None] - tips[None, :], axis=-1)) / scale),
        "thumb_reach": float(np.linalg.norm(points[4] - palm_center) / scale),
        "other_tip_separation": float(np.mean(np.linalg.norm(points[[12, 16, 20]] - contact_center, axis=1)) / scale),
    }


def _thumb_extended(points, scale):
    points = np.asarray(points, dtype=float)
    thumb = points[1:5]
    lengths = np.linalg.norm(np.diff(thumb, axis=0), axis=1)
    if np.any(lengths < 0.025 * scale) or np.any(lengths > 1.3 * scale):
        return False
    before, after = thumb[1] - thumb[2], thumb[3] - thumb[2]
    thumb_ip_cosine = np.dot(before, after) / (lengths[1] * lengths[2])
    thumb_extension = np.linalg.norm(thumb[3] - thumb[0]) / np.sum(lengths)
    return bool(thumb_ip_cosine < np.cos(np.deg2rad(145)) and thumb_extension > 0.8)


def _is_open_palm(points, features):
    if not (
        np.all(features["pip_angles"] > 145) and np.all(features["extension"] > 0.85)
        and np.all(features["dip_angles"] > 145)
    ):
        return False
    points = np.asarray(points, dtype=float)
    scale = features["palm_scale"]
    if not _thumb_extended(points, scale):
        return False
    # Require thumb abduction even when a tucked thumb happens to remain straight.
    lateral = points[5] - points[17]
    thumb_outward = np.dot(points[4] - points[5], lateral) / np.linalg.norm(lateral) / scale
    return bool(thumb_outward > 0.25)


def classify_gesture(points, image_points=None, ok_distance=DEFAULT_OK_DISTANCE,
                     pinch_distance=DEFAULT_PINCH_DISTANCE):
    features = gesture_features(points)
    if features is None:
        return GESTURE_UNKNOWN
    angles, extension = features["pip_angles"], features["extension"]
    # Preserve the working fist rule before relaxing contact-based gestures.
    curled = (angles < 120) & (extension < 0.78)
    if (
        np.count_nonzero(curled) >= 3
        and np.all(extension < 0.85)
        and np.max(angles) < 155
        and features["thumb_reach"] < 0.9
    ):
        return FIST
    world = fingertip_distances(points)
    image = fingertip_distances(image_points) if image_points is not None else None
    if image is not None and ("palm_area" not in image or image["palm_area"] < 0.15):
        image = None

    # The fingertips can be bent in a pinch: their contact matters more than PIP angles.
    pinch_3d = (
        world["thumb_max"] < pinch_distance
        and world["thumb_mean"] < 0.75 * pinch_distance
    )
    pinch_2d = (
        image is not None
        and image["thumb_max"] < 0.35
        and image["thumb_mean"] < 0.25
        and world["thumb_max"] < 1.6 * pinch_distance
        and world["thumb_mean"] < pinch_distance
    )
    if pinch_3d or pinch_2d:
        return FIVE_TIP_PINCH

    # Other fingertips must remain away from the contact and their own MCPs.
    other_free_3d = (world["thumb_distances"][1:] > 0.55) & (world["other_tip_reach"] > 0.48)
    ok_3d = world["thumb_index"] < ok_distance and np.count_nonzero(other_free_3d) >= 2
    ok_2d = (
        image is not None
        and image["thumb_index"] < 0.25
        and world["thumb_index"] < 1.8 * ok_distance
        and np.count_nonzero(
            (image["thumb_distances"][1:] > 0.55) & (image["other_tip_reach"] > 0.45)
        ) >= 2
    )
    if ok_3d or ok_2d:
        return OK
    if _is_open_palm(points, features):
        return OPEN_PALM
    if (
        image is not None and np.all(extension > 0.85)
        and np.all(features["dip_angles"] > 130)
        and _thumb_extended(points, features["palm_scale"])
    ):
        # A broad palm projection can corroborate extension when world PIP angles drift.
        projected = np.column_stack((np.asarray(image_points, dtype=float), np.zeros(21)))
        projected_features = gesture_features(projected)
        if projected_features is not None and _is_open_palm(projected, projected_features):
            return OPEN_PALM
    return OTHER


class GestureTracker:
    """Confirm independent hand gestures for 150 ms; clear missing data at once."""

    def __init__(self, settle_time=0.15):
        self.settle_time = settle_time
        self.pending = {}

    def update(self, detections, now):
        output = {"Left": GESTURE_UNKNOWN, "Right": GESTURE_UNKNOWN}
        for label in output:
            matches = [item for item in detections if item["label"] == label]
            candidate = GESTURE_UNKNOWN
            if len(matches) == 1 and matches[0]["score"] >= 0.6:
                candidate = matches[0]["gesture"]
            if candidate == GESTURE_UNKNOWN:
                self.pending.pop(label, None)
                continue
            previous = self.pending.get(label)
            if previous is None or previous[0] != candidate or now < previous[1]:
                self.pending[label] = candidate, now
            if now - self.pending[label][1] >= self.settle_time:
                output[label] = candidate
        return output
