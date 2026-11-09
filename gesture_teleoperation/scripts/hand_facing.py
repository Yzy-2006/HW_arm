"""Classify and display which surface of each measured palm faces the camera."""

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


FACING_WINDOW_NAME = "D435i Palm-Back Flags"
UNKNOWN, BACK, PALM = -1, 0, 1


def classify_palm_facing(hand, angle_margin=0.3):
    """Compare the anatomical palmar normal with the hand-to-camera direction.

    For the existing little-to-index X / wrist-to-middle Y convention, the
    palmar normal is +Z on Right hands and -Z on Left hands. Display mirroring
    must not be applied to these optical-frame coordinates.
    """
    label = hand.get("label")
    if label not in ("Left", "Right") or hand.get("handedness_score", 0.0) < 0.6:
        return UNKNOWN
    position = np.asarray(hand["position"], dtype=float)
    quaternion = np.asarray(hand["orientation"], dtype=float)
    if (
        position.shape != (3,) or quaternion.shape != (4,)
        or not np.isfinite(position).all() or not np.isfinite(quaternion).all()
        or position[2] <= 0 or np.linalg.norm(quaternion) < 1e-6
    ):
        return UNKNOWN
    normal = Rotation.from_quat(quaternion).as_matrix()[:, 2]
    if label == "Left":
        normal = -normal
    toward_camera = -position / np.linalg.norm(position)
    cosine = float(np.dot(normal, toward_camera))
    if abs(cosine) <= angle_margin:
        return UNKNOWN
    return PALM if cosine > 0 else BACK


class FacingTracker:
    """Confirm a new surface for 120 ms; never hold a flag across invalid data."""

    def __init__(self, settle_time=0.12):
        self.settle_time = settle_time
        self.pending = {}

    def update(self, hands, now):
        observed = {hand["label"]: classify_palm_facing(hand) for hand in hands}
        flags = {"Left": UNKNOWN, "Right": UNKNOWN}
        for label in flags:
            candidate = observed.get(label, UNKNOWN)
            if candidate == UNKNOWN:
                self.pending.pop(label, None)
                continue
            previous = self.pending.get(label)
            if previous is None or previous[0] != candidate or now < previous[1]:
                self.pending[label] = candidate, now
            if now - self.pending[label][1] >= self.settle_time:
                flags[label] = candidate
        return flags


def draw_facing_flags(flags, gesture_labels=None):
    gesture_labels = {} if gesture_labels is None else gesture_labels
    output = np.full((220, 540, 3), (30, 32, 35), dtype=np.uint8)
    cv2.putText(output, "HAND STATUS", (20, 29), cv2.FONT_HERSHEY_SIMPLEX,
                0.62, (235, 235, 235), 1, cv2.LINE_AA)
    for text, x in (("HAND", 20), ("FLAG", 180), ("SURFACE", 280)):
        cv2.putText(output, text, (x, 57), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, (150, 155, 165), 1, cv2.LINE_AA)
    cv2.line(output, (20, 66), (520, 66), (65, 68, 73), 1)
    styles = {
        PALM: ("PALM", (105, 225, 100)),
        BACK: ("BACK", (85, 175, 255)),
        UNKNOWN: ("UNKNOWN", (170, 175, 185)),
    }
    for label, baseline in (("Left", 100), ("Right", 173)):
        flag = flags.get(label, UNKNOWN)
        text, color = styles[flag]
        cv2.putText(output, label, (20, baseline), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (235, 235, 235), 1, cv2.LINE_AA)
        cv2.putText(output, str(flag), (180, baseline), cv2.FONT_HERSHEY_SIMPLEX,
                    0.9, color, 2, cv2.LINE_AA)
        cv2.putText(output, text, (280, baseline), cv2.FONT_HERSHEY_SIMPLEX,
                    0.75, color, 2, cv2.LINE_AA)
        cv2.circle(output, (507, baseline - 9), 7, color, -1, cv2.LINE_AA)
        cv2.putText(output, "Gesture", (20, baseline + 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, (150, 155, 165), 1, cv2.LINE_AA)
        cv2.putText(output, gesture_labels.get(label, "UNKNOWN [-1]"),
                    (180, baseline + 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (225, 225, 225), 1, cv2.LINE_AA)
    return output
