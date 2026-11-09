"""Compare two independently estimated poses from the same hand detection."""

import numpy as np
from scipy.spatial.transform import Rotation


def compare_hands(rgbd_hands, rgb_hands):
    reference = {hand["label"]: hand for hand in rgbd_hands}
    estimated = {hand["label"]: hand for hand in rgb_hands}
    output = {"Left": None, "Right": None}
    for label in output:
        depth, rgb = reference.get(label), estimated.get(label)
        if depth is None or rgb is None:
            continue
        if depth.get("detection_index") is None or depth["detection_index"] != rgb.get("detection_index"):
            continue
        # RGB-D may use four anchors if ring MCP depth is absent; compare equal origins only.
        if any(depth["joints"][index] is None for index in (0, 5, 9, 13, 17)):
            continue
        delta = np.asarray(rgb["position"]) - depth["position"]
        angle = Rotation.from_quat(depth["orientation"]).inv() * Rotation.from_quat(rgb["orientation"])
        output[label] = {
            "position_difference_m": float(np.linalg.norm(delta)),
            "orientation_difference_deg": float(np.degrees(angle.magnitude())),
            "delta_m": delta.tolist(), "rgb_position": rgb["position"],
        }
    return output
