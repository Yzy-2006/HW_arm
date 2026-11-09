import copy
import time
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from scripts.pose_comparison import compare_hands
from scripts.pose_protocol import FRAME_ID, validate_packet


def hand(label="Left", detection_index=0):
    return {
        "label": label, "detection_index": detection_index,
        "position": [0, 0, 0.6], "orientation": [0, 0, 0, 1],
        "joints": [[0, 0, 0.6] for _ in range(21)], "quality": 0.9,
    }


class ComparisonTests(unittest.TestCase):
    def test_metric_deltas_and_quaternion_sign(self):
        reference = hand()
        rgb = hand()
        rgb["position"] = [0.03, -0.04, 0.6]
        rgb["orientation"] = (-Rotation.from_euler("z", 20, degrees=True).as_quat()).tolist()
        difference = compare_hands([reference], [rgb])["Left"]
        self.assertAlmostEqual(difference["position_difference_m"], 0.05)
        self.assertAlmostEqual(difference["orientation_difference_deg"], 20)
        np.testing.assert_allclose(difference["delta_m"], [0.03, -0.04, 0])
        self.assertEqual(difference["rgb_position"], rgb["position"])

    def test_missing_identity_mismatch_and_inconsistent_origin_are_not_compared(self):
        reference = hand()
        self.assertEqual(compare_hands([reference], []), {"Left": None, "Right": None})
        self.assertIsNone(compare_hands([reference], [hand(detection_index=1)])["Left"])
        self.assertIsNone(compare_hands([reference], [hand("Right")])["Left"])
        reference["joints"][13] = None
        self.assertIsNone(compare_hands([reference], [hand()])["Left"])

    def test_protocol_checks_comparison_metadata(self):
        reference = hand()
        packet = {
            "version": 1, "frame_id": FRAME_ID, "sequence": 1,
            "capture_monotonic_ns": time.monotonic_ns(), "capture_time_ns": time.time_ns(),
            "hands": [reference], "comparison": compare_hands([reference], [hand()]),
        }
        validate_packet(packet, time.monotonic_ns())
        for field, value in (("orientation_difference_deg", 181), ("position_difference_m", float("nan")),
                             ("delta_m", [0, 0]), ("rgb_position", [0, 0, float("inf")])):
            invalid = copy.deepcopy(packet)
            invalid["comparison"]["Left"][field] = value
            with self.assertRaises(ValueError):
                validate_packet(invalid, time.monotonic_ns())


if __name__ == "__main__":
    unittest.main()
