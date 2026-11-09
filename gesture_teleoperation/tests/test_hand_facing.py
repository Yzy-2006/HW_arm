import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from scripts.hand_facing import BACK, PALM, UNKNOWN, FacingTracker, classify_palm_facing, draw_facing_flags


def hand(label="Right", rotation=None, position=(0.0, 0.0, 0.6), score=0.95):
    rotation = Rotation.identity() if rotation is None else rotation
    return {
        "label": label, "position": list(position),
        "orientation": rotation.as_quat().tolist(), "handedness_score": score,
    }


class FacingTests(unittest.TestCase):
    def test_both_hand_conventions_front_and_back(self):
        turned = Rotation.from_euler("y", 180, degrees=True)
        self.assertEqual(classify_palm_facing(hand("Right")), BACK)
        self.assertEqual(classify_palm_facing(hand("Right", turned)), PALM)
        self.assertEqual(classify_palm_facing(hand("Left")), PALM)
        self.assertEqual(classify_palm_facing(hand("Left", turned)), BACK)

    def test_side_on_and_ambiguous_handedness_are_unknown(self):
        for angle in (75, 90, 105):
            self.assertEqual(
                classify_palm_facing(hand(rotation=Rotation.from_euler("y", angle, degrees=True))),
                UNKNOWN,
            )
        self.assertEqual(classify_palm_facing(hand(score=0.55)), UNKNOWN)
        invalid = hand()
        invalid["orientation"] = [0, 0, 0, 0]
        self.assertEqual(classify_palm_facing(invalid), UNKNOWN)
        invalid = hand(position=(0.0, 0.0, -0.6))
        self.assertEqual(classify_palm_facing(invalid), UNKNOWN)

    def test_actual_camera_ray_and_quaternion_sign(self):
        tilted = hand(rotation=Rotation.from_euler("y", 75, degrees=True), position=(0.6, 0, 0.3))
        self.assertEqual(classify_palm_facing(tilted), BACK)
        tilted["orientation"] = (-np.asarray(tilted["orientation"])).tolist()
        self.assertEqual(classify_palm_facing(tilted), BACK)

    def test_confirm_flip_and_clear_on_missing_or_invalid_depth_pose(self):
        tracker = FacingTracker()
        self.assertEqual(tracker.update([hand()], 1.0)["Right"], UNKNOWN)
        self.assertEqual(tracker.update([hand()], 1.15)["Right"], BACK)
        flipped = hand(rotation=Rotation.from_euler("y", 180, degrees=True))
        self.assertEqual(tracker.update([flipped], 1.16)["Right"], UNKNOWN)
        self.assertEqual(tracker.update([flipped], 1.30)["Right"], PALM)
        self.assertEqual(tracker.update([], 1.31), {"Left": UNKNOWN, "Right": UNKNOWN})
        self.assertEqual(tracker.update([flipped], 1.32)["Right"], UNKNOWN)
        self.assertEqual(tracker.update([flipped], 1.46)["Right"], PALM)
        edge = hand(rotation=Rotation.from_euler("y", 90, degrees=True))
        self.assertEqual(tracker.update([edge], 1.47)["Right"], UNKNOWN)

    def test_independent_hands_and_fixed_display_dimensions(self):
        tracker = FacingTracker()
        tracker.update([hand("Right"), hand("Left")], 1.0)
        flags = tracker.update([hand("Left"), hand("Right")], 1.15)
        self.assertEqual(flags, {"Left": PALM, "Right": BACK})
        display = draw_facing_flags(flags)
        self.assertEqual(display.shape, (220, 540, 3))
        self.assertGreater(np.count_nonzero(display != [30, 32, 35]), 1000)
        self.assertEqual(draw_facing_flags({}).shape, display.shape)


if __name__ == "__main__":
    unittest.main()
