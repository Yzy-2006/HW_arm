import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from scripts.hand_gestures import (
    FINGER_BASES, FIST, FIVE_TIP_PINCH, GESTURE_UNKNOWN, OK, OPEN_PALM, OTHER,
    GestureTracker, classify_gesture,
)
from scripts.hand_facing import draw_facing_flags


def skeleton(gesture=OTHER):
    points = np.zeros((21, 3), dtype=float)
    points[1:5] = [[0.025, 0.02, 0], [0.05, 0.035, 0], [0.065, 0.055, 0], [0.075, 0.075, 0]]
    for base, position in zip(FINGER_BASES, (
        (0.035, 0.075, 0), (0.008, 0.085, 0), (-0.018, 0.078, 0), (-0.04, 0.065, 0),
    )):
        points[base] = position
        for offset, reach in enumerate((0.035, 0.057, 0.074), start=1):
            points[base + offset] = points[base] + [0, reach, 0]
    if gesture == FIST:
        for base in FINGER_BASES:
            points[base + 1] = points[base] + [0, 0.03, -0.005]
            points[base + 2] = points[base] + [0, 0.027, -0.03]
            points[base + 3] = points[base] + [0, 0.003, -0.03]
        points[3:5] = [[0.04, 0.055, -0.025], [0.02, 0.06, -0.025]]
    elif gesture == OK:
        points[6:9] = [[0.035, 0.11, -0.005], [0.065, 0.115, -0.015], [0.06, 0.09, -0.02]]
        points[4] = points[8]
    elif gesture == FIVE_TIP_PINCH:
        target = np.array([0.0, 0.112, -0.065])
        points[4] = target
        points[3] = points[2] + 0.65 * (target - points[2])
        for base in FINGER_BASES:
            for offset, fraction in enumerate((0.45, 0.78, 1.0), start=1):
                points[base + offset] = points[base] + fraction * (target - points[base])
    elif gesture == OTHER:
        points[6] += [0.022, 0, 0]
    return points


class GestureGeometryTests(unittest.TestCase):
    def test_four_gestures_and_partial_open_hand(self):
        for gesture in (FIST, OK, FIVE_TIP_PINCH, OPEN_PALM, OTHER):
            with self.subTest(gesture=gesture):
                self.assertEqual(classify_gesture(skeleton(gesture)), gesture)

    def test_rotation_translation_scale_and_left_right_reflection(self):
        rotation = Rotation.from_euler("xyz", [55, -35, 120], degrees=True)
        for gesture in (FIST, OK, FIVE_TIP_PINCH, OPEN_PALM, OTHER):
            for scale in (0.4, 1.0, 2.0):
                for reflection in (1, -1):
                    points = skeleton(gesture) * [reflection, 1, 1]
                    transformed = rotation.apply(points) * scale + [0.1, -0.3, 0.9]
                    self.assertEqual(classify_gesture(transformed), gesture)

    def test_thumb_up_and_pointing_are_not_fists(self):
        thumb_up = skeleton(FIST)
        thumb_up[4] = [0.04, 0.19, 0]
        self.assertEqual(classify_gesture(thumb_up), OTHER)
        pointing = skeleton(FIST)
        pointing[5:9] = skeleton(OPEN_PALM)[5:9]
        self.assertEqual(classify_gesture(pointing), OTHER)

    def test_near_miss_ok_and_incomplete_five_tip_pinch(self):
        almost_ok = skeleton(OK)
        almost_ok[4] += [0.04, 0, 0]
        self.assertEqual(classify_gesture(almost_ok), OTHER)
        incomplete = skeleton(FIVE_TIP_PINCH)
        incomplete[20] = skeleton(OTHER)[20]
        self.assertNotEqual(classify_gesture(incomplete), FIVE_TIP_PINCH)

    def test_clustered_curled_fingers_are_not_five_tip_pinch(self):
        fist = skeleton(FIST)
        target = np.array([0.0, 0.075, -0.025])
        for tip in (4, 8, 12, 16, 20):
            fist[tip] = target
        self.assertNotEqual(classify_gesture(fist), FIVE_TIP_PINCH)

    def test_contact_gestures_allow_bent_fingers(self):
        for gesture, offset in ((OK, 0.023), (FIVE_TIP_PINCH, 0.035)):
            points = skeleton(gesture)
            for base in (9, 13, 17):
                points[base + 1] += [offset, 0, 0]
            self.assertEqual(classify_gesture(points), gesture)

    def test_image_contact_recovers_moderate_world_depth_error(self):
        for gesture in (OK, FIVE_TIP_PINCH):
            original = skeleton(gesture)
            points = original.copy()
            if gesture == OK:
                points[4, 2] += 0.04
            else:
                points[[4, 8, 12, 16, 20], 2] += [0, 0.015, 0.03, -0.01, 0.06]
            self.assertEqual(classify_gesture(points, pinch_distance=0.65), OTHER)
            for scale in (0.5, 2.0):
                pixels = original[:, :2] * 1000 * scale + [100, 200]
                self.assertEqual(classify_gesture(points, pixels, pinch_distance=0.65), gesture)

    def test_image_overlap_does_not_override_large_world_separation(self):
        for gesture in (OK, FIVE_TIP_PINCH):
            original = skeleton(gesture)
            points = original.copy()
            points[4, 2] += 0.15
            self.assertNotEqual(classify_gesture(points, original[:, :2] * 1000), gesture)

    def test_edge_on_projection_does_not_supply_contact(self):
        points = skeleton(OK)
        pixels = points[:, :2].copy() * 1000
        pixels[:, 0] = 100
        points[4, 2] += 0.04
        self.assertEqual(classify_gesture(points, pixels), OTHER)

    def test_distance_thresholds_can_be_tuned(self):
        points = skeleton(OK)
        points[4] += [0.04, 0, 0]
        self.assertEqual(classify_gesture(points), OTHER)
        self.assertEqual(classify_gesture(points, ok_distance=0.6), OK)
        points = skeleton(FIVE_TIP_PINCH)
        points[20, 2] += 0.085
        self.assertEqual(classify_gesture(points), OTHER)
        self.assertEqual(classify_gesture(points, pinch_distance=1.25), FIVE_TIP_PINCH)

    def test_invalid_image_data_preserves_world_classification(self):
        for gesture in (FIST, OK, FIVE_TIP_PINCH, OPEN_PALM, OTHER):
            for pixels in (np.zeros((20, 2)), np.zeros((21, 2)), np.full((21, 2), np.nan)):
                self.assertEqual(classify_gesture(skeleton(gesture), pixels), gesture)

    def test_open_palm_with_parallel_or_spread_fingers(self):
        points = skeleton(OPEN_PALM)
        self.assertEqual(classify_gesture(points), OPEN_PALM)
        for base, sideways in zip(FINGER_BASES, (0.3, 0, -0.15, -0.3)):
            direction = np.array([sideways, 1, 0], dtype=float)
            direction /= np.linalg.norm(direction)
            for offset, reach in enumerate((0.035, 0.057, 0.074), start=1):
                points[base + offset] = points[base] + reach * direction
        self.assertEqual(classify_gesture(points), OPEN_PALM)

    def test_open_palm_requires_every_finger_extended(self):
        for base in FINGER_BASES:
            points = skeleton(OPEN_PALM)
            points[base:base + 4] = skeleton(FIST)[base:base + 4]
            self.assertEqual(classify_gesture(points), OTHER)
        hooked = skeleton(OPEN_PALM)
        hooked[8] = hooked[7] + [0.018, 0.006, 0]
        self.assertEqual(classify_gesture(hooked), OTHER)

    def test_open_palm_requires_unfolded_and_abducted_thumb(self):
        folded = skeleton(OPEN_PALM)
        folded[4] = [0.01, 0.06, -0.01]
        self.assertEqual(classify_gesture(folded), OTHER)
        tucked = skeleton(OPEN_PALM)
        for offset, fraction in enumerate((0.3, 0.65, 1.0), start=2):
            tucked[offset] = tucked[1] + fraction * (np.array([0.035, 0.10, 0]) - tucked[1])
        self.assertEqual(classify_gesture(tucked), OTHER)
        invalid = skeleton(OPEN_PALM)
        invalid[3] = invalid[4]
        self.assertNotEqual(classify_gesture(invalid), OPEN_PALM)

    def test_image_open_palm_recovers_world_angle_error(self):
        original = skeleton(OPEN_PALM)
        points = original.copy()
        points[6, 2] += 0.018
        self.assertEqual(classify_gesture(points), OTHER)
        for scale in (0.5, 2.0):
            pixels = original[:, :2] * 1000 * scale + [100, 200]
            self.assertEqual(classify_gesture(points, pixels), OPEN_PALM)

    def test_image_open_palm_does_not_override_curled_world_fingers(self):
        pixels = skeleton(OPEN_PALM)[:, :2] * 1000
        for base in FINGER_BASES:
            points = skeleton(OPEN_PALM)
            points[base:base + 4] = skeleton(FIST)[base:base + 4]
            self.assertNotEqual(classify_gesture(points, pixels), OPEN_PALM)
        folded = skeleton(OPEN_PALM)
        folded[4] = [0.01, 0.06, -0.01]
        self.assertNotEqual(classify_gesture(folded, pixels), OPEN_PALM)
        points = skeleton(OPEN_PALM)
        points[6, 2] += 0.018
        pixels[:, 0] = 100
        self.assertNotEqual(classify_gesture(points, pixels), OPEN_PALM)

    def test_invalid_skeleton(self):
        for points in (np.zeros((21, 3)), np.zeros((20, 3)), np.full((21, 3), np.nan)):
            self.assertEqual(classify_gesture(points), GESTURE_UNKNOWN)


class GestureTrackingTests(unittest.TestCase):
    def detection(self, label, gesture, score=0.95):
        return {"label": label, "gesture": gesture, "score": score}

    def test_depth_independent_two_hand_results_and_transitions(self):
        tracker = GestureTracker()
        detections = [self.detection("Left", FIST), self.detection("Right", OK)]
        self.assertEqual(tracker.update(detections, 1.0), {"Left": -1, "Right": -1})
        self.assertEqual(tracker.update(detections[::-1], 1.2), {"Left": FIST, "Right": OK})
        detections[0]["gesture"] = FIVE_TIP_PINCH
        self.assertEqual(tracker.update(detections, 1.21), {"Left": -1, "Right": OK})
        self.assertEqual(tracker.update(detections, 1.4), {"Left": FIVE_TIP_PINCH, "Right": OK})
        self.assertEqual(tracker.update([], 1.41), {"Left": -1, "Right": -1})

    def test_uncertain_or_duplicate_identity_clears_results(self):
        tracker = GestureTracker(settle_time=0)
        self.assertEqual(tracker.update([self.detection("Left", FIST)], 1.0)["Left"], FIST)
        duplicate = [self.detection("Left", FIST), self.detection("Left", OK)]
        self.assertEqual(tracker.update(duplicate, 1.1)["Left"], GESTURE_UNKNOWN)
        self.assertEqual(tracker.update([self.detection("Left", FIST, score=0.55)], 1.2)["Left"], GESTURE_UNKNOWN)

    def test_open_palm_confirmation_and_independent_hand_transition(self):
        tracker = GestureTracker()
        detections = [self.detection("Left", OPEN_PALM), self.detection("Right", FIST)]
        self.assertEqual(tracker.update(detections, 0), {"Left": -1, "Right": -1})
        self.assertEqual(tracker.update(detections, 0.2), {"Left": OPEN_PALM, "Right": FIST})
        detections[0]["gesture"] = OTHER
        self.assertEqual(tracker.update(detections, 0.21), {"Left": -1, "Right": FIST})
        self.assertEqual(tracker.update(detections, 0.4), {"Left": OTHER, "Right": FIST})

    def test_status_panel_fits_longest_gesture(self):
        panel = draw_facing_flags(
            {"Left": -1, "Right": 1},
            {"Left": "FIVE_TIP_PINCH [3]", "Right": "OPEN_PALM [4]"},
        )
        self.assertEqual(panel.shape, (220, 540, 3))


if __name__ == "__main__":
    unittest.main()
