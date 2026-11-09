import copy
import time
from types import SimpleNamespace
import unittest

import numpy as np
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation

from scripts.hand_pose import PALM_IDS, PalmTracker, estimate_palm
from scripts.pose_protocol import FRAME_ID, validate_packet
from scripts.rgb_hand_pose import ANCHOR_IDS, RGBPalmEstimator, camera_parameters, palm_rotation_hint


def synthetic_hand(rotation=None, translation=None):
    rotation = Rotation.identity() if rotation is None else rotation
    translation = np.array([0.0, 0.0, 0.6]) if translation is None else translation
    local = np.array([
        [0.0, -0.05, 0], [0.04, -0.03, 0], [0.06, -0.01, 0],
        [0.07, 0.01, 0], [0.08, 0.03, 0],
        [0.035, 0.02, 0], [0.035, 0.05, 0], [0.035, 0.075, 0], [0.035, 0.095, 0],
        [0.0, 0.025, 0], [0.0, 0.06, 0], [0.0, 0.085, 0], [0.0, 0.11, 0],
        [-0.018, 0.025, 0], [-0.018, 0.05, 0], [-0.018, 0.075, 0], [-0.018, 0.09, 0],
        [-0.035, 0.02, 0], [-0.04, 0.04, 0], [-0.04, 0.06, 0], [-0.04, 0.075, 0],
    ])
    points = rotation.apply(local) + translation
    intrinsics = rs.intrinsics()
    intrinsics.width, intrinsics.height = 640, 480
    intrinsics.fx = intrinsics.fy = 600.0
    intrinsics.ppx, intrinsics.ppy = 320.0, 240.0
    intrinsics.model = rs.distortion.none
    landmarks = []
    for point in points:
        u, v = rs.rs2_project_point_to_pixel(intrinsics, point.tolist())
        landmarks.append(SimpleNamespace(x=u / 640, y=v / 480))
    yy, xx = np.mgrid[:480, :640]
    rays = np.stack(((xx - 320) / 600, (yy - 240) / 600, np.ones_like(xx)), axis=-1)
    normal = rotation.apply([0, 0, 1])
    depth = (normal @ translation) / (rays @ normal)
    depth = (np.rint(depth * 1000) / 1000).astype(np.float32)
    return landmarks, depth, intrinsics, points, rotation


def observation(label, x, quaternion=None, score=0.95):
    position = np.array([x, 0.0, 0.6])
    return {
        "detected_label": label, "score": score,
        "position": position,
        "orientation": np.array([0, 0, 0, 1] if quaternion is None else quaternion, dtype=float),
        "joints": np.tile(position, (21, 1)),
        "quality": 0.8, "plane_rms_m": 0.002,
    }


class GeometryTests(unittest.TestCase):
    def test_rgb_pnp_and_depth_frames_agree_when_the_palm_tilts(self):
        for angle in (-35, -15, 0, 15, 35):
            expected = Rotation.from_euler("y", angle, degrees=True) * Rotation.from_euler("x", 180, degrees=True)
            landmarks, depth, intrinsics, points, _ = synthetic_hand(expected)
            measured, reason = estimate_palm(landmarks, depth, intrinsics)
            self.assertIsNone(reason)
            measured_rotation = Rotation.from_quat(measured["orientation"])
            local = measured_rotation.inv().apply(measured["joints"][list(ANCHOR_IDS)] - measured["position"])
            local[:, 2] = 0
            local -= local.mean(axis=0)
            calibration = {
                "version": 1, "units": "m", "camera": camera_parameters(intrinsics, "synthetic-camera"),
                "hands": {"Right": {"anchor_ids": list(ANCHOR_IDS), "points_m": local.tolist()}},
            }
            pose, reason = RGBPalmEstimator(calibration).estimate("Right", landmarks, 0, world_landmarks=points)
            self.assertIsNone(reason)
            actual = Rotation.from_quat(pose["orientation"])
            self.assertLess((measured_rotation.inv() * actual).magnitude(), np.deg2rad(5))
            self.assertLess((measured_rotation.inv() * palm_rotation_hint(points)).magnitude(), np.deg2rad(3))

    def test_known_metric_pose_and_rotation(self):
        for angles in ([0, 0, 0], [25, -15, 10], [-20, 35, -40]):
            landmarks, depth, intrinsics, points, expected_rotation = synthetic_hand(
                Rotation.from_euler("xyz", angles, degrees=True),
            )
            pose, reason = estimate_palm(landmarks, depth, intrinsics)
            self.assertIsNone(reason)
            np.testing.assert_allclose(pose["position"], points[list(PALM_IDS)].mean(axis=0), atol=0.003)
            error = Rotation.from_quat(pose["orientation"]).inv() * expected_rotation
            self.assertLess(error.magnitude(), np.deg2rad(3))
            self.assertAlmostEqual(np.linalg.det(Rotation.from_quat(pose["orientation"]).as_matrix()), 1.0)

    def test_invalid_and_background_depth_is_rejected(self):
        landmarks, depth, intrinsics, _, _ = synthetic_hand()
        for invalid in (np.zeros_like(depth), np.full_like(depth, np.nan), np.full_like(depth, 3.0)):
            pose, reason = estimate_palm(landmarks, invalid, intrinsics)
            self.assertIsNone(pose)
            self.assertIsNotNone(reason)
        wrist = landmarks[0]
        x, y = round(wrist.x * 640), round(wrist.y * 480)
        depth[y - 5:y + 6, x - 5:x + 6] = 3.0
        pose, reason = estimate_palm(landmarks, depth, intrinsics)
        self.assertIsNone(pose)
        self.assertIn("anchor", reason)

    def test_offscreen_and_degenerate_palm_is_rejected(self):
        landmarks, depth, intrinsics, _, _ = synthetic_hand()
        landmarks[0].x = -0.1
        self.assertIsNone(estimate_palm(landmarks, depth, intrinsics)[0])
        landmarks = [SimpleNamespace(x=0.5, y=0.5) for _ in range(21)]
        self.assertIsNone(estimate_palm(landmarks, depth, intrinsics)[0])


class TrackingTests(unittest.TestCase):
    def test_order_and_classifier_jitter_preserve_identity(self):
        tracker = PalmTracker()
        first = tracker.update([observation("Left", -0.1), observation("Right", 0.1)], 1.0)
        self.assertEqual({item["label"] for item in first}, {"Left", "Right"})
        next_frame = tracker.update([
            observation("Right", 0.11), observation("Right", -0.11, score=0.7),
        ], 1.03)
        by_side = {item["label"]: item for item in next_frame}
        self.assertLess(by_side["Left"]["position"][0], 0)
        self.assertGreater(by_side["Right"]["position"][0], 0)

    def test_no_stale_output_and_filter_reset(self):
        tracker = PalmTracker()
        first = tracker.update([observation("Left", 0.0)], 1.0)[0]
        second = tracker.update([observation("Left", 0.03, [0, 0, 0, -1])], 1.03)[0]
        self.assertGreater(second["position"][0], 0)
        self.assertLess(second["position"][0], 0.03)
        self.assertGreater(np.dot(first["orientation"], second["orientation"]), 0.99)
        self.assertEqual(tracker.update([], 1.1), [])
        recovered = tracker.update([observation("Left", 0.5)], 1.8)[0]
        self.assertEqual(recovered["position"][0], 0.5)

    def test_duplicate_initial_label_is_not_invented(self):
        output = PalmTracker().update([
            observation("Left", -0.1, score=0.95), observation("Left", 0.1, score=0.7),
        ], 1.0)
        self.assertEqual(len(output), 1)
        self.assertEqual(output[0]["label"], "Left")


class ProtocolTests(unittest.TestCase):
    def packet(self):
        return {
            "version": 1, "frame_id": FRAME_ID, "sequence": 1,
            "capture_monotonic_ns": time.monotonic_ns(), "capture_time_ns": time.time_ns(),
            "hands": PalmTracker().update([observation("Left", 0.0), observation("Right", 0.1)], 1.0),
        }

    def test_valid_two_hand_packet(self):
        packet = self.packet()
        self.assertIs(validate_packet(packet, time.monotonic_ns()), packet)

    def test_stale_duplicate_nan_and_bad_quaternion_are_rejected(self):
        packet = self.packet()
        variants = []
        stale = copy.deepcopy(packet)
        stale["capture_monotonic_ns"] -= 1_000_000_000
        variants.append(stale)
        duplicate = copy.deepcopy(packet)
        duplicate["hands"][1]["label"] = "Left"
        variants.append(duplicate)
        invalid = copy.deepcopy(packet)
        invalid["hands"][0]["position"][0] = float("nan")
        variants.append(invalid)
        quaternion = copy.deepcopy(packet)
        quaternion["hands"][0]["orientation"] = [0, 0, 0, 0]
        variants.append(quaternion)
        for variant in variants:
            with self.assertRaises(ValueError):
                validate_packet(variant, time.monotonic_ns())


if __name__ == "__main__":
    unittest.main()
