import copy
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from scripts.hand_pose import PalmTracker
from scripts.pose_protocol import FRAME_ID, validate_packet
from scripts.rgb_hand_pose import (
    ANCHOR_IDS, PalmCalibrator, RGBPalmEstimator, camera_parameters,
    check_camera, load_calibration, palm_rotation_hint, validate_calibration,
)


def camera():
    return camera_parameters(SimpleNamespace(
        width=640, height=480, fx=600.0, fy=605.0, ppx=320.0, ppy=240.0,
        coeffs=[0.02, -0.01, 0.001, -0.001, 0.0], model="distortion.brown_conrady",
    ), "test-camera")


def palm_points(scale=1.0):
    points = np.array([
        [0, -0.05, 0], [0.035, 0.02, 0], [0, 0.025, 0],
        [-0.018, 0.025, 0], [-0.035, 0.02, 0],
    ]) * scale
    return points - points.mean(axis=0)


def calibration():
    return {
        "version": 1, "units": "m", "camera": camera(),
        "hands": {label: {"anchor_ids": list(ANCHOR_IDS), "points_m": palm_points(scale).tolist()}
                  for label, scale in (("Left", 0.9), ("Right", 1.1))},
    }


def asymmetric_calibration():
    model = calibration()
    points = np.array([
        [0.008, -0.073, 0], [0.030, 0.022, 0], [0.008, 0.025, 0],
        [-0.013, 0.019, 0], [-0.033, 0.007, 0],
    ])
    points -= points.mean(axis=0)
    for label in model["hands"]:
        model["hands"][label]["points_m"] = points.tolist()
    return model


def world_skeleton(model, label, rotation):
    points = np.zeros((21, 3))
    points[list(ANCHOR_IDS)] = rotation.apply(model["hands"][label]["points_m"])
    return points


def observation(points, rotation=None, translation=None):
    rotation = Rotation.identity() if rotation is None else rotation
    translation = np.array([0, 0, 0.6]) if translation is None else np.asarray(translation)
    joints = np.full((21, 3), np.nan)
    joints[list(ANCHOR_IDS)] = rotation.apply(points) + translation
    return {
        "joints": joints, "position": translation, "orientation": rotation.as_quat(),
        "quality": 0.9, "plane_rms_m": 0.001,
    }


def project(model, label="Right", angles=(35, -25, 15), position=(0.02, -0.01, 0.65), noise=0):
    points = np.asarray(model["hands"][label]["points_m"])
    rotation = Rotation.from_euler("xyz", angles, degrees=True)
    camera = model["camera"]
    pixels = cv2.projectPoints(
        points, rotation.as_rotvec(), np.asarray(position, dtype=float),
        np.asarray(camera["matrix"]), np.asarray(camera["distortion_coefficients"]),
    )[0].reshape(-1, 2)
    pixels += np.random.default_rng(7).normal(0, noise, pixels.shape)
    landmarks = [SimpleNamespace(x=0.5, y=0.5) for _ in range(21)]
    for index, pixel in zip(ANCHOR_IDS, pixels):
        landmarks[index] = SimpleNamespace(x=pixel[0] / camera["width"], y=pixel[1] / camera["height"])
    return landmarks, rotation


class CalibrationTests(unittest.TestCase):
    def test_metric_geometry_is_recovered_across_rotations_and_saved(self):
        collector = PalmCalibrator(camera(), samples=20, interval=0)
        rng = np.random.default_rng(9)
        for index in range(20):
            for label, scale in (("Left", 0.9), ("Right", 1.1)):
                rotation = Rotation.from_euler("xyz", rng.uniform(-40, 40, 3), degrees=True)
                pose = observation(palm_points(scale), rotation, [0.05, -0.1, 0.5 + index * 0.01])
                self.assertTrue(collector.add(label, pose, index))
        self.assertTrue(collector.complete)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "nested/geometry.json"
            collector.save(path)
            result = load_calibration(path)
            self.assertFalse(path.with_suffix(".json.tmp").exists())
        for label, scale in (("Left", 0.9), ("Right", 1.1)):
            np.testing.assert_allclose(result["hands"][label]["points_m"], palm_points(scale), atol=1e-10)
            self.assertEqual(result["hands"][label]["sample_count"], 20)

    def test_incomplete_calibration_preserves_existing_file(self):
        collector = PalmCalibrator(camera(), samples=20)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "geometry.json"
            path.write_text(json.dumps(calibration()))
            before = path.read_bytes()
            with self.assertRaisesRegex(ValueError, "incomplete"):
                collector.save(path)
            self.assertEqual(path.read_bytes(), before)

    def test_invalid_depth_quality_and_oversampling_are_rejected(self):
        collector = PalmCalibrator(camera(), labels=("Right",), samples=20)
        pose = observation(palm_points())
        self.assertFalse(collector.add("Left", pose, 0))
        for field, value in (("quality", 0.1), ("plane_rms_m", 0.02)):
            bad = dict(pose, **{field: value})
            self.assertFalse(collector.add("Right", bad, 0))
        bad = copy.deepcopy(pose)
        bad["joints"][13] = np.nan
        self.assertFalse(collector.add("Right", bad, 0))
        self.assertTrue(collector.add("Right", pose, 0))
        self.assertFalse(collector.add("Right", pose, 0.01))

    def test_unstable_samples_are_replaced(self):
        collector = PalmCalibrator(camera(), labels=("Right",), samples=20, interval=0)
        for index in range(20):
            points = palm_points()
            if index >= 15:
                points[1, 0] += 0.03
            collector.add("Right", observation(points), index)
        self.assertFalse(collector.complete)
        for index in range(20, 25):
            collector.add("Right", observation(palm_points()), index)
        self.assertTrue(collector.complete)
        np.testing.assert_allclose(collector.models["Right"]["points_m"], palm_points(), atol=1e-10)

    def test_schema_and_camera_mismatch_are_rejected(self):
        original = calibration()
        for mutate in (
            lambda data: data.update(units="mm"),
            lambda data: data["camera"].update(matrix=[[0] * 3] * 3),
            lambda data: data["hands"]["Left"].update(points_m=[[0, 0, 0]] * 5),
            lambda data: data["hands"].clear(),
            lambda data: data["camera"].update(distortion_model="inverse_brown_conrady"),
        ):
            data = copy.deepcopy(original)
            mutate(data)
            with self.assertRaises(ValueError):
                validate_calibration(data)
        check_camera(original, camera())
        for key, value in (("serial", "other"), ("width", 1280), ("matrix", [[500, 0, 320], [0, 500, 240], [0, 0, 1]])):
            current = camera()
            current[key] = value
            with self.assertRaises(ValueError):
                check_camera(original, current)
        with self.assertRaisesRegex(ValueError, "missing"):
            load_calibration("/tmp/nonexistent-hand-calibration-test.json")


class PnPTests(unittest.TestCase):
    def test_frontal_tilt_sweep_preserves_normal_direction_for_both_hands(self):
        model = asymmetric_calibration()
        for label, frontal in (("Left", 0), ("Right", 180)):
            for axis in ("x", "y", "z"):
                for angle in (-40, -20, 0, 20, 40):
                    angles = {"x": (frontal + angle, 0, 0), "y": (frontal, angle, 0), "z": (frontal, 0, angle)}[axis]
                    with self.subTest(label=label, axis=axis, angle=angle):
                        landmarks, expected = project(model, label, angles, position=(0, 0, 0.6))
                        pose, reason = RGBPalmEstimator(model).estimate(label, landmarks, 0)
                        self.assertIsNone(reason)
                        error = (expected.inv() * Rotation.from_quat(pose["orientation"])).magnitude()
                        self.assertLess(error, np.deg2rad(0.1))

    def test_rgb_orientation_disambiguates_noisy_mirrored_tilt(self):
        model = asymmetric_calibration()
        for label, frontal in (("Left", 0), ("Right", 180)):
            for angle in (-25, 25):
                landmarks, expected = project(model, label, (frontal, angle, 0), noise=0.7)
                world = world_skeleton(model, label, expected)
                pose, reason = RGBPalmEstimator(model).estimate(label, landmarks, 0, world_landmarks=world)
                self.assertIsNone(reason)
                actual = Rotation.from_quat(pose["orientation"])
                self.assertLess((expected.inv() * actual).magnitude(), np.deg2rad(10))
                self.assertGreater(expected.as_matrix()[0, 2] * actual.as_matrix()[0, 2], 0)
                self.assertLess(pose["pnp_hint_angle_deg"], 10)

    def test_invalid_rgb_orientation_hint_does_not_change_clean_solution(self):
        model = asymmetric_calibration()
        landmarks, expected = project(model, angles=(180, 35, 0))
        for world in (np.zeros((21, 3)), np.zeros((20, 3)), np.full((21, 3), np.nan)):
            self.assertIsNone(palm_rotation_hint(world))
            pose, reason = RGBPalmEstimator(model).estimate("Right", landmarks, 0, world_landmarks=world)
            self.assertIsNone(reason)
            self.assertFalse(pose["pnp_orientation_hint_used"])
            self.assertIsNone(pose["pnp_hint_angle_deg"])
            self.assertLess((expected.inv() * Rotation.from_quat(pose["orientation"])).magnitude(), 1e-3)

    def test_rgb_orientation_cannot_rescue_an_inconsistent_reprojection(self):
        model = asymmetric_calibration()
        landmarks, expected = project(model, angles=(180, 35, 0))
        wrong = expected * Rotation.from_euler("y", 90, degrees=True)
        world = world_skeleton(model, "Right", wrong)
        pose, reason = RGBPalmEstimator(model).estimate("Right", landmarks, 0, world_landmarks=world)
        self.assertIsNone(pose)
        self.assertEqual(reason, "RGB orientation/PnP inconsistent")

    def test_known_metric_poses_both_hands_with_distortion(self):
        model = calibration()
        for label in ("Left", "Right"):
            for angles in ((0, 0, 0), (35, -25, 15), (-40, 30, -25), (150, -20, 10)):
                estimator = RGBPalmEstimator(model)
                landmarks, expected = project(model, label, angles)
                pose, reason = estimator.estimate(label, landmarks, 0)
                self.assertIsNone(reason, (angles, reason))
                np.testing.assert_allclose(pose["position"], [0.02, -0.01, 0.65], atol=1e-5)
                self.assertLess((expected.inv() * Rotation.from_quat(pose["orientation"])).magnitude(), 1e-3)
                self.assertEqual(pose["pose_source"], "rgb_pnp")
                self.assertLess(pose["reprojection_rms_px"], 1e-4)
                self.assertTrue(np.isnan(pose["joints"][4]).all())

    def test_small_image_noise_and_temporal_following(self):
        model = calibration()
        estimator = RGBPalmEstimator(model)
        for index in range(5):
            position = [0.02 + index * 0.002, -0.01, 0.65]
            landmarks, expected = project(model, angles=(45, -30, 15), position=position, noise=0.2)
            pose, reason = estimator.estimate("Right", landmarks, index * 0.033)
            self.assertIsNone(reason)
            self.assertLess(np.linalg.norm(pose["position"] - position), 0.01)
            self.assertLess((expected.inv() * Rotation.from_quat(pose["orientation"])).magnitude(), np.deg2rad(5))

    def test_outliers_offscreen_degenerate_and_depth_range(self):
        model = calibration()
        estimator = RGBPalmEstimator(model, max_error=2)
        landmarks, _ = project(model)
        landmarks[13].x += 0.1
        self.assertIsNone(estimator.estimate("Right", landmarks, 0)[0])
        landmarks, _ = project(model)
        landmarks[0].x = -0.1
        self.assertEqual(estimator.estimate("Right", landmarks, 0)[1], "palm outside image")
        for index in ANCHOR_IDS:
            landmarks[index] = SimpleNamespace(x=0.5, y=0.5)
        self.assertIsNone(estimator.estimate("Right", landmarks, 0)[0])
        landmarks, _ = project(model, position=(0, 0, 2.0))
        self.assertIsNone(estimator.estimate("Right", landmarks, 0)[0])
        landmarks[9].x = np.nan
        self.assertEqual(estimator.estimate("Right", landmarks, 0)[1], "invalid landmarks")

    def test_uncalibrated_hand_and_stale_rotation_prior(self):
        model = calibration()
        del model["hands"]["Left"]
        estimator = RGBPalmEstimator(model)
        landmarks, _ = project(model)
        self.assertEqual(estimator.estimate("Left", landmarks, 0)[1], "hand not calibrated")
        self.assertIsNotNone(estimator.estimate("Right", landmarks, 0)[0])
        turned, _ = project(model, angles=(150, -20, 10))
        self.assertEqual(estimator.estimate("Right", turned, 0.03)[1], "PnP orientation jump")
        self.assertIsNotNone(estimator.estimate("Right", turned, 1.0)[0])

    def test_rgb_pose_survives_tracking_and_json_protocol(self):
        model = calibration()
        estimator = RGBPalmEstimator(model)
        observations = []
        for label in ("Left", "Right"):
            landmarks, _ = project(model, label)
            pose, reason = estimator.estimate(label, landmarks, 0)
            self.assertIsNone(reason)
            pose.update(detected_label=label, score=0.99, detection_index=len(observations))
            observations.append(pose)
        hands = PalmTracker().update(observations, 0)
        packet = json.loads(json.dumps({
            "version": 1, "frame_id": FRAME_ID, "sequence": 1,
            "capture_monotonic_ns": time.monotonic_ns(), "capture_time_ns": time.time_ns(),
            "hands": hands,
        }, allow_nan=False))
        self.assertIs(validate_packet(packet, time.monotonic_ns()), packet)
        for hand in hands:
            self.assertEqual(hand["pose_source"], "rgb_pnp")
            self.assertIsNone(hand["joints"][4])
            self.assertIsNone(hand["plane_rms_m"])


if __name__ == "__main__":
    unittest.main()
