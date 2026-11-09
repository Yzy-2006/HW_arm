"""Gesture transitions, coordinate mapping, bounded IK and simulation recordings."""

import copy
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "teleop"))
sys.path.insert(0, str(ROOT / "simulation"))
from gesture_control import FIST, OPEN_PALM, GestureMapper, decode_packet, load_config
from gesture_recording import SimulationRecorder
from gesture_pose_fusion import PosePacketFusion


def packet(stamp, left=0, right=0, position=(0, 0, 0.5), rotation=None, missing=()):
    quaternion = [0, 0, 0, 1] if rotation is None else rotation.as_quat().tolist()
    return {"version": 1, "frame_id": "camera_color_optical_frame", "sequence": int(stamp * 1000),
            "capture_monotonic_ns": round(stamp * 1e9), "capture_time_ns": round(stamp * 1e9),
            "gestures": {"Left": left, "Right": right},
            "hands": [{"label": label, "position": list(position), "orientation": quaternion,
                       "quality": 1.0, "joints": [None] * 21} for label in ("Left", "Right") if label not in missing]}


class GestureMappingTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config()
        self.mapper = GestureMapper(self.config)

    def release(self, start=1.0):
        for i in range(7):
            self.mapper.consume(packet(start + i * 0.03))

    def test_camera_facing_axes_and_quaternion_conjugation(self):
        self.release()
        self.mapper.consume(packet(1.21, left=OPEN_PALM))
        result = self.mapper.consume(packet(1.24, left=OPEN_PALM, position=(0.001, -0.002, 0.497),
                                            rotation=Rotation.from_rotvec([0.01, 0, 0])))
        hand = result["hands"]["left"]
        np.testing.assert_allclose(hand["position_delta"], [0.003, 0.001, 0.002], atol=1e-9)
        np.testing.assert_allclose(hand["rotation_delta"], [0, 0.01, 0], atol=1e-9)
        self.assertFalse(result["hands"]["right"]["active"])

    def test_initial_open_palm_cannot_move_and_reclutch_has_no_jump(self):
        self.assertFalse(self.mapper.consume(packet(1, left=OPEN_PALM))["hands"]["left"]["active"])
        self.release(1.03)
        result = self.mapper.consume(packet(1.24, left=OPEN_PALM, position=(0.2, 0, 0.5)))
        self.assertTrue(result["hands"]["left"]["active"])
        np.testing.assert_array_equal(result["hands"]["left"]["position_delta"], np.zeros(3))

    def test_fist_stops_motion_and_reopening_reanchors(self):
        self.release()
        self.mapper.consume(packet(1.21, left=OPEN_PALM))
        self.mapper.consume(packet(1.24, left=OPEN_PALM, position=(0.001, 0, 0.5)))
        for i in range(7):
            hand = self.mapper.consume(packet(1.27 + i * 0.03, left=FIST,
                                               position=(0.1 + i * 0.001, 0, 0.5)))["hands"]["left"]
            self.assertFalse(hand["active"])
            self.assertEqual(hand["motion_state"], "HOLD")
            np.testing.assert_array_equal(hand["position_delta"], np.zeros(3))
        hand = self.mapper.consume(packet(1.48, left=OPEN_PALM, position=(0.2, 0, 0.5)))["hands"]["left"]
        self.assertEqual(hand["motion_state"], "ANCHOR")
        np.testing.assert_array_equal(hand["position_delta"], np.zeros(3))

    def test_open_palm_packet_is_accepted_but_unknown_id_is_rejected(self):
        self.assertIsNotNone(decode_packet(json.dumps(packet(10, left=OPEN_PALM, right=OPEN_PALM)), 10.1, 0.35))
        self.assertIsNone(decode_packet(json.dumps(packet(10, left=5)), 10.1, 0.35))

    def test_recognizer_unknown_transition_does_not_prevent_clutch(self):
        self.release()
        self.mapper.consume(packet(1.21, left=-1))
        self.mapper.consume(packet(1.24, left=-1))
        self.assertTrue(self.mapper.consume(packet(1.27, left=OPEN_PALM))["hands"]["left"]["active"])
        self.mapper.consume(packet(1.30, left=-1))
        result = self.mapper.consume(packet(1.33, left=OPEN_PALM, position=(0.1, 0, 0.5)))
        np.testing.assert_array_equal(result["hands"]["left"]["position_delta"], np.zeros(3))

    def test_long_stream_timeout_requires_release(self):
        self.release()
        self.mapper.consume(packet(1.21, left=OPEN_PALM))
        self.assertTrue(self.mapper.expired(2.3))
        hand = self.mapper.consume(packet(2.33, left=OPEN_PALM))["hands"]["left"]
        self.assertFalse(hand["active"])
        self.assertEqual(hand["release_reason"], "stream_lost")

    def test_brief_stream_gap_resumes_with_zero_first_increment(self):
        self.release()
        self.mapper.consume(packet(1.21, left=OPEN_PALM))
        self.assertTrue(self.mapper.expired(1.7))
        hand = self.mapper.consume(packet(1.73, left=OPEN_PALM, position=(0.2, 0, 0.5)))["hands"]["left"]
        self.assertEqual(hand["motion_state"], "ANCHOR")
        np.testing.assert_array_equal(hand["position_delta"], np.zeros(3))
        hand = self.mapper.consume(packet(1.76, left=OPEN_PALM, position=(0.201, 0, 0.5)))["hands"]["left"]
        np.testing.assert_allclose(hand["position_delta"], [0, 0.001, 0])

    def test_jump_is_not_applied_and_recovers_after_consistent_poses(self):
        self.release()
        self.mapper.consume(packet(1.21, left=OPEN_PALM))
        hand = self.mapper.consume(packet(1.24, left=OPEN_PALM, position=(0.3, 0, 0.5)))["hands"]["left"]
        self.assertEqual(hand["motion_state"], "RECOVER")
        self.assertFalse(hand["active"])
        for i in range(7):
            hand = self.mapper.consume(packet(1.27 + i * 0.03, left=OPEN_PALM))["hands"]["left"]
            np.testing.assert_array_equal(hand["position_delta"], np.zeros(3))
        hand = self.mapper.consume(packet(1.48, left=OPEN_PALM, position=(0.001, 0, 0.5)))["hands"]["left"]
        self.assertEqual(hand["motion_state"], "MOVE")
        np.testing.assert_allclose(hand["position_delta"], [0, 0.001, 0])

    def test_continuing_pose_jumps_require_release(self):
        self.release()
        self.mapper.consume(packet(1.21, left=OPEN_PALM))
        for i in range(40):
            hand = self.mapper.consume(packet(1.24 + i * 0.03, left=OPEN_PALM,
                                               position=(0.3 if i % 2 == 0 else 0, 0, 0.5)))["hands"]["left"]
            self.assertFalse(hand["active"])
            np.testing.assert_array_equal(hand["position_delta"], np.zeros(3))
        self.assertEqual(hand["motion_state"], "RELEASE")
        self.assertEqual(hand["release_reason"], "pose_unstable")

    def test_pose_loss_reanchors_without_losing_confirmed_open_palm_intent(self):
        # Pose is optional when confirming a release, but required for movement.
        for i in range(7):
            self.mapper.consume(packet(1 + i * 0.03, missing=("Left",)))
        lost = self.mapper.consume(packet(1.21, left=OPEN_PALM, missing=("Left",)))["hands"]["left"]
        self.assertFalse(lost["active"])
        self.assertEqual(lost["motion_state"], "NO_POSE")
        for i in range(20):
            self.mapper.consume(packet(1.24 + i * 0.03, left=OPEN_PALM, missing=("Left",)))
        recovered = self.mapper.consume(packet(1.84, left=OPEN_PALM, position=(0.2, 0, 0.5)))["hands"]["left"]
        self.assertEqual(recovered["motion_state"], "ANCHOR")
        np.testing.assert_array_equal(recovered["position_delta"], np.zeros(3))
        moved = self.mapper.consume(packet(1.87, left=OPEN_PALM, position=(0.201, 0, 0.5)))["hands"]["left"]
        np.testing.assert_allclose(moved["position_delta"], [0, 0.001, 0])

    def test_long_unknown_identity_requires_release(self):
        self.release()
        for i in range(38):
            self.mapper.consume(packet(1.21 + i * 0.03, left=-1))
        result = self.mapper.consume(packet(2.35, left=OPEN_PALM))["hands"]["left"]
        self.assertFalse(result["active"])
        self.assertEqual(result["motion_state"], "RELEASE")
        self.assertEqual(result["release_reason"], "gesture_lost")

    def test_brief_unknown_pauses_then_resumes_without_release(self):
        self.release()
        self.mapper.consume(packet(1.21, left=OPEN_PALM))
        for i in range(20):
            hand = self.mapper.consume(packet(1.24 + i * 0.03, left=-1))["hands"]["left"]
            self.assertFalse(hand["active"])
        hand = self.mapper.consume(packet(1.84, left=OPEN_PALM, position=(0.3, 0, 0.5)))["hands"]["left"]
        self.assertEqual(hand["motion_state"], "ANCHOR")
        np.testing.assert_array_equal(hand["position_delta"], np.zeros(3))
        hand = self.mapper.consume(packet(1.87, left=OPEN_PALM, position=(0.301, 0, 0.5)))["hands"]["left"]
        np.testing.assert_allclose(hand["position_delta"], [0, 0.001, 0])

    def test_pose_source_switch_reanchors_before_sending_deltas(self):
        self.release()
        self.mapper.consume(packet(1.21, left=OPEN_PALM))
        alternate = packet(1.24, left=OPEN_PALM, position=(0.2, 0, 0.5))
        alternate["hands"][0]["pose_source"] = "rgb_pnp"
        hand = self.mapper.consume(alternate)["hands"]["left"]
        self.assertEqual(hand["motion_state"], "ANCHOR")
        np.testing.assert_array_equal(hand["position_delta"], np.zeros(3))
        alternate = packet(1.27, left=OPEN_PALM, position=(0.201, 0, 0.5))
        alternate["hands"][0]["pose_source"] = "rgb_pnp"
        hand = self.mapper.consume(alternate)["hands"]["left"]
        np.testing.assert_allclose(hand["position_delta"], [0, 0.001, 0], atol=1e-10)

    def test_events_once_no_retrigger_on_unknown_and_work_without_pose(self):
        self.release()
        events = []
        for i in range(20):
            events += self.mapper.consume(packet(1.21 + i * 0.03, left=2, right=FIST,
                                                 missing=("Left", "Right")))["events"]
        self.assertEqual(events, ["record", "gripper_right"])
        self.mapper.consume(packet(1.81, left=-1, right=-1))
        for i in range(10):
            self.assertEqual(self.mapper.consume(packet(1.84 + i * 0.03, left=2, right=FIST))["events"], [])
        self.release(2.14)
        events = []
        for i in range(8):
            events += self.mapper.consume(packet(2.35 + i * 0.03, left=2, right=2))["events"]
        self.assertEqual(events, ["record", "home"])

    def test_home_cancels_both_arm_increments(self):
        self.release()
        for i in range(6):
            result = self.mapper.consume(packet(1.21 + i * 0.03, left=OPEN_PALM, right=2,
                                                position=(0, 0, 0.5 - i * 0.001)))
            if "home" in result["events"]:
                self.assertFalse(result["hands"]["left"]["active"])
                np.testing.assert_array_equal(result["hands"]["left"]["position_delta"], np.zeros(3))
                break
        else:
            self.fail("Home gesture did not fire")

    def test_duplicate_and_out_of_order_cannot_change_state(self):
        self.release()
        self.assertIsNone(self.mapper.consume(packet(1.18, left=2)))
        self.assertIsNone(self.mapper.consume(packet(1.01, left=2)))
        self.assertAlmostEqual(self.mapper.last_stamp, 1.18)

    def test_speed_limit_and_swap(self):
        self.config["swap_hands"] = True
        self.mapper = GestureMapper(self.config)
        self.release()
        self.mapper.consume(packet(1.21, right=OPEN_PALM))
        result = self.mapper.consume(packet(1.24, right=OPEN_PALM, position=(0, 0, 0.45)))
        self.assertTrue(result["hands"]["left"]["active"])
        self.assertAlmostEqual(np.linalg.norm(result["hands"]["left"]["position_delta"]), 0.25 * 0.03)

    def test_fist_toggles_each_gripper_once_and_pinch_no_longer_toggles(self):
        self.release()
        for i in range(12):
            self.assertEqual(self.mapper.consume(packet(1.21 + i * 0.03, left=3, right=3))["events"], [])
        events = []
        for i in range(20):
            result = self.mapper.consume(packet(1.60 + i * 0.03, left=FIST, right=FIST))
            events += result["events"]
            self.assertFalse(any(hand["active"] for hand in result["hands"].values()))
        self.assertEqual(events, ["gripper_left", "gripper_right"])
        self.release(2.23)
        events = []
        for i in range(12):
            events += self.mapper.consume(packet(2.44 + i * 0.03, left=FIST, right=FIST))["events"]
        self.assertEqual(events, ["gripper_left", "gripper_right"])

    def test_hardware_yaml_scales_without_input_speed_caps(self):
        for position_scale, orientation_scale in ((0.3, 0.5), (0.6, 0.8)):
            with self.subTest(position_scale=position_scale, orientation_scale=orientation_scale):
                config = load_config(ROOT / "config/gesture_robot.yaml")
                self.assertIsNone(config["position_speed_mps"])
                self.assertIsNone(config["orientation_speed_radps"])
                config.update(position_scale=position_scale, orientation_scale=orientation_scale)
                mapper = GestureMapper(config)
                for i in range(9):
                    mapper.consume(packet(1 + i * 0.03))
                mapper.consume(packet(1.27, left=OPEN_PALM))
                hand = mapper.consume(packet(1.30, left=OPEN_PALM, position=(0, 0, 0.47),
                                             rotation=Rotation.from_rotvec([0.12, 0, 0])))["hands"]["left"]
                self.assertEqual(hand["motion_state"], "MOVE")
                # Both exceed the old speed caps while remaining valid poses.
                np.testing.assert_allclose(hand["position_delta"], [0.03 * position_scale, 0, 0], atol=1e-12)
                np.testing.assert_allclose(hand["rotation_delta"], [0, 0.12 * orientation_scale, 0], atol=1e-12)

    def test_speed_caps_can_be_disabled_independently(self):
        for disabled in ("position_speed_mps", "orientation_speed_radps"):
            with self.subTest(disabled=disabled):
                config = dict(self.config, **{disabled: None})
                mapper = GestureMapper(config)
                for i in range(9):
                    mapper.consume(packet(1 + i * 0.03))
                mapper.consume(packet(1.27, left=OPEN_PALM))
                hand = mapper.consume(packet(1.30, left=OPEN_PALM, position=(0, 0, 0.47),
                                             rotation=Rotation.from_rotvec([0.12, 0, 0])))["hands"]["left"]
                self.assertAlmostEqual(np.linalg.norm(hand["position_delta"]),
                                       0.03 if disabled == "position_speed_mps" else 0.25 * 0.03)
                self.assertAlmostEqual(np.linalg.norm(hand["rotation_delta"]),
                                       0.12 if disabled == "orientation_speed_radps" else 1.0 * 0.03)

    def test_packet_validation(self):
        valid = packet(10)
        self.assertIsNotNone(decode_packet(json.dumps(valid), 10.1, 0.35))
        self.assertIsNone(decode_packet(json.dumps(valid), 11, 0.35))
        self.assertIsNone(decode_packet(json.dumps(valid), 9, 0.35))
        for key, value in (("gestures", {"Left": True, "Right": 1}), ("hands", [{"label": "Left"}]),
                           ("frame_id", "camera_link"), ("sequence", -1)):
            broken = dict(valid, **{key: value})
            self.assertIsNone(decode_packet(json.dumps(broken), 10.1, 0.35))
        for field, value in (("position", [float("nan"), 0, 0]), ("orientation", [0, 0, 0, 0])):
            broken = copy.deepcopy(valid)
            broken["hands"][0][field] = value
            self.assertIsNone(decode_packet(json.dumps(broken), 10.1, 0.35))

    def test_reflection_is_rejected(self):
        config = dict(self.config, camera_to_robot_rotation=np.diag([-1, 1, 1]).tolist())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.json"
            path.write_text(json.dumps(config))
            with self.assertRaises(ValueError):
                load_config(path)


@unittest.skipUnless(importlib.util.find_spec("pinocchio"), "Requires robot Pinocchio environment")
class KinematicsTests(unittest.TestCase):
    def test_home_loads_named_hardware_zero_and_rejects_invalid_limits(self):
        from gesture_kinematics import GestureRobot, ARM_JOINT_KEYS, DEFAULT_CONFIG
        configured = yaml.safe_load(DEFAULT_CONFIG.read_text())
        robot = GestureRobot()
        for side, keys in ARM_JOINT_KEYS.items():
            np.testing.assert_array_equal(robot.q[robot.indices[side]],
                                          [configured[f"{side}_arm"][key] for key in keys])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "zeropos.yaml"
            configured["left_arm"]["left_shoulder_pitch_joint"] += 0.05
            # Reordering keys must not change the joint assignment.
            configured["right_arm"] = dict(reversed(list(configured["right_arm"].items())))
            path.write_text(yaml.safe_dump(configured, sort_keys=False))
            updated = GestureRobot(zero_config=path)
            for side, keys in ARM_JOINT_KEYS.items():
                expected = [configured[f"{side}_arm"][key] for key in keys]
                updated.q[updated.indices[side]] += 0.01
                updated.start_home()
                updated.tick(1.0)
                np.testing.assert_allclose(updated.q[updated.indices[side]], expected, atol=1e-12)
            configured["right_arm"]["right_elbow_joint"] = -1.0
            path.write_text(yaml.safe_dump(configured))
            with self.assertRaisesRegex(ValueError, "within model limits"):
                GestureRobot(zero_config=path)

    def test_bilateral_motion_limits_gripper_and_home(self):
        from gesture_kinematics import GestureRobot
        robot = GestureRobot()
        original = robot.q.copy()
        start = {side: robot.pose(side).translation.copy() for side in ("left", "right")}
        for _ in range(15):
            before = robot.q.copy()
            for side in ("left", "right"):
                self.assertTrue(robot.move(side, np.array([0.001, 0, 0]), np.zeros(3), 1 / 30))
            self.assertLessEqual(np.max(np.abs(robot.q - before)), 0.8 / 30 + 1e-8)
        for side in ("left", "right"):
            self.assertGreater(robot.pose(side).translation[0] - start[side][0], 0.008)
        self.assertTrue(np.all(robot.q >= robot.model.lowerPositionLimit))
        self.assertTrue(np.all(robot.q <= robot.model.upperPositionLimit))
        robot.toggle_gripper("left")
        robot.start_home()
        for _ in range(60):
            robot.tick(1 / 30)
        self.assertFalse(robot.homing)
        for indices in robot.indices.values():
            np.testing.assert_allclose(robot.q[indices], original[indices], atol=1e-8)
        self.assertTrue(robot.closed["left"])
        self.assertFalse(robot.closed["right"])
        self.assertAlmostEqual(robot.q[robot.fingers["left"]], robot.finger_angle("left", 0.04))


class RecordingTests(unittest.TestCase):
    def test_episode_finalized_and_explicitly_simulation(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = SimulationRecorder(Path(directory), load_config())
            self.assertFalse(recorder.active)
            recorder.start()
            recorder.write({"type": "sample", "joint_positions": [1, 2]})
            result = recorder.stop("left_ok")
            self.assertFalse(recorder.active)
            self.assertEqual(list(Path(directory).glob("*.partial")), [])
            lines = [json.loads(line) for line in result.read_text().splitlines()]
            self.assertTrue(lines[0]["simulation"])
            self.assertEqual(lines[1]["joint_positions"], [1, 2])
            self.assertEqual(lines[-1]["reason"], "left_ok")


class PoseFusionTests(unittest.TestCase):
    def test_launcher_uses_existing_calibration_without_overriding_explicit_mode(self):
        spec = importlib.util.spec_from_file_location("gesture_launcher", ROOT / "scripts/run_gesture_rviz.py")
        launcher = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(launcher)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.assertEqual(launcher.perception_options([], root)[0], "rgbd")
            (root / "calibration").mkdir()
            (root / "calibration/hand_geometry.json").write_text("{}")
            self.assertEqual(launcher.perception_options([], root)[0], "compare")
            self.assertEqual(launcher.perception_options(["--mode", "rgbd"], root)[0], "rgbd")
            self.assertEqual(launcher.perception_options(["--mode=rgb"], root)[0], "rgb")

    def test_same_frame_fallback_and_sticky_source_while_open_palm_held(self):
        fusion = PosePacketFusion(hybrid=True)
        fusion.feed(packet(1, left=OPEN_PALM, missing=("Left",)), "rgbd", 1.01)
        self.assertIsNone(fusion.take(1.02))
        fusion.feed(packet(1, left=OPEN_PALM), "rgb", 1.02)
        result = fusion.take(1.02)
        self.assertEqual(result["hands"][0]["pose_source"], "rgb_pnp")
        for stamp, gesture, expected in ((1.03, OPEN_PALM, "rgb_pnp"), (1.06, -1, "rgb_pnp"), (1.09, 0, "rgbd")):
            for source in ("rgbd", "rgb"):
                fusion.feed(packet(stamp, left=gesture), source, stamp)
            result = fusion.take(stamp)
            self.assertEqual(result["hands"][0]["pose_source"], expected)

    def test_frames_never_mix_and_missing_stream_does_not_block_forever(self):
        fusion = PosePacketFusion(hybrid=True)
        fusion.feed(packet(1, left=OPEN_PALM, missing=("Left",)), "rgbd", 1.01)
        fusion.feed(packet(1.03, left=2), "rgb", 1.04)
        result = fusion.take(1.065)
        self.assertEqual(result["gestures"]["Left"], OPEN_PALM)
        self.assertNotIn("Left", [hand["label"] for hand in result["hands"]])
        result = fusion.take(1.1)
        self.assertEqual(result["gestures"]["Left"], 2)
        self.assertEqual(result["hands"][0]["pose_source"], "rgb_pnp")
        fusion.feed(packet(1), "rgbd", 1.11)
        self.assertIsNone(fusion.take(1.12))
        fusion.feed(packet(1.2), "rgb", 1.21)
        self.assertIsNone(fusion.take(1.6))

    def test_conflicting_gestures_are_not_combined(self):
        fusion = PosePacketFusion(hybrid=True)
        fusion.feed(packet(1, left=OPEN_PALM), "rgbd", 1)
        fusion.feed(packet(1, left=2), "rgb", 1)
        result = fusion.take(1)
        self.assertEqual(result["gestures"]["Left"], -1)
        self.assertNotIn("Left", [hand["label"] for hand in result["hands"]])


if __name__ == "__main__":
    unittest.main()
