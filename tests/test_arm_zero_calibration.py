from pathlib import Path
import sys

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import calibrate_arm_zero
from calibrate_arm_zero import FeedbackWindow, save_zero_pose


YAML_WITH_LEFT_AND_RIGHT = """\
left_arm:
  left_shoulder_pitch_joint: 0.0
  left_shoulder_roll_joint: 0.0
  left_shoulder_yaw_joint: 0.0
  left_elbow_joint: 0.0
  left_wrist_roll_joint: 0.0
  left_wrist_pitch_joint: 0.0
  left_wrist_yaw_joint: 0.0
right_arm:
  right_shoulder_pitch_joint: -0.4
  right_shoulder_roll_joint: 0.0
  right_shoulder_yaw_joint: 0.0
  right_elbow_joint: 1.4
  right_wrist_roll_joint: 0.0
  right_wrist_pitch_joint: -0.57
  right_wrist_yaw_joint: 0.0
"""


def test_calibration_branch_isolated_from_normal_homing():
    source = (
        ROOT / "ros2_ws/src/servo_control_hw/src/ros_controller/rosLoop.cpp"
    ).read_text()

    assert "arm_zero_calibration_side" in source
    assert "if (!left_arm_zero_calibration_)" in source
    assert "transMod = 5" in source
    assert "left arm calibration: zero-current" in source
    assert 'calibration_side != "both"' in source
    assert "both_arm_zero_calibration_" in source
    assert "both-arm calibration: zero-current" in source


def test_make_calibration_target_and_normal_servo_arguments():
    makefile = (ROOT / "Makefile").read_text()

    assert "calibrate-arm-zero:" in makefile
    assert "arm_zero_calibration_side:=$(SIDE)" in makefile
    assert '@bash "$(ROOT)/scripts/run.sh" servo $(ARGS)' in makefile
    assert "Usage: make calibrate-arm-zero SIDE={left|both}" in makefile


def test_save_left_zero_pose_uses_medians_and_preserves_right_arm(tmp_path):
    path = tmp_path / "zeropos.yaml"
    path.write_text(YAML_WITH_LEFT_AND_RIGHT)

    pose = save_zero_pose(
        path,
        "left",
        [(0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70)] * 20,
    )

    saved = yaml.safe_load(path.read_text())
    original = yaml.safe_load(YAML_WITH_LEFT_AND_RIGHT)
    assert pose["left_elbow_joint"] == pytest.approx(0.40)
    assert saved["left_arm"] == pose
    assert saved["right_arm"] == original["right_arm"]
    assert len(list(tmp_path.glob("zeropos.yaml.*.backup"))) == 1


def test_save_left_zero_pose_preserves_unrelated_yaml_text(tmp_path):
    original = "# Keep this file-level note.\n" + YAML_WITH_LEFT_AND_RIGHT.replace(
        "  left_elbow_joint: 0.0",
        "  left_elbow_joint: 0.0  # Keep this elbow note.",
    )
    path = tmp_path / "zeropos.yaml"
    path.write_text(original)

    save_zero_pose(
        path,
        "left",
        [(0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70)] * 20,
    )

    saved_text = path.read_text()
    assert "# Keep this file-level note." in saved_text
    assert "# Keep this elbow note." in saved_text
    assert "  right_elbow_joint: 1.4" in saved_text


def test_save_both_zero_poses_updates_both_arms_with_one_backup(tmp_path):
    path = tmp_path / "zeropos.yaml"
    path.write_text(YAML_WITH_LEFT_AND_RIGHT)

    saved = calibrate_arm_zero.save_zero_poses(
        path,
        {
            "left": [(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7)] * 20,
            "right": [(-0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7)] * 20,
        },
    )

    config = yaml.safe_load(path.read_text())
    assert saved["left"]["left_elbow_joint"] == pytest.approx(0.4)
    assert saved["right"]["right_elbow_joint"] == pytest.approx(-0.4)
    assert config["left_arm"] == saved["left"]
    assert config["right_arm"] == saved["right"]
    assert len(list(tmp_path.glob("zeropos.yaml.*.backup"))) == 1


def test_invalid_side_in_both_arm_capture_does_not_write_anything(tmp_path):
    path = tmp_path / "zeropos.yaml"
    path.write_text(YAML_WITH_LEFT_AND_RIGHT)
    before = path.read_text()

    with pytest.raises(ValueError):
        calibrate_arm_zero.save_zero_poses(
            path,
            {
                "left": [(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7)] * 20,
                "right": [(0.0,) * 7] * 20,
            },
        )

    assert path.read_text() == before
    assert not list(tmp_path.glob("zeropos.yaml.*.backup"))


@pytest.mark.parametrize(
    "samples",
    [
        [(0.0,) * 7] * 20,
        [(0.1,) * 6] * 20,
        [(0.1,) * 7] * 19,
        [(0.1,) * 6 + (float("nan"),)] * 20,
        [(0.1,) * 7] * 19 + [(0.2,) * 7],
    ],
)
def test_bad_capture_leaves_config_unchanged(tmp_path, samples):
    path = tmp_path / "zeropos.yaml"
    path.write_text(YAML_WITH_LEFT_AND_RIGHT)
    before = path.read_text()

    with pytest.raises(ValueError):
        save_zero_pose(path, "left", samples)

    assert path.read_text() == before


def test_feedback_gap_restarts_the_capture_window():
    window = FeedbackWindow(max_gap_seconds=0.2)
    pose = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7)

    window.add(0.0, pose)
    window.add(0.1, pose)
    window.add(0.4, pose)

    assert window.samples == [pose]
    assert not window.is_ready(duration_seconds=1.0)


def test_save_target_and_docs_cover_safe_calibration_sequence():
    makefile = (ROOT / "Makefile").read_text()
    document = ROOT / "docs/ARM_ZERO_CALIBRATION.md"

    assert "save-arm-zero:" in makefile
    assert "Usage: make save-arm-zero SIDE={left|both}" in makefile
    assert document.is_file()
    docs = document.read_text()
    for phrase in (
        "急停",
        "绿灯",
        "make calibrate-arm-zero SIDE=both",
        "make save-arm-zero SIDE=both",
        "make build",
        "备份",
    ):
        assert phrase in docs
