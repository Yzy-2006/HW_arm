from pathlib import Path
import sys
import subprocess
import math

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
    assert "if (!arm_zero_calibration_)" in source
    assert "transMod = 5" in source
    assert "left arm calibration: zero-current" in source
    assert 'calibration_side != "both"' in source
    assert 'calibration_side != "right"' in source
    assert "both_arm_zero_calibration_" in source
    assert "both-arm calibration: zero-current" in source
    assert "right arm calibration: zero-current" in source
    assert "Cannot hold left arm without feedback" in source
    assert "Cannot hold right arm without feedback" in source


def test_make_calibration_target_and_normal_servo_arguments():
    makefile = (ROOT / "Makefile").read_text()

    assert "calibrate-arm-zero:" in makefile
    assert "arm_zero_calibration_side:=$(SIDE)" in makefile
    assert '@bash "$(ROOT)/scripts/run.sh" servo $(ARGS)' in makefile
    assert "Usage: make calibrate-arm-zero SIDE={left|right|both}" in makefile


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


def test_save_left_zero_pose_accepts_point_two_rad_joint_spread(tmp_path):
    path = tmp_path / "zeropos.yaml"
    path.write_text(YAML_WITH_LEFT_AND_RIGHT)

    pose = save_zero_pose(
        path,
        "left",
        [(0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70)] * 19
        + [(0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90)],
    )

    assert pose["left_elbow_joint"] == pytest.approx(0.40)
    assert yaml.safe_load(path.read_text())["left_arm"] == pose


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
        [(0.1,) * 7] * 19 + [(0.3001,) * 7],
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
    assert "Usage: make save-arm-zero SIDE={left|right|both}" in makefile
    assert document.is_file()
    docs = document.read_text()
    for phrase in (
        "急停",
        "绿灯",
        "make calibrate-arm-zero SIDE=both",
        "make save-arm-zero SIDE=both",
        "make calibrate-arm-zero SIDE=right",
        "make save-arm-zero SIDE=right",
        "保持侧",
        "make build",
        "备份",
    ):
        assert phrase in docs


CARTESIAN_YAML = """\
# Keep Cartesian comments and inactive examples.
# right_arm:
#   position: {x: 99, y: 99, z: 99}
left_arm:
  position:
    x: 0.0  # x comment
    y: 0.17
    z: -0.6
  orientation:
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
right_arm:
  position: {x: 0.1, y: -0.17, z: -0.4}
  orientation: {roll: 0.0, pitch: -1.57, yaw: 0.0}
"""


class FakeKinematics:
    def __init__(self):
        self.calls = []

    def pose(self, side, angles):
        self.calls.append((side, angles))
        return [0.12, 0.17, -0.5, 0.2, -0.3, 0.4]


@pytest.mark.parametrize("target", ["joint", "cartesian", "both", "preset"])
@pytest.mark.parametrize("side", ["left", "right", "both"])
def test_calibration_target_selects_only_requested_files_and_arms(tmp_path, target, side):
    joint = tmp_path / "zeropos.yaml"
    desc = tmp_path / "desc_zeropos.yaml"
    preset = tmp_path / "vr_preset_pose.yaml"
    joint.write_text(YAML_WITH_LEFT_AND_RIGHT)
    desc.write_text(CARTESIAN_YAML)
    preset.write_text(YAML_WITH_LEFT_AND_RIGHT)
    sides = ("left", "right") if side == "both" else (side,)
    samples = {selected: [(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7)] * 20 for selected in sides}
    fk = FakeKinematics()

    joints, cartesian, backups = calibrate_arm_zero.save_calibration(
        joint, desc, samples, target=target, kinematics=fk, preset_path=preset
    )

    assert set(joints) == set(sides)
    expected_files = {
        "both": {joint, desc}, "joint": {joint}, "cartesian": {desc}, "preset": {preset},
    }[target]
    assert set(backups) == expected_files
    assert len({backup.name.split(".")[-2] for backup in backups.values()}) == 1
    for path, original in (
        (joint, YAML_WITH_LEFT_AND_RIGHT), (desc, CARTESIAN_YAML),
        (preset, YAML_WITH_LEFT_AND_RIGHT),
    ):
        if path not in expected_files:
            assert path.read_text() == original
        else:
            assert backups[path].read_text() == original
        for other in set(("left", "right")) - set(sides):
            assert yaml.safe_load(path.read_text())[f"{other}_arm"] == yaml.safe_load(original)[f"{other}_arm"]
    if target in ("joint", "preset"):
        assert not fk.calls
        assert not cartesian
        saved = yaml.safe_load((preset if target == "preset" else joint).read_text())
        for selected in sides:
            assert [saved[f"{selected}_arm"][key] for key in
                    calibrate_arm_zero.ARM_JOINT_KEYS[selected]] == pytest.approx(joints[selected])
    else:
        assert fk.calls == [(selected, joints[selected]) for selected in sides]
        saved_desc = yaml.safe_load(desc.read_text())
        for selected in sides:
            assert saved_desc[f"{selected}_arm"]["position"]["x"] == pytest.approx(0.12)
            assert saved_desc[f"{selected}_arm"]["orientation"]["pitch"] == pytest.approx(-0.3)
        assert "# x comment" in desc.read_text()
        assert "#   position: {x: 99, y: 99, z: 99}" in desc.read_text()


@pytest.mark.parametrize("failure", ["invalid_samples", "invalid_yaml", "fk_error", "replace_error"])
def test_both_files_remain_unchanged_on_failure(tmp_path, monkeypatch, failure):
    joint = tmp_path / "zeropos.yaml"
    desc = tmp_path / "desc_zeropos.yaml"
    original_desc = CARTESIAN_YAML.replace("    yaw: 0.0", "    other: 0.0") if failure == "invalid_yaml" else CARTESIAN_YAML
    joint.write_text(YAML_WITH_LEFT_AND_RIGHT)
    desc.write_text(original_desc)
    fk = FakeKinematics()
    samples = {"left": [(0.1,) * 7] * 20, "right": [(0.2,) * 7] * 20}
    if failure == "invalid_samples":
        samples["right"] = [(float("nan"),) * 7] * 20
    elif failure == "fk_error":
        def fail_fk(side, angles):
            raise ValueError("FK unavailable")
        fk.pose = fail_fk
    elif failure == "replace_error":
        replace = calibrate_arm_zero.os.replace

        def fail_second_file(source, destination):
            if destination == desc:
                raise OSError("Simulated second-file replacement failure")
            return replace(source, destination)

        monkeypatch.setattr(calibrate_arm_zero.os, "replace", fail_second_file)

    with pytest.raises((ValueError, OSError)):
        calibrate_arm_zero.save_calibration(joint, desc, samples, kinematics=fk)

    assert joint.read_text() == YAML_WITH_LEFT_AND_RIGHT
    assert desc.read_text() == original_desc
    if failure != "replace_error":
        assert not list(tmp_path.glob("*.backup"))
    assert not [path for path in tmp_path.iterdir() if path.suffix not in (".yaml", ".backup")]


def test_cartesian_only_does_not_require_joint_file(tmp_path):
    desc = tmp_path / "desc.yaml"
    desc.write_text(CARTESIAN_YAML)
    calibrate_arm_zero.save_calibration(
        tmp_path / "missing.yaml", desc, {"right": [(0.1,) * 7] * 20},
        target="cartesian", kinematics=FakeKinematics(),
    )
    assert not (tmp_path / "missing.yaml").exists()


def test_save_through_symlink_preserves_install_link(tmp_path):
    joint = tmp_path / "source.yaml"
    joint.write_text(YAML_WITH_LEFT_AND_RIGHT)
    installed = tmp_path / "install.yaml"
    installed.symlink_to(joint)
    save_zero_pose(installed, "left", [(0.1,) * 7] * 20)
    assert installed.is_symlink()
    assert yaml.safe_load(joint.read_text())["left_arm"]["left_elbow_joint"] == pytest.approx(0.1)


def test_fk_uses_arm_base_frame_and_correct_joint_names():
    pytest.importorskip("pinocchio")
    from arm_kinematics import ArmKinematics

    fk = ArmKinematics()
    # The URDF world-to-base translation is +1 m; it must not enter saved poses.
    assert fk.pose("left", [0.0] * 7) == pytest.approx([0, 0.17, -0.6, 0, 0, 0])
    assert fk.pose("right", [0.0] * 7) == pytest.approx([0, -0.17, -0.6, 0, 0, 0])
    # With all other joints straight, shoulder pitch rotates the 0.6 m arm.
    # The two shoulder axes have opposite signs in this URDF.
    angle = 0.3
    for side, sign in (("left", 1), ("right", -1)):
        assert fk.pose(side, [angle, 0, 0, 0, 0, 0, 0]) == pytest.approx([
            -sign * 0.6 * math.sin(angle), sign * 0.17, -0.6 * math.cos(angle),
            0, sign * angle, 0,
        ])


@pytest.mark.parametrize("target", ["joint", "cartesian", "both", "preset"])
def test_make_passes_calibration_target(target):
    result = subprocess.run(
        ["make", "-n", "save-arm-zero", "SIDE=right", f"TARGET={target}"],
        cwd=ROOT, capture_output=True, text=True, check=True,
    )
    assert f'--side "right" --target "{target}"' in result.stdout


def test_calibration_cli_defaults_to_both(monkeypatch):
    monkeypatch.setattr(sys, "argv", ["calibrate_arm_zero.py", "--side", "left"])
    assert calibrate_arm_zero.parse_arguments().target == "both"


def test_calibration_cli_selects_independent_preset_file(monkeypatch):
    monkeypatch.setattr(sys, "argv", [
        "calibrate_arm_zero.py", "--side", "both", "--target", "preset",
    ])
    args = calibrate_arm_zero.parse_arguments()
    assert args.target == "preset"
    assert args.preset_config == ROOT / "ros2_ws/src/ctl_msgs_hw/config/vr_preset_pose.yaml"
    assert args.preset_config not in (args.config, args.desc_config)


@pytest.mark.parametrize("failure", ["invalid_samples", "invalid_yaml", "zero_path_alias"])
def test_preset_calibration_failure_preserves_files(tmp_path, failure):
    joint = tmp_path / "zeropos.yaml"
    desc = tmp_path / "desc_zeropos.yaml"
    preset = tmp_path / "vr_preset_pose.yaml"
    joint.write_text(YAML_WITH_LEFT_AND_RIGHT)
    desc.write_text(CARTESIAN_YAML)
    original = YAML_WITH_LEFT_AND_RIGHT
    if failure == "invalid_yaml":
        original = original.replace("right_elbow_joint:", "unknown_joint:")
    preset.write_text(original)
    samples = {"left": [(0.1,) * 7] * 20, "right": [(0.2,) * 7] * 20}
    if failure == "invalid_samples":
        samples["right"] = [(float("nan"),) * 7] * 20
    path = preset
    if failure == "zero_path_alias":
        path = tmp_path / "preset_link.yaml"
        path.symlink_to(joint)
    with pytest.raises(ValueError):
        calibrate_arm_zero.save_calibration(
            joint, desc, samples, target="preset", preset_path=path,
        )
    assert joint.read_text() == YAML_WITH_LEFT_AND_RIGHT
    assert desc.read_text() == CARTESIAN_YAML
    assert preset.read_text() == original
    assert not list(tmp_path.glob("*.backup"))


def test_preset_only_does_not_require_zero_files_or_kinematics(tmp_path):
    preset = tmp_path / "vr_preset_pose.yaml"
    preset.write_text(YAML_WITH_LEFT_AND_RIGHT)
    calibrate_arm_zero.save_calibration(
        tmp_path / "missing_joint.yaml", tmp_path / "missing_desc.yaml",
        {"left": [(0.1,) * 7] * 20}, target="preset", preset_path=preset,
    )
    assert yaml.safe_load(preset.read_text())["left_arm"]["left_elbow_joint"] == pytest.approx(0.1)
    assert not (tmp_path / "missing_joint.yaml").exists()
    assert not (tmp_path / "missing_desc.yaml").exists()
