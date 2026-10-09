from pathlib import Path
import shlex
import subprocess

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "ros2_ws/src/ctl_msgs_hw/config"
ROBOT = ROOT / "ros2_ws/src/robot_control_hw"


@pytest.fixture(scope="module")
def preset_executable(tmp_path_factory):
    flags = ROOT / "ros2_ws/build/robot_control_hw/CMakeFiles/robot_control_node.dir/flags.make"
    if not flags.exists():
        pytest.skip("Build robot_control_hw first to provide ROS compiler include paths")
    includes = next(line.partition(" = ")[2] for line in flags.read_text().splitlines()
                    if line.startswith("CXX_INCLUDES = "))
    executable = tmp_path_factory.mktemp("vr-preset-build") / "vr_preset_pose_test"
    subprocess.run([
        "g++", "-std=c++17", "-ffunction-sections", "-fdata-sections",
        *shlex.split(includes), str(ROOT / "tests/vr_preset_pose_test.cpp"),
        str(ROBOT / "src/controller/xrTelepercationCom.cpp"),
        str(ROBOT / "src/controller/upper_Transmit.cpp"),
        "-Wl,--gc-sections", "-lyaml-cpp", "-o", str(executable),
    ], check=True, capture_output=True, text=True)
    return executable


@pytest.mark.parametrize("case", ["valid", "valid_no_feedback", "missing_file", "missing_joint", "nan", "malformed"])
def test_preset_selection_validation_and_hold(preset_executable, tmp_path, case):
    config = tmp_path / "config"
    config.mkdir()
    (config / "controllerConfig.yaml").write_text((CONFIG / "controllerConfig.yaml").read_text())
    pose = yaml.safe_load((CONFIG / "vr_preset_pose.yaml").read_text())
    for side, sign in (("left", 1), ("right", -1)):
        for i, key in enumerate(pose[f"{side}_arm"]):
            pose[f"{side}_arm"][key] = sign * (0.1 + 0.01 * i)
    if case == "missing_joint":
        del pose["right_arm"]["right_wrist_yaw_joint"]
    if case == "nan":
        pose["right_arm"]["right_wrist_yaw_joint"] = float("nan")
    if case != "missing_file":
        (config / "vr_preset_pose.yaml").write_text(
            "left_arm: [" if case == "malformed" else yaml.safe_dump(pose)
        )
    subprocess.run([str(preset_executable), str(tmp_path), case],
                   check=True, capture_output=True, text=True)


@pytest.mark.parametrize("enabled", [True, False])
def test_tool_axis_lock_preserves_swing_and_translation(preset_executable, tmp_path, enabled):
    config = tmp_path / "config"
    config.mkdir()
    control = yaml.safe_load((CONFIG / "controllerConfig.yaml").read_text())
    control["vrOrientationControl"] = {"lock_tool_twist": enabled}
    control["vrTeleoperationLimit"]["position_enabled"] = False
    control["vrTeleoperationLimit"]["orientation_enabled"] = False
    (config / "controllerConfig.yaml").write_text(yaml.safe_dump(control))
    (config / "vr_preset_pose.yaml").write_text((CONFIG / "vr_preset_pose.yaml").read_text())
    subprocess.run([
        str(preset_executable), str(tmp_path), "lock_enabled" if enabled else "lock_disabled",
    ], check=True, capture_output=True, text=True)
