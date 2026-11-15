import shlex
import subprocess
import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
ROBOT = ROOT / "ros2_ws/src/robot_control_hw"


def test_joint_stream_hold_timeout_resume_and_absolute_adapter(tmp_path):
    flags = ROOT / "ros2_ws/build/robot_control_hw/CMakeFiles/robot_control_node.dir/flags.make"
    if not flags.exists():
        pytest.skip("Build robot_control_hw first to provide ROS compiler include paths")
    includes = next(line.partition(" = ")[2] for line in flags.read_text().splitlines()
                    if line.startswith("CXX_INCLUDES = "))
    executable = tmp_path / "pi05_joint_stream_test"
    subprocess.run([
        "g++", "-std=c++17", "-ffunction-sections", "-fdata-sections",
        *shlex.split(includes), str(ROOT / "tests/pi05_joint_stream_test.cpp"),
        str(ROBOT / "src/controller/upper_Transmit.cpp"),
        str(ROBOT / "src/messageType/upper_msgType.cpp"),
        "-Wl,--gc-sections", "-o", str(executable),
    ], check=True, capture_output=True, text=True)
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)


def test_live_fk_topic_reaches_vr_handoff_and_rejects_missing_stale_nan(tmp_path):
    executable = ROOT / "ros2_ws/build/robot_control_hw/robot_control_node"
    if not executable.exists():
        pytest.skip("Build robot_control_hw first for the isolated ROS integration test")
    env = dict(os.environ, ROS_DOMAIN_ID="219", ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
               ROS_LOG_DIR=str(tmp_path / "ros_logs"))
    log = tmp_path / "robot.log"
    result = subprocess.run([sys.executable, str(ROOT / "tests/vr_fk_handoff_runtime.py"),
                             str(executable), str(log)], env=env, capture_output=True, text=True, timeout=45)
    assert result.returncode == 0, result.stderr + (log.read_text() if log.exists() else "")


def test_policy_speed_metadata_survives_hold_and_timeout_until_vr_takeover(tmp_path):
    build = ROOT / "ros2_ws/build/robot_control_hw"
    flags = build / "CMakeFiles/robot_control_node.dir/flags.make"
    link = build / "CMakeFiles/robot_control_node.dir/link.txt"
    if not flags.exists() or not link.exists():
        pytest.skip("Build robot_control_hw first to provide ROS compiler include paths")
    includes = next(line.partition(" = ")[2] for line in flags.read_text().splitlines()
                    if line.startswith("CXX_INCLUDES = "))
    executable = tmp_path / "policy_speed_metadata_test"
    arguments = shlex.split(link.read_text())
    libraries = [arg for arg in arguments[arguments.index("-o") + 2:] if arg != "-rdynamic"]
    result = subprocess.run([
        "g++", "-std=c++17", "-ffunction-sections", "-fdata-sections",
        *shlex.split(includes), str(ROOT / "tests/policy_speed_metadata_test.cpp"),
        str(ROBOT / "src/controller/ctlTransmit.cpp"), "-Wl,--gc-sections",
        *libraries, "-o", str(executable),
    ], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    env = dict(os.environ, ROS_DOMAIN_ID="218", ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
               ROS_LOG_DIR=str(tmp_path / "ros_logs"))
    result = subprocess.run([str(executable)], env=env, capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr
