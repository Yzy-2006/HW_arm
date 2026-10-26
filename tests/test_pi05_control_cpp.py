import shlex
import subprocess
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
