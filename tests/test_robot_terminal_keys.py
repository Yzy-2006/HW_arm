import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_terminal_keys_are_immediate_nonblocking_restore_settings_and_ignore_escape_sequences(tmp_path):
    executable = tmp_path / "terminal_keys_test"
    subprocess.run([
        "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
        "-I", str(ROOT / "ros2_ws/src/robot_control_hw/include"),
        str(ROOT / "tests/terminal_keys_test.cpp"), "-lutil", "-o", str(executable),
    ], check=True, capture_output=True, text=True)
    subprocess.run([str(executable)], check=True, capture_output=True, text=True, timeout=5)
