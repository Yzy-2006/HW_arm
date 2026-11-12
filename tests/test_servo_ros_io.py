import subprocess
import shlex
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SERVO = ROOT / "ros2_ws/src/servo_control_hw"


def test_native_ros_io_isolation_and_bounded_snapshots(tmp_path):
    executable = tmp_path / "servo_ros_io_test"
    subprocess.run([
        "g++", "-std=c++17", "-O2", "-pthread", "-I", str(SERVO / "include"),
        str(ROOT / "tests/servo_ros_io_test.cpp"), "-o", str(executable),
    ], check=True, capture_output=True, text=True)
    subprocess.run([str(executable)], check=True, capture_output=True, text=True, timeout=15)


def test_control_loop_cannot_enter_the_ros_executor():
    source = (SERVO / "src/ros_controller/rosLoop.cpp").read_text()
    run = source.split("void ServoNode::run(){", 1)[1].split("void ServoNode::desMsg_pack_func", 1)[0]
    drain = source.split("void ServoNode::ctlmsg_get(){", 1)[1].split("void ServoNode::EtherCATmsg_Get", 1)[0]
    assert "executor_->" not in run
    assert "executor_->" not in drain
    assert "->publish(" not in run
    assert "->publish(" not in drain
    assert run.index("shutdown_damping();") < run.index("stop_ros_io();")


def test_real_servo_drain_preserves_stop_reset_and_shutdown(tmp_path, monkeypatch):
    build = ROOT / "ros2_ws/build/servo_control_hw"
    flags = build / "CMakeFiles/servo_node.dir/flags.make"
    link = build / "CMakeFiles/servo_node.dir/link.txt"
    if not flags.exists() or not link.exists():
        pytest.skip("Build servo_control_hw first for the ROS node integration test")
    includes = next(line.partition(" = ")[2] for line in flags.read_text().splitlines()
                    if line.startswith("CXX_INCLUDES = "))
    arguments = shlex.split(link.read_text())
    output = arguments.index("-o")
    del arguments[output:output + 2]
    arguments = [arg for arg in arguments if not arg.endswith("/src/main.cpp.o")]
    executable = tmp_path / "servo_ros_node_io_test"
    compilation = subprocess.run([
        arguments[0], "-std=c++17", *shlex.split(includes),
        str(ROOT / "tests/servo_ros_node_io_test.cpp"), *arguments[1:], "-o", str(executable),
    ], cwd=build, capture_output=True, text=True)
    assert compilation.returncode == 0, compilation.stderr
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(ROOT / "ros2_ws/install/ctl_msgs_hw") + ":/opt/ros/jazzy")
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    monkeypatch.setenv("ROS_DOMAIN_ID", "232")
    result = subprocess.run([str(executable)], capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr
