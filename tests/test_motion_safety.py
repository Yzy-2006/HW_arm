import subprocess
import shlex
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "deployment"))
from motion_safety import SafetyState, CommandSafety, MotionFault
from minimum_snap import MinimumSnapTrajectory
from pi05_bridge import limit_action


def test_native_filter_and_fault_guard(tmp_path):
    stub = tmp_path / "robot_config/config.h"
    stub.parent.mkdir()
    stub.write_text("#pragma once\nextern double TEMPTIME;\n")
    executable = tmp_path / "control_safety_test"
    subprocess.run([
        "g++", "-std=c++17", "-O2", "-ffunction-sections", "-fdata-sections",
        "-I", str(tmp_path), "-I", str(ROOT / "ros2_ws/src/dual_arm_control_hw/include"),
        "-I", str(ROOT / "ros2_ws/src/servo_control_hw/include"),
        str(ROOT / "tests/control_safety_test.cpp"),
        str(ROOT / "ros2_ws/src/dual_arm_control_hw/src/controller/JntController.cpp"),
        "-Wl,--gc-sections", "-o", str(executable),
    ], check=True, capture_output=True, text=True)
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)


def test_safety_heartbeat_loss_latches_and_ready_cannot_reset():
    clock, stops = [10.], []
    state = SafetyState(stops.append, now=lambda: clock[0])
    assert not state.ready()
    state.receive("ready")
    assert state.ready()
    clock[0] += .21
    assert not state.ready()
    state.receive("ready")
    assert not state.ready()
    assert len(stops) == 1


def test_servo_filter_and_nonzero_startup(tmp_path):
    servo = ROOT / "ros2_ws/src/servo_control_hw"
    executable = tmp_path / "servo_filter_test"
    subprocess.run([
        "g++", "-std=c++17", "-O2", "-ffunction-sections", "-fdata-sections",
        "-I", str(servo / "include"), str(ROOT / "tests/servo_filter_test.cpp"),
        str(servo / "src/JntController/ForceComp.cpp"),
        str(servo / "src/JntController/runinController.cpp"),
        "-Wl,--gc-sections", "-lyaml-cpp", "-o", str(executable),
    ], check=True, capture_output=True, text=True)
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)


def test_missing_safety_server_rejects_startup():
    clock, stops = [10.], []
    state = SafetyState(stops.append, now=lambda: clock[0])
    clock[0] += 5.1
    assert not state.ready()
    assert len(stops) == 1


def test_incident_out_of_range_feedback_is_rejected():
    current = np.zeros(16)
    current[0] = -3.94693660736084
    target = np.zeros(16)
    target[0] = .2960975468158722
    with pytest.raises(MotionFault, match="feedback outside"):
        limit_action(target, current, np.full(14, -3.1416), np.full(14, 3.1416), np.full(14, .09))


def test_command_speed_acceleration_and_tracking_are_separate():
    clock = [10.]
    guard = CommandSafety({}, np.full(14, -3), np.full(14, 3), now=lambda: clock[0])
    action = np.zeros(16)
    guard.check(action, action, .02)
    clock[0] += .02
    target = action.copy()
    target[0] = .09  # Allowed by the old feedback clip, unsafe in 20ms.
    with pytest.raises(MotionFault, match="velocity or acceleration"):
        guard.check(target, action, .02)
    guard.reset()
    guard.check(action, action, .02)
    clock[0] += .02
    target[0] = .004  # Velocity is .2, acceleration is 10.
    with pytest.raises(MotionFault, match="acceleration"):
        guard.check(target, action, .02)


def test_minimum_snap_checks_extrema_between_waypoints():
    trajectory = MinimumSnapTrajectory([0, 1, 2, 3], [[0], [1], [1], [0]])
    with pytest.raises(MotionFault, match="derivative 0"):
        trajectory.validate_limits([-1], [1], 100, 100)
    trajectory = MinimumSnapTrajectory([0, 1], [[0], [1]])
    with pytest.raises(MotionFault, match="derivative 1"):
        trajectory.validate_limits([-1], [2], 1, 100)
    trajectory.validate_limits([-1], [2], 3, 10)


def test_all_compensation_disabled_in_source_and_install():
    for path in [ROOT / "ros2_ws/src/ctl_msgs_hw/config/controllerConfig.yaml",
                 ROOT / "ros2_ws/install/ctl_msgs_hw/share/ctl_msgs_hw/config/controllerConfig.yaml"]:
        config = yaml.safe_load(path.read_text())
        for key in ("ifUseGravComp", "ifUseInertiaComp", "ifUseCoriolisComp", "ifUseFricComp"):
            assert config["gravComp"][key] == 0


def test_servo_transport_fault_inhibits_entire_packet_and_rejects_partial_feedback(tmp_path):
    servo = ROOT / "ros2_ws/src/servo_control_hw"
    flags = ROOT / "ros2_ws/build/servo_control_hw/CMakeFiles/servo_node.dir/flags.make"
    if not flags.exists():
        pytest.skip("Build servo_control_hw first for SOEM include paths")
    includes = next(line.partition(" = ")[2] for line in flags.read_text().splitlines()
                    if line.startswith("CXX_INCLUDES = "))
    motor = tmp_path / "motor.o"
    subprocess.run(["gcc", "-std=c11", "-ffunction-sections", "-fdata-sections", "-I", str(servo / "include"),
                    "-c", str(servo / "src/servo_control/motor_control.c"), "-o", str(motor)],
                   check=True, capture_output=True, text=True)
    executable = tmp_path / "servo_transport_test"
    math_ops = tmp_path / "math_ops.o"
    subprocess.run(["gcc", "-std=c11", "-I", str(servo / "include"),
                    "-c", str(servo / "src/servo_control/math_ops.c"), "-o", str(math_ops)],
                   check=True, capture_output=True, text=True)
    compilation = subprocess.run([
        "g++", "-std=c++17", "-ffunction-sections", "-fdata-sections", *shlex.split(includes),
        str(ROOT / "tests/servo_transport_safety_test.cpp"), str(servo / "src/servo_control/transmit.cpp"),
        str(motor), str(math_ops), str(ROOT / "ros2_ws/build/servo_control_hw/libsoem.a"),
        "-Wl,--gc-sections", "-lpthread", "-o", str(executable),
    ], capture_output=True, text=True)
    assert compilation.returncode == 0, compilation.stderr
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)
