import subprocess
import shlex
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "deployment"))
from motion_safety import SafetyState, CommandSafety, MotionFault, configured_position_limits
from minimum_snap import MinimumSnapTrajectory
from pi05_bridge import limit_action


def test_homing_preflight_and_feedback_completion(tmp_path):
    servo = ROOT / "ros2_ws/src/servo_control_hw"
    executable = tmp_path / "homing_safety_test"
    compilation = subprocess.run([
        "g++", "-std=c++17", "-O2", "-I", str(servo / "include"),
        str(ROOT / "tests/homing_safety_test.cpp"),
        str(servo / "src/JntController/retZeroControl.cpp"),
        "-lyaml-cpp", "-o", str(executable),
    ], capture_output=True, text=True)
    assert compilation.returncode == 0, compilation.stderr
    result = subprocess.run([str(executable)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


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


def test_controller_handoff_after_homing_and_nonzero_home_pose(tmp_path):
    executable = tmp_path / "controller_handoff_test"
    subprocess.run([
        "g++", "-std=c++17", "-O2",
        "-I", str(ROOT / "ros2_ws/src/servo_control_hw/include"),
        "-I", str(ROOT / "ros2_ws/src/dual_arm_control_hw/include"),
        "-I", str(ROOT / "ros2_ws/src/robot_control_hw/include"),
        str(ROOT / "tests/controller_handoff_test.cpp"),
        "-lyaml-cpp", "-o", str(executable),
    ], check=True, capture_output=True, text=True)
    result = subprocess.run([str(executable)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_handoff_ros_serialization_and_shutdown_sleep(tmp_path, monkeypatch):
    flags = ROOT / "ros2_ws/build/servo_control_hw/CMakeFiles/servo_node.dir/flags.make"
    if not flags.exists():
        pytest.skip("Build servo_control_hw first for ROS compiler include paths")
    includes = next(line.partition(" = ")[2] for line in flags.read_text().splitlines()
                    if line.startswith("CXX_INCLUDES = "))
    executable = tmp_path / "controller_ros_runtime_test"
    compilation = subprocess.run([
        "g++", "-std=c++17", *shlex.split(includes),
        str(ROOT / "tests/controller_ros_runtime_test.cpp"),
        "-L/opt/ros/jazzy/lib", "-Wl,-rpath,/opt/ros/jazzy/lib",
        "-L" + str(ROOT / "ros2_ws/install/ctl_msgs_hw/lib"),
        "-Wl,-rpath," + str(ROOT / "ros2_ws/install/ctl_msgs_hw/lib"),
        "-lrclcpp", "-lrcutils", "-lctl_msgs_hw__rosidl_typesupport_cpp",
        "-o", str(executable),
    ], capture_output=True, text=True)
    assert compilation.returncode == 0, compilation.stderr
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    result = subprocess.run([str(executable)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_missing_safety_server_rejects_startup():
    clock, stops = [10.], []
    state = SafetyState(stops.append, now=lambda: clock[0])
    clock[0] += 5.1
    assert not state.ready()
    assert len(stops) == 1


@pytest.mark.parametrize("state,expected", [(None, "never received"), ("waiting", "state is not ready")])
def test_safety_startup_diagnostics(state, expected):
    clock, stops = [10.], []
    guard = SafetyState(stops.append, now=lambda: clock[0])
    assert not guard.ready() and not guard.fault
    clock[0] += 5.1
    if state is not None:
        guard.receive(state)
    assert not guard.ready()
    assert expected in guard.fault and "heartbeat_age_s=" in guard.fault
    assert len(stops) == 1


def test_concurrent_fault_reception_latches_once_and_blocks_publication():
    from concurrent.futures import ThreadPoolExecutor
    from threading import Barrier
    stops, commands = [], []
    guard = SafetyState(stops.append, defer_receive_fault=True)
    guard.receive("ready")
    assert guard.ready()
    barrier = Barrier(8)

    def receive():
        barrier.wait(timeout=2)
        guard.receive("fault:test")

    with ThreadPoolExecutor(max_workers=8) as pool:
        list(pool.map(lambda _: receive(), range(8)))
    assert guard.fault == "fault:test" and stops == []
    assert not guard.publish_if_ready(lambda: commands.append(1))
    guard.receive("ready")
    assert not guard.ready() and not commands
    assert stops == ["fault:test"]


def test_deployment_arms_on_publication_after_stable_heartbeat_not_first_ready():
    clock, stops, commands = [10.], [], []
    guard = SafetyState(stops.append, now=lambda: clock[0], arm_on_publish=True, startup_settle_s=.2)
    guard.receive("ready")
    assert not guard.ready()
    clock[0] += .44
    assert not guard.ready() and not guard.fault
    assert not guard.publish_if_ready(lambda: commands.append(1))
    guard.receive("ready")
    # Elapsed time alone cannot turn one retained ready message into a stable stream.
    clock[0] += .19
    assert not guard.ready()
    clock[0] += .02
    guard.receive("ready")
    assert guard.ready_samples == 1
    for _ in range(5):
        clock[0] += .05
        guard.receive("ready")
    assert guard.ready() and not guard.was_ready
    assert guard.publish_if_ready(lambda: commands.append(1))
    assert guard.was_ready and commands == [1]
    clock[0] += .201
    assert not guard.ready() and "heartbeat expired" in guard.fault
    guard.receive("ready")
    assert not guard.ready() and len(stops) == 1


@pytest.mark.parametrize("last_state", ["ready", "waiting", "fault:motor_error"])
def test_deployment_startup_never_suppresses_faults_or_waits_forever(last_state):
    clock, stops = [10.], []
    guard = SafetyState(stops.append, now=lambda: clock[0], arm_on_publish=True, startup_settle_s=.2)
    guard.receive(last_state)
    if last_state.startswith("fault:"):
        assert stops == [last_state]
    clock[0] += 5.1
    assert not guard.ready() and guard.fault
    assert len(stops) == 1


def test_disabled_heartbeat_accepts_missing_stale_waiting_but_latches_explicit_fault():
    clock, stops = [10.], []
    guard = SafetyState(stops.append, now=lambda: clock[0], monitor_heartbeat=False,
                        arm_on_publish=True, startup_settle_s=.2)
    assert guard.ready()
    guard.receive("waiting")
    clock[0] += 100
    assert guard.ready() and not stops
    assert guard.publish_if_ready(lambda: None)
    clock[0] += 100
    assert guard.ready()
    guard.receive("fault:feedback_speed_joint_0")
    assert not guard.ready() and len(stops) == 1
    guard.receive("ready")
    assert not guard.ready()


def test_speed_only_disables_non_speed_checks_but_preserves_velocity_and_finite_guards():
    clock = [10.]
    config = {"motion_safety": {"protection_mode": "speed_only", "max_velocity_rad_s": 5.,
        "max_acceleration_rad_s2": .01, "check_position_limits": True,
        "speed_limit_enabled_per_arm": [True, True, True, True, False, False, False]}}
    guard = CommandSafety(config, [-1.] * 14, [1.] * 14, now=lambda: clock[0])
    feedback = np.full(16, 10.)
    action = feedback.copy()
    guard.check(action, feedback, .01)
    for _ in range(4):
        clock[0] += 1  # No command-age, position, acceleration or tracking-error trip.
        action[0] += 1
        guard.check(action, feedback, .01)
    clock[0] += .01
    action[0] += .1
    with pytest.raises(MotionFault, match="velocity"):
        guard.check(action, feedback, .01)
    guard.reset()
    feedback[0] = np.nan
    with pytest.raises(MotionFault, match="Non-finite"):
        guard.check(action, feedback, .01)


def test_safety_subscription_group_is_independent_but_cleanup_timer_is_serial(monkeypatch):
    from types import SimpleNamespace
    from motion_safety import attach_safety
    subscriptions, timers = [], []
    node = SimpleNamespace(
        create_publisher=lambda *args: SimpleNamespace(publish=lambda msg: None),
        create_subscription=lambda *args, **kwargs: subscriptions.append((args, kwargs)),
        create_timer=lambda *args, **kwargs: timers.append((args, kwargs)),
    )
    monkeypatch.setitem(sys.modules, "std_msgs.msg", SimpleNamespace(String=lambda **kwargs: kwargs))
    monkeypatch.setitem(sys.modules, "rclpy.qos", SimpleNamespace(
        QoSProfile=lambda **kwargs: kwargs, DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL=1)))
    group, faults = object(), []
    state = attach_safety(node, faults.append, receive_callback_group=group)
    assert subscriptions[0][1]["callback_group"] is group
    assert "callback_group" not in timers[0][1]
    subscriptions[0][0][2](SimpleNamespace(data="fault:test"))
    assert state.fault == "fault:test" and faults == []
    timers[0][0][1]()
    assert faults == ["fault:test"]


@pytest.mark.parametrize('repeat', [False, True])
def test_stop_retries_until_servo_fault_acknowledgement(monkeypatch, repeat):
    from types import SimpleNamespace
    from motion_safety import attach_safety
    published, timers, faults = [], [], []
    node = SimpleNamespace(
        create_publisher=lambda *a: SimpleNamespace(publish=published.append),
        create_subscription=lambda *a, **kw: None,
        create_timer=lambda period, callback: timers.append(callback))
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', SimpleNamespace(String=SimpleNamespace))
    monkeypatch.setitem(sys.modules, 'rclpy.qos', SimpleNamespace(
        QoSProfile=lambda **kw: kw, DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL=1)))
    state = attach_safety(node, faults.append, monitor_heartbeat=False, repeat_stop_until_ack=repeat)
    state.trip('Replay tracking error')
    assert len(published) == 1 and published[0].data == 'Replay tracking error'
    timers[0]()
    assert len(published) == (2 if repeat else 1)
    state.receive('fault:upstream: Replay tracking error')
    timers[0]()
    assert len(published) == (2 if repeat else 1)
    assert faults == ['Replay tracking error']


def test_ros_executor_receives_heartbeat_while_control_group_is_blocked(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    script = '''
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from motion_safety import SafetyState
rclpy.init()
node = Node("heartbeat_scheduling_test")
executor = MultiThreadedExecutor(num_threads=3)
group = MutuallyExclusiveCallbackGroup()
state = SafetyState(lambda reason: None, defer_receive_fault=True)
entered, done = threading.Event(), threading.Event()
received_during_control = []
results = []
def heartbeat():
    state.receive("ready")
    if entered.is_set() and not done.is_set():
        received_during_control.append(time.monotonic())
def control():
    control_timer.cancel()
    entered.set()
    time.sleep(.35)
    results.append(state.ready())
    done.set()
node.create_timer(.01, heartbeat, callback_group=group)
control_timer = node.create_timer(.05, control)
executor.add_node(node)
try:
    deadline = time.monotonic() + 5
    while not done.is_set() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=.05)
    assert done.is_set() and results == [True], (results, state.diagnostics())
    assert len(received_during_control) >= 3, received_during_control
finally:
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
'''
    result = subprocess.run([sys.executable, "-c", script], cwd=ROOT / "deployment",
                            capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr


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
    with pytest.raises(MotionFault, match="command velocity"):
        guard.check(target, action, .02)
    guard.reset()
    guard.check(action, action, .02)
    clock[0] += .02
    target[0] = .004  # Velocity is .2, acceleration is 10.
    with pytest.raises(MotionFault, match="acceleration"):
        guard.check(target, action, .02)


@pytest.mark.parametrize("joint,name", [(0, "left_joint_1"), (10, "right_joint_4")])
def test_command_speed_fault_identifies_joint_delta_interval_and_limit(joint, name):
    clock = [10.]
    guard = CommandSafety({"motion_safety": {"protection_mode": "speed_only", "max_velocity_rad_s": 5.}},
                          [-3.] * 14, [3.] * 14, now=lambda: clock[0])
    action = np.zeros(16)
    guard.check(action, action, .01)
    clock[0] += .01
    target = action.copy()
    target[np.r_[0:7, 8:15][joint]] = .06
    with pytest.raises(MotionFault) as error:
        guard.check(target, action, .01)
    for field in ("command velocity", name, "previous_q=0", "target_q=0.06",
                  "delta_q=0.06rad", "dt_s=0.01", "velocity=6rad/s", "max_abs_velocity=5"):
        assert field in str(error.value)


def test_minimum_snap_checks_extrema_between_waypoints():
    trajectory = MinimumSnapTrajectory([0, 1, 2, 3], [[0], [1], [1], [0]])
    with pytest.raises(MotionFault, match="derivative 0"):
        trajectory.validate_limits([-1], [1], 100, 100)
    trajectory = MinimumSnapTrajectory([0, 1], [[0], [1]])
    with pytest.raises(MotionFault, match="derivative 1"):
        trajectory.validate_limits([-1], [2], 1, 100)
    trajectory.validate_limits([-1], [2], 3, 10)


def test_acceleration_checks_can_be_disabled_without_disabling_speed_or_finite_checks():
    clock = [10.]
    guard = CommandSafety({"motion_safety": {"max_acceleration_rad_s2": None}},
                          np.full(14, -3), np.full(14, 3), now=lambda: clock[0])
    action = np.zeros(16)
    guard.check(action, action, .02)
    clock[0] += .02
    target = action.copy()
    target[0] = .004  # v=.2, a=10: exceeds the default acceleration bound.
    guard.check(target, action, .02, np.full(14, .2), np.full(14, 1000.))
    clock[0] += .02
    with pytest.raises(MotionFault, match="derivative"):
        guard.check(target, action, .02, np.full(14, 2.), np.zeros(14))
    with pytest.raises(MotionFault, match="derivative"):
        guard.check(target, action, .02, np.zeros(14), np.full(14, np.nan))
    target[0] = .1
    with pytest.raises(MotionFault, match="command velocity"):
        guard.check(target, action, .02)

    trajectory = MinimumSnapTrajectory([0, .1], [[0], [.02]])
    with pytest.raises(MotionFault, match="derivative 2"):
        trajectory.validate_limits([-1], [1], 1, 5)
    trajectory.validate_limits([-1], [1], 1, None)
    with pytest.raises(MotionFault, match="derivative 1"):
        trajectory.validate_limits([-1], [1], .4, None)


def test_active_config_disables_position_and_acceleration_trips():
    config = yaml.safe_load((ROOT / "config/pi05_deployment.yaml").read_text())["bridge"]
    guard = CommandSafety(config, np.full(14, -3), np.full(14, 3))
    assert guard.max_acceleration is None
    assert guard.max_velocity == 5.
    assert config["replay_approach"]["max_velocity_rad_s"] == 5.
    assert np.isneginf(guard.lower).all() and np.isposinf(guard.upper).all()
    assert config["replay_approach"]["max_acceleration_rad_s2"] == 1.
    expected_mask = np.tile([True, True, True, True, False, False, False], 2)
    np.testing.assert_array_equal(np.isfinite(guard.velocity_limits), expected_mask)
    servo = yaml.safe_load((ROOT / "ros2_ws/src/ctl_msgs_hw/config/controllerConfig.yaml").read_text())
    assert servo["armSafety"]["speed_limit_enabled_per_arm"] == config["motion_safety"]["speed_limit_enabled_per_arm"]


@pytest.mark.parametrize("joint", range(14))
def test_speed_mask_applies_to_command_steps_and_analytic_velocity(joint):
    config = {"motion_safety": {
        "max_velocity_rad_s": 5., "max_acceleration_rad_s2": None,
        "speed_limit_enabled_per_arm": [True, True, True, True, False, False, False],
    }}
    for step in (0., .02):
        guard = CommandSafety(config, np.full(14, -3), np.full(14, 3), now=lambda: 10.)
        action, feedback, velocity = np.zeros(16), np.zeros(16), np.zeros(14)
        action[np.r_[0:7, 8:15][joint]] = step
        velocity[joint] = 6.
        if joint % 7 < 4:
            with pytest.raises(MotionFault):
                guard.check(action, feedback, .001, velocity)
        else:
            guard.check(action, feedback, .001, velocity)
            guard.reset()
            velocity[joint] = np.inf
            with pytest.raises(MotionFault):
                guard.check(action, feedback, .001, velocity)


@pytest.mark.parametrize("mask", [[True] * 6, [1] * 7, "true", None])
def test_invalid_speed_mask_is_rejected(mask):
    with pytest.raises(ValueError, match="7 booleans"):
        CommandSafety({"motion_safety": {"speed_limit_enabled_per_arm": mask}}, [-3] * 14, [3] * 14)


def test_disabled_position_limits_apply_to_preflight_clipping_and_feedback():
    config = {"motion_safety": {"check_position_limits": False, "max_acceleration_rad_s2": None}}
    lower, upper = configured_position_limits(config, np.zeros(14), np.ones(14))
    current = np.zeros(16)
    current[1] = -.1
    target = current.copy()
    target[1] = -.11
    limited = limit_action(target, current, lower, upper, np.full(14, .02))
    assert limited[1] == -.11  # No implicit clipping back to the URDF lower bound.
    segment = MinimumSnapTrajectory.rest_to_rest([-.1], [-.11], .3, .5)
    segment.validate_limits(lower[:1], upper[:1], 1, None)
    clock = [10.]
    guard = CommandSafety(config, np.zeros(14), np.ones(14), now=lambda: clock[0])
    guard.check(current, current, .02)
    clock[0] += .02
    guard.check(limited, current, .02)
    clock[0] += .02
    target[1] = -.2
    # Feedback-relative allowance and emitted speed remain independent protections.
    assert limit_action(target, current, lower, upper, np.full(14, .02))[1] == pytest.approx(-.12)
    with pytest.raises(MotionFault, match="velocity"):
        guard.check(target, current, .02)
    target[1] = np.nan
    with pytest.raises(ValueError, match="non-finite"):
        limit_action(target, current, lower, upper, np.full(14, .02))


@pytest.mark.parametrize("value", ["false", None, 0])
def test_position_limit_switch_requires_boolean(value):
    with pytest.raises(ValueError, match="boolean"):
        configured_position_limits({"motion_safety": {"check_position_limits": value}}, [0], [1])


def test_all_compensation_disabled_in_source_and_install():
    for path in [ROOT / "ros2_ws/src/ctl_msgs_hw/config/controllerConfig.yaml",
                 ROOT / "ros2_ws/install/ctl_msgs_hw/share/ctl_msgs_hw/config/controllerConfig.yaml"]:
        config = yaml.safe_load(path.read_text())
        assert config["armSafety"]["check_command_position_limits"] is False
        assert config["armSafety"]["check_feedback_position_limits"] is False
        assert config["motorPositionOffsets"]["left_shoulder_roll_deg"] == 0.0
        assert config["armSafety"]["protection_mode"] == "speed_only"
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
        "-Wl,--gc-sections", "-lpthread", "-lyaml-cpp", "-o", str(executable),
        "-Wl,--wrap=ec_send_processdata", "-Wl,--wrap=ec_receive_processdata",
    ], capture_output=True, text=True)
    assert compilation.returncode == 0, compilation.stderr
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)
