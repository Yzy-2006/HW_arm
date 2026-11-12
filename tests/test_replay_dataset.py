import sys
import datetime as dt
import json
import types
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "deployment"))

from replay_dataset import (  # noqa: E402
    ReplayRecorder, _make_bridge_message, _resolve_slow_factor, _rviz_joint_state,
    load_episode_actions, create_replay_node,
)
from minimum_snap import JOINT_INDICES
from motion_safety import MotionFault


def report_handoff(node, complete=True):
    if not node.command_pub.messages:
        return
    command = node.command_pub.messages[-1]
    node._handoff_callback(types.SimpleNamespace(
        source_header=command.header, complete=complete,
        commanded_position=command.l_arm_des_msg + command.r_arm_des_msg,
    ))


def test_replay_slow_factor_uses_yaml_unless_overridden():
    assert _resolve_slow_factor(None, {"slow_factor": 0.4}) == 0.4
    assert _resolve_slow_factor(0.1, {"slow_factor": 0.4}) == 0.1
    assert _resolve_slow_factor(None, {}) == 1.0


@pytest.mark.parametrize("value", [0, -0.1, 1.1, float("nan"), float("inf"), None, "invalid"])
def test_replay_slow_factor_rejects_invalid_yaml(value):
    with pytest.raises(ValueError, match="slow_factor"):
        _resolve_slow_factor(None, {"slow_factor": value})


def test_bridge_message_preserves_dataset_joint_order():
    class Message:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None)

    action = np.arange(16, dtype=float)
    msg = _make_bridge_message(
        Message, "stamp", action, np.ones(7), np.ones(7) * 2,
        np.ones(7) * 3, np.ones(7) * 4, hold=False, reanchor=True,
    )
    assert msg.l_arm_des_msg == list(range(7))
    assert msg.r_arm_des_msg == list(range(8, 15))
    assert msg.l_arm_rigidity_mod == [2] * 7
    assert msg.r_arm_rigidity_mod == [2] * 7
    assert msg.if_init_use_ret == 1
    assert not msg.hold_position
    assert msg.target_period_s == .02


def test_bridge_message_preserves_trajectory_derivatives():
    class Message:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None)
    v, a = np.arange(14, dtype=float), np.arange(14, dtype=float) * 2
    msg = _make_bridge_message(Message, 42, np.zeros(16), *[np.ones(7)] * 4,
        hold=False, reanchor=False, target_period=1 / 300, velocity=v, acceleration=a)
    assert msg.derivatives_valid
    assert msg.target_period_s == 1 / 300
    assert msg.l_arm_mdp_des_jnt_vel == v[:7].tolist()
    assert msg.r_arm_mdp_des_jnt_vel == v[7:].tolist()
    assert msg.l_arm_des_acc == a[:7].tolist()
    assert msg.r_arm_des_acc == a[7:].tolist()


def test_load_episode_actions_selects_frame_range(monkeypatch, tmp_path):
    (tmp_path / "meta/episodes").mkdir(parents=True)
    (tmp_path / "data/chunk-000").mkdir(parents=True)
    (tmp_path / "meta/info.json").write_text('{"fps": 30}')
    episode_path = tmp_path / "meta/episodes/file-000.parquet"
    data_path = tmp_path / "data/chunk-000/file-000.parquet"
    episode_path.touch()
    data_path.touch()

    episode_rows = [{
        "episode_index": 2, "length": 4, "data/chunk_index": 0,
        "data/file_index": 0, "dataset_from_index": 0, "dataset_to_index": 4,
    }]
    data_rows = [
        {"action": [float(frame)] * 16, "episode_index": 2, "frame_index": frame}
        for frame in range(4)
    ]

    fake_pyarrow = types.ModuleType("pyarrow")
    fake_pyarrow.__path__ = []
    fake_parquet = types.ModuleType("pyarrow.parquet")

    class Table:
        def __init__(self, rows):
            self.rows = rows

        def to_pylist(self):
            return self.rows

    def read_table(path, columns):
        return Table(episode_rows if "episodes" in str(path) else data_rows)

    fake_parquet.read_table = read_table
    fake_pyarrow.parquet = fake_parquet
    monkeypatch.setitem(sys.modules, "pyarrow", fake_pyarrow)
    monkeypatch.setitem(sys.modules, "pyarrow.parquet", fake_parquet)

    actions, fps = load_episode_actions(tmp_path, 2, start_frame=1, max_frames=2)
    assert fps == 30
    np.testing.assert_array_equal(actions[:, 0], [1, 2])
    assert actions.shape == (2, 16)


def test_replay_recorder_writes_command_and_feedback_csv(tmp_path):
    recorder = ReplayRecorder(tmp_path, episode=3, start_frame=10, make_plot=False)
    command = np.arange(14, dtype=float)
    feedback = command + 0.1
    dataset_action = command + 1.0
    recorder.record(10, dataset_action, command, feedback, gripper_feedback=np.arange(8, dtype=float))
    csv_path, plot_path = recorder.save()
    assert csv_path is not None and csv_path.is_file()
    assert plot_path is None
    dt.datetime.strptime(csv_path.stem, '%Y%m%d_%H%M%S_%f')
    metadata = json.loads(csv_path.with_suffix('.json').read_text())
    assert metadata['episode'] == 3 and metadata['start_frame'] == 10
    text = csv_path.read_text()
    assert "command_left_joint_1" in text
    assert "dataset_left_joint_1" in text
    assert "feedback_right_joint_7" in text
    assert "feedback_left_gripper_deg" in text
    assert "interpolated_left_joint_1" in text
    assert "source_knot" in text
    assert "0.1" in text


def test_replay_recorder_plot_marks_source_knots_and_interpolation(tmp_path):
    recorder = ReplayRecorder(tmp_path, episode=3, start_frame=10)
    for frame, value in enumerate((0.0, 0.5, 1.0)):
        positions = np.full(14, value)
        recorder.record(
            10 + frame, positions, positions, None,
            interpolated=positions + 0.1, source_knot=True,
        )
    csv_path, plot_path = recorder.save()
    assert csv_path is not None and csv_path.is_file()
    assert plot_path is not None and plot_path.is_file()
    assert plot_path.stem == csv_path.stem
    dt.datetime.strptime(plot_path.stem, '%Y%m%d_%H%M%S_%f')


def test_rviz_joint_state_uses_model_viewer_joint_names():
    class Message:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None)

    msg = _rviz_joint_state(Message, "stamp", np.arange(14, dtype=float))
    assert msg.header.stamp == "stamp"
    assert msg.name[0] == "arm_l_joint_1"
    assert msg.name[-1] == "arm_r_joint_7"
    assert msg.position == list(np.arange(14, dtype=float))


def test_rviz_joint_state_maps_binary_grippers_to_urdf_finger_limits():
    class Message:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None)

    msg = _rviz_joint_state(Message, "stamp", np.zeros(14), [0, 1])
    assert msg.name[7] == "arm_l_joint_finger"
    assert msg.name[-1] == "arm_r_joint_finger"
    assert msg.position[7] == pytest.approx(-1.5708)
    assert msg.position[-1] == pytest.approx(0.0)


def test_rviz_launch_does_not_inherit_opencv_qt_paths(monkeypatch):
    import os
    import replay_dataset as module

    monkeypatch.setenv("QT_QPA_PLATFORM_PLUGIN_PATH", "/venv/site-packages/cv2/qt/plugins")
    monkeypatch.setenv("QT_QPA_FONTDIR", "/venv/site-packages/cv2/qt/fonts")
    monkeypatch.setenv("QT_PLUGIN_PATH", "/custom/system/plugins")
    monkeypatch.setenv("DISPLAY", ":1")
    calls = []
    monkeypatch.setattr(module.subprocess, "Popen", lambda cmd, **kw: calls.append((cmd, kw)))
    module._start_rviz()
    env = calls[0][1]["env"]
    assert "QT_QPA_PLATFORM_PLUGIN_PATH" not in env
    assert "QT_QPA_FONTDIR" not in env
    assert env["QT_PLUGIN_PATH"] == "/custom/system/plugins"
    assert env["DISPLAY"] == ":1"
    assert "cv2" in os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"]
    assert calls[0][1]["start_new_session"]


def test_rviz_waits_before_resetting_playback_timer(monkeypatch):
    import replay_dataset as module

    clock, resets, previews = [0.0], [], []
    monkeypatch.setattr(module, "time", types.SimpleNamespace(monotonic=lambda: clock[0]))
    node = types.SimpleNamespace(
        timer=types.SimpleNamespace(cancel=lambda: resets.append("cancel"), reset=lambda: resets.append("reset")),
        actions=[np.zeros(16)], _publish_rviz=previews.append,
        get_node_names_and_namespaces=lambda: [("rviz2", "/model_viewer")] if clock[0] >= 2 else [],
        rviz_pub=types.SimpleNamespace(get_subscription_count=lambda: 1),
    )
    ros = types.SimpleNamespace(ok=lambda: True, spin_once=lambda *a, **kw: clock.__setitem__(0, clock[0] + .5))
    module._wait_for_rviz(node, types.SimpleNamespace(poll=lambda: None), ros)
    assert resets == ["cancel", "reset"]
    assert clock[0] == 3.0
    assert len(previews) == 7
    with pytest.raises(RuntimeError, match="exited during startup"):
        module._wait_for_rviz(node, types.SimpleNamespace(poll=lambda: 1), ros)
    with pytest.raises(RuntimeError, match="timed out"):
        module._wait_for_rviz(node, types.SimpleNamespace(poll=lambda: None), ros, timeout_s=0)


@pytest.fixture
def replay(monkeypatch, tmp_path, request):
    import replay_dataset as module

    clock = [10.0]

    class Message:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None)

    class Node:
        def __init__(self, name):
            self.publishers = {}

        def create_publisher(self, *args):
            messages = []
            publisher = types.SimpleNamespace(messages=messages, publish=messages.append)
            self.publishers[args[1]] = publisher
            return publisher

        def create_subscription(self, *args):
            pass

        def create_timer(self, period, callback):
            timer = types.SimpleNamespace(period=period, cancelled=False)
            timer.cancel = lambda: setattr(timer, 'cancelled', True)
            return timer

        def get_clock(self):
            return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda:
                types.SimpleNamespace(sec=int(clock[0]), nanosec=int((clock[0] % 1) * 1e9))))

        def get_logger(self):
            return types.SimpleNamespace(warning=lambda msg: None, info=lambda msg: None, error=lambda msg: None)

    for name, value in {
        "rclpy": types.ModuleType("rclpy"),
        "rclpy.node": types.SimpleNamespace(Node=Node),
        "rclpy.qos": types.SimpleNamespace(qos_profile_sensor_data=object(),
            QoSProfile=lambda **kw: kw, DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=1)),
        "ctl_msgs_hw.msg": types.SimpleNamespace(BridgeCtlMsg=Message, GripperCtl=Message,
                                                MidNodeCtlMsgRet=Message, HandoffStatus=Message),
        "std_msgs.msg": types.SimpleNamespace(Float64MultiArray=Message, String=types.SimpleNamespace),
    }.items():
        monkeypatch.setitem(sys.modules, name, value)
    monkeypatch.setattr(module, "time", types.SimpleNamespace(monotonic=lambda: clock[0]))
    # Timing tests supply a healthy servo; fault/heartbeat behavior is tested separately.
    from motion_safety import attach_safety
    def safety(node, on_fault, **kwargs):
        state = attach_safety(node, on_fault, **kwargs)
        state.now = lambda: clock[0]
        state.started = clock[0]
        if kwargs.get("monitor_heartbeat", True):
            state.ready = lambda: not state.fault
        return state
    monkeypatch.setattr(module, "attach_safety", safety)
    config = yaml.safe_load((ROOT / "config/pi05_deployment.yaml").read_text())
    # Timing tests use 300 Hz independently of the operator's publication rate.
    config["bridge"]["interpolation"] = {"enabled": True, "publish_rate_hz": 300}
    controller = yaml.safe_load((ROOT / config["controller_config"]).read_text())
    for name in ("robot_ctlNode_loop_ctl", "dual_arm_ctl_node_loop_ctl"):
        controller["TransLoopCtlInte"][name]["TempTime"] = 1 / 300
    controller_path = tmp_path / "controller.yaml"
    controller_path.write_text(yaml.safe_dump(controller))
    config["controller_config"] = str(controller_path)
    config["bridge"]["motion_safety"] = {"max_acceleration_rad_s2": 100.0}
    config["bridge"]["motion_safety"].update(getattr(request, "param", {}))
    config["bridge"]["replay_approach"] = {
        "max_velocity_rad_s": .3, "max_acceleration_rad_s2": .5, "min_duration_s": 1.,
    }
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(config))
    args = types.SimpleNamespace(config=path, slow_factor=None, speed=1.0, motion=True, start_frame=0)
    actions = np.zeros((4, 16))
    actions[:, 0] = np.arange(1, 5) * 0.002
    actions[1:, [7, 15]] = 1
    recorder = ReplayRecorder(tmp_path / "logs", 0, 0, make_plot=False)
    node = create_replay_node(args, actions, 30, recorder)

    def feedback(joints=None):
        joints = np.zeros(14) if joints is None else joints
        node._joint_callback(types.SimpleNamespace(l_arm_pos_fdback=joints[:7], r_arm_pos_fdback=joints[7:],
                                                  l_arm_spd_fdback=[0.] * 7, r_arm_spd_fdback=[0.] * 7))
        node._gripper_callback(types.SimpleNamespace(data=[0.0] * 8))
        report_handoff(node)

    feedback()
    return node, clock, feedback, actions, recorder


@pytest.mark.parametrize('replay', [{'protection_mode': 'speed_only'}], indirect=True)
@pytest.mark.parametrize('joint', range(14))
def test_tracking_error_stops_on_first_feedback_for_every_arm_joint(replay, joint):
    node, clock, feedback, _, recorder = replay
    node.step()
    assert node.last_command is not None
    before = (len(node.command_pub.messages), len(node.gripper_pub.messages))
    actual = node.last_command[JOINT_INDICES].copy()
    actual[joint] -= .301
    feedback(actual)  # No timer tick and no debounce required.
    assert node.done and node.timer.cancelled and node.failed
    assert f'{"left" if joint < 7 else "right"}_joint_{joint % 7 + 1}' in node.failed
    assert 'limit=0.3rad' in node.failed
    assert recorder.status == 'fault' and recorder.failure_reason == node.failed
    stops = node.publishers['/towel/safety/stop'].messages
    assert len(stops) == 1 and stops[0].data == node.failed
    feedback(node.last_command[JOINT_INDICES])
    node.step()
    node._publish_hold()
    assert (len(node.command_pub.messages), len(node.gripper_pub.messages)) == before


@pytest.mark.parametrize('replay', [{'protection_mode': 'speed_only'}], indirect=True)
def test_tracking_guard_checks_next_target_before_clipping_or_publication(replay):
    node, _, feedback, _, _ = replay
    node.step()
    node.approach.at = lambda elapsed, derivative=0: np.r_[.301, np.zeros(13)] if derivative == 0 else np.zeros(14)
    before = len(node.command_pub.messages)
    node.step()
    assert 'source=next_target' in node.failed
    assert len(node.command_pub.messages) == before
    assert node.publishers['/towel/safety/stop'].messages


def test_tracking_threshold_is_strict_and_does_not_apply_to_grippers(replay):
    node, _, _, _, _ = replay
    node.joints = np.zeros(14)
    target = np.zeros(16)
    target[0], target[7], target[15] = .3, 1., 1.
    node._check_tracking(target, 'test')
    target[0] = .300001
    with pytest.raises(MotionFault, match='Replay tracking error'):
        node._check_tracking(target, 'test')


def test_replay_mid_gains_match_original_preset_and_rigidity_mode(replay):
    from pi05_bridge import load_pd
    node, _, _, _, _ = replay
    kp_l, kd_l, kp_r, kd_r = load_pd(ROOT / 'ros2_ws/src/ctl_msgs_hw/config/initialPD.yaml', 'mid')
    node.step()
    node._publish_hold()
    for msg in node.command_pub.messages:
        assert msg.l_arm_rigidity_mod == [2] * 7 and msg.r_arm_rigidity_mod == [2] * 7
        np.testing.assert_array_equal(msg.l_arm_jnt_kp, kp_l)
        np.testing.assert_array_equal(msg.r_arm_jnt_kp, kp_r)
        np.testing.assert_array_equal(msg.l_arm_jnt_kd, kd_l)
        np.testing.assert_array_equal(msg.r_arm_jnt_kd, kd_r)
    config = yaml.safe_load((ROOT / 'config/pi05_deployment.yaml').read_text())
    assert config['bridge']['pd_level'] == 'mid' and config['bridge']['rigidity_mode'] == 2


@pytest.mark.parametrize('acknowledged', [False, True])
def test_replay_exit_waits_for_servo_fault_or_reports_unconfirmed(monkeypatch, acknowledged):
    import replay_dataset as module
    clock, messages = [0.], []
    monkeypatch.setattr(module, 'time', types.SimpleNamespace(monotonic=lambda: clock[0]))
    state = types.SimpleNamespace(state='ready')
    def spin(node, timeout_sec):
        clock[0] += timeout_sec
        if acknowledged:
            state.state = 'fault:upstream: Replay tracking error'
    node = types.SimpleNamespace(safety=state,
        get_logger=lambda: types.SimpleNamespace(info=messages.append, error=messages.append))
    ros = types.SimpleNamespace(ok=lambda: True, spin_once=spin)
    assert module._wait_for_servo_fault(node, ros) is acknowledged
    assert ('NOT confirmed' in messages[-1]) != acknowledged


@pytest.mark.parametrize("replay", [{"protection_mode": "speed_only", "max_velocity_rad_s": 5.}], indirect=True)
def test_speed_only_replay_skips_feedback_age_and_first_frame_arrival_gate(replay):
    node, clock, feedback, actions, recorder = replay
    assert not node.safety.monitor_heartbeat and node.start_gate.status_at is None
    clock[0] += 10
    node.step()
    assert node.approach is not None and not node.failed
    duration = node.approach.duration
    for _ in range(int(duration * 300) + 2):
        clock[0] += 1 / 300
        node.step()  # No feedback, velocity or native-handoff updates.
    assert node.dataset_started_at is not None and not node.failed
    assert node.start_gate.started is None
    assert any(row["phase"] == "replay" for row in recorder.rows)


def test_replay_interpolates_at_300hz_and_visits_all_30hz_knots(replay):
    node, clock, feedback, actions, recorder = replay
    assert node.timer.period == pytest.approx(1 / 300)
    for tick in range(600):
        clock[0] = 10 + tick / 300
        feedback()
        node.step()
        command = node.command_pub.messages[-1]
        if node.dataset_started_at is not None:
            replay_tick = round((clock[0] - node.dataset_started_at) * 300)
            if replay_tick % 10 == 0:
                assert command.l_arm_des_msg[0] == pytest.approx(actions[replay_tick // 10, 0])
        if tick < 300:
            assert node.index == 0
            assert recorder.rows[-1]["phase"] == "approach"
            assert recorder.rows[-1]["dataset_frame"] == -1
            assert not recorder.rows[-1]["source_knot"]
        assert not command.hold_position
        assert command.if_init_use_ret == int(tick == 0)
        if node.index == len(actions):
            break
    assert node.dataset_started_at >= 11.2
    assert any(row["phase"] == "settling" for row in recorder.rows)
    assert [row["dataset_frame"] for row in recorder.rows if row["source_knot"]] == [0, 1, 2, 3]
    node.step()
    assert node.done
    assert node.command_pub.messages[-1].hold_position


def test_replay_feedback_gap_latches_fault_without_automatic_resume(replay):
    node, clock, feedback, actions, _ = replay
    node.step()
    clock[0] += 0.07
    feedback()
    node.step()
    index = node.index
    clock[0] += 0.3
    count = len(node.command_pub.messages)
    node.step()
    assert node.failed
    assert node.index == index
    feedback()
    node.step()
    assert len(node.command_pub.messages) == count
    assert node.index == index


def test_replay_dry_run_does_not_publish(replay):
    node, clock, feedback, actions, recorder = replay
    node.motion = False
    node.step()
    clock[0] += 1
    node.step()
    node.step()
    assert node.done
    assert not node.command_pub.messages
    assert not node.gripper_pub.messages
    assert all(row["phase"] == "replay" for row in recorder.rows)


def test_large_start_gap_gets_independent_duration_and_preserves_dataset_timing(replay):
    node, clock, feedback, actions, recorder = replay
    actual = np.zeros(14)
    actual[0] = .5
    feedback(actual)
    node.step()
    assert not node.failed
    assert node.approach.duration > 3
    assert node.trajectory.duration == pytest.approx((len(actions) - 1) / 30)
    start = clock[0]
    for tick in range(1, int(np.ceil(node.approach.duration * 300)) + 1):
        clock[0] = start + tick / 300
        feedback(np.r_[node.last_command[:7], node.last_command[8:15]])
        node.step()
        assert not node.failed
    assert node.dataset_started_at is None
    assert recorder.rows[-1]["phase"] == "settling"
    for _ in range(90):
        clock[0] += 1 / 300
        feedback(np.r_[node.last_command[:7], node.last_command[8:15]])
        node.step()
        assert not node.failed
        if node.dataset_started_at is not None:
            break
    assert node.dataset_started_at == clock[0]
    assert node.index == 0
    assert node.last_command[0] == pytest.approx(actions[0, 0])
    assert recorder.rows[-1]["phase"] == "replay"
    assert recorder.rows[-1]["source_knot"]
    for derivative in (1, 2, 3):
        np.testing.assert_allclose(node.approach.at(node.approach.duration, derivative),
                                   node.trajectory.at(0, derivative), atol=1e-7)


def test_invalid_dataset_segment_is_rejected_before_approach_motion(replay):
    node, clock, feedback, actions, recorder = replay
    actions[1, 0] = .5
    node.step()
    assert node.failed.startswith("Dataset replay segment:")
    assert not node.command_pub.messages
    assert not node.gripper_pub.messages
    assert not recorder.rows


def test_single_frame_replay_holds_grippers_during_approach(replay):
    node, clock, feedback, actions, recorder = replay
    node.actions = actions[:1].copy()
    node.actions[0, [7, 15]] = 1
    for tick in range(600):
        clock[0] = 10 + tick / 300
        feedback()
        node.step()
        assert not node.failed
        if node.dataset_started_at is None:
            np.testing.assert_array_equal(node.last_command[[7, 15]], node.initial_grippers)
        else:
            np.testing.assert_array_equal(node.last_command[[7, 15]], [1, 1])
        if node.index == 1:
            break
    node.step()
    assert node.done
    assert [row["dataset_frame"] for row in recorder.rows if row["source_knot"]] == [0]


def test_noninterpolated_replay_also_approaches_before_consuming_frames(replay, monkeypatch, tmp_path):
    import replay_dataset as module
    _, clock, _, actions, _ = replay
    monkeypatch.setattr(module, "interpolation_settings", lambda config, rate: (False, rate))
    args = types.SimpleNamespace(config=tmp_path / "config.yaml", slow_factor=None,
                                 speed=1., motion=True, start_frame=12)
    actions = np.repeat(actions[:1], 2, axis=0)
    recorder = ReplayRecorder(tmp_path / "raw_logs", 0, 12, make_plot=False)
    node = create_replay_node(args, actions, 30, recorder)
    for tick in range(100):
        clock[0] = 10 + tick / 30
        node._joint_callback(types.SimpleNamespace(l_arm_pos_fdback=[0.] * 7, r_arm_pos_fdback=[0.] * 7,
                                                  l_arm_spd_fdback=[0.] * 7, r_arm_spd_fdback=[0.] * 7))
        node._gripper_callback(types.SimpleNamespace(data=[0.] * 8))
        report_handoff(node)
        node.step()
        assert not node.failed
        if tick < 30:
            assert node.index == 0
            assert not node.command_pub.messages[-1].derivatives_valid
        if node.index == 2:
            break
    node.step()
    assert node.done
    assert [row["dataset_frame"] for row in recorder.rows if row["source_knot"]] == [12, 13]


def test_zero_approach_limit_is_rejected_before_publication(replay, tmp_path):
    _, clock, _, actions, _ = replay
    path = tmp_path / "config.yaml"
    config = yaml.safe_load(path.read_text())
    config["bridge"]["replay_approach"]["max_velocity_rad_s"] = 0
    path.write_text(yaml.safe_dump(config))
    args = types.SimpleNamespace(config=path, slow_factor=None, speed=1., motion=True, start_frame=0)
    node = create_replay_node(args, actions, 30)
    node._joint_callback(types.SimpleNamespace(l_arm_pos_fdback=[0.] * 7, r_arm_pos_fdback=[0.] * 7))
    node._gripper_callback(types.SimpleNamespace(data=[0.] * 8))
    node.step()
    assert "Invalid replay_approach" in node.failed
    assert not node.command_pub.messages


def test_approach_out_of_bounds_feedback_does_not_move(replay):
    node, clock, feedback, _, recorder = replay
    initial = np.zeros(14)
    initial[0] = 10
    feedback(initial)
    node.step()
    assert node.failed.startswith("Approach to first frame:")
    assert not node.command_pub.messages
    assert not recorder.rows


def test_disabled_acceleration_trip_preserves_approach_planning(replay, tmp_path):
    _, _, _, actions, _ = replay
    path = tmp_path / "config.yaml"
    config = yaml.safe_load(path.read_text())
    config["bridge"]["motion_safety"]["max_acceleration_rad_s2"] = None
    path.write_text(yaml.safe_dump(config))
    args = types.SimpleNamespace(config=path, slow_factor=None, speed=1., motion=True, start_frame=0)
    node = create_replay_node(args, actions, 30)
    node._joint_callback(types.SimpleNamespace(l_arm_pos_fdback=[0.] * 7, r_arm_pos_fdback=[0.] * 7))
    node._gripper_callback(types.SimpleNamespace(data=[0.] * 8))
    node.step()
    assert not node.failed
    assert node.command_safety.max_acceleration is None
    assert node.command_pub.messages
    node.approach.validate_limits(np.full(14, -3), np.full(14, 3), .3, .5)


def test_disabled_position_limits_allow_approach_from_negative_left_j2(replay, tmp_path):
    _, _, _, actions, _ = replay
    path = tmp_path / "config.yaml"
    config = yaml.safe_load(path.read_text())
    config["bridge"]["motion_safety"]["check_position_limits"] = False
    path.write_text(yaml.safe_dump(config))
    actions[:, 1] = -.15
    args = types.SimpleNamespace(config=path, slow_factor=None, speed=1., motion=True, start_frame=0)
    node = create_replay_node(args, actions, 30)
    node._joint_callback(types.SimpleNamespace(l_arm_pos_fdback=[0., -.1, 0., 0., 0., 0., 0.],
                                              r_arm_pos_fdback=[0.] * 7))
    node._gripper_callback(types.SimpleNamespace(data=[0.] * 8))
    node.step()
    assert not node.failed
    assert node.command_pub.messages[-1].l_arm_des_msg[1] == pytest.approx(-.1)
    assert node.approach.at(node.approach.duration)[1] == pytest.approx(-.15)


def test_replay_waits_for_handoff_and_physical_arrival_then_starts_at_frame_zero(replay):
    node, clock, feedback, actions, recorder = replay
    node.step()
    for tick in range(1, 700):
        clock[0] = 10 + tick / 300
        feedback()
        if tick < 450:
            report_handoff(node, complete=False)
        elif tick < 550:
            node.joints[0] = .08  # Complete command handoff alone is insufficient.
        elif tick < 600:
            node.joint_velocity[0] = .1  # Near the target but still moving.
        node.step()
        assert not node.failed
        if tick < 660:
            assert node.dataset_started_at is None
            assert node.index == 0
            assert not any(row["source_knot"] for row in recorder.rows)
        if node.dataset_started_at is not None:
            assert node.dataset_started_at == clock[0]
            np.testing.assert_allclose(node.last_command[[0, 1, 2]], actions[0, [0, 1, 2]], atol=1e-9)
            assert recorder.rows[-1]["source_knot"]
            assert recorder.rows[-1]["phase"] == "replay"
            break
    else:
        pytest.fail("Did not start after fresh handoff and settled physical feedback")


def test_replay_missing_handoff_times_out_without_consuming_frames(replay):
    node, clock, feedback, _, recorder = replay
    node.start_gate.timeout = .5
    for tick in range(500):
        clock[0] = 10 + tick / 300
        feedback()
        node.start_gate.status_at = None
        node.step()
        if node.failed:
            break
    assert "First-frame settle timeout" in node.failed
    assert node.dataset_started_at is None and node.index == 0
    assert recorder.status == "fault"
    assert not any(row["source_knot"] for row in recorder.rows)
    count = len(node.command_pub.messages)
    feedback()
    node.step()
    assert len(node.command_pub.messages) == count
