import sys
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
def replay(monkeypatch, tmp_path):
    import replay_dataset as module

    clock = [10.0]

    class Message:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None)

    class Node:
        def __init__(self, name):
            pass

        def create_publisher(self, *args):
            messages = []
            return types.SimpleNamespace(messages=messages, publish=messages.append)

        def create_subscription(self, *args):
            pass

        def create_timer(self, period, callback):
            return types.SimpleNamespace(period=period, cancel=lambda: None)

        def get_clock(self):
            return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: clock[0]))

        def get_logger(self):
            return types.SimpleNamespace(warning=lambda msg: None, info=lambda msg: None, error=lambda msg: None)

    for name, value in {
        "rclpy": types.ModuleType("rclpy"),
        "rclpy.node": types.SimpleNamespace(Node=Node),
        "rclpy.qos": types.SimpleNamespace(qos_profile_sensor_data=object()),
        "ctl_msgs_hw.msg": types.SimpleNamespace(BridgeCtlMsg=Message, GripperCtl=Message, MidNodeCtlMsgRet=Message),
        "std_msgs.msg": types.SimpleNamespace(Float64MultiArray=Message),
    }.items():
        monkeypatch.setitem(sys.modules, name, value)
    monkeypatch.setattr(module, "time", types.SimpleNamespace(monotonic=lambda: clock[0]))
    # Timing tests supply a healthy servo; fault/heartbeat behavior is tested separately.
    from motion_safety import SafetyState
    def safety(node, on_fault):
        state = SafetyState(on_fault, now=lambda: clock[0])
        state.ready = lambda: not state.fault
        return state
    monkeypatch.setattr(module, "attach_safety", safety)
    config = yaml.safe_load((ROOT / "config/pi05_deployment.yaml").read_text())
    config["bridge"]["motion_safety"] = {"max_acceleration_rad_s2": 100.0}
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(config))
    args = types.SimpleNamespace(config=path, slow_factor=None, speed=1.0, motion=True, start_frame=0)
    actions = np.zeros((4, 16))
    actions[:, 0] = np.arange(1, 5) * 0.002
    actions[1:, [7, 15]] = 1
    node = create_replay_node(args, actions, 30)

    def feedback():
        node._joint_callback(types.SimpleNamespace(l_arm_pos_fdback=[0.0] * 7, r_arm_pos_fdback=[0.0] * 7))
        node._gripper_callback(types.SimpleNamespace(data=[0.0] * 8))

    feedback()
    return node, clock, feedback, actions


def test_replay_interpolates_at_300hz_and_visits_all_30hz_knots(replay):
    node, clock, feedback, actions = replay
    assert node.timer.period == pytest.approx(1 / 300)
    for tick in range(41):
        clock[0] = 10 + tick / 300
        feedback()
        node.step()
        command = node.command_pub.messages[-1]
        if tick and tick % 10 == 0:
            assert command.l_arm_des_msg[0] == pytest.approx(actions[tick // 10 - 1, 0])
        assert not command.hold_position
        assert command.if_init_use_ret == int(tick == 0)
    assert len(node.command_pub.messages) == 41
    node.step()
    assert node.done
    assert node.command_pub.messages[-1].hold_position


def test_replay_feedback_gap_latches_fault_without_automatic_resume(replay):
    node, clock, feedback, actions = replay
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
    node, clock, feedback, actions = replay
    node.motion = False
    node.step()
    clock[0] += 1
    node.step()
    node.step()
    assert node.done
    assert not node.command_pub.messages
    assert not node.gripper_pub.messages
