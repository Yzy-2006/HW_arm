import sys
import types
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "deployment"))

from replay_dataset import (  # noqa: E402
    ReplayRecorder, _make_bridge_message, _resolve_slow_factor, load_episode_actions, create_replay_node,
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
    assert "0.1" in text


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
            return types.SimpleNamespace(warning=lambda msg: None, info=lambda msg: None)

    for name, value in {
        "rclpy": types.ModuleType("rclpy"),
        "rclpy.node": types.SimpleNamespace(Node=Node),
        "rclpy.qos": types.SimpleNamespace(qos_profile_sensor_data=object()),
        "ctl_msgs_hw.msg": types.SimpleNamespace(BridgeCtlMsg=Message, GripperCtl=Message, MidNodeCtlMsgRet=Message),
        "std_msgs.msg": types.SimpleNamespace(Float64MultiArray=Message),
    }.items():
        monkeypatch.setitem(sys.modules, name, value)
    monkeypatch.setattr(module, "time", types.SimpleNamespace(monotonic=lambda: clock[0]))
    config = yaml.safe_load((ROOT / "config/pi05_deployment.yaml").read_text())
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


def test_replay_feedback_gap_holds_and_restarts_from_feedback(replay):
    node, clock, feedback, actions = replay
    node.step()
    clock[0] += 0.07
    feedback()
    node.step()
    index = node.index
    clock[0] += 0.3
    node.step()
    assert node.command_pub.messages[-1].hold_position
    assert node.index == index
    assert node.trajectory is None
    feedback()
    node.step()
    command = node.command_pub.messages[-1]
    assert command.if_init_use_ret == 1
    assert not command.hold_position
    assert command.l_arm_des_msg[0] == pytest.approx(0)
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
