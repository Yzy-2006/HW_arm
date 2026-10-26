import sys
from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path
from types import SimpleNamespace
from threading import Event

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "deployment"))

from pi05_bridge import GripperState, limit_action, load_joint_limits, load_pd  # noqa: E402
from pi05_common import recv_packet, send_packet  # noqa: E402
from pi05_worker import _is_openpi_checkpoint  # noqa: E402
from pi05_bridge import ActionChunk, create_bridge_node  # noqa: E402
from pi05_worker import PolicyRunner  # noqa: E402


def test_pi05_protocol_round_trip():
    class FakeSocket:
        def __init__(self):
            self.buffer = bytearray()

        def sendall(self, value):
            self.buffer.extend(value)

        def recv(self, size):
            value = bytes(self.buffer[:size])
            del self.buffer[:size]
            return value

    sock = FakeSocket()
    send_packet(sock, {"type": "health", "ok": True})
    assert recv_packet(sock) == {"type": "health", "ok": True}


def test_openpi_checkpoint_layout_is_detected(tmp_path):
    (tmp_path / "inference.py").write_text("# published OpenPI entry point\n")
    assert _is_openpi_checkpoint(tmp_path)
    (tmp_path / "config.json").write_text("{}\n")
    assert not _is_openpi_checkpoint(tmp_path)


def test_encos_limits_and_low_pd_have_14_joints():
    lower, upper = load_joint_limits(ROOT / "ros2_ws/src/dual_arm_hw/urdf/dual_arm.urdf")
    assert lower.shape == (14,)
    assert upper.shape == (14,)
    assert np.all(lower < upper)
    assert all(values.shape == (7,) for values in load_pd(ROOT / "ros2_ws/src/ctl_msgs_hw/config/initialPD.yaml", "low"))


def test_gripper_hysteresis_and_action_safety():
    gripper = GripperState(
        {
            "left": {"open_deg": 0.0, "closed_deg": 10.0},
            "right": {"open_deg": 0.0, "closed_deg": -10.0},
        }
    )
    feedback = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert np.array_equal(gripper.update(feedback), [0, 0])

    lower, upper = load_joint_limits(ROOT / "ros2_ws/src/dual_arm_hw/urdf/dual_arm.urdf")
    state = np.zeros(16)
    action = limit_action(np.full(16, 99.0), state, lower, upper, np.full(14, 0.02))
    assert action.shape == (16,)
    assert np.all(action[[0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14]] <= 0.02)
    assert action[7] == 1.0
    assert action[15] == 1.0


def test_chunk_uses_observation_time_and_expires():
    actions = np.repeat(np.arange(50)[:, None], 16, axis=1)
    chunk = ActionChunk(actions, 10.0, 30, 1.0)
    assert chunk.at(10.11)[0] == 3
    assert chunk.at(10.21)[0] == 6
    assert chunk.at(11.01) is None
    assert ActionChunk(actions[:2], 10, 30, 1).at(10.1) is None
    single = ActionChunk(actions[:1], 10, 30, 1)
    assert single.at(10.1)[0] == 0
    assert single.at(11.01) is None
    actions = actions.astype(float)
    actions[-1, 0] = np.nan
    with pytest.raises(ValueError):
        ActionChunk(actions, 10, 30, 1)


def test_openpi_keeps_the_entire_absolute_action_horizon():
    runner = PolicyRunner.__new__(PolicyRunner)
    runner.backend = "openpi"
    runner.camera_map = {}
    runner._decode_image = lambda value: np.zeros((2, 2, 3), dtype=np.uint8)
    actions = np.arange(800, dtype=np.float32).reshape(50, 16)
    runner.policy = SimpleNamespace(infer=lambda obs: {"actions": actions})
    payload = {"state": np.ones(16), "task": "fold", "images": {
        "head": "", "left_wrist": "", "right_wrist": ""}}
    np.testing.assert_array_equal(runner.infer(payload), actions)
    actions[-1, 0] = np.nan
    with pytest.raises(ValueError, match="non-finite"):
        runner.infer(payload)


class FakePool:
    def __init__(self, **kwargs):
        self.calls = []

    def submit(self, fn, *args):
        future = Future()
        self.calls.append((future, args))
        return future

    def shutdown(self, **kwargs):
        pass


@pytest.fixture
def bridge(monkeypatch):
    import pi05_bridge as module

    clock = [10.0]

    class Message:
        def __init__(self):
            self.header = SimpleNamespace(stamp=None)
            self.hold_position = False

    class Node:
        def __init__(self, name):
            self.subscriptions = []
            self.logs = []

        def create_publisher(self, *args):
            messages = []
            return SimpleNamespace(messages=messages, publish=messages.append)

        def create_subscription(self, *args, **kwargs):
            self.subscriptions.append(kwargs)

        def create_timer(self, period, callback):
            return SimpleNamespace(cancel=lambda: None)

        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: clock[0]))

        def get_logger(self):
            return SimpleNamespace(warning=self.logs.append, info=self.logs.append)

    class Cameras:
        def __init__(self, config):
            self.error = False

        def snapshot(self, now):
            if self.error:
                raise ValueError("Camera frame is stale")
            return {}, {}

        def close(self):
            pass

    for name, mod in {
        "rclpy.callback_groups": SimpleNamespace(MutuallyExclusiveCallbackGroup=object),
        "rclpy.node": SimpleNamespace(Node=Node),
        "rclpy.qos": SimpleNamespace(qos_profile_sensor_data=object()),
        "ctl_msgs_hw.msg": SimpleNamespace(BridgeCtlMsg=Message, GripperCtl=Message, MidNodeCtlMsgRet=Message),
        "std_msgs.msg": SimpleNamespace(Float64MultiArray=Message),
        "record": SimpleNamespace(Cameras=Cameras),
    }.items():
        monkeypatch.setitem(sys.modules, name, mod)
    monkeypatch.setattr(module, "ThreadPoolExecutor", FakePool)
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: clock[0]))
    args = SimpleNamespace(config=ROOT / "config/pi05_deployment.yaml", worker_host=None,
                           worker_port=None, token=None, slow_factor=None, motion=True)
    node = create_bridge_node(args)

    def feedback(value=0.0):
        joints = [value] + [0.0] * 6
        node._joint_callback(SimpleNamespace(l_arm_pos_fdback=joints, r_arm_pos_fdback=joints))
        node._gripper_callback(SimpleNamespace(data=[0.0] * 8))

    feedback()
    yield node, clock, feedback
    node.close()


def complete(node, value=1.0):
    node.pending.set_result({"actions": np.full((50, 16), value).tolist()})


def test_continuous_mode_and_latest_feedback_limit(bridge):
    node, clock, feedback = bridge
    node.step()
    assert not node.command_pub.messages
    clock[0] += 0.11
    feedback(0.2)
    complete(node)
    node.step()
    msg = node.command_pub.messages[-1]
    assert msg.ctl_trans_mod == 3
    assert not msg.hold_position
    assert msg.if_init_use_ret == 1
    assert msg.l_arm_des_msg[0] == pytest.approx(0.22)
    assert msg.r_arm_des_msg[0] == pytest.approx(0.22)
    assert all(s["callback_group"] is node.feedback_group for s in node.subscriptions)
    clock[0] += 0.04
    feedback(0.3)
    node.step()
    assert node.command_pub.messages[-1].l_arm_des_msg[0] == pytest.approx(0.32)
    assert node.command_pub.messages[-1].if_init_use_ret == 0


@pytest.mark.parametrize("failure", ["feedback", "camera", "inference"])
def test_failure_holds_without_gripper_commands_and_reanchors(bridge, failure):
    node, clock, feedback = bridge
    node.step()
    complete(node)
    clock[0] += 0.04
    node.step()
    gripper_count = len(node.gripper_pub.messages)
    clock[0] += 0.3
    if failure == "camera":
        feedback()
        node.cameras.error = True
    elif failure == "inference":
        feedback()
        node.step()
        node.pending.set_exception(RuntimeError("inference failed"))
        gripper_count = len(node.gripper_pub.messages)
    node.step()
    assert node.chunk is None
    assert node.command_pub.messages[-1].hold_position
    assert len(node.gripper_pub.messages) == gripper_count
    node.cameras.error = False
    feedback()
    node.step()
    complete(node)
    clock[0] += 0.04
    node.step()
    assert not node.command_pub.messages[-1].hold_position
    assert node.command_pub.messages[-1].if_init_use_ret == 1
    assert node.inference_pool.calls[-1][1][-1] is True  # policy cache reset


def test_inflight_response_is_discarded_after_feedback_loss(bridge):
    node, clock, feedback = bridge
    node.step()
    pending = node.pending
    clock[0] += 0.3
    node.step()
    feedback()
    pending.set_result({"actions": np.ones((50, 16)).tolist()})
    node.step()
    assert not node.command_pub.messages
    assert node.pending is not pending
    assert node.chunk is None


def test_control_continues_during_inference_then_holds_on_expiry(bridge):
    node, clock, feedback = bridge
    node.step()
    complete(node)
    clock[0] += 0.3
    feedback()
    node.step()
    assert node.pending is not None and not node.pending.done()
    clock[0] += 0.1
    feedback(0.1)
    node.step()
    assert not node.command_pub.messages[-1].hold_position
    assert node.command_pub.messages[-1].l_arm_des_msg[0] == pytest.approx(0.12)
    clock[0] += 0.7
    feedback()
    node.step()
    assert node.command_pub.messages[-1].hold_position


def test_expired_inference_never_publishes_motion(bridge):
    node, clock, feedback = bridge
    node.step()
    complete(node)
    clock[0] += 1.1
    feedback()
    node.step()
    assert not node.command_pub.messages
    assert node.chunk is None


def test_dry_run_never_publishes_including_close(bridge):
    node, clock, feedback = bridge
    node.motion = False
    node.step()
    complete(node)
    clock[0] += 0.1
    node.step()
    clock[0] += 0.3
    node.step()
    node.close()
    assert not node.command_pub.messages
    assert not node.gripper_pub.messages


def test_slow_inference_does_not_block_feedback_or_control(bridge):
    node, clock, feedback = bridge
    entered, release = Event(), Event()

    def infer(*args):
        entered.set()
        assert release.wait(2)
        return {"actions": np.ones((50, 16)).tolist()}

    node._infer = infer
    node.inference_pool = ThreadPoolExecutor(max_workers=1)
    try:
        node.step()
        assert entered.wait(1)
        clock[0] += 0.1
        feedback(0.2)
        node.step()
        assert not node.pending.done()
        assert node._snapshot_state()[0] == pytest.approx(0.2)
        release.set()
        node.pending.result(timeout=1)
        node.step()
        assert node.command_pub.messages[-1].l_arm_des_msg[0] == pytest.approx(0.22)
    finally:
        release.set()
        node.inference_pool.shutdown(wait=True)


def test_close_cleans_up_after_ros_publication_is_unavailable(bridge):
    node, clock, feedback = bridge
    node.last_command = np.zeros(16)
    cleaned = []

    def failed_publish(msg):
        raise RuntimeError("ROS context is already closed")

    node.command_pub.publish = failed_publish
    node.cameras.close = lambda: cleaned.append("cameras")
    node.client.close = lambda: cleaned.append("client")
    node.inference_pool.shutdown = lambda **kwargs: cleaned.append("inference")
    node.close()
    assert cleaned == ["inference", "client", "cameras"]
