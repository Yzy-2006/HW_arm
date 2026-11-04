import sys
from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path
from types import SimpleNamespace
from threading import Event

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "deployment"))

from pi05_bridge import GripperState, limit_action, load_joint_limits, load_pd  # noqa: E402
from pi05_common import recv_packet, send_packet  # noqa: E402
from pi05_worker import _is_openpi_checkpoint  # noqa: E402
from pi05_bridge import ActionChunk, create_bridge_node  # noqa: E402
from pi05_worker import PolicyRunner, parse_args  # noqa: E402


def test_worker_reads_yaml_inference_settings_and_cli_overrides(tmp_path):
    config = tmp_path / "deployment.yaml"
    config.write_text(yaml.safe_dump({"worker": {"num_steps": 7, "action_horizon": 50}}))
    args = parse_args(["--config", str(config), "--model", "/unused"])
    assert args.num_steps == 7
    assert args.action_horizon == 50
    args = parse_args(["--config", str(config), "--model", "/unused", "--num-steps", "3"])
    assert args.num_steps == 3
    assert parse_args(["--model", "/unused"]).num_steps == 10


@pytest.mark.parametrize("settings", [
    {"num_steps": 0}, {"num_steps": 1.5}, {"num_steps": True},
    {"action_horizon": -1}, {"action_horizon": 1001},
])
def test_worker_rejects_invalid_yaml_inference_settings(tmp_path, settings):
    config = tmp_path / "deployment.yaml"
    config.write_text(yaml.safe_dump({"worker": settings}))
    with pytest.raises(SystemExit) as error:
        parse_args(["--config", str(config), "--model", "/unused"])
    assert error.value.code == 2


def test_yaml_integration_steps_reach_openpi_loader_and_horizon_is_checked(tmp_path, monkeypatch):
    import pi05_worker

    (tmp_path / "inference.py").touch()
    (tmp_path / "step-49999/params").mkdir(parents=True)
    calls = []

    def load_policy(**kwargs):
        calls.append(kwargs)
        return SimpleNamespace(_model=SimpleNamespace(action_horizon=50))

    monkeypatch.setitem(sys.modules, "jax", SimpleNamespace(devices=lambda: [SimpleNamespace(platform="gpu")]))
    monkeypatch.setattr(pi05_worker, "_load_openpi_inference_module",
                        lambda path: SimpleNamespace(load_policy=load_policy))
    config = tmp_path / "deployment.yaml"
    config.write_text(yaml.safe_dump({"worker": {"num_steps": 7, "action_horizon": 50}}))
    args = parse_args(["--config", str(config), "--model", str(tmp_path), "--checkpoint", "step-49999"])
    runner = PolicyRunner(args.model, args.device, args.allow_cpu, {}, args.checkpoint,
                          args.num_steps, args.action_horizon)
    assert runner.action_horizon == 50
    assert calls == [{"checkpoint": str(tmp_path / "step-49999"), "num_steps": 7}]
    with pytest.raises(ValueError, match="model produces 50 steps"):
        PolicyRunner(str(tmp_path), "cuda", False, {}, "step-49999", 7, 25)


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
    runner.action_horizon = 25
    with pytest.raises(ValueError, match="action_horizon=25"):
        runner.infer(payload)
    runner.action_horizon = 50
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
def bridge(monkeypatch, tmp_path, request):
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
            return SimpleNamespace(period=period, cancel=lambda: None)

        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: clock[0]))

        def get_logger(self):
            return SimpleNamespace(warning=self.logs.append, info=self.logs.append, error=self.logs.append)

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
    from motion_safety import SafetyState
    def safety(node, on_fault):
        state = SafetyState(on_fault, now=lambda: clock[0])
        state.ready = lambda: not state.fault
        return state
    monkeypatch.setattr(module, "attach_safety", safety)
    config = yaml.safe_load((ROOT / "config/pi05_deployment.yaml").read_text())
    # Control tests use an explicit limit, independent of the operator's tuning.
    config["bridge"]["max_joint_step_rad"] = 0.02
    config["bridge"]["interpolation"] = {"enabled": False}
    config["bridge"]["replan_interval_s"] = 0.25
    # These tests isolate inference/handoff timing, not physical motion limits.
    config["bridge"]["motion_safety"] = {"max_velocity_rad_s": 100, "max_acceleration_rad_s2": 100000}
    config["bridge"].update(getattr(request, "param", {}))
    config_path = tmp_path / "deployment.yaml"
    config_path.write_text(yaml.safe_dump(config))
    args = SimpleNamespace(config=config_path, worker_host=None,
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
    actions = np.full((50, 16), value)
    actions[:, [4 - 1, 9]] *= -1  # Left elbow and right shoulder roll have nonpositive ranges.
    node.pending.set_result({"actions": actions.tolist()})


@pytest.mark.parametrize("bridge", [{"interpolation": {"enabled": True, "publish_rate_hz": 300}}], indirect=True)
def test_intervention_blocks_all_policy_ticks_and_discards_inflight_result(bridge):
    node, clock, feedback = bridge
    node.step()
    old = node.pending
    generation = node.generation
    node.intervention = SimpleNamespace(mode="paused")
    node.pause_policy()
    assert node.generation > generation
    assert node.command_pub.messages[-1].hold_position
    count = len(node.command_pub.messages)
    old.set_result({"actions": np.full((50, 16), 999).tolist()})
    node.step()
    node.publish_step()
    assert len(node.command_pub.messages) == count
    assert not node.gripper_pub.messages
    node.resume_policy()
    node.intervention.mode = "policy"
    feedback()
    node.step()
    assert node.chunk is None
    assert node.pending is not old
    assert node.inference_pool.calls[-1][1][-1] is True
    assert node.last_command is None


@pytest.mark.parametrize("bridge", [{"rate_hz": 60, "action_rate_hz": 15}], indirect=True)
def test_action_timeline_rate_is_independent_of_control_timer(bridge):
    node, clock, feedback = bridge
    node.step()
    actions = np.repeat(np.arange(50)[:, None], 16, axis=1)
    node.pending.set_result({"actions": actions.tolist()})
    clock[0] += 0.11
    feedback()
    node.step()
    assert node.control_timer.period == pytest.approx(1 / 60)
    assert node.chunk.rate_hz == 15
    np.testing.assert_array_equal(node.chunk.at(clock[0]), np.ones(16))


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
    clock[0] += 0.22 if failure == "inference" else 0.3
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
    if failure == "feedback":
        assert node.safety.fault
        count = len(node.command_pub.messages)
        feedback()
        node.step()
        assert len(node.command_pub.messages) == count
        return
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


@pytest.mark.parametrize("bridge", [{"interpolation": {"enabled": True, "publish_rate_hz": 300}}], indirect=True)
def test_interpolated_bridge_publishes_between_source_ticks_and_holds(bridge):
    node, clock, feedback = bridge
    node.step()
    complete(node, value=0.01)
    clock[0] += 0.1
    feedback()
    node.step()
    assert node.control_timer.period == pytest.approx(1 / 30)
    assert node.publish_timer.period == pytest.approx(1 / 300)
    assert not node.command_pub.messages
    node.publish_step()
    first = node.command_pub.messages[-1]
    assert first.l_arm_des_msg[0] == pytest.approx(0)
    assert first.if_init_use_ret == 1
    for _ in range(9):
        clock[0] += 1 / 300
        feedback()
        node.publish_step()
    assert len(node.command_pub.messages) == 10
    assert node.command_pub.messages[-1].l_arm_des_msg[0] > 0
    assert node.gripper_pub.messages[-1].ctl_time == pytest.approx(1 / 300)
    clock[0] += 0.3
    feedback()
    node.publish_step()  # A stalled observation loop must also stop the output.
    assert node.command_pub.messages[-1].hold_position
    assert node.interpolator is None
    assert node.chunk is None


@pytest.mark.parametrize("bridge", [{"interpolation": {"enabled": True}}], indirect=True)
def test_interpolated_bridge_replans_continuously_and_dry_run_never_publishes(bridge):
    node, clock, feedback = bridge
    node.motion = False
    node.step()
    complete(node, value=0.001)
    clock[0] += 0.1
    feedback()
    node.step()
    node.publish_step()
    clock[0] += 0.2
    feedback()
    node.step()
    old = node.interpolator
    complete(node, value=-0.001)
    clock[0] += 0.01
    node.step()
    for derivative in range(4):
        np.testing.assert_allclose(
            node.interpolator.trajectory.at(0, derivative),
            old.trajectory.at(clock[0] - old.started_at, derivative), atol=1e-6,
        )
    node.publish_step()
    assert not node.command_pub.messages
    assert not node.gripper_pub.messages
