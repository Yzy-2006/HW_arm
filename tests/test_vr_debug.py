import importlib.util
from pathlib import Path
import subprocess
import sys
import types

import pytest
import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def load_vr_publisher(monkeypatch):
    pytest.importorskip("ctl_msgs_hw.msg")
    sys.path.insert(0, str(ROOT / "teleop" / "VR_controller"))
    televuer = types.ModuleType("televuer")
    televuer.TeleVuerWrapper = object
    monkeypatch.setitem(sys.modules, "televuer", televuer)
    spec = importlib.util.spec_from_file_location(
        "towel_vr_debug_test", ROOT / "teleop" / "VR_controller" / "vr_pose_increment_publisher.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_debug_tick_reports_controller_state_without_publishing_controls(monkeypatch):
    module = load_vr_publisher(monkeypatch)
    calls = []
    snapshot = {"events": 3, "errors": 0, "left_seen": True, "right_seen": False}

    node = types.SimpleNamespace(
        report_controller_debug=lambda received: calls.append(("debug", received)),
        publish_increment=lambda received: calls.append(("increment", received)),
        update_gripper=lambda received: calls.append(("gripper", received)),
    )
    wrapper = types.SimpleNamespace(
        get_motion_state_data=lambda: "telemetry",
        controller_debug_snapshot=lambda: snapshot,
    )

    module.run_vr_tick(node, wrapper, debug_controller_input=True)

    assert calls == [("debug", snapshot)]


def test_normal_vr_tick_publishes_only_position_and_gripper(monkeypatch):
    module = load_vr_publisher(monkeypatch)
    calls = []
    node = types.SimpleNamespace(
        publish_increment=lambda data: calls.append(("increment", data)),
        update_gripper=lambda data: calls.append(("gripper", data)),
    )
    wrapper = types.SimpleNamespace(get_motion_state_data=lambda: "telemetry")

    module.run_vr_tick(node, wrapper)

    assert calls == [("increment", "telemetry"), ("gripper", "telemetry")]


def test_debug_summary_includes_event_counts_and_raw_button_state(monkeypatch):
    module = load_vr_publisher(monkeypatch)

    summary = module.format_controller_debug(
        {
            "events": 12,
            "errors": 1,
            "left_seen": True,
            "right_seen": False,
            "left_trigger": True,
            "left_trigger_value": 0.75,
            "right_trigger": False,
            "right_trigger_value": 0.0,
            "left_a": False,
            "left_b": True,
            "right_a": False,
            "right_b": False,
        }
    )

    assert "events=12" in summary
    assert "errors=1" in summary
    assert "left(seen=yes, trigger=yes/0.75" in summary
    assert "left_a=no, left_b=yes" in summary
    assert "right(seen=no, trigger=no/0.00" in summary


def test_make_vr_debug_starts_only_the_vr_publisher_with_debug_flag():
    result = subprocess.run(
        ["make", "-n", "vr-debug"],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    assert "vr_pose_increment_publisher.py" in result.stdout
    assert "--debug-controller-input" in result.stdout
    assert "servo" not in result.stdout


def test_managed_vr_does_not_emit_arm_motion_without_fresh_grant(monkeypatch):
    module = load_vr_publisher(monkeypatch)
    cls = module.VRPoseIncrementPublisher
    invalidations = []
    node = types.SimpleNamespace(
        intervention_enabled=True, intervention_mode="policy", intervention_received_at=10.0,
        intervention_was_active=False, first_frame=False,
        dual_grippers=types.SimpleNamespace(sides={
            "left": types.SimpleNamespace(invalidate=lambda: invalidations.append("left")),
            "right": types.SimpleNamespace(invalidate=lambda: invalidations.append("right")),
        }),
    )
    monkeypatch.setattr(module.time, "monotonic", lambda: 10.1)
    node._intervention_active = lambda: cls._intervention_active(node)
    cls.publish_increment(node, object())  # Invalid telemetry must not even be inspected.
    assert node.first_frame
    node.intervention_mode = "vr"
    assert node._intervention_active()
    assert invalidations == []  # Standby feedback already anchors the grippers.
    node.intervention_received_at = 9.0
    assert not node._intervention_active()
    assert invalidations == ["left", "right"]


def test_managed_gripper_commands_only_publish_during_vr(monkeypatch):
    module = load_vr_publisher(monkeypatch)
    commands, states, toggles = [], [], []

    def step(now, buttons):
        toggles.append(buttons)
        return [0.2, -0.3], [0, 1, 0, 1]

    active = [False]
    node = types.SimpleNamespace(
        intervention_enabled=True, _intervention_active=lambda: active[0],
        prev_record_button=False, prev_discard_button=False,
        prev_left_gripper_button=False, prev_right_gripper_button=False,
        dual_grippers=types.SimpleNamespace(step=step),
        gripper_ctl_pub=types.SimpleNamespace(publish=commands.append),
        gripper_state_pub=types.SimpleNamespace(publish=states.append),
    )
    telemetry = types.SimpleNamespace(tele_state=types.SimpleNamespace(left_squeeze_ctrl_state=True))
    module.VRPoseIncrementPublisher.update_gripper(node, telemetry)
    assert not commands
    assert len(states) == 1
    assert not toggles[-1]["left"]
    active[0] = True
    module.VRPoseIncrementPublisher.update_gripper(node, telemetry)
    assert len(commands) == 1
    assert not toggles[-1]["left"]  # A squeeze held during policy cannot toggle on takeover.
    np.testing.assert_allclose([commands[0].l_arm_gripper_pos, commands[0].r_arm_gripper_pos], [.2, -.3])
