from concurrent.futures import Future
from types import SimpleNamespace as NS, MethodType
import subprocess

import pytest

from test_vr_debug import load_vr_publisher, ROOT


@pytest.fixture
def managed_vr(monkeypatch):
    module = load_vr_publisher(monkeypatch)
    from grippers import DualGrippers
    clock, logs, commands, states, increments, recording, requests = [10.], [], [], [], [], [], []
    monkeypatch.setattr(module, "time", NS(monotonic=lambda: clock[0]))
    grippers = DualGrippers({side: dict(calibrated=True, open_deg=0., closed_deg=sign * 10.,
                                      motion_time_s=.6) for side, sign in (("left", 1), ("right", -1))})

    def request(message):
        future = Future()
        requests.append(future)
        return future

    node = NS(policy_intervention=True, policy_mode="unknown", policy_mode_received_at=-float("inf"),
              _vr_enabled=False, _await_controls_release=True, _intervention_future=None,
              _intervention_requested_at=0., first_frame=True, _last_sample_time=None,
              _record_request_pending=False,
              prev_record_button=False, prev_discard_button=False,
              prev_left_gripper_button=False, prev_right_gripper_button=False,
              dual_grippers=grippers,
              intervention_client=NS(service_is_ready=lambda: True, call_async=request),
              get_logger=lambda: NS(info=logs.append, error=logs.append),
              gripper_ctl_pub=NS(publish=commands.append), gripper_state_pub=NS(publish=states.append),
              toggle_recording=lambda: recording.append("record"),
              discard_recording=lambda: recording.append("discard"),
              publish_increment=lambda data: increments.append(data))
    for name in ("_reset_intervention_tracking", "_policy_mode_callback", "_request_intervention",
                 "_managed_controls_ready", "update_gripper"):
        setattr(node, name, MethodType(getattr(module.VRPoseIncrementPublisher, name), node))
    buttons = NS(left_aButton=False, left_bButton=False, left_trigger_state=False,
                 right_trigger_state=False, left_squeeze_ctrl_state=False,
                 right_squeeze_ctrl_state=False, right_aButton=False)
    data = NS(tele_state=buttons)
    wrapper = NS(get_motion_state_data=lambda: data)

    def mode(value):
        node._policy_mode_callback(NS(data=value))

    def tick():
        grippers.feedback([0.] * 8, clock[0])
        module.run_vr_tick(node, wrapper)

    return NS(node=node, clock=clock, buttons=buttons, mode=mode, tick=tick, commands=commands,
              increments=increments, recording=recording, requests=requests, logs=logs)


def test_standby_never_publishes_and_discard_requests_takeover_once(managed_vr):
    vr = managed_vr
    vr.mode("policy")
    vr.buttons.left_squeeze_ctrl_state = vr.buttons.left_trigger_state = True
    vr.buttons.left_bButton = True
    vr.tick()
    assert not vr.commands and not vr.increments and not vr.recording
    vr.buttons.left_aButton = True
    vr.tick()
    vr.tick()
    assert len(vr.requests) == 1
    assert not vr.commands and not vr.increments
    vr.requests[0].set_result(NS(success=True, message="accepted"))
    vr.mode("vr")
    vr.tick()
    assert not vr.commands and not vr.increments, "Held controls must be released after takeover"
    vr.buttons.left_trigger_state = vr.buttons.left_squeeze_ctrl_state = False
    vr.tick()
    assert vr.commands and vr.increments and not vr.recording
    assert not vr.node._await_controls_release
    assert vr.node.dual_grippers.sides["left"].command == 0


def test_vr_records_discards_and_stops_outputs_for_home_and_lost_heartbeat(managed_vr):
    vr = managed_vr
    vr.mode("vr")
    vr.tick()
    vr.buttons.left_bButton = True
    vr.tick()
    vr.tick()
    assert vr.recording == ["record"]
    vr.buttons.left_bButton = False
    vr.tick()
    vr.buttons.left_bButton = True
    vr.tick()
    vr.buttons.left_aButton = True
    vr.tick()
    assert vr.recording == ["record", "record"]
    assert len(vr.requests) == 1
    vr.requests[0].set_result(NS(success=True, message="discarded"))
    counts = len(vr.commands), len(vr.increments)
    vr.mode("home")
    vr.tick()
    vr.mode("policy")
    vr.tick()
    assert counts == (len(vr.commands), len(vr.increments))
    vr.mode("vr")
    vr.tick()
    counts = len(vr.commands), len(vr.increments)
    vr.clock[0] += .61
    vr.tick()
    assert counts == (len(vr.commands), len(vr.increments))
    assert vr.node._await_controls_release and vr.node.first_frame


def test_missing_takeover_service_does_not_enable_vr(managed_vr):
    vr = managed_vr
    vr.node.intervention_client.service_is_ready = lambda: False
    vr.mode("policy")
    vr.buttons.left_aButton = True
    vr.tick()
    assert not vr.requests and not vr.commands and not vr.increments
    assert vr.logs


def test_discard_during_pending_record_request_cannot_resume_policy(managed_vr):
    vr = managed_vr
    vr.mode("vr")
    vr.tick()
    vr.node._record_request_pending = True
    vr.buttons.left_aButton = True
    vr.tick()
    assert not vr.requests
    vr.node._record_request_pending = False
    vr.tick()
    assert not vr.requests, "Held discard must not become a second action after the save finishes"
    vr.buttons.left_aButton = False
    vr.tick()
    vr.buttons.left_aButton = True
    vr.tick()
    assert len(vr.requests) == 1


def test_make_vr_intervene_starts_managed_vr_only():
    result = subprocess.run(["make", "-n", "vr-intervene"], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0
    assert "--policy-intervention" in result.stdout
    assert "vr_pose_increment_publisher.py" in result.stdout
    assert "collection/record.py" not in result.stdout
