import sys
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "deployment"))
from intervention import Intervention


@pytest.fixture
def handoff():
    calls = []
    clock = [0.0]

    def call(service, value):
        future = Future()
        calls.append((service, value, future))
        return future

    manager = Intervention(lambda: calls.append(("pause",)), lambda: calls.append(("resume",)),
                           lambda v: call("controller", v), lambda v: call("recorder", v),
                           lambda text: None, now=lambda: clock[0])
    return manager, calls, clock


def ack(manager, calls, success=True):
    calls[-1][2].set_result(SimpleNamespace(success=success, message="ack"))
    manager.tick()


def test_takeover_waits_for_controller_and_recording_then_save_before_resume(handoff):
    manager, calls, clock = handoff
    manager.mode = "policy"
    assert manager.request(True)[0]
    assert calls[0] == ("pause",)
    assert calls[-1][:2] == ("controller", True)
    assert manager.mode == "arming_vr"
    assert not manager.request(False)[0]
    ack(manager, calls)
    assert calls[-1][:2] == ("recorder", True)
    assert manager.mode == "arming_vr"
    ack(manager, calls)
    assert manager.mode == "vr"
    assert manager.request(False)[0]
    assert calls[-1][:2] == ("controller", False)
    assert manager.mode == "saving"
    ack(manager, calls)
    assert calls[-1][:2] == ("recorder", False)
    assert manager.mode == "saving"
    ack(manager, calls)
    assert manager.mode == "policy"
    assert calls[-1] == ("resume",)


@pytest.mark.parametrize("stage", ["controller", "recording", "vr", "saving"])
def test_safety_fault_blocks_handoff_and_late_ack(handoff, stage):
    manager, calls, clock = handoff
    manager.request(True)
    if stage != "controller":
        ack(manager, calls)
    if stage in {"vr", "saving"}:
        ack(manager, calls)
    if stage == "saving":
        manager.request(False)
    manager.latch_fault("overspeed")
    assert not manager.request(True)[0]
    assert not manager.request(False)[0]
    if manager.pending is not None:
        manager.pending.set_result(SimpleNamespace(success=True, message="late ack"))
    manager.tick()
    assert manager.mode == "fault"
    assert ("resume",) not in calls
    if stage != "controller":
        assert calls[-1][:2] == ("recorder", False)
        ack(manager, calls)
        assert not manager.recording
    assert manager.mode == "fault"


@pytest.mark.parametrize("stage", ["controller", "recorder", "save", "resume"])
def test_failed_ack_never_enables_policy_or_vr(handoff, stage):
    manager, calls, clock = handoff
    manager.request(True)
    if stage != "controller":
        ack(manager, calls)
    if stage in {"save", "resume"}:
        ack(manager, calls)
        manager.request(False)
    if stage == "resume":
        ack(manager, calls)
    ack(manager, calls, success=False)
    assert manager.mode == "paused"
    assert ("resume",) not in calls


def test_late_ack_after_timeout_cannot_grant_ownership(handoff):
    manager, calls, clock = handoff
    manager.request(True)
    clock[0] = 6.0
    manager.tick()
    assert manager.mode == "paused"
    assert not manager.request(False)[0]
    ack(manager, calls)
    assert manager.mode == "paused"
    assert len(calls) == 2


def test_missing_controller_still_pauses_immediately(handoff):
    manager, calls, _ = handoff
    manager.mode = "policy"
    manager.controller = lambda value: (_ for _ in ()).throw(RuntimeError("unavailable"))
    assert not manager.request(True)[0]
    assert calls == [("pause",)]
    assert manager.mode == "paused"


def test_save_waits_for_writer_ack_beyond_controller_timeout(handoff):
    manager, calls, clock = handoff
    manager.mode = 'vr'
    manager.recording = True
    manager.request(False)
    ack(manager, calls)
    assert manager.stage == 'saved'
    clock[0] = 6.
    manager.tick()
    assert manager.mode == 'saving' and ('resume',) not in calls
    ack(manager, calls)
    assert manager.mode == 'policy' and calls[-1] == ('resume',)


@pytest.mark.parametrize('outcome', ['ready', 'failed', 'timeout'])
def test_vr_waits_for_first_captured_frame(handoff, outcome):
    manager, calls, clock = handoff
    ready = [False]
    manager.recorder_ready = lambda: ready[0]
    manager.request(True)
    ack(manager, calls)
    ack(manager, calls)
    manager.tick()
    assert manager.mode == 'arming_vr' and manager.pending is None
    assert not manager.request(False)[0]
    if outcome == 'ready':
        ready[0] = True
    elif outcome == 'failed':
        manager.recorder_ready = lambda: (_ for _ in ()).throw(RuntimeError('Missing streams: tcp'))
    else:
        clock[0] += manager.startup_timeout_s + .1
    manager.tick()
    assert manager.mode == ('vr' if outcome == 'ready' else 'paused')
    if outcome == 'failed':
        assert 'tcp' in manager.error
    assert ('resume',) not in calls
