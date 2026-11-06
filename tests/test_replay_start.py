import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "deployment"))
from replay_start import ReplayStartGate
from motion_safety import MotionFault


def test_gate_requires_current_ack_complete_and_sustained_arrival():
    gate = ReplayStartGate({}, .25)
    q, v = np.zeros(14), np.zeros(14)
    assert not gate.update(10., q, q, v, 10.)
    gate.command_sent(100)
    gate.command_sent(200)  # Repeated endpoint commands do not move the required marker.
    gate.observe(99, True, q, 10.1)
    assert not gate.update(10.1, q, q, v, 10.1)
    assert "acknowledged" in gate.reason
    gate.observe(100, False, q, 10.2)
    assert not gate.update(10.2, q, q, v, 10.2)
    assert "still active" in gate.reason
    gate.observe(100, True, q + .1, 10.3)
    assert not gate.update(10.3, q, q, v, 10.3)
    assert "command has not reached" in gate.reason
    gate.observe(101, True, q, 10.4)
    assert not gate.update(10.4, q, q + .1, v, 10.4)
    assert "position error" in gate.reason
    assert not gate.update(10.4, q, q, v + .1, 10.4)
    assert "feedback speed" in gate.reason
    assert not gate.update(10.4, q, q, v, 10.4)
    gate.observe(102, True, q, 10.61)
    assert gate.update(10.61, q, q, v, 10.61)


@pytest.mark.parametrize("failure", ["status", "feedback", "velocity", "position", "handoff"])
def test_gate_restarts_settle_timer_on_interrupted_readiness(failure):
    gate = ReplayStartGate({}, .25)
    q, v = np.zeros(14), np.zeros(14)
    gate.command_sent(100)
    gate.observe(100, True, q, 10.)
    assert not gate.update(10., q, q, v, 10.)
    now = 10.1
    gate.observe(101, failure != "handoff", q, 9. if failure == "status" else now)
    assert not gate.update(now, q, q + (.1 if failure == "position" else 0),
                           None if failure == "velocity" else v, 9. if failure == "feedback" else now)
    assert gate.settled_since is None
    gate.observe(102, True, q, 10.15)
    assert not gate.update(10.15, q, q, v, 10.15)
    assert not gate.update(10.25, q, q, v, 10.25)
    gate.observe(103, True, q, 10.36)
    assert gate.update(10.36, q, q, v, 10.36)


def test_missing_handoff_status_times_out_instead_of_playing():
    gate = ReplayStartGate({"settle_timeout_s": 1.}, .25)
    q = np.zeros(14)
    assert not gate.update(10., q, q, q, 10.)
    with pytest.raises(MotionFault, match="settle timeout.*status missing"):
        gate.update(11., q, q, q, 11.)


@pytest.mark.parametrize("settings", [{"settle_timeout_s": .1}, {"settle_time_s": 0},
                                     {"position_tolerance_rad": float("nan")}])
def test_invalid_gate_settings_are_rejected(settings):
    with pytest.raises(ValueError):
        ReplayStartGate(settings, .25)
