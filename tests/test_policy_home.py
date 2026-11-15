import os
from pathlib import Path
import pty
import subprocess
import sys
import termios
from concurrent.futures import Future
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "deployment"))
from policy_home import PolicyHomeSession, TerminalKeys


def test_terminal_home_key_is_immediate_ignores_escape_and_restores_settings():
    master, slave = pty.openpty()
    original = termios.tcgetattr(slave)
    keys = TerminalKeys(slave)
    try:
        assert keys.enabled
        assert termios.tcgetattr(slave)[3] & termios.ISIG
        assert keys.poll() is None
        os.write(master, b"\x1b[H")
        assert keys.poll() is None
        os.write(master, b"h")
        assert keys.poll() == "h"
        os.write(master, b"H")
        assert keys.poll() == "H"
    finally:
        keys.close()
        assert termios.tcgetattr(slave) == original
        os.close(master)
        os.close(slave)


def test_terminal_errors_disable_keyboard_without_blocking_cleanup(monkeypatch):
    monkeypatch.setattr(os, "isatty", lambda fd: True)

    def unavailable(fd):
        raise termios.error("terminal unavailable")

    monkeypatch.setattr(termios, "tcgetattr", unavailable)
    keys = TerminalKeys(0)
    assert not keys.enabled and keys.poll() is None
    keys.close()


@pytest.mark.parametrize("result", ["success", "rejected", "timeout"])
def test_home_session_waits_for_controller_confirmation_and_never_auto_resumes_on_failure(result):
    clock, logs, starts, resumes = [10.], [], [], []

    def request(calls):
        future = Future()
        calls.append(future)
        return future

    session = PolicyHomeSession(lambda: request(starts), lambda: request(resumes),
                                lambda: clock[0], logs.append)
    session.begin()
    session.begin()
    assert len(starts) == 1 and session.active
    starts[-1].set_result(SimpleNamespace(success=result != "rejected", message="rejected"))
    session.tick()
    if result == "rejected":
        assert session.phase == "failed" and not resumes
        return
    clock[0] += .3
    session.tick()
    assert len(resumes) == 1
    resumes[-1].set_result(SimpleNamespace(success=False, message="home still pending"))
    session.tick()
    assert session.active
    clock[0] += .3
    session.tick()
    if result == "timeout":
        clock[0] += 30
        assert not session.tick() and session.phase == "failed"
        assert resumes[-1].cancelled()
    else:
        resumes[-1].set_result(SimpleNamespace(success=True))
        assert session.tick() and not session.active


def test_native_home_resume_requires_finished_trajectory_timing_and_three_seconds(tmp_path):
    executable = tmp_path / "policy_home_guard"
    subprocess.run([
        "g++", "-std=c++17", "-I", str(ROOT / "ros2_ws/src/robot_control_hw/include"),
        str(ROOT / "tests/policy_home_guard_test.cpp"), "-lyaml-cpp", "-o", str(executable),
    ], check=True, capture_output=True, text=True)
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)
