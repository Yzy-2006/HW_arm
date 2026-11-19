from concurrent.futures import Future
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from deployment.voice_control import MotionProcess, VoiceSession, match_command


@pytest.mark.parametrize("text,command", [
    ("毛巾", "start"), ("开始叠毛巾。", "start"), ("请停止！", "stop"),
    ("停止叠毛巾", "stop"), ("你好", None),
])
def test_matching(text, command):
    assert match_command(text) == command


def session_fixture():
    events, homes, clock = [], [], [10.0]

    class Motion:
        running = False

        def start(self):
            self.running = True
            events.append("start")

        def stop(self):
            self.running = False
            events.append("stop")

    def home():
        events.append("home")
        future = Future()
        homes.append(future)
        return future

    session = VoiceSession(Motion(), home, lambda text: None, lambda: clock[0])
    return session, events, homes, clock


def test_stop_before_home_waits_for_fresh_start_and_drops_buffered_commands():
    session, events, homes, clock = session_fixture()
    session.handle("毛巾", 10.)
    session.handle("毛巾", 10.)
    assert events == ["start"]
    session.handle("停止毛巾", 10.)
    assert events == ["start", "stop", "home"]
    clock[0] = 11.
    session.handle("毛巾", 11.)
    assert len(events) == 3
    homes[0].set_result(SimpleNamespace(success=True))
    session.tick()
    assert not session.motion.running
    session.handle("毛巾", 10.5)
    assert len(events) == 3
    clock[0] = 12.
    session.handle("毛巾", 12.)
    assert events[-1] == "start"


@pytest.mark.parametrize("failure", ["rejected", "timeout", "exception"])
def test_home_failure_blocks_start_until_successful_stop_retry(failure):
    session, events, homes, clock = session_fixture()
    session.handle("停止", 10.)
    if failure == "timeout":
        clock[0] = 16.
    elif failure == "exception":
        homes[0].set_exception(RuntimeError("lost service"))
    else:
        homes[0].set_result(SimpleNamespace(success=False, message="not ready"))
    session.tick()
    clock[0] += 1
    session.handle("毛巾", clock[0])
    assert "start" not in events
    session.handle("停止", clock[0])
    assert len(homes) == 2
    homes[1].set_result(SimpleNamespace(success=True))
    session.tick()
    clock[0] += 1
    session.handle("毛巾", clock[0])
    assert events[-1] == "start"


def test_stale_speech_cannot_start_motion():
    session, events, _, clock = session_fixture()
    session.handle("毛巾", clock[0] - 6)
    assert not events


def test_stop_cleans_child_even_after_make_like_parent_exits(tmp_path):
    pid_file = tmp_path / "child.pid"
    child_code = "import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); time.sleep(60)"
    parent_code = (
        "import subprocess,sys,time; "
        f"p=subprocess.Popen([sys.executable,'-c',{child_code!r}]); "
        f"open({str(pid_file)!r},'w').write(str(p.pid)); time.sleep(60)"
    )
    motion = MotionProcess([sys.executable, "-c", parent_code])
    try:
        motion.start()
        deadline = time.monotonic() + 5
        while not pid_file.exists() and time.monotonic() < deadline:
            time.sleep(.02)
        pid = int(pid_file.read_text())
        time.sleep(.1)
        motion.stop()
        assert motion.process is None
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            stat = Path(f"/proc/{pid}/stat")
            if not stat.exists() or stat.read_text().split()[2] == "Z":
                break
            time.sleep(.02)
        else:
            pytest.fail("motion child survived stop")
    finally:
        motion.stop()


def test_text_dry_run_needs_no_ros_or_microphone():
    result = subprocess.run([
        sys.executable, str(ROOT / "scripts/run_voice_control.py"), "--dry-run", "--stdin",
    ], input="毛巾\n停止\n", text=True, capture_output=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert "Started make motion" in result.stdout
    assert "DRY RUN: /dexbotCtl/keyboard_home" in result.stdout
