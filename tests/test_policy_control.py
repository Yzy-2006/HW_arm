import os
import runpy
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("outcome", ["success", "rejected", "unavailable", "timeout"])
def test_policy_handoff_requires_success_and_cleans_up(monkeypatch, capsys, outcome):
    events = []
    response = SimpleNamespace(success=outcome == "success", message="test response")
    future = SimpleNamespace(
        done=lambda: outcome != "timeout", result=lambda: response,
        cancel=lambda: events.append("cancel"),
    )

    class Client:
        def wait_for_service(self, timeout_sec):
            assert timeout_sec == 5.0
            return outcome != "unavailable"

        def call_async(self, request):
            assert request.data is False
            events.append("request")
            return future

    class Node:
        def create_client(self, service_type, service_name):
            assert service_name == "/dexbotCtl/vr_override"
            return Client()

        def destroy_node(self):
            events.append("destroy")

    monkeypatch.setitem(sys.modules, "rclpy", SimpleNamespace(
        init=lambda: None, create_node=lambda name: Node(),
        spin_until_future_complete=lambda node, future, timeout_sec: None,
        ok=lambda: True, shutdown=lambda: events.append("shutdown"),
    ))
    monkeypatch.setitem(sys.modules, "std_srvs.srv", SimpleNamespace(
        SetBool=SimpleNamespace(Request=SimpleNamespace),
    ))
    main = runpy.run_path(str(ROOT / "scripts/request_policy_control.py"))["main"]
    assert main() == (0 if outcome == "success" else 2)
    assert events[-2:] == ["destroy", "shutdown"]
    assert ("request" in events) == (outcome != "unavailable")
    assert ("cancel" in events) == (outcome == "timeout")
    captured = capsys.readouterr()
    if outcome == "success":
        assert "Policy control granted" in captured.out
    else:
        assert "ERROR:" in captured.err


@pytest.mark.parametrize("handoff_status", [0, 2])
def test_make_motion_only_starts_bridge_after_handoff_succeeds(tmp_path, handoff_status):
    venv = tmp_path / ".venv-robot"
    (venv / "bin").mkdir(parents=True)
    (venv / "bin/activate").write_text("")
    (tmp_path / "scripts").mkdir()
    (tmp_path / "scripts/env.sh").write_text("")
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    fake_python = fake_bin / "python3"
    fake_python.write_text(
        "#!/usr/bin/env bash\n"
        'printf "%s\\n" "$1" >> "$TEST_CALLS"\n'
        'if [[ "$1" == */request_policy_control.py ]]; then exit "$TEST_HANDOFF_STATUS"; fi\n'
        "exit 0\n"
    )
    fake_python.chmod(0o755)
    calls = tmp_path / "calls"
    environment = dict(os.environ, PATH=f"{fake_bin}:{os.environ['PATH']}",
                       TEST_CALLS=str(calls), TEST_HANDOFF_STATUS=str(handoff_status))
    result = subprocess.run([
        "make", "-f", str(ROOT / "Makefile"), f"ROOT={tmp_path}",
        f"VENV={venv}", "motion",
    ], env=environment, capture_output=True, text=True, timeout=10)
    assert (result.returncode == 0) == (handoff_status == 0), result.stderr
    expected = [str(tmp_path / "scripts/request_policy_control.py")]
    if handoff_status == 0:
        expected.append(str(tmp_path / "deployment/pi05_bridge.py"))
    assert calls.read_text().splitlines() == expected
