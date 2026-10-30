import os
from pathlib import Path
import select
import signal
import socket
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "deployment"))


def read_ready(process):
    assert select.select([process.stdout], [], [], 10)[0], "worker startup timed out"
    return int(process.stdout.readline().strip())


@pytest.mark.parametrize("signum", [signal.SIGINT, signal.SIGTERM])
@pytest.mark.parametrize("ignore_signals", [False, True])
def test_launcher_waits_for_worker_exit_even_when_signals_are_ignored(signum, ignore_signals):
    handler = "signal.SIG_IGN" if ignore_signals else "lambda *args: sys.exit(0)"
    worker_code = (
        "import os, signal, sys, time\n"
        f"signal.signal(signal.SIGINT, {handler})\n"
        f"signal.signal(signal.SIGTERM, {handler})\n"
        "print(os.getpid(), flush=True)\n"
        "while True: time.sleep(1)\n"
    )
    launcher_code = (
        f"import sys; sys.path.insert(0, {str(ROOT / 'scripts')!r}); "
        "from run_policy_worker import run_supervised; "
        f"sys.exit(run_supervised({[sys.executable, '-c', worker_code]!r}, 0.2))"
    )
    process = subprocess.Popen(
        [sys.executable, "-c", launcher_code],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, start_new_session=True,
    )
    worker_pid = None
    try:
        worker_pid = read_ready(process)
        # Signal the launcher alone, as happens when a parent exits first.
        process.send_signal(signum)
        _, stderr = process.communicate(timeout=5)
        assert process.returncode == 128 + signum, stderr
        with pytest.raises(ProcessLookupError):
            os.kill(worker_pid, 0)
        if ignore_signals:
            assert "SIGKILL" in stderr
    finally:
        if worker_pid is not None:
            try:
                os.killpg(worker_pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        if process.poll() is None:
            process.kill()
        process.communicate(timeout=5)


def test_worker_rejects_occupied_port_before_loading_model(monkeypatch):
    import pi05_worker

    def unexpected_model_load(*args):
        pytest.fail("An occupied worker port must not allocate another model")

    with socket.socket() as occupied:
        occupied.bind(("127.0.0.1", 0))
        occupied.listen(1)
        port = occupied.getsockname()[1]
        monkeypatch.setattr(sys, "argv", ["worker", "--model", "/unused", "--port", str(port)])
        monkeypatch.setattr(pi05_worker, "PolicyRunner", unexpected_model_load)
        monkeypatch.setattr(pi05_worker.signal, "signal", lambda *args: None)
        assert pi05_worker.main() == 2


@pytest.mark.parametrize("signum", [signal.SIGINT, signal.SIGTERM])
@pytest.mark.parametrize("phase", ["loading", "serving"])
def test_worker_handles_shutdown_during_load_and_socket_accept(signum, phase):
    worker_code = (
        "import os, signal, sys, time\n"
        f"sys.path.insert(0, {str(ROOT / 'deployment')!r})\n"
        "import pi05_worker\n"
        "class Runner:\n"
        "    def __init__(self, *args):\n"
        f"        if {phase == 'loading'!r}:\n"
        "            print(os.getpid(), flush=True)\n"
        "            while True: time.sleep(1)\n"
        "pi05_worker.PolicyRunner = Runner\n"
        "original_listen = pi05_worker.socket.socket.listen\n"
        "def listen(self, backlog):\n"
        "    original_listen(self, backlog)\n"
        "    print(os.getpid(), flush=True)\n"
        "pi05_worker.socket.socket.listen = listen\n"
        "sys.argv = ['worker', '--model', '/unused', '--port', '0']\n"
        "sys.exit(pi05_worker.main())\n"
    )
    process = subprocess.Popen(
        [sys.executable, "-c", worker_code],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    try:
        assert read_ready(process) == process.pid
        process.send_signal(signum)
        _, stderr = process.communicate(timeout=5)
        assert process.returncode == 0, stderr
        assert "Stopping policy worker" in stderr
        assert "Traceback" not in stderr
    finally:
        if process.poll() is None:
            process.kill()
        process.communicate(timeout=5)
