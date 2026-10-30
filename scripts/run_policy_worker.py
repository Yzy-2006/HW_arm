#!/usr/bin/env python3
"""Activate Conda and supervise a policy worker until its process exits."""

from __future__ import annotations

import argparse
import logging
import os
from pathlib import Path
import signal
import subprocess


LOG = logging.getLogger("policy_worker_launcher")
ROOT = Path(__file__).resolve().parents[1]


def run_supervised(command: list[str], shutdown_timeout: float = 5.0) -> int:
    requested_signal = None
    process = None

    def request_shutdown(signum, _frame):
        nonlocal requested_signal
        if requested_signal is None:
            requested_signal = signum

    previous_handlers = {
        signum: signal.signal(signum, request_shutdown)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        # A private process group lets us stop the worker and any of its children
        # without signaling make, the terminal, or unrelated robot nodes.
        process = subprocess.Popen(command, start_new_session=True)
        LOG.info("Worker process started: pid=%d", process.pid)
        while requested_signal is None:
            try:
                returncode = process.wait(timeout=0.2)
            except subprocess.TimeoutExpired:
                continue
            LOG.info("Worker process exited: pid=%d status=%d", process.pid, returncode)
            return returncode if returncode >= 0 else 128 - returncode

        for signum in (requested_signal, signal.SIGTERM):
            LOG.info("Sending %s to worker pid=%d", signal.Signals(signum).name, process.pid)
            try:
                os.killpg(process.pid, signum)
            except ProcessLookupError:
                break
            try:
                process.wait(timeout=shutdown_timeout)
                break
            except subprocess.TimeoutExpired:
                LOG.warning("Worker has not exited after %.1f seconds", shutdown_timeout)
        return 128 + requested_signal
    finally:
        if process is not None:
            if process.poll() is None:
                LOG.warning("Forcing worker pid=%d to exit with SIGKILL", process.pid)
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()
            LOG.info("Worker process reaped: pid=%d; GPU context owner has exited", process.pid)
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--env", required=True, help="Conda environment name")
    parser.add_argument("--shutdown-timeout", type=float, default=5.0)
    parser.add_argument("worker_args", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.shutdown_timeout <= 0:
        parser.error("--shutdown-timeout must be greater than zero")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    worker_args = args.worker_args
    if worker_args[:1] == ["--"]:
        worker_args = worker_args[1:]
    try:
        conda_base = subprocess.check_output(["conda", "info", "--base"], text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        LOG.exception("Cannot locate Conda; initialize it before starting the worker")
        return 2
    # Exec preserves this PID after activation and avoids conda run's extra
    # wrapper process. An absolute interpreter also avoids an active robot venv.
    command = [
        "bash", "-c",
        'set -e\nsource "$1/etc/profile.d/conda.sh"\nconda activate "$2"\n'
        'exec "$CONDA_PREFIX/bin/python" "$3" "${@:4}"',
        "policy-worker", conda_base, args.env,
        str(ROOT / "deployment/pi05_worker.py"), *worker_args,
    ]
    return run_supervised(command, args.shutdown_timeout)


if __name__ == "__main__":
    raise SystemExit(main())
