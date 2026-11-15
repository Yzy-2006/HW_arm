"""Dependency-light management of a separate GPU-owning worker process."""

from __future__ import annotations

import hmac
import logging
import os
import signal
import socket
import socketserver
import subprocess
import threading

from pi05_common import recv_packet, send_packet


LOG = logging.getLogger("worker_lifecycle")


class WorkerProcess:
    def __init__(self, command, shutdown_timeout=5.0):
        self.command = list(command)
        self.shutdown_timeout = shutdown_timeout
        self.process = None
        self.closed = False
        self.lock = threading.RLock()

    def status(self):
        with self.lock:
            code = None if self.process is None else self.process.poll()
            running = self.process is not None and code is None
            return dict(running=running, pid=self.process.pid if running else None, exit_code=code)

    def start(self):
        with self.lock:
            if self.closed:
                raise RuntimeError("Worker manager is shutting down")
            if not self.status()["running"]:
                self.process = subprocess.Popen(self.command, start_new_session=True)
                LOG.info("Worker process started: pid=%d", self.process.pid)
            return self.status()

    def stop(self):
        with self.lock:
            if self.process is not None:
                process = self.process
                if process.poll() is None:
                    LOG.info("Stopping GPU worker pid=%d", process.pid)
                    try:
                        os.killpg(process.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    try:
                        process.wait(timeout=self.shutdown_timeout)
                    except subprocess.TimeoutExpired:
                        LOG.warning("Forcing GPU worker pid=%d to exit with SIGKILL", process.pid)
                        try:
                            os.killpg(process.pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                process.wait()
                LOG.info("Worker process reaped: pid=%d; GPU context owner has exited", process.pid)
            return self.status()

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            self.stop()


class WorkerManagementServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address, worker, token="", on_shutdown=None):
        self.worker = worker
        self.token = token
        self.on_shutdown = on_shutdown
        super().__init__(address, WorkerManagementHandler)


class WorkerManagementHandler(socketserver.BaseRequestHandler):
    def handle(self):
        self.request.settimeout(5.0)
        try:
            message = recv_packet(self.request)
            if self.server.token and not hmac.compare_digest(str(message.get("token", "")), self.server.token):
                send_packet(self.request, dict(ok=False, error="invalid token"))
                return
            action = message.get("type")
            if action == "shutdown":
                self.server.worker.close()
                send_packet(self.request, dict(ok=True, **self.server.worker.status()))
                if self.server.on_shutdown is not None:
                    self.server.on_shutdown()
                return
            if action not in ("status", "start", "stop"):
                raise ValueError("Expected worker management action: status/start/stop/shutdown")
            result = getattr(self.server.worker, action)()
            send_packet(self.request, dict(ok=True, **result))
        except (OSError, ConnectionError, ValueError, RuntimeError) as exc:
            try:
                send_packet(self.request, dict(ok=False, error=str(exc)))
            except OSError:
                pass


class WorkerLifecycleClient:
    def __init__(self, host, port, token="", timeout_s=15.0):
        self.address = host, port
        self.token = token
        self.timeout_s = timeout_s

    def request(self, action):
        with socket.create_connection(self.address, timeout=self.timeout_s) as connection:
            connection.settimeout(self.timeout_s)
            send_packet(connection, dict(type=action, token=self.token))
            result = recv_packet(connection)
        if not result.get("ok"):
            raise RuntimeError(result.get("error", "Worker management request failed"))
        return result
