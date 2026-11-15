"""Terminal input and asynchronous policy pause/home/resume coordination."""

import os
import select
import sys
import termios


class TerminalKeys:
    def __init__(self, fd=None):
        self.fd = fd
        self.original = None
        self.escape = 0
        try:
            if self.fd is None:
                self.fd = sys.stdin.fileno()
            if not os.isatty(self.fd):
                return
            self.original = termios.tcgetattr(self.fd)
            settings = termios.tcgetattr(self.fd)
            settings[3] &= ~(termios.ICANON | termios.ECHO)
            settings[6][termios.VMIN] = 0
            settings[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, settings)
        except (OSError, ValueError, termios.error):
            self.original = None

    @property
    def enabled(self):
        return self.original is not None

    def poll(self):
        if not self.enabled:
            return None
        for _ in range(64):
            if not select.select([self.fd], [], [], 0)[0]:
                return None
            value = os.read(self.fd, 1)
            if not value:
                return None
            key = value.decode("ascii", errors="ignore")
            if key == "\x1b":
                self.escape = 1
            elif self.escape == 1:
                self.escape = 2 if key in ("[", "O") else 0
            elif self.escape == 2:
                if "@" <= key <= "~":
                    self.escape = 0
            elif key:
                return key
        return None

    def close(self):
        if self.enabled:
            try:
                termios.tcsetattr(self.fd, termios.TCSANOW, self.original)
            except (OSError, termios.error):
                pass  # The terminal may already have been closed during shutdown.
            finally:
                self.original = None


class PolicyHomeSession:
    def __init__(self, request_home, request_resume, now, log, timeout_s=30.0):
        self.request_home = request_home
        self.request_resume = request_resume
        self.now = now
        self.log = log
        self.timeout_s = timeout_s
        self.phase = "idle"
        self.pending = None
        self.started_at = self.next_poll_at = 0.0

    @property
    def active(self):
        return self.phase != "idle"

    def begin(self):
        if self.phase not in ("idle", "failed"):
            return
        self.phase = "requesting"
        self.started_at = self.now()
        self.log("H: model arm/gripper commands paused; requesting home")
        try:
            self.pending = self.request_home()
        except Exception as exc:
            self.fail(str(exc))

    def fail(self, reason):
        if self.pending is not None:
            self.pending.cancel()
            self.pending = None
        self.phase = "failed"
        self.log(f"Policy home stopped: {reason}; model remains paused")

    def tick(self):
        if self.phase in ("idle", "failed"):
            return False
        if self.now() - self.started_at >= self.timeout_s:
            self.fail("timed out waiting for home trajectory timing and resume")
            return False
        try:
            if self.pending is not None:
                if not self.pending.done():
                    return False
                response = self.pending.result()
                self.pending = None
                if response is None:
                    raise RuntimeError("empty home service response")
                if self.phase == "requesting":
                    if not response.success:
                        raise RuntimeError(response.message)
                    self.log("Home accepted; waiting for trajectory timing and the robot's resume delay")
                elif response.success:
                    self.phase = "idle"
                    self.log("Home trajectory timing and delay elapsed; controller handoff acknowledged")
                    return True
                self.phase = "waiting"
                self.next_poll_at = self.now() + 0.25
            if self.phase == "waiting" and self.now() >= self.next_poll_at:
                self.pending = self.request_resume()
                self.phase = "resuming"
        except Exception as exc:
            self.fail(str(exc))
        return False

    def close(self):
        if self.pending is not None:
            self.pending.cancel()
