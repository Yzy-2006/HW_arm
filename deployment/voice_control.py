"""Voice command matching, owned motion process, and explicit home handoff."""
from concurrent.futures import Future
import argparse
import fcntl
import os
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time
import unicodedata

ROOT = Path(__file__).resolve().parents[1]


def match_command(text):
    text = unicodedata.normalize("NFKC", text)
    # A sentence containing both commands must never start motion.
    if "停止" in text:
        return "stop"
    if "毛巾" in text:
        return "start"
    return None


class MotionProcess:
    def __init__(self, command):
        self.command = command
        self.process = None

    @property
    def running(self):
        return self.process is not None and self.process.poll() is None

    def start(self):
        if self.running:
            return
        self.stop()
        self.process = subprocess.Popen(self.command, cwd=ROOT, stdin=subprocess.DEVNULL,
                                        start_new_session=True)

    def stop(self):
        process = self.process
        if process is None:
            return

        def send(sig):
            try:
                os.killpg(process.pid, sig)
                return True
            except ProcessLookupError:
                return False

        # make can exit before its bridge child. Track the whole process group.
        send(signal.SIGINT)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            process.poll()
            if not send(0):
                break
            time.sleep(0.05)
        send(signal.SIGKILL)
        process.wait(timeout=2.0)
        self.process = None


class VoiceSession:
    def __init__(self, motion, request_home, log=print, now=time.monotonic):
        self.motion, self.request_home, self.log, self.now = motion, request_home, log, now
        self.pending = None
        self.home_started = 0.0
        self.barrier = -float("inf")
        self.blocked = False

    def handle(self, text, captured_at):
        if captured_at <= self.barrier or self.now() - captured_at > 5.0:
            self.log(f"Ignored stale speech: {text}")
            return
        command = match_command(text)
        if command == "stop":
            if self.pending is not None:
                return
            self.blocked = True
            self.motion.stop()
            self.barrier = self.now()
            try:
                self.pending = self.request_home()
                self.home_started = self.now()
                self.log("Motion stopped; requesting robot H home")
            except Exception as exc:
                self.log(f"Home failed; start blocked, say 停止 to retry: {exc}")
        elif command == "start":
            if self.blocked or self.pending is not None:
                self.log("Start ignored: home request unresolved; say 停止 to retry on failure")
            elif self.motion.running:
                self.log("Motion already running")
            else:
                self.motion.start()
                self.log("Started make motion")

    def tick(self):
        if self.pending is None:
            return
        try:
            if not self.pending.done():
                if self.now() - self.home_started < 5.0:
                    return
                raise TimeoutError("robot home service timed out; outcome unknown")
            response = self.pending.result()
            if response is None or not response.success:
                raise RuntimeError(response.message if response else "empty response")
            self.blocked = False
            self.log("Home accepted; policy remains blocked until the next 毛巾 command")
        except Exception as exc:
            self.log(f"Home failed; start blocked, say 停止 to retry: {exc}")
        finally:
            if self.pending is not None and (self.pending.done() or self.now() - self.home_started >= 5.0):
                self.pending.cancel()
                self.pending = None
                self.barrier = self.now()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/voice_control.yaml")
    parser.add_argument("--robot-venv", default=str(ROOT / ".venv-robot"))
    parser.add_argument("--policy-config", default=str(ROOT / "config/pi05_deployment.yaml"))
    parser.add_argument("--motion-args", default="")
    parser.add_argument("--slow-factor", default="")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--stdin", action="store_true", help="Read recognized text instead of microphone")
    parser.add_argument("--list-devices", action="store_true")
    args = parser.parse_args(argv)
    if args.list_devices:
        from deployment.voice.audio import list_devices_main
        list_devices_main()
        return 0

    # One supervisor per checkout, even when launched from different terminals.
    lock = (ROOT / ".voice-control.lock").open("a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print("Voice controller already running", flush=True)
        lock.close()
        return 2

    command = ["make", "--no-print-directory", "motion", f"VENV={args.robot_venv}",
               f"PI05_CONFIG={args.policy_config}", f"ARGS={args.motion_args}",
               f"PI05_SLOW_FACTOR={args.slow_factor}"]
    motion = MotionProcess(command)
    events = queue.Queue(maxsize=32)
    stop = threading.Event()
    node = worker = ros = None
    failed = False

    def emit(kind, value, captured_at):
        try:
            events.put_nowait((kind, value, captured_at))
        except queue.Full:
            pass  # Drop overflow instead of replaying a command backlog.

    def log(message):
        print(message, flush=True)

    try:
        if args.dry_run:
            class DryMotion:
                running = False

                def start(self):
                    self.running = True
                    log(f"DRY RUN: {command}")

                def stop(self):
                    self.running = False

            motion = DryMotion()

            def request_home():
                from types import SimpleNamespace
                future = Future()
                future.set_result(SimpleNamespace(success=True))
                log("DRY RUN: /dexbotCtl/keyboard_home")
                return future
        else:
            import rclpy as ros
            from std_srvs.srv import Trigger
            ros.init()
            node = ros.create_node("towel_voice_control")
            client = node.create_client(Trigger, "/dexbotCtl/keyboard_home")
            if not client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError("/dexbotCtl/keyboard_home unavailable; rebuild and restart make robot")

            def request_home():
                if not client.service_is_ready():
                    raise RuntimeError("robot home service unavailable")
                return client.call_async(Trigger.Request())

        session = VoiceSession(motion, request_home, log)
        if args.stdin:
            def read_text():
                import sys
                for line in sys.stdin:
                    if stop.is_set():
                        break
                    emit("transcript", line.strip(), time.monotonic())
                emit("eof", "", time.monotonic())
            target = read_text
        else:
            import yaml
            from deployment.voice.audio import AudioSettings, run_microphone
            with args.config.open(encoding="utf-8") as stream:
                settings = AudioSettings.from_dict(yaml.safe_load(stream)["audio"])
            target = lambda: run_microphone(settings, stop, emit)
        worker = threading.Thread(target=target, daemon=True)
        worker.start()
        log("Voice ready: 毛巾 / 停止" + (" (DRY RUN)" if args.dry_run else ""))
        while ros is None or ros.ok():
            if ros is not None:
                ros.spin_once(node, timeout_sec=0.05)
            session.tick()
            try:
                kind, value, captured_at = events.get(timeout=0.05)
            except queue.Empty:
                continue
            if kind == "eof":
                break
            if kind == "error":
                raise RuntimeError(value)
            if kind == "transcript":
                log(f"ASR: {value}")
                session.handle(value, captured_at)
            else:
                log(f"{kind}: {value}")
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        log(f"Voice controller failed: {exc}")
        failed = True
    finally:
        stop.set()
        motion.stop()
        if worker is not None and not args.stdin:
            worker.join(timeout=2.0)
        if node is not None:
            node.destroy_node()
        if ros is not None and ros.ok():
            ros.shutdown()
        lock.close()
    return 2 if failed else 0
