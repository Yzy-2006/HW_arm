"""Launch gesture input for isolated RViz simulation or the hardware adapter."""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]


def perception_environment():
    environment = os.environ.copy()
    for key in list(environment):
        if key.startswith(("CONDA", "QT_")) or key in (
                "PYTHONPATH", "PYTHONHOME", "LD_LIBRARY_PATH", "LD_PRELOAD", "VIRTUAL_ENV",
                "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH"):
            environment.pop(key, None)
    environment["PYTHONNOUSERSITE"] = "1"
    return environment


def perception_options(extra, gesture_root):
    probe = argparse.ArgumentParser(add_help=False)
    probe.add_argument("--mode", choices=("rgbd", "rgb", "compare"))
    probe.add_argument("--calibration", type=Path,
                       default=gesture_root / "calibration/hand_geometry.json")
    selected, _ = probe.parse_known_args(extra)
    mode = selected.mode or ("compare" if selected.calibration.is_file() else "rgbd")
    return mode, [*extra, "--mode", mode]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gesture-root", type=Path, default=ROOT / "gesture_teleoperation")
    parser.add_argument("--perception-python", type=Path, default=Path.home() / "anaconda3/envs/mediapipe/bin/python")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--robot", action="store_true", help="Use hardware VR topics; does not start robot controllers")
    parser.add_argument("--dry-run", action="store_true", help="Hardware diagnostics without commands or service calls")
    parser.add_argument("--recording-root", type=Path)
    parser.add_argument("--port", type=int)
    parser.add_argument("--rgb-port", type=int, help="Optional second UDP port for calibrated PnP")
    parser.add_argument("--domain-id", type=int)
    parser.add_argument("--no-camera", action="store_true")
    parser.add_argument("--no-rviz", action="store_true")
    parser.add_argument("--duration", type=float, default=0)
    parser.add_argument("perception_args", nargs=argparse.REMAINDER, help="Detector options after --")
    args = parser.parse_args()
    if args.dry_run and not args.robot:
        parser.error("--dry-run requires --robot")
    if args.robot and args.recording_root is not None:
        parser.error("Hardware episodes are configured through make record, not --recording-root")
    args.config = args.config or ROOT / ("config/gesture_robot.yaml" if args.robot else "config/gesture_rviz.yaml")
    args.port = args.port if args.port is not None else (9874 if args.robot else 9872)
    if args.domain_id is None:
        args.domain_id = int(os.environ.get("ROS_DOMAIN_ID", "0")) if args.robot else 84
    if not 1 <= args.port <= 65535 or not 0 <= args.domain_id <= 232 or args.duration < 0:
        parser.error("Invalid port, domain ID or duration")
    detector = args.gesture_root / "scripts/track_hands.py"
    if not args.no_camera and (not detector.is_file() or not args.perception_python.is_file()):
        parser.error("Detector or perception Python missing; set --gesture-root / --perception-python")
    if not args.robot and not args.no_rviz and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        parser.error("RViz requires a desktop; use --no-rviz -- --no-display for headless checks")
    extra = args.perception_args
    if extra and extra[0] == "--":
        extra = extra[1:]
    if any(value.split("=")[0] in ("--udp-port", "--comparison-udp-port", "--calibrate") for value in extra):
        parser.error("UDP routing and calibration are managed separately from this launcher")
    mode, extra = perception_options(extra, args.gesture_root) if not args.no_camera else ("external", extra)
    rgb_port = args.rgb_port
    if mode == "compare" and rgb_port is None:
        rgb_port = args.port + 1
    if rgb_port is not None and (not 1 <= rgb_port <= 65535 or rgb_port == args.port):
        parser.error("RGB port must be in 1-65535 and differ from the primary UDP port")
    if rgb_port is not None and mode not in ("compare", "external"):
        parser.error("--rgb-port requires compare mode or --no-camera")
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(args.domain_id)
    if not args.robot:
        environment["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    environment["TOWEL_ROOT"] = str(ROOT)
    environment.setdefault("ROS_LOG_DIR", str(ROOT / ".ros/gesture_rviz"))
    node_script = "teleop/gesture_robot.py" if args.robot else "simulation/gesture_rviz.py"
    node = [sys.executable, str(ROOT / node_script), "--port", str(args.port),
            "--config", str(args.config.resolve())]
    if args.dry_run:
        node.append("--dry-run")
    if args.recording_root:
        node += ["--recording-root", str(args.recording_root.resolve())]
    if rgb_port is not None:
        node += ["--rgb-port", str(rgb_port)]
    viewer = ["ros2", "launch", str(ROOT / "ros2_ws/src/towel_bringup/launch/model_viewer.launch.py"),
              "gui:=false", "joint_publisher:=false", f"rviz:={str(not args.no_rviz).lower()}",
              f"rviz_config:={ROOT / 'config/gesture_rviz.rviz'}"]
    commands = [("Hardware input" if args.robot else "Simulation", node, environment)]
    if not args.robot:
        commands.append(("Model/RViz", viewer, environment))
    if not args.no_camera:
        if rgb_port is not None:
            extra += ["--comparison-udp-port", str(rgb_port)]
        commands.append(("Perception", [str(args.perception_python), str(detector), *extra,
                                         "--udp-port", str(args.port)], perception_environment()))
    processes = []

    def interrupted(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    try:
        for name, command, env in commands:
            process = subprocess.Popen(command, cwd=ROOT, env=env, start_new_session=True)
            processes.append((name, process))
            # Fail before opening the camera if the UDP receiver/model cannot start.
            time.sleep(0.8)
            for running_name, running in processes:
                if running.poll() is not None:
                    raise RuntimeError(f"{running_name} exited during startup ({running.returncode})")
        backend = "hardware DRY RUN" if args.dry_run else "hardware LIVE" if args.robot else "RViz simulation"
        print(f"Gesture {backend}: ROS domain {args.domain_id}, UDP {args.port}, pose mode {mode}. Ctrl+C stops all processes.", flush=True)
        start = time.monotonic()
        while not args.duration or time.monotonic() - start < args.duration:
            for name, process in processes:
                code = process.poll()
                if code is not None:
                    if code:
                        raise RuntimeError(f"{name} exited with status {code}")
                    return
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        for _, process in reversed(processes):
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        for _, process in reversed(processes):
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
