"""Launch RGB-D or calibrated RGB perception, the ROS 2 bridge, and RViz."""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]


def ros_environment(setup):
    # Keep Conda's Python/Qt/native libraries out of system ROS processes.
    environment = os.environ.copy()
    for key in list(environment):
        if key.startswith(("CONDA", "QT_")) or key in (
            "PYTHONPATH", "PYTHONHOME", "LD_LIBRARY_PATH", "LD_PRELOAD", "AMENT_PREFIX_PATH",
            "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH",
        ):
            environment.pop(key, None)
    environment["PATH"] = "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
    environment["PYTHONNOUSERSITE"] = "1"
    result = subprocess.run(
        ["/bin/bash", "-c", 'source "$1" && env -0', "ros-env", str(setup)],
        env=environment, check=True, capture_output=True,
    )
    return {
        key.decode(): value.decode()
        for entry in result.stdout.split(b"\0") if b"=" in entry
        for key, value in (entry.split(b"=", 1),)
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ros-setup", type=Path, default=Path("/opt/ros/jazzy/setup.bash"))
    parser.add_argument("--python", type=Path, default=Path(sys.executable),
                        help="Python executable from the mediapipe Conda environment")
    parser.add_argument("--port", type=int, default=9870)
    parser.add_argument("--mode", choices=("rgbd", "rgb", "compare"), default="rgbd")
    parser.add_argument("--comparison-port", type=int, help="Second bridge port; defaults to PORT + 1")
    parser.add_argument("--no-rviz", action="store_true")
    parser.add_argument("--duration", type=float, default=0, help="Optional total runtime in seconds")
    args, perception_args = parser.parse_known_args()
    if not args.ros_setup.is_file():
        parser.error(f"ROS setup missing: {args.ros_setup}")
    if not (1 <= args.port <= 65535) or args.duration < 0:
        parser.error("Invalid port or duration")
    comparison_port = args.comparison_port if args.comparison_port is not None else args.port + 1
    if args.mode == "compare" and (not 1 <= comparison_port <= 65535 or comparison_port == args.port):
        parser.error("Comparison port must be in 1-65535 and differ from the main port")
    if not args.no_rviz and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        parser.error("RViz needs a desktop session; use --no-rviz --no-display for headless execution")
    environment = ros_environment(args.ros_setup)
    processes = []

    def stop_signal(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop_signal)
    try:
        bridges = [("ROS bridge", ["--port", str(args.port)])]
        if args.mode == "compare":
            bridges = [
                ("RGB-D bridge", ["--port", str(args.port), "--namespace", "/hands/rgbd",
                                  "--tf-prefix", "rgbd_", "--style", "rgbd"]),
                ("RGB bridge", ["--port", str(comparison_port), "--namespace", "/hands/rgb",
                                "--tf-prefix", "rgb_", "--style", "rgb"]),
            ]
        for name, arguments in bridges:
            bridge = subprocess.Popen(
                ["/usr/bin/python3", str(ROOT / "scripts/ros_hand_bridge.py")] + arguments,
                env=environment, cwd=ROOT, start_new_session=True,
            )
            processes.append((name, bridge))
            time.sleep(0.6)
            if bridge.poll() is not None:
                raise RuntimeError(f"{name} failed to start; check whether the port is in use")
        if not args.no_rviz:
            config = "hands_compare.rviz" if args.mode == "compare" else "hands.rviz"
            rviz = subprocess.Popen(
                [str(args.ros_setup.parent / "bin/rviz2"), "-d", str(ROOT / "config" / config)],
                env=environment, cwd=ROOT, start_new_session=True,
            )
            processes.append(("RViz", rviz))
        perception = subprocess.Popen(
            [str(args.python), str(ROOT / "scripts/track_hands.py"), "--udp-port", str(args.port),
             "--mode", args.mode, "--comparison-udp-port", str(comparison_port if args.mode == "compare" else 9871)]
            + perception_args,
            cwd=ROOT, start_new_session=True,
        )
        processes.append(("Perception", perception))
        started = time.monotonic()
        print("Running hand tracking. Close any window or press Ctrl+C to stop all processes.", flush=True)
        if args.mode == "compare":
            print("Comparison: RGB-D cyan; RGB/PnP magenta; yellow links show pose differences.", flush=True)
        while not args.duration or time.monotonic() - started < args.duration:
            for name, process in processes:
                code = process.poll()
                if code is not None:
                    if code != 0:
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
                process.kill()
                process.wait()


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
