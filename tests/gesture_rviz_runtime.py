"""Hardware-free UDP -> gesture -> IK -> ROS -> recording integration check.

Run in the robot/ROS environment, with local sockets enabled:
    python tests/gesture_rviz_runtime.py
"""

import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time

import numpy as np
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests"))
from test_gesture_teleop import packet
from gesture_control import FIST, OPEN_PALM


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rviz", action="store_true")
    parser.add_argument("--hybrid", action="store_true", help="Exercise same-frame PnP fallback during open palm motion")
    parser.add_argument("--domain-id", type=int, default=85)
    args = parser.parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    rclpy.init()
    node = rclpy.create_node("gesture_runtime_check")
    received = {"joints": None, "status": None, "tf": False, "marker": None}
    node.create_subscription(JointState, "/model_viewer/joint_states", lambda m: received.update(joints=m), 10)
    node.create_subscription(String, "/gesture_sim/status", lambda m: received.update(status=json.loads(m.data)), 10)
    node.create_subscription(TFMessage, "/model_viewer/tf", lambda m: received.update(tf=bool(m.transforms)), 10)
    node.create_subscription(Marker, "/gesture_sim/status_marker", lambda m: received.update(marker=m), 10)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sender.bind(("127.0.0.1", 0))
    # Reserve a free receiver port before the child starts.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    rgb_port = None
    if args.hybrid:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
            reservation.bind(("127.0.0.1", 0))
            rgb_port = reservation.getsockname()[1]
    process = None
    sequence = 0

    def drive(duration, left=0, right=0, moving=False, send=True, pose_loss=False):
        nonlocal sequence
        start = time.monotonic()
        while time.monotonic() - start < duration:
            if process.poll() is not None:
                raise AssertionError(f"Launcher exited unexpectedly: {process.returncode}")
            now = time.monotonic()
            if send:
                sample = packet(now, left, right, position=(0, 0, 0.5 - (now - start) * 0.02 if moving else 0.5))
                sample["sequence"] = sequence
                sequence += 1
                if pose_loss:
                    sample["hands"] = []
                if rgb_port is not None:
                    rgb = dict(sample, hands=[dict(hand, pose_source="rgb_pnp") for hand in sample["hands"]])
                    sample["hands"] = [hand for hand in sample["hands"]
                                       if sample["gestures"][hand["label"]] != OPEN_PALM]
                    sender.sendto(json.dumps(rgb).encode(), ("127.0.0.1", rgb_port))
                sender.sendto(json.dumps(sample).encode(), ("127.0.0.1", port))
            deadline = now + 1 / 30
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=max(0, deadline - time.monotonic()))

    try:
        with tempfile.TemporaryDirectory(prefix="gesture-rviz-check-") as directory:
            command = [sys.executable, str(ROOT / "scripts/run_gesture_rviz.py"), "--no-camera",
                       "--domain-id", str(args.domain_id), "--port", str(port), "--recording-root", directory]
            if not args.rviz:
                command.append("--no-rviz")
            if rgb_port is not None:
                command += ["--rgb-port", str(rgb_port)]
            process = subprocess.Popen(command, cwd=ROOT, start_new_session=True)
            for _ in range(10):
                drive(1)
                if (received["joints"] is not None and received["tf"]
                        and received["status"] is not None and received["marker"] is not None):
                    break
            assert received["joints"] is not None and received["tf"], "No joint states / model TF"
            assert received["marker"] is not None and "SIM" in received["marker"].text
            baseline = np.array(received["joints"].position)
            drive(0.6, left=2)
            assert received["status"]["recording"], "Left OK did not start recording"
            drive(0.3)
            drive(0.5, left=FIST, right=FIST)
            assert all(received["status"]["gripper_closed"].values()), "Fists did not close both grippers"
            drive(0.3)
            drive(1.0, left=OPEN_PALM, right=OPEN_PALM, moving=True)
            moved = np.array(received["joints"].position)
            arm_indices = [i for i, name in enumerate(received["joints"].name) if not name.endswith("finger")]
            assert np.max(np.abs(moved[arm_indices] - baseline[arm_indices])) > 0.01, "Open palms did not move arms"
            if args.hybrid:
                assert all(hand["pose_source"] == "rgb_pnp" for hand in received["status"]["hands"].values())
            drive(0.15, left=FIST, right=FIST)
            stopped = np.array(received["joints"].position)
            drive(0.3, left=FIST, right=FIST, moving=True)
            np.testing.assert_allclose(np.array(received["joints"].position)[arm_indices], stopped[arm_indices], atol=1e-8)
            assert all(hand["motion_state"] == "HOLD" for hand in received["status"]["hands"].values())
            drive(0.4, left=OPEN_PALM, right=OPEN_PALM, pose_loss=True)
            assert all(hand["motion_state"] == "NO_POSE" for hand in received["status"]["hands"].values())
            paused = np.array(received["joints"].position)
            drive(0.5, left=OPEN_PALM, right=OPEN_PALM, moving=True)
            assert np.max(np.abs(np.array(received["joints"].position)[arm_indices] - paused[arm_indices])) > 0.005
            drive(0.6, send=False)
            assert received["status"]["tracking_stale"], "Timeout did not clear tracking"
            paused = np.array(received["joints"].position)
            drive(0.3, left=OPEN_PALM, right=OPEN_PALM, moving=True)
            assert np.max(np.abs(np.array(received["joints"].position)[arm_indices] - paused[arm_indices])) > 0.002
            drive(1.2, send=False)
            held = np.array(received["joints"].position)
            drive(0.3, left=OPEN_PALM, right=OPEN_PALM, moving=True)
            np.testing.assert_allclose(received["joints"].position, held, atol=1e-8)
            drive(0.3)
            drive(1.0, right=2)
            np.testing.assert_allclose(np.array(received["joints"].position)[arm_indices], baseline[arm_indices], atol=1e-5)
            drive(0.3)
            drive(0.6, left=2)
            assert not received["status"]["recording"], "Left OK did not stop recording"
            files = list(Path(directory).glob("*.jsonl"))
            assert len(files) == 1, "Expected exactly one completed episode"
            rows = [json.loads(line) for line in files[0].read_text().splitlines()]
            assert rows[0]["simulation"] and rows[-1]["reason"] == "left_ok"
            samples = [row for row in rows if row["type"] == "sample"]
            assert len(samples) > 30 and any(row["input"] is None for row in samples)
            topics = dict(node.get_topic_names_and_types())
            assert "/gripper/ctl" not in topics and "/towel/recording/control" not in topics
            names = node.get_node_names()
            assert not any(name in names for name in ("servo_node", "robot_control_node", "dual_arm_control_node"))
            print(f"PASS: UDP, both arms, grippers, timeout, home, TF, isolated ROS and {len(samples)} recorded samples", flush=True)
    finally:
        if process is not None:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        sender.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
