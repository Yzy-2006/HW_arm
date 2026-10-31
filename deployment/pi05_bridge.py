#!/usr/bin/env python3
"""ROS2 bridge between the ENCOS robot and the PI05 worker process.

The bridge owns camera capture, state assembly, action safety limits, and the
existing ``BridgeCtlMsg`` control path. Motion is disabled unless ``--motion``
is supplied explicitly.
"""

from __future__ import annotations

import argparse
import logging
import os
import socket
import sys
import threading
import time
import xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml

from pi05_common import encode_bytes, recv_packet, send_packet
from minimum_snap import ChunkInterpolator, interpolation_settings, validate_controller_rate


LOG = logging.getLogger("pi05_bridge")
ROOT = Path(__file__).resolve().parents[1]
LEFT_JOINT_NAMES = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
]
RIGHT_JOINT_NAMES = [
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]
JOINT_NAMES = LEFT_JOINT_NAMES + RIGHT_JOINT_NAMES
URDF_JOINT_NAMES = [f"arm_l_joint_{index}" for index in range(1, 8)] + [
    f"arm_r_joint_{index}" for index in range(1, 8)
]
JOINT_ACTION_INDICES = list(range(7)) + list(range(8, 15))
GRIPPER_ACTION_INDICES = (7, 15)


def _read_yaml(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(os.path.expandvars(path.read_text())) or {}
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")
    return data


def _resolve_path(value: str | None, default: Path) -> Path:
    if not value:
        return default
    path = Path(os.path.expandvars(value)).expanduser()
    return path if path.is_absolute() else ROOT / path


def _as_vector(value: Any, size: int, name: str) -> np.ndarray:
    if isinstance(value, (int, float)):
        result = np.full(size, float(value), dtype=np.float64)
    else:
        result = np.asarray(value, dtype=np.float64)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain {size} finite values")
    return result


def load_joint_limits(path: Path) -> tuple[np.ndarray, np.ndarray]:
    tree = ET.parse(path)
    limits = {}
    for joint in tree.getroot().findall("joint"):
        limit = joint.find("limit")
        if limit is None or "lower" not in limit.attrib or "upper" not in limit.attrib:
            continue
        limits[joint.attrib.get("name", "")] = (
            float(limit.attrib["lower"]),
            float(limit.attrib["upper"]),
        )
    missing = [name for name in URDF_JOINT_NAMES if name not in limits]
    if missing:
        raise ValueError(f"URDF has no limits for: {missing}")
    lower = np.asarray([limits[name][0] for name in URDF_JOINT_NAMES], dtype=np.float64)
    upper = np.asarray([limits[name][1] for name in URDF_JOINT_NAMES], dtype=np.float64)
    return lower, upper


def load_pd(path: Path, level: str) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    data = _read_yaml(path)

    def read(prefix: str, suffix: str, names: list[str]) -> np.ndarray:
        section = data.get(f"{prefix}_{level}")
        if not isinstance(section, dict):
            raise ValueError(f"Missing PD section {prefix}_{level} in {path}")
        return np.asarray(
            [float(section[f"{name}_{suffix}"]) for name in names], dtype=np.float64
        )

    return (
        read("left_arm_Kp", "kp", LEFT_JOINT_NAMES),
        read("left_arm_Kd", "kd", LEFT_JOINT_NAMES),
        read("right_arm_Kp", "kp", RIGHT_JOINT_NAMES),
        read("right_arm_Kd", "kd", RIGHT_JOINT_NAMES),
    )


class GripperState:
    """Convert calibrated degree feedback into the dataset's binary state."""

    def __init__(self, config: dict[str, Any]):
        self.open_deg = np.asarray(
            [float(config[side]["open_deg"]) for side in ("left", "right")], dtype=np.float64
        )
        self.closed_deg = np.asarray(
            [float(config[side]["closed_deg"]) for side in ("left", "right")], dtype=np.float64
        )
        if np.any(self.open_deg == self.closed_deg):
            raise ValueError("Gripper open and closed endpoints must differ")
        self.states = np.array([-1, -1], dtype=np.int8)

    def update(self, feedback: np.ndarray) -> np.ndarray | None:
        if feedback.shape != (8,):
            return None
        positions = feedback[[0, 4]]
        if not np.isfinite(positions).all():
            return None
        fraction = (positions - self.open_deg) / (self.closed_deg - self.open_deg)
        for index, value in enumerate(fraction):
            if self.states[index] < 0:
                self.states[index] = int(value >= 0.5)
            elif value >= 0.6:
                self.states[index] = 1
            elif value <= 0.4:
                self.states[index] = 0
        if np.any(self.states < 0):
            return None
        return self.states.copy()

    def target_degrees(self, states: np.ndarray) -> np.ndarray:
        return np.where(states.astype(bool), self.closed_deg, self.open_deg)


def limit_action(
    action: Any,
    state: Any,
    lower: np.ndarray,
    upper: np.ndarray,
    max_step: np.ndarray,
) -> np.ndarray:
    """Apply shape, finite, joint-range, and per-cycle delta checks."""
    result = np.asarray(action, dtype=np.float64).reshape(-1)
    current = np.asarray(state, dtype=np.float64).reshape(-1)
    if result.shape != (16,) or current.shape != (16,):
        raise ValueError("PI05 state/action must both have shape (16,)")
    if not np.isfinite(result).all() or not np.isfinite(current).all():
        raise ValueError("PI05 state/action contains non-finite values")
    safe = result.copy()
    for action_index, joint_index in zip(JOINT_ACTION_INDICES, range(14)):
        state_index = joint_index if joint_index < 7 else joint_index + 1
        target = np.clip(
            safe[action_index],
            current[state_index] - max_step[joint_index],
            current[state_index] + max_step[joint_index],
        )
        safe[action_index] = np.clip(target, lower[joint_index], upper[joint_index])
    for index in GRIPPER_ACTION_INDICES:
        safe[index] = float(safe[index] >= 0.5)
    return safe


class ActionChunk:
    """Absolute targets indexed by observation time, never by inference completion."""

    def __init__(self, actions: Any, observed_at: float, rate_hz: float, max_age_s: float):
        self.actions = np.asarray(actions, dtype=np.float64)
        if (self.actions.ndim != 2 or self.actions.shape[1] != 16
                or not 1 <= len(self.actions) <= 1000 or not np.isfinite(self.actions).all()):
            raise ValueError("Expected finite actions with shape (horizon, 16)")
        self.observed_at = observed_at
        self.rate_hz = rate_hz
        self.max_age_s = max_age_s

    def at(self, now: float) -> np.ndarray | None:
        age = max(0.0, now - self.observed_at)
        # LeRobot ACT returns one action at a time. Keep that target available
        # while the next asynchronous inference is running, subject to the
        # same bounded age safety limit used for action chunks.
        if len(self.actions) == 1:
            if age > self.max_age_s:
                return None
            return self.actions[0].copy()
        index = int(age * self.rate_hz)
        if age > self.max_age_s or index >= len(self.actions):
            return None
        return self.actions[index].copy()


class PolicyClient:
    def __init__(self, host: str, port: int, token: str, timeout_s: float):
        self.address = (host, port)
        self.token = token
        self.timeout_s = timeout_s
        self.sock = None
        self.lock = threading.Lock()

    def close(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            finally:
                self.sock = None

    def _connect(self) -> None:
        self.close()
        self.sock = socket.create_connection(self.address, timeout=self.timeout_s)
        self.sock.settimeout(self.timeout_s)
        send_packet(self.sock, {"type": "health", "token": self.token})
        response = recv_packet(self.sock)
        if not response.get("ok"):
            raise RuntimeError(response.get("error", "worker health check failed"))

    def infer(self, state: np.ndarray, images: dict[str, str], task: str, request_id: int,
              reset: bool = False) -> dict[str, Any]:
        with self.lock:
            if self.sock is None:
                self._connect()
            try:
                send_packet(
                    self.sock,
                    {
                        "type": "infer",
                        "token": self.token,
                        "request_id": request_id,
                        "state": [float(value) for value in state],
                        "images": images,
                        "task": task,
                        "reset": reset,
                    },
                )
                response = recv_packet(self.sock)
                if not response.get("ok"):
                    raise RuntimeError(response.get("error", "worker inference failed"))
                if response.get("request_id") != request_id:
                    raise ValueError("Worker response request_id does not match")
                return response
            except (ConnectionError, OSError, TimeoutError):
                self.close()
                raise


def _encode_images(images: dict[str, np.ndarray], quality: int) -> dict[str, str]:
    encoded = {}
    for name, image in images.items():
        ok, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            raise RuntimeError(f"Could not JPEG encode camera {name}")
        encoded[name] = encode_bytes(buffer.tobytes())
    return encoded


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(ROOT / "config/pi05_deployment.yaml"))
    parser.add_argument("--motion", action="store_true", help="Publish robot and gripper commands")
    parser.add_argument("--worker-host")
    parser.add_argument("--worker-port", type=int)
    parser.add_argument("--token")
    parser.add_argument(
        "--slow-factor",
        type=float,
        help="Scale the per-tick joint target offset (0 < factor <= 1)",
    )
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper()), format="%(asctime)s %(levelname)s %(message)s")

    import rclpy
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

    rclpy.init()
    node = None
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node = create_bridge_node(args)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.close()
            node.destroy_node()
        # A ROS signal handler may already have shut down the context while the
        # executor was stopping. Avoid reporting that expected second shutdown
        # as a bridge failure.
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def create_bridge_node(args):

    config_path = Path(args.config).expanduser().resolve()
    config = _read_yaml(config_path)
    bridge_cfg = config.get("bridge", {})
    worker_cfg = config.get("worker", {})
    camera_cfg_path = _resolve_path(config.get("camera_config"), ROOT / "config/recording.yaml")
    camera_cfg = _read_yaml(camera_cfg_path)
    camera_cfg["publish_rviz"] = False
    gripper_cfg = _read_yaml(_resolve_path(config.get("gripper_config"), ROOT / "teleop/VR_controller/config/grippers.yaml"))
    controller_cfg = _read_yaml(_resolve_path(config.get("controller_config"), ROOT / "ros2_ws/src/ctl_msgs_hw/config/controllerConfig.yaml"))
    urdf_path = _resolve_path(config.get("urdf"), ROOT / "ros2_ws/src/dual_arm_hw/urdf/dual_arm.urdf")
    pd_path = _resolve_path(config.get("pd_config"), ROOT / "ros2_ws/src/ctl_msgs_hw/config/initialPD.yaml")

    worker_host = args.worker_host or str(worker_cfg.get("host", "127.0.0.1"))
    worker_port = args.worker_port or int(worker_cfg.get("port", 8765))
    token = args.token if args.token is not None else str(worker_cfg.get("token", ""))
    task = str(config.get("task", camera_cfg.get("task", "fold a towel with both arms")))
    rate_hz = float(bridge_cfg.get("rate_hz", camera_cfg.get("fps", 30)))
    action_rate_hz = float(bridge_cfg.get("action_rate_hz", camera_cfg.get("fps", 30)))
    interpolate, publish_rate_hz = interpolation_settings(bridge_cfg, rate_hz)
    if interpolate:
        validate_controller_rate(controller_cfg, publish_rate_hz)
    state_timeout_s = float(bridge_cfg.get("state_timeout_s", 0.25))
    replan_interval_s = float(bridge_cfg.get("replan_interval_s", 0.25))
    max_action_age_s = float(bridge_cfg.get("max_action_age_s", 1.0))
    for name, value in (("rate_hz", rate_hz), ("action_rate_hz", action_rate_hz), ("state_timeout_s", state_timeout_s),
                        ("replan_interval_s", replan_interval_s), ("max_action_age_s", max_action_age_s)):
        if not np.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    slow_factor = (
        args.slow_factor
        if args.slow_factor is not None
        else float(bridge_cfg.get("slow_factor", 1.0))
    )
    if not np.isfinite(slow_factor) or not 0.0 < slow_factor <= 1.0:
        raise ValueError("slow_factor must be finite and satisfy 0 < slow_factor <= 1")
    max_step = _as_vector(bridge_cfg.get("max_joint_step_rad", 0.02), 14, "max_joint_step_rad")
    if np.any(max_step <= 0):
        raise ValueError("max_joint_step_rad must be positive")
    max_step *= slow_factor
    pd_level = str(bridge_cfg.get("pd_level", "mid")).strip().lower()
    rigidity_by_level = {"low": 1, "mid": 2, "high": 3, "superhigh": 7}
    if pd_level not in rigidity_by_level:
        raise ValueError(f"pd_level must be one of {sorted(rigidity_by_level)}")
    rigidity_mode = int(bridge_cfg.get("rigidity_mode", rigidity_by_level[pd_level]))
    if rigidity_mode not in set(rigidity_by_level.values()):
        raise ValueError("rigidity_mode must be one of 1 (low), 2 (mid), 3 (high), or 7 (superhigh)")
    jpeg_quality = int(bridge_cfg.get("jpeg_quality", 80))
    if not 30 <= jpeg_quality <= 100:
        raise ValueError("jpeg_quality must be between 30 and 100")
    lower, upper = load_joint_limits(urdf_path)
    kp_l, kd_l, kp_r, kd_r = load_pd(pd_path, pd_level)
    gripper = GripperState(gripper_cfg)
    controller_grippers = controller_cfg.get("gripperControl", {})
    zero_deg = np.asarray(
        [
            float(controller_grippers.get("l_gripper_ctl", {}).get("zero_pos_deg", 0.0)),
            float(controller_grippers.get("r_gripper_ctl", {}).get("zero_pos_deg", 0.0)),
        ],
        dtype=np.float64,
    )

    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from ctl_msgs_hw.msg import BridgeCtlMsg, GripperCtl, MidNodeCtlMsgRet
    from std_msgs.msg import Float64MultiArray

    if not hasattr(BridgeCtlMsg(), "hold_position"):
        raise RuntimeError("Rebuild ctl_msgs_hw and robot_control_hw, then source scripts/env.sh")

    sys.path.insert(0, str(ROOT / "collection"))
    from record import Cameras

    class BridgeNode(Node):
        def __init__(self):
            super().__init__("pi05_encos_bridge")
            self.motion = bool(args.motion)
            self.client = PolicyClient(worker_host, worker_port, token, float(worker_cfg.get("timeout_s", 5.0)))
            self.cameras = Cameras(camera_cfg)
            self.state_lock = threading.Lock()
            self.joints = None
            self.joint_received_at = 0.0
            self.gripper_feedback = np.full(8, np.nan, dtype=np.float64)
            self.gripper_received_at = 0.0
            self.last_command = None
            self.inference_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="pi05-inference")
            self.pending = None
            self.chunk = None
            self.interpolator = None
            self.interpolation_clipped = False
            self.last_observation_check = -float("inf")
            self.generation = 0
            self.last_request_at = -float("inf")
            self.last_infer_generation = -1
            self.last_latency_ms = 0.0
            # The low-level controller requires one feedback-anchored cycle when
            # switching away from its startup/zeroing command stream.
            self.reanchor_next_command = True
            self.request_id = 0
            self.last_log = 0.0
            self.feedback_group = MutuallyExclusiveCallbackGroup()
            self.command_pub = self.create_publisher(BridgeCtlMsg, "/dexbotCtl/bridge_ctl_msg", 1)
            self.gripper_pub = self.create_publisher(GripperCtl, "/gripper/ctl", 10)
            self.create_subscription(
                MidNodeCtlMsgRet,
                "/dexbotCtl/midMonitor_msg_send",
                self._joint_callback,
                qos_profile_sensor_data,
                callback_group=self.feedback_group,
            )
            self.create_subscription(Float64MultiArray, "/gripper/fdb", self._gripper_callback,
                                     qos_profile_sensor_data, callback_group=self.feedback_group)
            self.control_timer = self.create_timer(1.0 / rate_hz, self.step)
            self.publish_timer = self.create_timer(1.0 / publish_rate_hz, self.publish_step) if interpolate else None
            self.get_logger().warning(
                f"PI05 bridge started in {'MOTION' if self.motion else 'DRY-RUN'} "
                f"mode; worker={worker_host}:{worker_port} task={task!r} "
                f"pd_level={pd_level} rigidity_mode={rigidity_mode} "
                f"rate_hz={rate_hz:g} action_rate_hz={action_rate_hz:g} "
                f"minimum_snap={interpolate} publish_rate_hz={publish_rate_hz:g} "
                f"slow_factor={slow_factor:g} max_step_rad={max_step.max():g}"
            )

        def _joint_callback(self, msg):
            left = np.asarray(msg.l_arm_pos_fdback, dtype=np.float64)
            right = np.asarray(msg.r_arm_pos_fdback, dtype=np.float64)
            if left.shape != (7,) or right.shape != (7,) or not np.isfinite(np.r_[left, right]).all():
                return
            with self.state_lock:
                self.joints = np.r_[left, right]
                self.joint_received_at = time.monotonic()

        def _gripper_callback(self, msg):
            values = np.asarray(msg.data, dtype=np.float64)
            if values.shape == (8,) and np.isfinite(values[[0, 4]]).all():
                with self.state_lock:
                    self.gripper_feedback = values
                    self.gripper_received_at = time.monotonic()

        def _snapshot_state(self):
            with self.state_lock:
                joints = None if self.joints is None else self.joints.copy()
                feedback = self.gripper_feedback.copy()
                joint_age = time.monotonic() - self.joint_received_at
                gripper_age = time.monotonic() - self.gripper_received_at
            states = gripper.update(feedback)
            if joints is None or joint_age > state_timeout_s or states is None or gripper_age > state_timeout_s:
                return None
            return np.r_[joints[:7], states[0], joints[7:], states[1]].astype(np.float32)

        def _publish_command(self, action: np.ndarray, hold: bool = False):
            joints_l = action[:7].tolist()
            joints_r = action[8:15].tolist()
            bridge_msg = BridgeCtlMsg()
            bridge_msg.header.stamp = self.get_clock().now().to_msg()
            bridge_msg.ctl_trans_mod = 3
            bridge_msg.torque_feedforward_ctl = 0
            bridge_msg.if_reach_end = False
            bridge_msg.hold_position = bool(hold)
            bridge_msg.l_arm_if_use_jntlim = False
            bridge_msg.r_arm_if_use_jntlim = False
            bridge_msg.l_arm_limjnt_num = 0
            bridge_msg.r_arm_limjnt_num = 0
            bridge_msg.l_arm_des_msg = joints_l
            bridge_msg.r_arm_des_msg = joints_r
            bridge_msg.l_arm_des_tor_msg = [0.0] * 7
            bridge_msg.r_arm_des_tor_msg = [0.0] * 7
            bridge_msg.l_arm_mdp_des_jnt_vel = [0.0] * 7
            bridge_msg.r_arm_mdp_des_jnt_vel = [0.0] * 7
            bridge_msg.if_init_use_ret = int(self.reanchor_next_command and not hold)
            bridge_msg.l_arm_rigidity_mod = [rigidity_mode] * 7
            bridge_msg.r_arm_rigidity_mod = [rigidity_mode] * 7
            bridge_msg.l_arm_jnt_kp = kp_l.tolist()
            bridge_msg.l_arm_jnt_kd = kd_l.tolist()
            bridge_msg.r_arm_jnt_kp = kp_r.tolist()
            bridge_msg.r_arm_jnt_kd = kd_r.tolist()
            bridge_msg.l_arm_if_jnt_stab = [0] * 7
            bridge_msg.r_arm_if_jnt_stab = [0] * 7
            self.command_pub.publish(bridge_msg)

            if hold:
                self.reanchor_next_command = True
                return

            gripper_msg = GripperCtl()
            gripper_msg.ctl_time = 1.0 / publish_rate_hz
            target_deg = gripper.target_degrees(np.asarray([action[7], action[15]], dtype=np.float64))
            gripper_msg.l_arm_gripper_pos = float(target_deg[0] - zero_deg[0])
            gripper_msg.r_arm_gripper_pos = float(target_deg[1] - zero_deg[1])
            gripper_msg.l_arm_gripper_tor = 0.0
            gripper_msg.r_arm_gripper_tor = 0.0
            self.gripper_pub.publish(gripper_msg)
            if not hold:
                self.reanchor_next_command = False

        def _hold(self):
            if self.motion and self.last_command is not None:
                # Targets in this packet are ignored by the controller: it
                # captures its own joint feedback once on entry to hold.
                self._publish_command(self.last_command, hold=True)

        def _invalidate(self):
            self.chunk = None
            self.interpolator = None
            self.interpolation_clipped = False
            self.generation += 1

        def publish_step(self):
            try:
                now = time.monotonic()
                current = self._snapshot_state()
                if current is None or now - self.last_observation_check > state_timeout_s:
                    raise ValueError("Feedback or observation check expired before interpolation publication")
                raw = None if self.chunk is None else self.chunk.at(now)
                if raw is None or self.interpolator is None:
                    self.interpolator = None
                    self._hold()
                    return
                target = self.interpolator.at(now, raw)
                action = limit_action(target, current, lower, upper, max_step)
                self.interpolation_clipped = not np.allclose(
                    action[JOINT_ACTION_INDICES], target[JOINT_ACTION_INDICES], atol=1e-9, rtol=0,
                )
                self.last_command = action
                if self.motion:
                    self._publish_command(action)
            except Exception as exc:
                self._invalidate()
                self._warn(f"PI05 interpolation failed: {exc}")
                self._hold()

        def _infer(self, state, images, request_id, reset):
            encoded = _encode_images(images, jpeg_quality)
            return self.client.infer(state, encoded, task, request_id, reset=reset)

        def step(self):
            try:
                state = self._snapshot_state()
                if state is None:
                    raise ValueError("Waiting for fresh joint and gripper feedback")
                now = time.monotonic()
                images, _ = self.cameras.snapshot(now)
                self.last_observation_check = now
                if self.pending is not None and self.pending.done():
                    pending, self.pending = self.pending, None
                    response = pending.result()
                    if self.pending_generation == self.generation:
                        if now - self.last_request_at > max_action_age_s:
                            raise ValueError("Inference result is already expired")
                        if "actions" not in response:
                            raise ValueError("Worker does not return action chunks; restart the updated worker")
                        actions = response["actions"]
                        self.last_latency_ms = (now - self.last_request_at) * 1000.0
                        # LeRobot select_action supplies one queued target per
                        # request; it has no future horizon to latency-align.
                        observed_at = now if len(actions) == 1 else self.last_request_at
                        self.chunk = ActionChunk(actions, observed_at, action_rate_hz, max_action_age_s)
                        if self.chunk.at(now) is None:
                            raise ValueError("Inference result is already expired")
                        if interpolate:
                            anchor = state if self.interpolator is None or self.last_command is None else self.last_command
                            previous = None if self.interpolation_clipped else self.interpolator
                            self.interpolator = ChunkInterpolator(self.chunk, now, anchor, previous)
                            self.interpolation_clipped = False
                if self.pending is None and (
                    self.chunk is None or len(self.chunk.actions) == 1
                    or now - self.last_request_at >= replan_interval_s
                ):
                    self.pending_generation = self.generation
                    self.last_request_at = now
                    reset = self.last_infer_generation != self.generation
                    self.last_infer_generation = self.generation
                    self.pending = self.inference_pool.submit(self._infer, state.copy(), images, self.request_id, reset)
                    self.request_id += 1
                target = None if self.chunk is None else self.chunk.at(now)
                if target is None:
                    self.interpolator = None
                    self._hold()
                    self._warn("Waiting for a current action chunk; holding position")
                    return
                current = self._snapshot_state()
                if current is None:
                    raise ValueError("Feedback expired before command publication")
                action = limit_action(target, current, lower, upper, max_step)
                if not interpolate:
                    self.last_command = action
                if self.motion and not interpolate:
                    self._publish_command(action)
                now = time.monotonic()
                if now - self.last_log >= 1.0:
                    self.last_log = now
                    self.get_logger().info(
                        f"PI05 step={self.request_id} "
                        f"latency_ms={self.last_latency_ms:.1f} "
                        f"chunk_age_ms={(now - self.chunk.observed_at) * 1000:.1f} "
                        f"motion={self.motion} "
                        f"action={np.array2string(action, precision=3, suppress_small=True)}"
                    )
            except Exception as exc:
                self._invalidate()
                self._warn(f"PI05 step failed: {exc}")
                self._hold()

        def _warn(self, message: str):
            now = time.monotonic()
            if now - self.last_log >= 1.0:
                self.last_log = now
                self.get_logger().warning(message)

        def close(self):
            self.control_timer.cancel()
            if self.publish_timer is not None:
                self.publish_timer.cancel()
            try:
                # Ctrl+C can cause ROS to invalidate publishers before node.close
                # runs. In that case the controller watchdog handles the stop.
                if rclpy.ok():
                    self._hold()
            except Exception as exc:
                # SIGINT may have already closed ROS. The controller watchdog
                # still holds the arms when this stream disappears.
                LOG.warning("Could not publish shutdown hold: %s", exc)
            finally:
                self.inference_pool.shutdown(wait=True, cancel_futures=True)
                self.client.close()
                self.cameras.close()

    return BridgeNode()


if __name__ == "__main__":
    raise SystemExit(main())
