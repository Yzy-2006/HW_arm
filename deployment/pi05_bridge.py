#!/usr/bin/env python3
"""ROS2 bridge between the ENCOS robot and the PI05 worker process.

The bridge assembles observations and publishes the existing ``BridgeCtlMsg``
control path. Motion requires ``--motion``. Optional VR intervention pauses
policy commands and records corrections using the same camera streams.
"""

from __future__ import annotations

import argparse
import logging
import os
import socket
import signal
import sys
import threading
import time
import xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from motion_safety import MotionFault, CommandSafety, attach_safety, configured_position_limits, speed_only_mode
import yaml

from pi05_common import encode_bytes, recv_packet, send_packet
from minimum_snap import ChunkInterpolator, interpolation_settings, validate_controller_rate
from policy_home import PolicyHomeSession, TerminalKeys
from worker_lifecycle import WorkerLifecycleClient


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
    """Clip targets to the intersection of joint and feedback-relative limits.

    This allowance is not a velocity limiter; CommandSafety checks emitted motion.
    """
    result = np.asarray(action, dtype=np.float64).reshape(-1)
    current = np.asarray(state, dtype=np.float64).reshape(-1)
    if result.shape != (16,) or current.shape != (16,):
        raise ValueError("PI05 state/action must both have shape (16,)")
    if not np.isfinite(result).all() or not np.isfinite(current).all():
        raise ValueError("PI05 state/action contains non-finite values")
    from motion_safety import MotionFault
    feedback = current[JOINT_ACTION_INDICES]
    if np.any(feedback < lower - 0.02) or np.any(feedback > upper + 0.02):
        raise MotionFault("Joint feedback outside absolute limits")
    safe = result.copy()
    for action_index, joint_index in zip(JOINT_ACTION_INDICES, range(14)):
        state_index = joint_index if joint_index < 7 else joint_index + 1
        lo = max(current[state_index] - max_step[joint_index], lower[joint_index])
        hi = min(current[state_index] + max_step[joint_index], upper[joint_index])
        if lo > hi:
            raise MotionFault("Feedback allowance does not intersect joint limits")
        safe[action_index] = np.clip(safe[action_index], lo, hi)
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

    def at(self, now: float, *, timeline_time: float | None = None) -> np.ndarray | None:
        age = max(0.0, now - self.observed_at)
        # LeRobot ACT returns one action at a time. Keep that target available
        # while the next asynchronous inference is running, subject to the
        # same bounded age safety limit used for action chunks.
        if len(self.actions) == 1:
            if age > self.max_age_s:
                return None
            return self.actions[0].copy()
        playback_age = max(0.0, (now if timeline_time is None else timeline_time) - self.observed_at)
        index = int(playback_age * self.rate_hz)
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

    def _connect(self, timeout_s=None) -> None:
        self.close()
        timeout_s = self.timeout_s if timeout_s is None else timeout_s
        self.sock = socket.create_connection(self.address, timeout=timeout_s)
        self.sock.settimeout(timeout_s)
        send_packet(self.sock, {"type": "health", "token": self.token})
        response = recv_packet(self.sock)
        if not response.get("ok"):
            raise RuntimeError(response.get("error", "worker health check failed"))
        self.sock.settimeout(self.timeout_s)

    def health(self):
        with self.lock:
            try:
                if self.sock is None:
                    self._connect(timeout_s=0.5)
                else:
                    self.sock.settimeout(0.5)
                    send_packet(self.sock, {"type": "health", "token": self.token})
                    response = recv_packet(self.sock)
                    if not response.get("ok"):
                        raise RuntimeError(response.get("error", "worker health check failed"))
                    self.sock.settimeout(self.timeout_s)
            except Exception:
                self.close()
                raise

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

    return run_bridge(lambda: create_bridge_node(args))


def run_bridge(node_factory):
    """Drain callbacks before destroying ROS entities on SIGINT/SIGTERM."""

    import rclpy
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from ros_shutdown import drain_executor

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = threading.Event()
    previous_handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    for sig in previous_handlers:
        signal.signal(sig, lambda *_: stop_requested.set())
    node = None
    executor = MultiThreadedExecutor(num_threads=3)
    try:
        node = node_factory()
        executor.add_node(node)
        for companion in getattr(node, "companion_nodes", ()):
            executor.add_node(companion)
        while rclpy.ok() and not stop_requested.is_set():
            executor.spin_once(timeout_sec=0.05)
            if getattr(node, "recording_only_requested", False):
                from vr_recording import switch_to_recording_only

                executor, node = switch_to_recording_only(executor, node)
            elif getattr(node, "resume_policy_requested", False):
                from vr_recording import switch_to_policy

                executor, node = switch_to_policy(executor, node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            drain_executor(executor)
            if node is not None:
                try:
                    node.close()
                finally:
                    node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
            for sig, handler in previous_handlers.items():
                signal.signal(sig, handler)
    return 0


def create_bridge_node(args, cameras=None, recorder=None):

    config_path = Path(args.config).expanduser().resolve()
    config = _read_yaml(config_path)
    bridge_cfg = config.get("bridge", {})
    intervention_cfg = bridge_cfg.get("intervention", {})
    intervention_enabled = intervention_cfg.get("enabled", False)
    if not isinstance(intervention_enabled, bool):
        raise ValueError("bridge.intervention.enabled must be a boolean")
    stop_worker = intervention_cfg.get("stop_worker", False)
    if not isinstance(stop_worker, bool):
        raise ValueError("bridge.intervention.stop_worker must be a boolean")
    worker_stop_timeout_s = float(intervention_cfg.get("worker_stop_timeout_s", 15.0))
    if not np.isfinite(worker_stop_timeout_s) or worker_stop_timeout_s <= 0:
        raise ValueError("bridge.intervention.worker_stop_timeout_s must be finite and positive")
    takeover_timeout_s = float(intervention_cfg.get("takeover_timeout_s", 5.0))
    if not np.isfinite(takeover_timeout_s) or takeover_timeout_s <= 0:
        raise ValueError("bridge.intervention.takeover_timeout_s must be finite and positive")
    home_timeout_s = float(bridge_cfg.get("keyboard_home", {}).get("timeout_s", 30.0))
    if not np.isfinite(home_timeout_s) or home_timeout_s <= 0:
        raise ValueError("bridge.keyboard_home.timeout_s must be finite and positive")
    speed_only = speed_only_mode(bridge_cfg)
    policy_disable_all_speed_checks = bridge_cfg.get("motion_safety", {}).get("policy_disable_all_speed_checks", False)
    if not isinstance(policy_disable_all_speed_checks, bool):
        raise ValueError("motion_safety.policy_disable_all_speed_checks must be a boolean")
    policy_speed_check = bridge_cfg.get("motion_safety", {}).get("policy_command_speed_check", True)
    if not isinstance(policy_speed_check, bool):
        raise ValueError("motion_safety.policy_command_speed_check must be a boolean")
    if policy_disable_all_speed_checks:
        policy_speed_check = False
    policy_safety_cfg = dict(bridge_cfg)
    policy_safety_cfg["motion_safety"] = dict(bridge_cfg.get("motion_safety", {}))
    if not policy_speed_check:
        policy_safety_cfg["motion_safety"]["speed_limit_enabled_per_arm"] = [False] * 7
    worker_cfg = config.get("worker", {})
    camera_cfg_path = _resolve_path(config.get("camera_config"), ROOT / "config/recording.yaml")
    camera_cfg = _read_yaml(camera_cfg_path)
    camera_cfg["publish_rviz"] = False
    if speed_only:
        camera_cfg["camera_timing_checks_enabled"] = False
    gripper_cfg = _read_yaml(_resolve_path(config.get("gripper_config"), ROOT / "teleop/VR_controller/config/grippers.yaml"))
    controller_cfg = _read_yaml(_resolve_path(config.get("controller_config"), ROOT / "ros2_ws/src/ctl_msgs_hw/config/controllerConfig.yaml"))
    urdf_path = _resolve_path(config.get("urdf"), ROOT / "ros2_ws/src/dual_arm_hw/urdf/dual_arm.urdf")
    pd_path = _resolve_path(config.get("pd_config"), ROOT / "ros2_ws/src/ctl_msgs_hw/config/initialPD.yaml")

    worker_host = args.worker_host or str(worker_cfg.get("host", "127.0.0.1"))
    worker_port = args.worker_port or int(worker_cfg.get("port", 8765))
    control_port = worker_cfg.get("control_port")
    if control_port is None:
        control_port = worker_port + 1
    if stop_worker and (type(control_port) is not int or not 1 <= control_port <= 65535 or control_port == worker_port):
        raise ValueError("worker.control_port must be in 1..65535 and different from the inference port")
    token = args.token if args.token is not None else str(worker_cfg.get("token", ""))
    task = str(config.get("task", camera_cfg.get("task", "fold a towel with both arms")))
    rate_hz = float(bridge_cfg.get("rate_hz", camera_cfg.get("fps", 30)))
    action_rate_hz = float(bridge_cfg.get("action_rate_hz", camera_cfg.get("fps", 30)))
    interpolate, publish_rate_hz = interpolation_settings(bridge_cfg, rate_hz)
    if args.motion and publish_rate_hz < 4.0:
        raise ValueError("Motion publication must be at least 4 Hz; enable minimum_snap for slower source trajectories")
    if interpolate:
        validate_controller_rate(controller_cfg, publish_rate_hz)
    state_timeout_s = float(bridge_cfg.get("state_timeout_s", 0.25))
    replan_interval_s = float(bridge_cfg.get("replan_interval_s", 0.25))
    max_action_age_s = float(bridge_cfg.get("max_action_age_s", 1.0))
    for name, value in (("rate_hz", rate_hz), ("action_rate_hz", action_rate_hz), ("state_timeout_s", state_timeout_s),
                        ("replan_interval_s", replan_interval_s), ("max_action_age_s", max_action_age_s)):
        if not np.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    if speed_only:
        state_timeout_s = max_action_age_s = float("inf")
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
    if speed_only:
        max_step.fill(np.inf)
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
    lower, upper = configured_position_limits(bridge_cfg, lower, upper)
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
    from ctl_msgs_hw.msg import BridgeCtlMsg, GripperCtl, MidNodeCtlMsgRet
    from std_msgs.msg import Float64MultiArray

    if not hasattr(BridgeCtlMsg(), "hold_position"):
        raise RuntimeError("Rebuild ctl_msgs_hw and robot_control_hw, then source scripts/env.sh")
    if policy_disable_all_speed_checks and not hasattr(BridgeCtlMsg(), "policy_speed_checks_disabled"):
        raise RuntimeError("Rebuild ctl_msgs_hw, robot_control_hw, dual_arm_control_hw and servo_control_hw for policy speed override")

    sys.path.insert(0, str(ROOT / "collection"))
    from record import Cameras
    from ros_capture import feedback_qos, feedback_event_callbacks, make_feedback_group

    class BridgeNode(Node):
        def __init__(self):
            super().__init__("pi05_encos_bridge")
            self.motion = bool(args.motion)
            self.keyboard = None
            self.home_session = None
            self.control_mode = "policy"
            self.recording_only_requested = False
            self.policy_resume_pending = None
            self.policy_resume_started_at = 0.
            self.resume_observations_after = None
            self.companion_nodes = []
            self.recorder = None
            self.mode_pub = None
            self.takeover_pending = None
            self.takeover_started_at = 0.0
            self.worker_lifecycle = None
            self.lifecycle_pool = None
            self.lifecycle_pending = None
            self.lifecycle_operation = None
            self.lifecycle_cancel = threading.Event()
            self.worker_suspended = False
            self.worker_lifecycle_error = ""
            self.safety = None
            self.command_safety = CommandSafety(policy_safety_cfg, lower, upper, now=time.monotonic)
            self.sample_velocity = self.sample_acceleration = None
            self.last_sample_derivatives = None
            self.client = PolicyClient(worker_host, worker_port, token, float(worker_cfg.get("timeout_s", 5.0)))
            self.cameras = cameras if cameras is not None else Cameras(camera_cfg)
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
            self.startup_ready = False
            self.generation = 0
            self.last_request_at = -float("inf")
            self.last_infer_generation = -1
            self.last_latency_ms = 0.0
            # The low-level controller requires one feedback-anchored cycle when
            # switching away from its startup/zeroing command stream.
            self.reanchor_next_command = True
            self.request_id = 0
            self.last_log = 0.0
            self.feedback_group = make_feedback_group(self, camera_cfg.get("feedback_rate_hz", 120))
            self.safety_receive_group = MutuallyExclusiveCallbackGroup()
            self.command_pub = self.create_publisher(BridgeCtlMsg, "/dexbotCtl/bridge_ctl_msg", 1)
            self.gripper_pub = self.create_publisher(GripperCtl, "/gripper/ctl", 10)
            self.create_subscription(
                MidNodeCtlMsgRet,
                "/dexbotCtl/midMonitor_msg_send",
                self._joint_callback,
                feedback_qos(),
                callback_group=self.feedback_group,
                event_callbacks=feedback_event_callbacks(),
            )
            self.create_subscription(Float64MultiArray, "/gripper/fdb", self._gripper_callback,
                                     feedback_qos(), callback_group=self.feedback_group,
                                     event_callbacks=feedback_event_callbacks())
            self.control_timer = self.create_timer(1.0 / rate_hz, self.step)
            self.publish_timer = self.create_timer(1.0 / publish_rate_hz, self.publish_step) if interpolate else None
            if self.motion:
                self.safety = attach_safety(self, self._safety_fault,
                    receive_callback_group=self.safety_receive_group, diagnostics=self._input_diagnostics,
                    arm_on_publish=True, startup_settle_s=0.2, monitor_heartbeat=not speed_only)
                if speed_only:
                    self.get_logger().warning("Speed-only protection: heartbeat, age, position, acceleration, tracking and feedback-offset checks disabled; explicit faults/stops remain active")
            self.get_logger().warning(
                f"PI05 bridge started in {'MOTION' if self.motion else 'DRY-RUN'} "
                f"mode; worker={worker_host}:{worker_port} task={task!r} "
                f"pd_level={pd_level} rigidity_mode={rigidity_mode} "
                f"rate_hz={rate_hz:g} action_rate_hz={action_rate_hz:g} "
                f"minimum_snap={interpolate} publish_rate_hz={publish_rate_hz:g} "
                f"slow_factor={slow_factor:g} max_step_rad={max_step.max():g} "
                f"policy_command_speed_check={policy_speed_check}"
            )
            if self.motion and policy_disable_all_speed_checks:
                self.get_logger().warning("All policy software speed checks disabled, including servo command/feedback and policy holds; VR/replay takeover or controller transport timeout restores configured servo limits")
            elif self.motion and not policy_speed_check:
                self.get_logger().warning("Policy command speed checks disabled (preflight and publication); servo speed guards remain active")
            if self.motion:
                from std_srvs.srv import SetBool, Trigger

                self.home_client = self.create_client(Trigger, "/dexbotCtl/policy_home")
                self.resume_client = self.create_client(SetBool, "/dexbotCtl/vr_override")

                def request_home():
                    if not self.home_client.service_is_ready():
                        raise RuntimeError("/dexbotCtl/policy_home unavailable; rebuild and restart make robot")
                    return self.home_client.call_async(Trigger.Request())

                def request_resume():
                    if not self.resume_client.service_is_ready():
                        raise RuntimeError("/dexbotCtl/vr_override unavailable")
                    request = SetBool.Request()
                    request.data = False
                    return self.resume_client.call_async(request)

                self.home_session = PolicyHomeSession(request_home, request_resume, time.monotonic,
                    self.get_logger().warning, home_timeout_s)
                self.keyboard = TerminalKeys()
                if self.keyboard.enabled:
                    self.get_logger().info("H: pause model, return home, then automatically resume after the configured delay")
                if intervention_enabled:
                    from rclpy.qos import QoSProfile, DurabilityPolicy
                    from std_msgs.msg import String
                    from record import make_node

                    if stop_worker:
                        self.worker_lifecycle = WorkerLifecycleClient(worker_host, control_port, token, worker_stop_timeout_s)
                        self.lifecycle_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="worker-lifecycle")

                    self.mode_message = String
                    self.mode_pub = self.create_publisher(String, "/towel/policy_mode",
                        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
                    self.takeover_client = self.create_client(Trigger, "/dexbotCtl/policy_intervene")
                    self.create_service(Trigger, "/towel/intervene", self.request_intervention)
                    self.create_timer(0.2, self.publish_mode)
                    recording_cfg = dict(camera_cfg)
                    recording_cfg["dataset_root"] = str(_resolve_path(
                        intervention_cfg.get("dataset_root"), ROOT / "data/raw/interventions"))
                    recording_cfg["task"] = task
                    recording_cfg["episode_metadata"] = {"source": "vr_policy_intervention"}
                    self.recorder = recorder if recorder is not None else make_node(recording_cfg, self.cameras,
                        start_guard=self.recording_guard, control_callback_group=self.default_callback_group)
                    self.recorder.start_guard = self.recording_guard
                    self.companion_nodes.append(self.recorder)
                    self.publish_mode()
                    self.get_logger().info(f"VR intervention ready; shared-camera recorder: {recording_cfg['dataset_root']}")

        def publish_mode(self):
            if self.mode_pub is not None:
                self.mode_pub.publish(self.mode_message(data=self.control_mode))

        def _set_mode(self, mode):
            self.control_mode = mode
            self.publish_mode()

        def recording_guard(self):
            if self.control_mode != "vr":
                return "Recording requires acknowledged VR intervention ownership"
            if self.worker_lifecycle is not None and not self.worker_suspended:
                if self.worker_lifecycle_error and self.lifecycle_pending is None:
                    self._begin_worker_operation("stop")
                return self.worker_lifecycle_error or "Waiting for GPU worker to exit before recording; press B again when stopped"
            if self.safety is not None and not self.safety.ready():
                return "Recording blocked by servo safety state"
            return "Waiting for dedicated VR recording-only runtime; press B again when ready"

        def _stop_worker(self):
            result = self.worker_lifecycle.request("shutdown")
            if result.get("running") is not False:
                raise RuntimeError("Worker manager has not confirmed GPU process exit")
            # Stop also interrupts an inference already in flight. Drain that
            # future before closing its socket; none of its results may be reused.
            if self.pending is not None:
                try:
                    self.pending.result(timeout=worker_stop_timeout_s)
                except Exception:
                    if not self.pending.done():
                        raise RuntimeError("Old inference thread did not exit after worker stop")
            self.client.close()

        def _begin_worker_operation(self, operation):
            if operation != "stop":
                raise ValueError("VR intervention only shuts workers down; restart deployment manually")
            self.worker_lifecycle_error = ""
            self.lifecycle_operation = operation
            self.lifecycle_pending = self.lifecycle_pool.submit(self._stop_worker)

        def _poll_worker_operation(self):
            if self.lifecycle_pending is None or not self.lifecycle_pending.done():
                return
            future, operation = self.lifecycle_pending, self.lifecycle_operation
            self.lifecycle_pending = self.lifecycle_operation = None
            try:
                future.result()
                if operation == "stop":
                    self.pending = None
                    self.worker_suspended = True
                    self.get_logger().warning("GPU worker exited; preparing dedicated VR recording-only runtime")
            except Exception as exc:
                self.worker_lifecycle_error = f"Worker {operation} failed: {exc}; check make pi05/make act terminal"
                self.get_logger().error(self.worker_lifecycle_error)

        def release_to_recording_only(self):
            if not self.recording_only_requested:
                raise RuntimeError("VR handoff and worker shutdown must finish before detaching deployment")
            from vr_recording import make_recording_session

            self.control_timer.cancel()
            if self.publish_timer is not None:
                self.publish_timer.cancel()
            if self.keyboard is not None:
                self.keyboard.close()
            if self.home_session is not None:
                self.home_session.close()
            self.inference_pool.shutdown(wait=True, cancel_futures=True)
            if self.lifecycle_pool is not None:
                self.lifecycle_pool.shutdown(wait=True, cancel_futures=True)
            self.client.close()
            resume_factory = None if stop_worker else lambda cameras, recorder: create_bridge_node(args, cameras, recorder)
            client = self.client

            def check_worker():
                try:
                    client.health()
                finally:
                    client.close()

            session = make_recording_session(self.recorder, self.cameras,
                resume_factory=resume_factory, check_worker=check_worker if not stop_worker else None)
            self.recorder = self.cameras = None
            self.companion_nodes.clear()
            return session

        def _reset_policy(self):
            self._invalidate()
            self.command_safety.reset()
            self.last_command = None
            self.sample_velocity = self.sample_acceleration = None
            self.startup_ready = False
            self.reanchor_next_command = True
            self.last_observation_check = -float("inf")

        def resume_from_vr(self):
            from std_srvs.srv import SetBool

            self._reset_policy()
            self._set_mode("resuming_policy")
            self.policy_resume_started_at = time.monotonic()
            try:
                if not self.resume_client.service_is_ready():
                    raise RuntimeError("/dexbotCtl/vr_override unavailable")
                self.policy_resume_pending = self.resume_client.call_async(SetBool.Request(data=False))
            except Exception as exc:
                self._set_mode("failed")
                self.get_logger().error(f"Policy resume blocked: {exc}; discard requests VR takeover again")

        def _poll_policy_resume(self):
            try:
                future = self.policy_resume_pending
                if future is None:
                    return
                if not future.done():
                    if time.monotonic() - self.policy_resume_started_at < takeover_timeout_s:
                        return
                    future.cancel()
                    raise TimeoutError("Policy ownership response timed out")
                response = future.result()
                if response is None or not response.success:
                    raise RuntimeError(getattr(response, "message", "Empty policy ownership response"))
                self.policy_resume_pending = None
                self._reset_policy()
                self.resume_observations_after = time.monotonic()
                self._set_mode("policy")
                self.get_logger().info("Policy resumed from current observations; old chunks cleared; no home trajectory")
            except Exception as exc:
                self.policy_resume_pending = None
                self._set_mode("failed")
                self.get_logger().error(f"Policy resume blocked: {exc}; discard requests VR takeover again")

        def request_intervention(self, request, response):
            if self.control_mode == "vr":
                if self.worker_lifecycle is not None and not self.worker_suspended and self.lifecycle_pending is None:
                    self._begin_worker_operation("stop")
                response.success, response.message = True, "VR already owns control"
                return response
            if self.control_mode not in ("policy", "failed") or (
                    self.home_session is not None and self.home_session.active):
                response.success, response.message = False, "Control handoff or home is already active"
                return response
            # Stop publications and invalidate any result still in flight before
            # requesting the native controller's feedback-anchored joint hold.
            try:
                self._hold()
                with self.state_lock:
                    gripper_positions = self.gripper_feedback[[0, 4]].copy()
                    gripper_age = time.monotonic() - self.gripper_received_at
                if (np.isfinite(gripper_positions).all() and gripper_age <= 0.25 and
                        self.safety is not None):
                    hold_grippers = GripperCtl()
                    hold_grippers.ctl_time = 1.0 / publish_rate_hz
                    hold_grippers.l_arm_gripper_pos, hold_grippers.r_arm_gripper_pos = (
                        gripper_positions - zero_deg).tolist()
                    self.safety.publish_if_ready(lambda: self.gripper_pub.publish(hold_grippers))
            except Exception as exc:
                self.get_logger().warning(f"Initial intervention hold failed; requesting native hold: {exc}")
            self._reset_policy()
            self._set_mode("requesting_vr")
            if self.worker_lifecycle is not None:
                self._begin_worker_operation("stop")
            try:
                if not self.takeover_client.service_is_ready():
                    raise RuntimeError("/dexbotCtl/policy_intervene unavailable; rebuild and restart make robot")
                from std_srvs.srv import Trigger
                self.takeover_started_at = time.monotonic()
                self.takeover_pending = self.takeover_client.call_async(Trigger.Request())
                response.success, response.message = True, "Policy paused; VR handoff requested"
            except Exception as exc:
                self._set_mode("failed")
                response.success, response.message = False, str(exc)
                self.get_logger().error(f"Intervention failed; policy remains paused: {exc}")
            return response

        def _poll_intervention(self):
            if self.takeover_pending is None:
                return
            try:
                if not self.takeover_pending.done():
                    if time.monotonic() - self.takeover_started_at < takeover_timeout_s:
                        return
                    self.takeover_pending.cancel()
                    raise TimeoutError("VR handoff timed out; policy remains paused")
                response = self.takeover_pending.result()
                if response is None or not response.success:
                    raise RuntimeError(getattr(response, "message", "Empty VR handoff response"))
                self._set_mode("vr")
                self.get_logger().warning("VR owns control; switching to single-thread recording. " +
                    ("Finish recording, then restart deployment manually" if stop_worker else
                     "Discard while idle resumes policy from current feedback without homing"))
            except Exception as exc:
                self._set_mode("failed")
                self.get_logger().error(f"Intervention failed; policy remains paused: {exc}")
            finally:
                if self.control_mode != "requesting_vr":
                    self.takeover_pending = None

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

        def _input_diagnostics(self):
            now = time.monotonic()
            with self.state_lock:
                joint_age = "missing" if self.joints is None else f"{now - self.joint_received_at:.3f}"
                gripper_age = "missing" if not self.gripper_received_at else f"{now - self.gripper_received_at:.3f}"
            observation_age = (f"{now - self.last_observation_check:.3f}"
                               if np.isfinite(self.last_observation_check) else "not_checked")
            return (f"joint_age_s={joint_age} gripper_age_s={gripper_age} "
                    f"observation_check_age_s={observation_age} state_timeout_s={state_timeout_s:.3f}")

        def _publish_command(self, action: np.ndarray, hold: bool = False):
            if self.control_mode != "policy":
                return False
            if self.home_session is not None and self.home_session.active:
                return False
            if self.safety is not None and not self.safety.ready():
                return False
            joints_l = action[:7].tolist()
            joints_r = action[8:15].tolist()
            bridge_msg = BridgeCtlMsg()
            bridge_msg.header.stamp = self.get_clock().now().to_msg()
            bridge_msg.header.frame_id = "pi05_policy"
            bridge_msg.policy_speed_checks_disabled = policy_disable_all_speed_checks
            bridge_msg.ctl_trans_mod = 3
            bridge_msg.torque_feedforward_ctl = 0
            bridge_msg.if_reach_end = False
            bridge_msg.hold_position = bool(hold)
            bridge_msg.target_period_s = 1.0 / publish_rate_hz
            bridge_msg.derivatives_valid = self.sample_velocity is not None and not hold
            bridge_msg.l_arm_des_acc = [0.0] * 7 if hold or self.sample_acceleration is None else self.sample_acceleration[:7].tolist()
            bridge_msg.r_arm_des_acc = [0.0] * 7 if hold or self.sample_acceleration is None else self.sample_acceleration[7:].tolist()
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
            if bridge_msg.derivatives_valid:
                bridge_msg.l_arm_mdp_des_jnt_vel = self.sample_velocity[:7].tolist()
                bridge_msg.r_arm_mdp_des_jnt_vel = self.sample_velocity[7:].tolist()
            bridge_msg.if_init_use_ret = int(self.reanchor_next_command and not hold)
            bridge_msg.l_arm_rigidity_mod = [rigidity_mode] * 7
            bridge_msg.r_arm_rigidity_mod = [rigidity_mode] * 7
            bridge_msg.l_arm_jnt_kp = kp_l.tolist()
            bridge_msg.l_arm_jnt_kd = kd_l.tolist()
            bridge_msg.r_arm_jnt_kp = kp_r.tolist()
            bridge_msg.r_arm_jnt_kd = kd_r.tolist()
            bridge_msg.l_arm_if_jnt_stab = [0] * 7
            bridge_msg.r_arm_if_jnt_stab = [0] * 7
            if not hold:
                gripper_msg = GripperCtl()
                gripper_msg.ctl_time = 1.0 / publish_rate_hz
                target_deg = gripper.target_degrees(np.asarray([action[7], action[15]], dtype=np.float64))
                gripper_msg.l_arm_gripper_pos = float(target_deg[0] - zero_deg[0])
                gripper_msg.r_arm_gripper_pos = float(target_deg[1] - zero_deg[1])
                gripper_msg.l_arm_gripper_tor = 0.0
                gripper_msg.r_arm_gripper_tor = 0.0

            def publish():
                self.command_pub.publish(bridge_msg)
                if not hold:
                    self.gripper_pub.publish(gripper_msg)
                self.reanchor_next_command = bool(hold)

            if self.safety is None:
                publish()
                return True
            else:
                return self.safety.publish_if_ready(publish)

        def _hold(self):
            self.command_safety.reset()
            self.last_sample_derivatives = None
            if self.motion and self.last_command is not None:
                # Targets in this packet are ignored by the controller: it
                # captures its own joint feedback once on entry to hold.
                self._publish_command(self.last_command, hold=True)

        def _invalidate(self):
            self.chunk = None
            self.interpolator = None
            self.interpolation_clipped = False
            self.last_sample_derivatives = None
            self.generation += 1

        def publish_step(self):
            if self.control_mode != "policy":
                return
            if self.home_session is not None and self.home_session.active:
                return
            if self.safety is not None and not self.safety.ready():
                return
            if not self.startup_ready:
                return
            try:
                now = time.monotonic()
                current = self._snapshot_state()
                if self.motion and current is None and self.last_command is not None:
                    raise MotionFault(f"Joint or gripper feedback lost during motion; {self._input_diagnostics()}")
                if current is None or now - self.last_observation_check > state_timeout_s:
                    raise ValueError(f"Feedback or observation check expired before interpolation publication; {self._input_diagnostics()}")
                if self.interpolator is not None:
                    self.interpolator.start(now)
                raw = self._chunk_action(now)
                if raw is None or self.interpolator is None:
                    self.interpolator = None
                    self._hold()
                    return
                target = self.interpolator.at(now, raw)
                elapsed = now - self.interpolator.started_at
                self.sample_velocity = self.interpolator.trajectory.at(elapsed, 1)
                self.sample_acceleration = self.interpolator.trajectory.at(elapsed, 2)
                action = limit_action(target, current, lower, upper, max_step)
                self.interpolation_clipped = not np.allclose(
                    action[JOINT_ACTION_INDICES], target[JOINT_ACTION_INDICES], atol=1e-9, rtol=0,
                )
                if self.motion:
                    if self.interpolation_clipped:
                        raise MotionFault("Trajectory exceeds feedback allowance")
                    self.command_safety.check(action, current, 1.0 / publish_rate_hz,
                                              self.sample_velocity, self.sample_acceleration)
                if not self.motion or self._publish_command(action):
                    self.last_command = action.copy()
                    self.last_sample_derivatives = np.array([
                        self.sample_velocity, self.sample_acceleration,
                        self.interpolator.trajectory.at(elapsed, 3),
                    ])
            except MotionFault as exc:
                self._trip_safety(exc)
            except Exception as exc:
                self._invalidate()
                self._warn(f"PI05 interpolation failed: {exc}")
                self._hold()

        def _infer(self, state, images, request_id, reset):
            encoded = _encode_images(images, jpeg_quality)
            return self.client.infer(state, encoded, task, request_id, reset=reset)

        def _chunk_action(self, now):
            if self.chunk is None:
                return None
            if self.interpolator is not None:
                return self.interpolator.source_action(self.chunk, now)
            return self.chunk.at(now)

        def step(self):
            if self.control_mode == "resuming_policy":
                self._poll_policy_resume()
                return
            if self.keyboard is not None:
                key = self.keyboard.poll()
                if key in ("h", "H"):
                    self.request_home()
            self._poll_intervention()
            self._poll_worker_operation()
            if self.control_mode == "vr" and (self.worker_lifecycle is None or self.worker_suspended):
                self.recording_only_requested = True
                return
            if self.control_mode in ("requesting_vr", "vr", "failed", "stopped"):
                return
            if self.safety is not None and not self.safety.ready():
                if not self.startup_ready and not self.safety.fault:
                    self._warn(f"Waiting for startup servo safety; {self.safety.diagnostics()}; {self._input_diagnostics()}")
                return
            if self.home_session is not None and self.home_session.active:
                if self.home_session.tick():
                    self._reset_policy()
                    self._set_mode("policy")
                elif self.home_session.phase == "failed":
                    self._set_mode("failed")
                return
            try:
                state = self._snapshot_state()
                if state is None:
                    if self.motion and self.last_command is not None:
                        raise MotionFault(f"Joint or gripper feedback lost during motion; {self._input_diagnostics()}")
                    if not self.startup_ready:
                        self._warn(f"Waiting for startup joint/gripper feedback; {self._input_diagnostics()}")
                        return
                    raise ValueError(f"Waiting for fresh joint and gripper feedback; {self._input_diagnostics()}")
                now = time.monotonic()
                images, camera_meta = self.cameras.snapshot(now)
                if self.resume_observations_after is not None:
                    if (self.joint_received_at <= self.resume_observations_after or
                            self.gripper_received_at <= self.resume_observations_after or
                            any(camera_meta.get(name, {}).get("receive_monotonic_s", -float("inf")) <=
                                self.resume_observations_after for name in images)):
                        return
                    self.resume_observations_after = None
                self.last_observation_check = now
                if not self.startup_ready:
                    self.startup_ready = True
                    self.get_logger().info("Startup inputs available: joint/gripper feedback and camera observations; waiting for policy actions")
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
                        observed_at = now if speed_only or len(actions) == 1 else self.last_request_at
                        self.chunk = ActionChunk(actions, observed_at, action_rate_hz, max_action_age_s)
                        if self.chunk.at(now) is None:
                            raise ValueError("Inference result is already expired")
                        if interpolate:
                            continuing = self.last_command is not None and self.last_sample_derivatives is not None
                            anchor = self.last_command if continuing else state
                            derivatives = self.last_sample_derivatives if continuing else None
                            self.interpolator = ChunkInterpolator(self.chunk, now, anchor, derivatives)
                            if self.motion:
                                self.interpolator.trajectory.validate_limits(lower, upper,
                                    self.command_safety.velocity_limits, self.command_safety.max_acceleration)
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
                target = self._chunk_action(time.monotonic())
                if target is None:
                    self.interpolator = None
                    self._hold()
                    self._warn("Waiting for a current action chunk; holding position")
                    return
                current = self._snapshot_state()
                if current is None:
                    raise ValueError("Feedback expired before command publication")
                action = limit_action(target, current, lower, upper, max_step)
                if self.motion and not interpolate:
                    self.command_safety.check(action, current, 1.0 / publish_rate_hz)
                    if self._publish_command(action):
                        self.last_command = action.copy()
                elif not interpolate:
                    self.last_command = action.copy()
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
            except MotionFault as exc:
                self._trip_safety(exc)
            except Exception as exc:
                if not self.startup_ready:
                    self._warn(f"Waiting for startup observations: {exc}; {self._input_diagnostics()}")
                    return
                self._invalidate()
                self._warn(f"PI05 step failed: {exc}; {self._input_diagnostics()}")
                self._hold()

        def request_home(self):
            if self.control_mode == "vr" or self.worker_suspended:
                self.get_logger().warning("H disabled after VR intervention; save/discard, exit this session, and restart deployment manually")
                return
            if self.home_session is None or self.home_session.phase not in ("idle", "failed"):
                return
            if self.lifecycle_pending is not None:
                self.get_logger().warning("H ignored: worker shutdown is still pending")
                return
            if self.worker_lifecycle is not None and self.worker_lifecycle_error and not self.worker_suspended:
                self._begin_worker_operation("stop")
                self.get_logger().warning("H deferred: retrying unconfirmed worker stop; press H again after the worker has exited")
                return
            if self.takeover_pending is not None:
                self.get_logger().warning("H ignored: VR ownership handoff is pending")
                return
            if self.recorder is not None and self.recorder.busy():
                self.get_logger().warning("H ignored: save or discard the recording and wait for its writer to finish")
                return
            self._hold()
            self._reset_policy()
            self._set_mode("home")
            self.home_session.begin()

        def _trip_safety(self, reason):
            if self.safety is not None:
                self.safety.trip(str(reason))
            else:
                self._warn(str(reason))

        def _safety_fault(self, reason):
            self._invalidate()
            self.last_command = None
            self.get_logger().error(f"Motion fault latched; policy blocked: {reason}")

        def _warn(self, message: str):
            now = time.monotonic()
            if now - self.last_log >= 1.0:
                self.last_log = now
                self.get_logger().warning(message)

        def close(self):
            self.lifecycle_cancel.set()
            if self.keyboard is not None:
                self.keyboard.close()
            if self.home_session is not None:
                self.home_session.close()
            if self.takeover_pending is not None:
                self.takeover_pending.cancel()
            if self.policy_resume_pending is not None:
                self.policy_resume_pending.cancel()
            self.control_timer.cancel()
            if self.publish_timer is not None:
                self.publish_timer.cancel()
            try:
                import rclpy
                # Ctrl+C can cause ROS to invalidate publishers before node.close
                # runs. In that case the controller watchdog handles the stop.
                if rclpy.ok():
                    self._hold()
            except Exception as exc:
                # SIGINT may have already closed ROS. The controller watchdog
                # still holds the arms when this stream disappears.
                LOG.warning("Could not publish shutdown hold: %s", exc)
            finally:
                try:
                    self._set_mode("stopped")
                except Exception as exc:
                    LOG.warning("Could not publish stopped mode: %s", exc)
                try:
                    for companion in self.companion_nodes:
                        companion.close()
                finally:
                    self.companion_nodes.clear()
                    try:
                        if self.lifecycle_pool is not None:
                            self.lifecycle_pool.shutdown(wait=True, cancel_futures=True)
                    finally:
                        try:
                            self.inference_pool.shutdown(wait=True, cancel_futures=True)
                        finally:
                            try:
                                self.client.close()
                            finally:
                                self.cameras.close()

    return BridgeNode()


if __name__ == "__main__":
    raise SystemExit(main())
