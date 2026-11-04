#!/usr/bin/env python3
"""Replay recorded LeRobot actions through the ENCOS ROS2 control path.

The command is a dry-run unless ``--motion`` is supplied.  It reads only the
``action`` column from a selected episode; camera frames and the PI05 worker are
not involved in this mode.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import logging
import time
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from pi05_bridge import _as_vector, limit_action, load_joint_limits, load_pd
from minimum_snap import JOINT_INDICES, MinimumSnapTrajectory, interpolation_settings, validate_controller_rate


LOG = logging.getLogger("replay_dataset")
ROOT = Path(__file__).resolve().parents[1]
JOINT_LABELS = [
    *(f"left_joint_{index}" for index in range(1, 8)),
    *(f"right_joint_{index}" for index in range(1, 8)),
]


class ReplayRecorder:
    """Collect commanded and feedback arm angles for one replay run."""

    def __init__(self, output_dir: str | Path, episode: int, start_frame: int, *, make_plot: bool = True):
        self.output_dir = Path(output_dir).expanduser().resolve()
        self.episode = episode
        self.start_frame = start_frame
        self.make_plot = make_plot
        self.started_at = time.monotonic()
        self.rows: list[dict[str, Any]] = []
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.prefix = f"episode_{episode}_start_{start_frame}_{stamp}"

    def record(self, frame: int, dataset_action: np.ndarray, command: np.ndarray,
               feedback: np.ndarray | None,
               *, gripper_feedback: np.ndarray | None = None) -> None:
        dataset_action = np.asarray(dataset_action, dtype=np.float64).reshape(-1)
        if dataset_action.shape != (14,) or not np.isfinite(dataset_action).all():
            raise ValueError("Dataset action angles must contain 14 finite values")
        command = np.asarray(command, dtype=np.float64).reshape(-1)
        if command.shape != (14,) or not np.isfinite(command).all():
            raise ValueError("Replay command angles must contain 14 finite values")
        actual = np.full(14, np.nan, dtype=np.float64)
        if feedback is not None:
            actual = np.asarray(feedback, dtype=np.float64).reshape(-1)
            if actual.shape != (14,) or not np.isfinite(actual).all():
                raise ValueError("Replay feedback angles must contain 14 finite values")
        gripper = np.full(8, np.nan, dtype=np.float64)
        if gripper_feedback is not None:
            gripper = np.asarray(gripper_feedback, dtype=np.float64).reshape(-1)
            if gripper.shape != (8,) or not np.isfinite(gripper).all():
                raise ValueError("Replay gripper feedback must contain 8 finite values")
        row: dict[str, Any] = {
            "sample": len(self.rows),
            "dataset_frame": int(frame),
            "elapsed_s": time.monotonic() - self.started_at,
        }
        for label, value in zip(JOINT_LABELS, dataset_action):
            row[f"dataset_{label}"] = float(value)
        for label, value in zip(JOINT_LABELS, command):
            row[f"command_{label}"] = float(value)
        for label, value in zip(JOINT_LABELS, actual):
            row[f"feedback_{label}"] = float(value)
        row["feedback_left_gripper_deg"] = float(gripper[0])
        row["feedback_right_gripper_deg"] = float(gripper[4])
        self.rows.append(row)

    @property
    def fields(self) -> list[str]:
        return [
            "sample", "dataset_frame", "elapsed_s",
            *(f"dataset_{label}" for label in JOINT_LABELS),
            *(f"command_{label}" for label in JOINT_LABELS),
            *(f"feedback_{label}" for label in JOINT_LABELS),
            "feedback_left_gripper_deg", "feedback_right_gripper_deg",
        ]

    def save(self) -> tuple[Path | None, Path | None]:
        if not self.rows:
            LOG.warning("No replay samples were recorded; no CSV or plot was written")
            return None, None
        self.output_dir.mkdir(parents=True, exist_ok=True)
        csv_path = self.output_dir / f"{self.prefix}.csv"
        with csv_path.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=self.fields)
            writer.writeheader()
            writer.writerows(self.rows)

        plot_path = None
        if self.make_plot:
            try:
                plot_path = self._save_plot()
            except Exception as exc:  # plotting must not discard the CSV
                LOG.warning("Could not generate replay plot: %s", exc)
        LOG.info("Replay measurements saved: csv=%s plot=%s", csv_path, plot_path or "disabled/unavailable")
        return csv_path, plot_path

    def _save_plot(self) -> Path:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        times = np.asarray([row["elapsed_s"] for row in self.rows], dtype=np.float64)
        dataset_actions = np.asarray([
            [row[f"dataset_{label}"] for label in JOINT_LABELS]
            for row in self.rows
        ], dtype=np.float64)
        feedback = np.asarray([
            [row[f"feedback_{label}"] for label in JOINT_LABELS]
            for row in self.rows
        ], dtype=np.float64)
        fig, axes = plt.subplots(7, 2, figsize=(15, 22), sharex=True)
        axes = np.asarray(axes).reshape(-1)
        for index, (axis, label) in enumerate(zip(axes, JOINT_LABELS)):
            axis.plot(times, dataset_actions[:, index], label="dataset action",
                      color="tab:blue", linewidth=1.2)
            if np.isfinite(feedback[:, index]).any():
                axis.plot(times, feedback[:, index], label="motor feedback",
                          color="tab:orange", linewidth=1.0)
            else:
                axis.text(0.5, 0.5, "no feedback", transform=axis.transAxes,
                          ha="center", va="center")
            axis.set_title(label)
            axis.set_ylabel("rad")
            axis.grid(True, alpha=0.3)
            if index == 0:
                axis.legend(loc="best")
        axes[-1].set_xlabel("elapsed time (s)")
        axes[-2].set_xlabel("elapsed time (s)")
        fig.suptitle(f"Replay episode {self.episode}: dataset action vs motor feedback")
        fig.tight_layout()
        plot_path = self.output_dir / f"{self.prefix}.png"
        fig.savefig(plot_path, dpi=150)
        plt.close(fig)
        return plot_path


def _read_yaml(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text()) or {}
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")
    return data


def _parquet_files(root: Path, relative_dir: str) -> list[Path]:
    files = sorted((root / relative_dir).glob("**/*.parquet"))
    if not files:
        raise FileNotFoundError(f"No Parquet files found under {root / relative_dir}")
    return files


def _read_episode_table(root: Path):
    try:
        import pyarrow.parquet as parquet
    except ImportError as exc:
        raise RuntimeError(
            "Dataset replay requires pyarrow in the robot Python environment. "
            "Install it with: .venv-robot/bin/python -m pip install 'pyarrow>=14,<22'"
        ) from exc
    tables = []
    for path in _parquet_files(root, "meta/episodes"):
        tables.append(parquet.read_table(path, columns=[
            "episode_index", "length", "data/chunk_index", "data/file_index",
            "dataset_from_index", "dataset_to_index",
        ]))
    if len(tables) == 1:
        return tables[0]
    return __import__("pyarrow").concat_tables(tables)


def list_episodes(dataset_root: str | Path) -> list[dict[str, int]]:
    """Return compact episode metadata without loading actions."""
    root = Path(dataset_root).expanduser().resolve()
    table = _read_episode_table(root)
    return [
        {
            "episode": int(row["episode_index"]),
            "length": int(row["length"]),
            "from": int(row["dataset_from_index"]),
            "to": int(row["dataset_to_index"]),
        }
        for row in table.to_pylist()
    ]


def load_episode_actions(
    dataset_root: str | Path,
    episode_index: int,
    start_frame: int = 0,
    max_frames: int | None = None,
) -> tuple[np.ndarray, float]:
    """Load one episode's actions and dataset FPS.

    The returned action order is the dataset order:
    ``left joints, left gripper, right joints, right gripper``.
    """
    root = Path(dataset_root).expanduser().resolve()
    info_path = root / "meta/info.json"
    if not info_path.is_file():
        raise FileNotFoundError(f"Missing LeRobot metadata: {info_path}")
    info = json.loads(info_path.read_text())
    fps = float(info.get("fps", 0.0))
    if not np.isfinite(fps) or fps <= 0:
        raise ValueError(f"Invalid dataset fps in {info_path}: {fps}")
    if start_frame < 0:
        raise ValueError("start_frame must be non-negative")
    if max_frames is not None and max_frames <= 0:
        raise ValueError("max_frames must be positive")

    episodes = list_episodes(root)
    selected = next((row for row in episodes if row["episode"] == episode_index), None)
    if selected is None:
        available = ", ".join(str(row["episode"]) for row in episodes[:20])
        raise ValueError(f"Unknown episode {episode_index}; available episode indices start with: {available}")
    if start_frame >= selected["length"]:
        raise ValueError(f"start_frame {start_frame} is outside episode length {selected['length']}")

    try:
        import pyarrow.parquet as parquet
    except ImportError as exc:
        raise RuntimeError(
            "Dataset replay requires pyarrow in the robot Python environment. "
            "Install it with: .venv-robot/bin/python -m pip install 'pyarrow>=14,<22'"
        ) from exc

    values: list[list[float]] = []
    data_files = _parquet_files(root, "data")
    for path in data_files:
        table = parquet.read_table(path, columns=["action", "episode_index", "frame_index"])
        for row in table.to_pylist():
            if row["episode_index"] != episode_index:
                continue
            frame = int(row["frame_index"])
            if frame < start_frame:
                continue
            if frame >= selected["length"]:
                continue
            values.append(row["action"])

    actions = np.asarray(values, dtype=np.float64)
    if max_frames is not None:
        actions = actions[:max_frames]
    if actions.ndim != 2 or actions.shape[1] != 16 or not len(actions):
        raise ValueError(
            f"Episode {episode_index} has no valid action rows after frame {start_frame}; "
            f"got shape {actions.shape}"
        )
    if not np.isfinite(actions).all():
        raise ValueError(f"Episode {episode_index} contains non-finite actions")
    return actions, fps


def _make_bridge_message(message_type, stamp, action: np.ndarray, kp_l, kd_l, kp_r, kd_r,
                         *, hold: bool, reanchor: bool, rigidity_mode: int = 2):
    msg = message_type()
    msg.header.stamp = stamp
    msg.ctl_trans_mod = 3
    msg.torque_feedforward_ctl = 0
    msg.if_reach_end = False
    msg.hold_position = bool(hold)
    msg.l_arm_if_use_jntlim = False
    msg.r_arm_if_use_jntlim = False
    msg.l_arm_limjnt_num = 0
    msg.r_arm_limjnt_num = 0
    msg.l_arm_des_msg = action[:7].tolist()
    msg.r_arm_des_msg = action[8:15].tolist()
    msg.l_arm_des_tor_msg = [0.0] * 7
    msg.r_arm_des_tor_msg = [0.0] * 7
    msg.l_arm_mdp_des_jnt_vel = [0.0] * 7
    msg.r_arm_mdp_des_jnt_vel = [0.0] * 7
    msg.if_init_use_ret = int(reanchor and not hold)
    msg.l_arm_rigidity_mod = [rigidity_mode] * 7
    msg.r_arm_rigidity_mod = [rigidity_mode] * 7
    msg.l_arm_jnt_kp = kp_l.tolist()
    msg.l_arm_jnt_kd = kd_l.tolist()
    msg.r_arm_jnt_kp = kp_r.tolist()
    msg.r_arm_jnt_kd = kd_r.tolist()
    msg.l_arm_if_jnt_stab = [0] * 7
    msg.r_arm_if_jnt_stab = [0] * 7
    return msg


def _make_gripper_message(message_type, action: np.ndarray, gripper, zero_deg, rate_hz):
    msg = message_type()
    msg.ctl_time = 1.0 / rate_hz
    target_deg = gripper.target_degrees(np.asarray([action[7], action[15]], dtype=np.float64))
    msg.l_arm_gripper_pos = float(target_deg[0] - zero_deg[0])
    msg.r_arm_gripper_pos = float(target_deg[1] - zero_deg[1])
    msg.l_arm_gripper_tor = 0.0
    msg.r_arm_gripper_tor = 0.0
    return msg


def _resolve_slow_factor(override: float | None, bridge_cfg: dict[str, Any]) -> float:
    value = override if override is not None else bridge_cfg.get("slow_factor", 1.0)
    try:
        factor = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("slow_factor must be finite and satisfy 0 < value <= 1") from exc
    if not np.isfinite(factor) or not 0.0 < factor <= 1.0:
        raise ValueError("slow_factor must be finite and satisfy 0 < value <= 1")
    return factor


def create_replay_node(args, actions: np.ndarray, fps: float, recorder: ReplayRecorder | None = None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from ctl_msgs_hw.msg import BridgeCtlMsg, GripperCtl, MidNodeCtlMsgRet
    from std_msgs.msg import Float64MultiArray

    config_path = Path(args.config).expanduser().resolve()
    config = _read_yaml(config_path)
    bridge_cfg = config.get("bridge", {})
    slow_factor = _resolve_slow_factor(args.slow_factor, bridge_cfg)
    gripper_cfg = _read_yaml((ROOT / config.get("gripper_config", "teleop/VR_controller/config/grippers.yaml")).resolve())
    controller_cfg = _read_yaml((ROOT / config.get("controller_config", "ros2_ws/src/ctl_msgs_hw/config/controllerConfig.yaml")).resolve())
    urdf_path = (ROOT / config.get("urdf", "ros2_ws/src/dual_arm_hw/urdf/dual_arm.urdf")).resolve()
    pd_path = (ROOT / config.get("pd_config", "ros2_ws/src/ctl_msgs_hw/config/initialPD.yaml")).resolve()
    lower, upper = load_joint_limits(urdf_path)
    pd_level = str(bridge_cfg.get("pd_level", "mid")).strip().lower()
    rigidity_by_level = {"low": 1, "mid": 2, "high": 3, "superhigh": 7}
    if pd_level not in rigidity_by_level:
        raise ValueError(f"pd_level must be one of {sorted(rigidity_by_level)}")
    rigidity_mode = int(bridge_cfg.get("rigidity_mode", rigidity_by_level[pd_level]))
    if rigidity_mode not in set(rigidity_by_level.values()):
        raise ValueError("rigidity_mode must be one of 1 (low), 2 (mid), 3 (high), or 7 (superhigh)")
    kp_l, kd_l, kp_r, kd_r = load_pd(pd_path, pd_level)
    max_step = _as_vector(bridge_cfg.get("max_joint_step_rad", 0.02), 14, "max_joint_step_rad")
    if np.any(max_step <= 0):
        raise ValueError("max_joint_step_rad must be positive")
    max_step *= slow_factor
    from pi05_bridge import GripperState
    gripper = GripperState(gripper_cfg)
    controller_grippers = controller_cfg.get("gripperControl", {})
    zero_deg = np.asarray([
        float(controller_grippers.get("l_gripper_ctl", {}).get("zero_pos_deg", 0.0)),
        float(controller_grippers.get("r_gripper_ctl", {}).get("zero_pos_deg", 0.0)),
    ], dtype=np.float64)
    period = 1.0 / (fps * args.speed)
    interpolate, publish_rate_hz = interpolation_settings(bridge_cfg, fps * args.speed)
    if interpolate:
        validate_controller_rate(controller_cfg, publish_rate_hz)
    state_timeout = float(bridge_cfg.get("state_timeout_s", 0.25))

    class ReplayNode(Node):
        def __init__(self):
            super().__init__("towel_dataset_replay")
            self.motion = bool(args.motion)
            self.actions = actions
            self.index = 0
            self.joints = None
            self.joint_received_at = 0.0
            self.gripper_feedback = None
            self.gripper_feedback_at = 0.0
            self.last_command = None
            self.reanchor = True
            self.done = False
            self.trajectory = None
            self.trajectory_started_at = None
            self.trajectory_start_index = 0
            self.last_log_at = -float("inf")
            self.initial_grippers = None
            self.command_pub = self.create_publisher(BridgeCtlMsg, "/dexbotCtl/bridge_ctl_msg", 1)
            self.gripper_pub = self.create_publisher(GripperCtl, "/gripper/ctl", 10)
            self.create_subscription(MidNodeCtlMsgRet, "/dexbotCtl/midMonitor_msg_send", self._joint_callback,
                                     qos_profile_sensor_data)
            self.create_subscription(Float64MultiArray, "/gripper/fdb", self._gripper_callback,
                                     qos_profile_sensor_data)
            self.timer = self.create_timer(1.0 / publish_rate_hz, self.step)
            self.get_logger().warning(
                f"dataset replay {'MOTION' if self.motion else 'DRY-RUN'}: "
                f"frames={len(actions)} rate={fps * args.speed:g}Hz slow_factor={slow_factor:g} "
                f"minimum_snap={interpolate} publish_rate_hz={publish_rate_hz:g}"
            )

        def _joint_callback(self, msg):
            left = np.asarray(msg.l_arm_pos_fdback, dtype=np.float64)
            right = np.asarray(msg.r_arm_pos_fdback, dtype=np.float64)
            if left.shape == (7,) and right.shape == (7,) and np.isfinite(np.r_[left, right]).all():
                self.joints = np.r_[left, right]
                self.joint_received_at = time.monotonic()

        def _gripper_callback(self, msg):
            values = np.asarray(msg.data, dtype=np.float64)
            if values.shape == (8,) and np.isfinite(values[[0, 4]]).all():
                self.gripper_feedback = values
                self.gripper_feedback_at = time.monotonic()

        def _publish_hold(self):
            if not self.motion or self.last_command is None:
                return
            stamp = self.get_clock().now().to_msg()
            self.command_pub.publish(_make_bridge_message(
                BridgeCtlMsg, stamp, self.last_command, kp_l, kd_l, kp_r, kd_r,
                hold=True, reanchor=True, rigidity_mode=rigidity_mode,
            ))

        def step(self):
            if self.index >= len(self.actions):
                self._publish_hold()
                self.timer.cancel()
                self.done = True
                self.get_logger().info("Replay completed; holding position")
                return
            now = time.monotonic()
            if self.motion:
                if (self.joints is None or now - self.joint_received_at > state_timeout
                        or now - self.gripper_feedback_at > state_timeout):
                    self._publish_hold()
                    self.reanchor = True
                    self.trajectory = None
                    if now - self.last_log_at >= 1.0:
                        self.last_log_at = now
                        self.get_logger().warning(
                            "Waiting for fresh joint and gripper feedback; replay frame is paused"
                        )
                    return
                state = np.r_[self.joints[:7], 0.0, self.joints[7:], 0.0]
            else:
                state = self.actions[self.index].copy()
            finished = False
            if interpolate:
                if self.trajectory is None:
                    remaining = self.actions[self.index:]
                    self.trajectory = MinimumSnapTrajectory(
                        np.arange(len(remaining) + 1) * period,
                        np.vstack([state[JOINT_INDICES], remaining[:, JOINT_INDICES]]),
                    )
                    self.trajectory_start_index = self.index
                    self.trajectory_started_at = time.monotonic()
                    self.initial_grippers = gripper.update(self.gripper_feedback) if self.motion else state[[7, 15]]
                elapsed = max(0.0, time.monotonic() - self.trajectory_started_at)
                finished = elapsed >= self.trajectory.duration - 1e-9
                if finished:
                    elapsed = self.trajectory.duration
                # The initial interval joins feedback to frame zero. Grippers
                # stay discrete and advance only when their source frame is due.
                offset = max(0, int(np.floor(elapsed / period + 1e-9)) - 1)
                self.index = min(self.trajectory_start_index + offset, len(self.actions) - 1)
                raw = self.actions[self.index]
                target = raw.copy()
                target[JOINT_INDICES] = self.trajectory.at(elapsed)
                if elapsed < period:
                    target[[7, 15]] = self.initial_grippers
            else:
                raw = self.actions[self.index]
                target = raw.copy()
            if self.motion and (time.monotonic() - self.joint_received_at > state_timeout
                                or time.monotonic() - self.gripper_feedback_at > state_timeout):
                self._publish_hold()
                self.reanchor = True
                self.trajectory = None
                return
            action = limit_action(target, state, lower, upper, max_step) if self.motion else target
            self.last_command = action
            if self.motion:
                stamp = self.get_clock().now().to_msg()
                self.command_pub.publish(_make_bridge_message(
                    BridgeCtlMsg, stamp, action, kp_l, kd_l, kp_r, kd_r,
                    hold=False, reanchor=self.reanchor, rigidity_mode=rigidity_mode,
                ))
                self.gripper_pub.publish(_make_gripper_message(GripperCtl, action, gripper, zero_deg,
                                                                publish_rate_hz))
                self.reanchor = False
            if recorder is not None:
                recorder.record(
                    args.start_frame + self.index,
                    np.r_[raw[:7], raw[8:15]],
                    np.r_[action[:7], action[8:15]],
                    self.joints,
                    gripper_feedback=self.gripper_feedback,
                )
            if now - self.last_log_at >= 1.0:
                self.last_log_at = now
                self.get_logger().info(
                    f"replay frame {self.index + 1}/{len(self.actions)} "
                    f"action={np.array2string(action, precision=3, suppress_small=True)}"
                )
            if not interpolate:
                self.index += 1
            elif finished:
                self.index = len(self.actions)

    return ReplayNode()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", default="/home/yanziyu/datasets/fold-towel-93")
    parser.add_argument("--episode", type=int, help="Episode index; required unless --list-episodes is used")
    parser.add_argument("--list-episodes", action="store_true")
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--max-frames", type=int)
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier")
    parser.add_argument("--slow-factor", type=float,
                        help="Override bridge.slow_factor from YAML (fallback: 1.0; 0 < value <= 1)")
    parser.add_argument("--config", default=str(ROOT / "config/pi05_deployment.yaml"))
    parser.add_argument("--motion", action="store_true", help="Publish robot and gripper commands")
    parser.add_argument(
        "--output-dir",
        default=str(ROOT / "outputs/replay_logs"),
        help="Directory for replay CSV and PNG measurements",
    )
    parser.add_argument("--no-plot", action="store_true", help="Save CSV without generating a PNG plot")
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper()),
                        format="%(asctime)s %(levelname)s %(message)s")
    if args.speed <= 0 or not np.isfinite(args.speed):
        parser.error("--speed must be finite and positive")
    if args.slow_factor is not None and (
        args.slow_factor <= 0 or args.slow_factor > 1 or not np.isfinite(args.slow_factor)
    ):
        parser.error("--slow-factor must satisfy 0 < value <= 1")
    if args.list_episodes:
        for row in list_episodes(args.dataset):
            print(f"episode={row['episode']} length={row['length']} frames={row['from']}:{row['to']}")
        return 0
    if args.episode is None:
        parser.error("--episode is required for replay; use --list-episodes first")
    if args.motion and args.max_frames is None:
        parser.error("--motion requires --max-frames; choose an explicit finite test window")
    actions, fps = load_episode_actions(args.dataset, args.episode, args.start_frame, args.max_frames)
    LOG.info("Loaded episode=%d frames=%d fps=%g", args.episode, len(actions), fps)
    recorder = ReplayRecorder(
        args.output_dir,
        args.episode,
        args.start_frame,
        make_plot=not args.no_plot,
    )

    import rclpy
    rclpy.init()
    node = None
    try:
        node = create_replay_node(args, actions, fps, recorder)
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        if node is not None:
            node._publish_hold()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        recorder.save()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
