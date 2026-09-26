#!/usr/bin/env python3
"""Validate and export complete raw episodes to LeRobot; no ROS needed."""
import argparse
import bisect
import json
import math
from pathlib import Path

import cv2
import numpy as np

from core import ACTION_NAMES, STATE_NAMES, REQUIRED, SCHEMA_VERSION, finite_vector, vectors, binary_states


def load_episode(path):
    meta = json.loads((path / "episode.json").read_text())
    if meta["status"] != "complete" or meta["schema_version"] != SCHEMA_VERSION:
        raise ValueError(f"Unsupported or incomplete episode: {path}")
    rows = [json.loads(line) for line in (path / "frames.jsonl").read_text().splitlines()]
    if len(rows) != meta["frames"] or not rows:
        raise ValueError(f"Frame count mismatch or empty episode: {path}")
    config = meta["config"]
    previous = None
    for index, row in enumerate(rows):
        if row["frame_index"] != index:
            raise ValueError("Noncontiguous frame indexes")
        for key, size in REQUIRED.items():
            finite_vector(row[key], size)
        binary_states(row["gripper_state"])
        stamp = float(row["timestamp_s"])
        if not math.isfinite(stamp) or (previous is not None and (
                stamp <= previous or stamp-previous > config["max_frame_gap_s"])):
            raise ValueError("Invalid capture timestamps or excessive gap")
        previous = stamp
        if set(row["images"]) != set(config["cameras"]):
            raise ValueError("Camera keys changed")
        for image_path in row["images"].values():
            img = cv2.imread(str(path / image_path))
            if img is None or img.shape != (config["height"], config["width"], 3):
                raise ValueError(f"Missing or corrupt image: {path / image_path}")
    return meta, rows


def resample(rows, fps):
    """Previous-sample hold on a real time grid, without compressing dropped time."""
    stamps = [row["timestamp_s"] for row in rows]
    count = int(math.floor((stamps[-1] - stamps[0]) * fps + 1e-8)) + 1
    for index in range(count):
        stamp = stamps[0] + index / fps
        yield index / fps, rows[bisect.bisect_right(stamps, stamp + 1e-9) - 1]


def features(config):
    result = {
        "observation.state": {"dtype": "float32", "shape": (16,), "names": STATE_NAMES},
        "action": {"dtype": "float32", "shape": (16,), "names": ACTION_NAMES},
        "observation.joint_velocity": {"dtype": "float32", "shape": (14,)},
        "observation.tcp": {"dtype": "float32", "shape": (12,)},
        "auxiliary.tcp_target": {"dtype": "float32", "shape": (12,)},
        "auxiliary.capture_time_s": {"dtype": "float64", "shape": (1,)},
    }
    for name in config["cameras"]:
        result[f"observation.images.{name}"] = {
            "dtype": "video", "shape": (config["height"], config["width"], 3),
            "names": ["height", "width", "channels"],
        }
    return result


def export(root, output, repo_id, validate_only=False):
    episodes = [path for path in sorted(root.glob("episode_*"))
                if path.is_dir() and not path.name.endswith(".partial")]
    if not episodes:
        raise ValueError("No complete episodes found")
    # Validate every image before creating an output directory or importing LeRobot.
    reference = None
    for path in episodes:
        meta, _ = load_episode(path)
        cfg = meta["config"]
        signature = ({key: cfg[key] for key in ("fps", "width", "height", "cameras")},
                     meta.get("calibration", {}))
        if reference is not None and signature != reference:
            raise ValueError("Camera or robot calibration changed; export as separate datasets")
        reference = signature
    if validate_only:
        print(f"Validated {len(episodes)} complete episodes")
        return
    if output.exists():
        raise ValueError(f"Output already exists: {output}")
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    dataset = LeRobotDataset.create(repo_id=repo_id, root=output, fps=cfg["fps"],
                                    robot_type="encos_dual_arm_towel", features=features(cfg),
                                    use_videos=True, image_writer_threads=4)
    try:
        for path in episodes:
            meta, rows = load_episode(path)
            for _, row in resample(rows, cfg["fps"]):
                state, action = vectors(row)
                frame = {
                    "observation.state": np.asarray(state, dtype=np.float32),
                    "action": np.asarray(action, dtype=np.float32),
                    "observation.joint_velocity": np.asarray(row["dq"], dtype=np.float32),
                    "observation.tcp": np.asarray(row["tcp"], dtype=np.float32),
                    "auxiliary.tcp_target": np.asarray(row["tcp_target"], dtype=np.float32),
                    "auxiliary.capture_time_s": np.asarray([row["timestamp_s"]], dtype=np.float64),
                    "task": meta["config"]["task"],
                }
                for name, image_path in row["images"].items():
                    frame[f"observation.images.{name}"] = cv2.cvtColor(
                        cv2.imread(str(path / image_path)), cv2.COLOR_BGR2RGB)
                dataset.add_frame(frame)
            dataset.save_episode()
        dataset.finalize()
        (output / "meta/towel_schema.json").write_text(json.dumps({
            "state_names": STATE_NAMES, "action_names": ACTION_NAMES,
            "schema_version": SCHEMA_VERSION,
            "gripper_encoding": "0=open, 1=closed; observed state vs commanded action",
            "joint_action_source": "/dexbotCtl/ctl_data_pub (post-IK position target)",
            "resampling": "previous sample hold on fixed FPS grid",
            "raw_episodes": [str(path.resolve()) for path in episodes],
            "source_calibration": reference[1],
        }, ensure_ascii=False, indent=2) + "\n")
    except BaseException:
        # Prevent a partially exported dataset from being mistaken for a complete one.
        (output / "EXPORT_INCOMPLETE").write_text("Export failed; use a new output path to retry.\n")
        raise
    print(f"Exported {len(episodes)} episodes to {output}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True, type=Path)
    parser.add_argument("--output", type=Path, default=Path("data/lerobot/towel_wipe"))
    parser.add_argument("--repo-id", default="local/towel_wipe")
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()
    export(args.root, args.output, args.repo_id, args.validate_only)
