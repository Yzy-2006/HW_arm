#!/usr/bin/env python3
"""Validate and export complete raw episodes to LeRobot; no ROS needed."""
import argparse
import bisect
import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml

from core import ACTION_NAMES, STATE_NAMES, REQUIRED, SCHEMA_VERSION, finite_vector, vectors, binary_states


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "config/export_lerobot.yaml"

# These are the fields that can be represented directly in a LeRobot frame.
# Raw dictionary-valued timestamps are exported as JSONL sidecars instead.
FIELD_SPECS = {
    "observation.state": {
        "dtype": "float32", "shape": (16,), "names": STATE_NAMES,
        "value": lambda row: vectors(row)[0],
    },
    "action": {
        "dtype": "float32", "shape": (16,), "names": ACTION_NAMES,
        "value": lambda row: vectors(row)[1],
    },
    "observation.joint_velocity": {
        "dtype": "float32", "shape": (14,),
        "value": lambda row: row["dq"],
    },
    "observation.tcp": {
        "dtype": "float32", "shape": (12,),
        "value": lambda row: row["tcp"],
    },
    "auxiliary.tcp_target": {
        "dtype": "float32", "shape": (12,),
        "value": lambda row: row["tcp_target"],
    },
    "auxiliary.capture_time_s": {
        "dtype": "float64", "shape": (1,),
        "value": lambda row: [row["timestamp_s"]],
    },
    "raw.q": {
        "dtype": "float32", "shape": (14,),
        "value": lambda row: row["q"],
    },
    "raw.dq": {
        "dtype": "float32", "shape": (14,),
        "value": lambda row: row["dq"],
    },
    "raw.joint_target": {
        "dtype": "float32", "shape": (14,),
        "value": lambda row: row["joint_target"],
    },
    "raw.gripper_state": {
        "dtype": "float32", "shape": (4,),
        "value": lambda row: row["gripper_state"],
    },
    "raw.frame_index": {
        "dtype": "int64", "shape": (1,),
        "value": lambda row: [row["frame_index"]],
    },
    "raw.timestamp_s": {
        "dtype": "float64", "shape": (1,),
        "value": lambda row: [row["timestamp_s"]],
    },
    "raw.monotonic_s": {
        "dtype": "float64", "shape": (1,),
        "value": lambda row: [row["monotonic_s"]],
    },
    "raw.ros_time_s": {
        "dtype": "float64", "shape": (1,),
        "value": lambda row: [row["ros_time_s"]],
    },
}


def load_export_config(path=None):
    """Load and validate the field-selection YAML used by export."""
    config_path = Path(path) if path is not None else DEFAULT_CONFIG
    try:
        data = yaml.safe_load(config_path.read_text()) or {}
    except OSError as exc:
        raise ValueError(f"Cannot read export config: {config_path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError("Export config must contain a YAML mapping")

    fields = data.get("fields", {})
    if not isinstance(fields, dict):
        raise ValueError("Export config 'fields' must be a mapping")
    unknown_fields = set(fields) - set(FIELD_SPECS)
    if unknown_fields:
        raise ValueError(f"Unknown export fields: {sorted(unknown_fields)}")
    selected_fields = {name: bool(fields.get(name, False)) for name in FIELD_SPECS}
    if not any(selected_fields.values()):
        raise ValueError("Export config selects no LeRobot fields")

    images = data.get("images", {})
    if not isinstance(images, dict):
        raise ValueError("Export config 'images' must be a mapping")
    metadata = data.get("metadata", {})
    if not isinstance(metadata, dict):
        raise ValueError("Export config 'metadata' must be a mapping")
    sidecars = data.get("raw_sidecars", {})
    if not isinstance(sidecars, dict):
        raise ValueError("Export config 'raw_sidecars' must be a mapping")
    unknown_images = set(images) - {"head", "left_wrist", "right_wrist"}
    if unknown_images:
        raise ValueError(f"Unknown export cameras: {sorted(unknown_images)}")
    unknown_metadata = set(metadata) - {"task", "raw_episodes", "source_calibration", "episode_metadata"}
    if unknown_metadata:
        raise ValueError(f"Unknown export metadata options: {sorted(unknown_metadata)}")
    unknown_sidecars = set(sidecars) - {"stream_timestamps", "camera_timestamps"}
    if unknown_sidecars:
        raise ValueError(f"Unknown raw sidecars: {sorted(unknown_sidecars)}")
    for name, value in (*fields.items(), *images.items(), *metadata.items(), *sidecars.items()):
        if not isinstance(value, bool):
            raise ValueError(f"Export config value for {name!r} must be true or false")
    return {
        "path": str(config_path),
        "fields": selected_fields,
        "images": {str(name): bool(value) for name, value in images.items()},
        "metadata": {
            "task": bool(metadata.get("task", True)),
            "raw_episodes": bool(metadata.get("raw_episodes", True)),
            "source_calibration": bool(metadata.get("source_calibration", True)),
            "episode_metadata": bool(metadata.get("episode_metadata", False)),
        },
        "raw_sidecars": {
            "stream_timestamps": bool(sidecars.get("stream_timestamps", False)),
            "camera_timestamps": bool(sidecars.get("camera_timestamps", False)),
        },
    }


def motion_filter_enabled(config):
    threshold = float(config.get("min_joint_delta_rad", 0.0))
    if not math.isfinite(threshold) or threshold < 0:
        raise ValueError("min_joint_delta_rad must be finite and nonnegative")
    return threshold > 0


def load_episode(path):
    meta = json.loads((path / "episode.json").read_text())
    if meta["status"] != "complete" or meta["schema_version"] != SCHEMA_VERSION:
        raise ValueError(f"Unsupported or incomplete episode: {path}")
    rows = [json.loads(line) for line in (path / "frames.jsonl").read_text().splitlines()]
    if len(rows) != meta["frames"] or not rows:
        raise ValueError(f"Frame count mismatch or empty episode: {path}")
    config = meta["config"]
    filtered = motion_filter_enabled(config)
    previous = None
    for index, row in enumerate(rows):
        if row["frame_index"] != index:
            raise ValueError("Noncontiguous frame indexes")
        for key, size in REQUIRED.items():
            finite_vector(row[key], size)
        binary_states(row["gripper_state"])
        stamp = float(row["timestamp_s"])
        if not math.isfinite(stamp) or (previous is not None and (
                stamp <= previous or (not filtered and stamp-previous > config["max_frame_gap_s"]))):
            raise ValueError("Invalid capture timestamps or excessive gap")
        previous = stamp
        if set(row["images"]) != set(config["cameras"]):
            raise ValueError("Camera keys changed")
        for image_path in row["images"].values():
            img = cv2.imread(str(path / image_path))
            if img is None or img.shape != (config["height"], config["width"], 3):
                raise ValueError(f"Missing or corrupt image: {path / image_path}")
    return meta, rows


def resample(rows, fps, *, compact=False):
    """Compact filtered samples, otherwise hold samples on the capture-time grid."""
    if compact:
        for index, row in enumerate(rows):
            yield index / fps, row
        return
    stamps = [row["timestamp_s"] for row in rows]
    count = int(math.floor((stamps[-1] - stamps[0]) * fps + 1e-8)) + 1
    for index in range(count):
        stamp = stamps[0] + index / fps
        yield index / fps, rows[bisect.bisect_right(stamps, stamp + 1e-9) - 1]


def features(config, export_config=None):
    export_config = export_config or load_export_config()
    result = {}
    for name, spec in FIELD_SPECS.items():
        if export_config["fields"].get(name):
            result[name] = {key: value for key, value in spec.items() if key != "value"}
    for name in config["cameras"]:
        if not export_config["images"].get(name, True):
            continue
        result[f"observation.images.{name}"] = {
            "dtype": "video", "shape": (config["height"], config["width"], 3),
            "names": ["height", "width", "channels"],
        }
    return result


def _selected_images(config, export_config):
    known = set(config["cameras"])
    unknown = set(export_config["images"]) - known
    if unknown:
        raise ValueError(f"Export config selects unknown cameras: {sorted(unknown)}")
    return [name for name in config["cameras"] if export_config["images"].get(name, True)]


def _write_sidecar(sidecars, name, episode_index, frame_index, row):
    if name not in sidecars:
        return
    value = row.get(name)
    if value is None:
        raise ValueError(f"Raw frame is missing {name}")
    sidecars[name].write(json.dumps({
        "episode_index": episode_index,
        "frame_index": frame_index,
        "value": value,
    }, ensure_ascii=False) + "\n")


def export(root, output, repo_id, validate_only=False, config_path=None):
    export_config = load_export_config(config_path)
    episodes = [path for path in sorted(root.glob("episode_*"))
                if path.is_dir() and not path.name.endswith(".partial")]
    if not episodes:
        raise ValueError("No complete episodes found")
    # Validate every image before creating an output directory or importing LeRobot.
    reference = None
    for path in episodes:
        meta, _ = load_episode(path)
        cfg = meta["config"]
        _selected_images(cfg, export_config)
        signature = ({key: cfg[key] for key in ("fps", "width", "height", "cameras")},
                     meta.get("calibration", {}), motion_filter_enabled(cfg))
        if reference is not None and signature != reference:
            raise ValueError("Camera, robot calibration or motion-filter mode changed; export as separate datasets")
        reference = signature
    if validate_only:
        print(f"Validated {len(episodes)} complete episodes")
        return
    if output.exists():
        raise ValueError(f"Output already exists: {output}")
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    selected_images = _selected_images(cfg, export_config)
    selected_features = features(cfg, export_config)
    if not selected_features:
        raise ValueError("Export config selects no output features")
    dataset = LeRobotDataset.create(repo_id=repo_id, root=output, fps=cfg["fps"],
                                    robot_type="encos_dual_arm_towel", features=selected_features,
                                    use_videos=bool(selected_images), image_writer_threads=4)
    sidecar_files = {}
    sidecar_names = [name for name, enabled in export_config["raw_sidecars"].items() if enabled]
    if export_config["metadata"]["episode_metadata"]:
        sidecar_names.append("episode_metadata")
    try:
        for name in sidecar_names:
            sidecar_path = output / "meta" / f"raw_{name}.jsonl"
            sidecar_files[name] = sidecar_path.open("w")
            if name == "episode_metadata":
                for episode_index, path in enumerate(episodes):
                    metadata = json.loads((path / "episode.json").read_text())
                    sidecar_files[name].write(json.dumps({
                        "episode_index": episode_index,
                        "metadata": metadata,
                    }, ensure_ascii=False) + "\n")
        for episode_index, path in enumerate(episodes):
            meta, rows = load_episode(path)
            for source_index, row in enumerate(rows):
                _write_sidecar(sidecar_files, "stream_timestamps", episode_index, source_index, row)
                _write_sidecar(sidecar_files, "camera_timestamps", episode_index, source_index, row)
            for _, row in resample(rows, cfg["fps"], compact=motion_filter_enabled(meta["config"])):
                frame = {}
                for name, spec in FIELD_SPECS.items():
                    if export_config["fields"].get(name):
                        frame[name] = np.asarray(spec["value"](row), dtype=spec["dtype"])
                if export_config["metadata"]["task"]:
                    frame["task"] = meta["config"]["task"]
                for name in selected_images:
                    image_path = row["images"][name]
                    frame[f"observation.images.{name}"] = cv2.cvtColor(
                        cv2.imread(str(path / image_path)), cv2.COLOR_BGR2RGB)
                dataset.add_frame(frame)
            dataset.save_episode()
        dataset.finalize()
        schema = {
            "state_names": STATE_NAMES, "action_names": ACTION_NAMES,
            "schema_version": SCHEMA_VERSION,
            "gripper_encoding": "0=open, 1=closed; observed state vs commanded action",
            "joint_action_source": "/dexbotCtl/ctl_data_pub (post-IK position target)",
            "resampling": ("saved frames in order on fixed FPS grid; capture gaps removed"
                           if motion_filter_enabled(cfg) else "previous sample hold on fixed FPS grid"),
            "export_config": export_config,
            "selected_features": list(selected_features),
            "selected_images": selected_images,
        }
        if export_config["metadata"]["raw_episodes"]:
            schema["raw_episodes"] = [str(path.resolve()) for path in episodes]
        if export_config["metadata"]["source_calibration"]:
            schema["source_calibration"] = reference[1]
        (output / "meta/towel_schema.json").write_text(
            json.dumps(schema, ensure_ascii=False, indent=2) + "\n")
    except BaseException:
        # Prevent a partially exported dataset from being mistaken for a complete one.
        (output / "EXPORT_INCOMPLETE").write_text("Export failed; use a new output path to retry.\n")
        raise
    finally:
        for handle in sidecar_files.values():
            handle.close()
    print(f"Exported {len(episodes)} episodes to {output}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True, type=Path)
    parser.add_argument("--output", type=Path, default=Path("data/lerobot/towel_wipe"))
    parser.add_argument("--repo-id", default="local/towel_wipe")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG,
                        help="YAML file selecting exported fields")
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()
    export(args.root, args.output, args.repo_id, args.validate_only, args.config)
