#!/usr/bin/env python3
"""Filter an exported towel LeRobot v3 dataset into a new local dataset."""

import argparse
from contextlib import ExitStack, closing
import copy
import json
import math
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq

from core import ACTION_NAMES, STATE_NAMES


JOINT_INDICES = [*range(7), *range(8, 15)]
GRIPPER_INDICES = [7, 15]
GENERATED_FEATURES = {"timestamp", "frame_index", "episode_index", "index", "task_index"}


def select_frames(states, actions, threshold, tasks=None):
    """Compare measured joints to the last kept frame; retain gripper events/endpoints."""
    if not math.isfinite(threshold) or threshold <= 0:
        raise ValueError("min_joint_delta_rad must be finite and positive")
    states = np.asarray(states, dtype=np.float64)
    actions = np.asarray(actions, dtype=np.float64)
    if states.ndim != 2 or states.shape[1] != 16 or not len(states) or actions.shape != states.shape:
        raise ValueError("observation.state and action must be nonempty (N, 16) arrays")
    if not np.isfinite(states).all() or not np.isfinite(actions).all():
        raise ValueError("observation.state and action must be finite")
    grippers = np.concatenate((states[:, GRIPPER_INDICES], actions[:, GRIPPER_INDICES]), axis=1)
    if not np.isin(grippers, [0, 1]).all():
        raise ValueError("Gripper states and actions must be binary (0 or 1)")
    if tasks is not None and len(tasks) != len(states):
        raise ValueError("Task count does not match frame count")
    kept = [0]
    joints = states[:, JOINT_INDICES]
    for index in range(1, len(states)):
        previous = kept[-1]
        moved = np.max(np.abs(joints[index] - joints[previous])) >= threshold
        event = np.any(grippers[index] != grippers[previous])
        task_changed = tasks is not None and tasks[index] != tasks[previous]
        if moved or event or task_changed or index == len(states) - 1:
            kept.append(index)
    return kept


def load_source(root):
    if (root / "FILTER_INCOMPLETE").exists() or (root / "EXPORT_INCOMPLETE").exists():
        raise ValueError("Source dataset is marked incomplete")
    info = json.loads((root / "meta/info.json").read_text())
    if info.get("codebase_version") != "v3.0":
        raise ValueError("Only exported LeRobot v3.0 datasets are supported")
    fps = info["fps"]
    if not math.isfinite(fps) or fps <= 0 or int(fps) != fps:
        raise ValueError("Dataset FPS must be a positive integer")
    features = info["features"]
    for name, names in (("observation.state", STATE_NAMES), ("action", ACTION_NAMES)):
        spec = features.get(name, {})
        if spec.get("shape") != [16] or spec.get("names") != names:
            raise ValueError(f"{name} must use the project's 16-dimensional towel schema")
    for name, spec in features.items():
        if spec["dtype"] == "video":
            if len(spec["shape"]) != 3 or spec["shape"][2] != 3 or spec.get("info", {}).get("is_depth_map"):
                raise ValueError(f"Only RGB video features are supported: {name}")
        elif name not in GENERATED_FEATURES:
            # Image-backed or nested features need a different source reader.
            try:
                np.dtype(spec["dtype"])
            except TypeError as exc:
                raise ValueError(f"Unsupported feature type: {name} ({spec['dtype']})") from exc
    files = sorted((root / "meta/episodes").glob("**/*.parquet"))
    episodes = [row for path in files for row in pq.read_table(path).to_pylist()]
    episodes.sort(key=lambda row: row["episode_index"])
    ids = [row["episode_index"] for row in episodes]
    if not episodes or len(episodes) != info["total_episodes"] or ids != list(range(len(episodes))):
        raise ValueError("Episode metadata count or indexes are invalid")
    if sum(row["length"] for row in episodes) != info["total_frames"]:
        raise ValueError("Episode lengths do not match total_frames")
    task_rows = pq.read_table(root / "meta/tasks.parquet").to_pylist()
    tasks = {row["task_index"]: row.get("task", row.get("__index_level_0__")) for row in task_rows}
    if not tasks or not all(isinstance(task, str) for task in tasks.values()):
        raise ValueError("Invalid task metadata")
    return info, episodes, tasks


def read_episode(root, info, episode):
    path = root / info["data_path"].format(
        chunk_index=episode["data/chunk_index"], file_index=episode["data/file_index"]
    )
    table = pq.read_table(path, filters=[("episode_index", "=", episode["episode_index"])])
    table = table.sort_by([("frame_index", "ascending")])
    rows = table.to_pylist()
    if len(rows) != episode["length"] or not rows:
        raise ValueError(f"Episode {episode['episode_index']}: frame count mismatch")
    if [row["frame_index"] for row in rows] != list(range(len(rows))):
        raise ValueError("Source frame indexes are not contiguous")
    if [row["index"] for row in rows] != list(range(episode["dataset_from_index"], episode["dataset_to_index"])):
        raise ValueError("Source global indexes do not match episode metadata")
    timestamps = np.asarray([row["timestamp"] for row in rows], dtype=np.float64)
    if not np.isfinite(timestamps).all() or not np.allclose(
        timestamps, np.arange(len(rows)) / info["fps"], atol=1e-4, rtol=0
    ):
        raise ValueError("Source timestamps do not match its fixed FPS grid")
    return rows


def video_path(root, info, episode, key):
    prefix = f"videos/{key}"
    return root / info["video_path"].format(
        video_key=key, chunk_index=episode[f"{prefix}/chunk_index"],
        file_index=episode[f"{prefix}/file_index"],
    )


def selected_video_frames(path, timestamps, fps):
    """Decode sequentially from a keyframe, matching absolute video timestamps."""
    import av

    tolerance = min(1e-3, 0.25 / fps)
    with av.open(str(path)) as container:
        stream = container.streams.video[0]
        container.seek(max(0, int(timestamps[0] / stream.time_base) - 1), stream=stream, backward=True)
        decoded = container.decode(stream)
        for target in timestamps:
            for frame in decoded:
                if frame.pts is None:
                    continue
                stamp = float(frame.pts * stream.time_base)
                if stamp < target - tolerance:
                    continue
                if abs(stamp - target) > tolerance:
                    raise ValueError(f"Video timestamp mismatch in {path}: requested {target}, decoded {stamp}")
                yield frame.to_ndarray(format="rgb24")
                break
            else:
                raise ValueError(f"Video ended before timestamp {target}: {path}")


def plan_filter(root, threshold, episode_ids=None):
    info, episodes, tasks = load_source(root)
    if episode_ids is not None:
        requested = set(episode_ids)
        if not requested or requested - {row["episode_index"] for row in episodes}:
            raise ValueError("--episodes must select existing episode indexes")
        episodes = [row for row in episodes if row["episode_index"] in requested]
    report = {
        "source_dataset": str(root), "min_joint_delta_rad": threshold, "fps": info["fps"],
        "comparison": "maximum absolute measured-joint delta from the last retained frame",
        "preserve": ["first and last frame", "observed and commanded gripper changes", "task changes"],
        "timeline": "retained frames on the original fixed FPS grid; removed time gaps are compressed",
        "episodes": [],
    }
    for output_index, episode in enumerate(episodes):
        rows = read_episode(root, info, episode)
        task_ids = [row["task_index"] for row in rows]
        if set(task_ids) - tasks.keys():
            raise ValueError("Frame references an unknown task")
        kept = select_frames([r["observation.state"] for r in rows], [r["action"] for r in rows], threshold, task_ids)
        for key, spec in info["features"].items():
            if spec["dtype"] == "video" and not video_path(root, info, episode, key).is_file():
                raise ValueError(f"Missing source video: {key}, episode {episode['episode_index']}")
        entry = {
            "source_episode_index": episode["episode_index"], "output_episode_index": output_index,
            "source_frames": len(rows), "retained_frames": len(kept),
            "source_frame_indices": kept,
            "source_timestamps_s": [rows[index]["timestamp"] for index in kept],
        }
        report["episodes"].append(entry)
        print(f"episode={episode['episode_index']} frames={len(rows)} -> {len(kept)} "
              f"retained={len(kept) / len(rows):.1%}", flush=True)
    report["source_frames"] = sum(row["source_frames"] for row in report["episodes"])
    report["retained_frames"] = sum(row["retained_frames"] for row in report["episodes"])
    print(f"Total: {report['source_frames']} -> {report['retained_frames']} frames "
          f"({report['retained_frames'] / report['source_frames']:.1%} retained)", flush=True)
    return info, episodes, tasks, report


def write_filtered(root, output, info, episodes, tasks, report):
    validate_output(root, output)
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    features = {
        name: {key: copy.deepcopy(spec[key]) for key in ("dtype", "shape", "names") if key in spec}
        for name, spec in info["features"].items() if name not in GENERATED_FEATURES
    }
    video_keys = [key for key, spec in features.items() if spec["dtype"] == "video"]
    numeric_keys = [key for key in features if key not in video_keys]
    dataset = None
    # A failed conversion must never look like a complete training dataset.
    try:
        dataset = LeRobotDataset.create(
            repo_id=f"local/{output.name}", root=output, fps=int(info["fps"]),
            features=features, robot_type=info.get("robot_type"), use_videos=bool(video_keys),
            image_writer_threads=4,
        )
        (output / "FILTER_INCOMPLETE").write_text("Offline filtering is incomplete. Retry with a new output path.\n")
        for episode, entry in zip(episodes, report["episodes"]):
            rows = read_episode(root, info, episode)
            kept = entry["source_frame_indices"]
            with ExitStack() as stack:
                readers = {}
                for key in video_keys:
                    offset = episode[f"videos/{key}/from_timestamp"]
                    timestamps = [offset + rows[index]["timestamp"] for index in kept]
                    readers[key] = stack.enter_context(closing(selected_video_frames(
                        video_path(root, info, episode, key), timestamps, info["fps"]
                    )))
                for index in kept:
                    row = rows[index]
                    frame = {key: np.asarray(row[key], dtype=features[key]["dtype"]).reshape(features[key]["shape"])
                             for key in numeric_keys}
                    frame["task"] = tasks[row["task_index"]]
                    for key in video_keys:
                        frame[key] = next(readers[key])
                    dataset.add_frame(frame)
            dataset.save_episode()
            print(f"Saved episode={entry['output_episode_index']} "
                  f"(source={entry['source_episode_index']}, frames={len(kept)})", flush=True)
        dataset.finalize()
        schema_path = root / "meta/towel_schema.json"
        if schema_path.exists():
            schema = json.loads(schema_path.read_text())
            schema["resampling"] = report["timeline"]
            schema["offline_filter"] = {"min_joint_delta_rad": report["min_joint_delta_rad"],
                                        "provenance": "meta/offline_filter.json"}
            (output / "meta/towel_schema.json").write_text(json.dumps(schema, ensure_ascii=False, indent=2) + "\n")
        (output / "meta/offline_filter.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    except BaseException as exc:
        if output.exists() and not isinstance(exc, FileExistsError):
            (output / "FILTER_INCOMPLETE").write_text("Offline filtering failed. Retry with a new output path.\n")
        raise
    finally:
        if dataset is not None:
            dataset.finalize()
    (output / "FILTER_INCOMPLETE").unlink()
    print(f"Filtered dataset saved to {output}", flush=True)


def validate_output(root, output):
    if output == root or output.is_relative_to(root) or root.is_relative_to(output):
        raise ValueError("--output must be separate from the source dataset")
    if output.exists():
        raise ValueError(f"Output already exists: {output}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, type=Path, help="Existing local LeRobot v3 dataset")
    parser.add_argument("--output", type=Path, help="New dataset directory; must not exist")
    parser.add_argument("--min-joint-delta-rad", required=True, type=float)
    parser.add_argument("--episodes", nargs="+", type=int, help="Optional source episode indexes (renumbered in output)")
    parser.add_argument("--dry-run", action="store_true", help="Only report retained frame counts; no video decoding or writes")
    args = parser.parse_args()
    if not math.isfinite(args.min_joint_delta_rad) or args.min_joint_delta_rad <= 0:
        parser.error("--min-joint-delta-rad must be finite and positive")
    root = args.dataset.expanduser().resolve()
    output = args.output.expanduser().resolve() if args.output is not None else None
    if not args.dry_run and output is None:
        parser.error("--output is required unless --dry-run is specified")
    if output is not None:
        try:
            validate_output(root, output)
        except ValueError as exc:
            parser.error(str(exc))
    info, episodes, tasks, report = plan_filter(root, args.min_joint_delta_rad, args.episodes)
    if not args.dry_run:
        write_filtered(root, output, info, episodes, tasks, report)


if __name__ == "__main__":
    main()
