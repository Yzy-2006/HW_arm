import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "collection"))
import filter_lerobot as filtering
from core import ACTION_NAMES, STATE_NAMES


def test_filter_compares_to_last_retained_frame_and_keeps_boundary():
    states = np.zeros((7, 16))
    states[:, 0] = [0, 0.0625, 0.125, 0.25, 0.3125, 0.5, 0.51]
    assert filtering.select_frames(states, states, 0.25) == [0, 3, 5, 6]
    assert filtering.select_frames(np.zeros((1, 16)), np.zeros((1, 16)), 1) == [0]


@pytest.mark.parametrize("key,column", [("state", 7), ("state", 15), ("action", 7), ("action", 15)])
def test_gripper_changes_survive_stationary_joints(key, column):
    arrays = {"state": np.zeros((6, 16)), "action": np.zeros((6, 16))}
    arrays[key][2:3, column] = 1
    assert filtering.select_frames(arrays["state"], arrays["action"], 1) == [0, 2, 3, 5]


def test_selection_uses_measured_joints_and_preserves_task_changes():
    states = np.zeros((5, 16))
    actions = states.copy()
    actions[:, 0] = np.arange(5)
    assert filtering.select_frames(states, actions, 0.02, [0, 0, 1, 0, 0]) == [0, 2, 3, 4]
    states[2:, 14] = 0.1
    assert filtering.select_frames(states, actions, 0.02) == [0, 2, 4]


@pytest.mark.parametrize("threshold", [0, -1, float("nan"), float("inf")])
def test_invalid_threshold(threshold):
    with pytest.raises(ValueError, match="finite and positive"):
        filtering.select_frames(np.zeros((2, 16)), np.zeros((2, 16)), threshold)


def test_invalid_frames_are_rejected():
    states = np.zeros((2, 16))
    states[1, 0] = np.nan
    with pytest.raises(ValueError, match="finite"):
        filtering.select_frames(states, np.zeros((2, 16)), 0.02)
    states[1, 0] = 0
    states[1, 7] = 0.5
    with pytest.raises(ValueError, match="binary"):
        filtering.select_frames(states, np.zeros((2, 16)), 0.02)


def make_source(root, *, videos=False):
    (root / "meta/episodes/chunk-000").mkdir(parents=True)
    (root / "data/chunk-000").mkdir(parents=True)
    features = {
        "observation.state": {"dtype": "float32", "shape": [16], "names": STATE_NAMES},
        "action": {"dtype": "float32", "shape": [16], "names": ACTION_NAMES},
        "auxiliary.capture_time_s": {"dtype": "float64", "shape": [1]},
    }
    if videos:
        for key in ("observation.images.head", "observation.images.left_wrist"):
            features[key] = {"dtype": "video", "shape": [64, 64, 3], "names": ["height", "width", "channels"]}
    info = {
        "codebase_version": "v3.0", "fps": 30, "total_episodes": 2, "total_frames": 12,
        "features": features, "robot_type": "encos_dual_arm_towel",
        "data_path": "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet",
        "video_path": "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4",
    }
    rows, episodes = [], []
    for episode_index in range(2):
        for index in range(6):
            state = np.zeros(16)
            state[0] = index * 0.004 + episode_index
            state[15] = int(index >= 3)
            action = np.zeros(16)
            action[0] = index * 0.1 + episode_index
            action[7] = int(index >= 2)
            rows.append({"observation.state": state.tolist(), "action": action.tolist(),
                         "auxiliary.capture_time_s": [100 + index * 0.2],
                         "timestamp": index / 30, "frame_index": index,
                         "episode_index": episode_index, "index": episode_index * 6 + index, "task_index": 0})
        episode = {"episode_index": episode_index, "length": 6, "data/chunk_index": 0, "data/file_index": 0,
                   "dataset_from_index": episode_index * 6, "dataset_to_index": (episode_index + 1) * 6}
        for camera_index, key in enumerate(name for name, spec in features.items() if spec["dtype"] == "video"):
            prefix = f"videos/{key}"
            episode.update({f"{prefix}/chunk_index": 0, f"{prefix}/file_index": 0,
                            f"{prefix}/from_timestamp": (episode_index * 6 + camera_index * 2) / 30,
                            f"{prefix}/to_timestamp": ((episode_index + 1) * 6 + camera_index * 2) / 30})
        episodes.append(episode)
    pq.write_table(pa.Table.from_pylist(rows), root / "data/chunk-000/file-000.parquet")
    pq.write_table(pa.Table.from_pylist(episodes), root / "meta/episodes/chunk-000/file-000.parquet")
    pq.write_table(pa.Table.from_pylist([{"task_index": 0, "task": "fold towel"}]), root / "meta/tasks.parquet")
    (root / "meta/info.json").write_text(json.dumps(info))
    (root / "meta/towel_schema.json").write_text(json.dumps({"schema_version": 2}))
    if videos:
        av = pytest.importorskip("av")
        for camera_index, key in enumerate(name for name, spec in features.items() if spec["dtype"] == "video"):
            path = filtering.video_path(root, info, episodes[0], key)
            path.parent.mkdir(parents=True)
            with av.open(str(path), "w") as container:
                stream = container.add_stream("libx264", rate=30)
                stream.width, stream.height, stream.pix_fmt = 64, 64, "yuv444p"
                stream.options = {"crf": "0", "preset": "ultrafast"}
                for index in range(12 + camera_index * 2):
                    color = 30 + (index - camera_index * 2) * 12 + camera_index * 20
                    frame = av.VideoFrame.from_ndarray(np.full((64, 64, 3), color, np.uint8), format="rgb24")
                    for packet in stream.encode(frame):
                        container.mux(packet)
                for packet in stream.encode():
                    container.mux(packet)
    return info, episodes, rows


def test_dry_run_selects_episodes_without_writing(tmp_path, monkeypatch):
    root = tmp_path / "source"
    make_source(root)
    info, episodes, tasks, report = filtering.plan_filter(root, 0.02, [1])
    assert [row["episode_index"] for row in episodes] == [1]
    assert tasks == {0: "fold towel"}
    assert report["source_frames"] == 6
    assert report["retained_frames"] == 4
    assert report["episodes"][0]["source_frame_indices"] == [0, 2, 3, 5]
    assert report["episodes"][0]["output_episode_index"] == 0
    output = tmp_path / "output"
    monkeypatch.setattr(sys, "argv", ["filter_lerobot.py", "--dataset", str(root), "--output", str(output),
                                     "--min-joint-delta-rad", "0.02", "--dry-run"])
    filtering.main()
    assert not output.exists()
    with pytest.raises(ValueError, match="existing episode"):
        filtering.plan_filter(root, 0.02, [99])


@pytest.mark.parametrize("output_name", ["source", "source/filtered", ".", "existing"])
def test_refuses_overwrites_and_overlapping_paths(tmp_path, monkeypatch, output_name):
    root = tmp_path / "source"
    make_source(root)
    (tmp_path / "existing").mkdir()
    monkeypatch.setattr(sys, "argv", ["filter_lerobot.py", "--dataset", str(root), "--output",
                                     str(tmp_path / output_name), "--min-joint-delta-rad", "0.02"])
    with pytest.raises(SystemExit):
        filtering.main()


def test_invalid_source_timestamps_and_incomplete_marker(tmp_path):
    root = tmp_path / "source"
    info, episodes, rows = make_source(root)
    rows[1]["timestamp"] = 0
    pq.write_table(pa.Table.from_pylist(rows), root / "data/chunk-000/file-000.parquet")
    with pytest.raises(ValueError, match="timestamps"):
        filtering.read_episode(root, info, episodes[0])
    (root / "FILTER_INCOMPLETE").touch()
    with pytest.raises(ValueError, match="incomplete"):
        filtering.load_source(root)


def fingerprints(root):
    return {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in root.rglob("*") if path.is_file()}


def test_real_export_rebuilds_aligned_videos_indexes_and_stats(tmp_path):
    pytest.importorskip("lerobot.datasets.lerobot_dataset")
    root = tmp_path / "source"
    make_source(root, videos=True)
    before = fingerprints(root)
    info, episodes, tasks, report = filtering.plan_filter(root, 0.02)
    output = tmp_path / "filtered"
    filtering.write_filtered(root, output, info, episodes, tasks, report)
    assert not (output / "FILTER_INCOMPLETE").exists()
    assert fingerprints(root) == before
    out_info, out_episodes, out_tasks = filtering.load_source(output)
    assert out_info["total_frames"] == 8
    assert out_info["total_episodes"] == 2
    assert out_info["fps"] == 30
    assert out_tasks == tasks
    for episode_index, episode in enumerate(out_episodes):
        rows = filtering.read_episode(output, out_info, episode)
        np.testing.assert_allclose([row["action"][0] for row in rows], np.array([0, 0.2, 0.3, 0.5]) + episode_index)
        np.testing.assert_allclose(np.asarray([row["auxiliary.capture_time_s"] for row in rows]).reshape(-1),
                                   [100, 100.4, 100.6, 101])
        assert [row["index"] for row in rows] == list(range(episode_index * 4, episode_index * 4 + 4))
        for camera_index, key in enumerate(["observation.images.head", "observation.images.left_wrist"]):
            timestamps = [episode[f"videos/{key}/from_timestamp"] + row["timestamp"] for row in rows]
            images = list(filtering.selected_video_frames(filtering.video_path(output, out_info, episode, key), timestamps, 30))
            expected = 30 + (np.array([0, 2, 3, 5]) + episode_index * 6) * 12 + camera_index * 20
            np.testing.assert_allclose([image.mean() for image in images], expected, atol=8)
    stats = json.loads((output / "meta/stats.json").read_text())
    assert stats["action"]["count"] == [8]
    np.testing.assert_allclose(stats["action"]["mean"][0], 0.75, atol=1e-6)
    saved_report = json.loads((output / "meta/offline_filter.json").read_text())
    assert saved_report == report


def test_failed_export_leaves_incomplete_marker(tmp_path, monkeypatch):
    module = pytest.importorskip("lerobot.datasets.lerobot_dataset")
    root = tmp_path / "source"
    make_source(root)
    info, episodes, tasks, report = filtering.plan_filter(root, 0.02)
    output = tmp_path / "failed"

    def fail(*args, **kwargs):
        raise RuntimeError("injected write failure")

    monkeypatch.setattr(module.LeRobotDataset, "add_frame", fail)
    with pytest.raises(RuntimeError, match="injected write failure"):
        filtering.write_filtered(root, output, info, episodes, tasks, report)
    assert (output / "FILTER_INCOMPLETE").exists()
