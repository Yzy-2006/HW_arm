"""Hardware-independent schema, stream validation and atomic episode storage."""
import json
import math
import shutil
import time
import uuid
from pathlib import Path

SCHEMA_VERSION = 2
JOINT_NAMES = [f"{side}_joint_{i}" for side in ("left", "right") for i in range(1, 8)]
TCP_NAMES = [f"{side}_tcp_{axis}" for side in ("left", "right")
             for axis in ("x", "y", "z", "roll", "pitch", "yaw")]
STATE_NAMES = JOINT_NAMES[:7] + ["left_gripper_closed"] + JOINT_NAMES[7:] + ["right_gripper_closed"]
ACTION_NAMES = JOINT_NAMES[:7] + ["left_gripper_closed"] + JOINT_NAMES[7:] + ["right_gripper_closed"]
REQUIRED = {"q": 14, "dq": 14, "joint_target": 14, "tcp": 12,
            "tcp_target": 12, "gripper_state": 4}


def finite_vector(values, size):
    result = [float(x) for x in values]
    if len(result) != size or not all(math.isfinite(x) for x in result):
        raise ValueError(f"Expected {size} finite values")
    return result


def binary_states(values):
    result = finite_vector(values, 4)
    if any(value not in (0, 1) for value in result):
        raise ValueError("Gripper states must be binary and calibrated (0=open, 1=closed)")
    return [int(value) for value in result]


def vectors(sample):
    """Joint angles plus measured/commanded binary gripper states."""
    q = finite_vector(sample["q"], 14)
    g = binary_states(sample["gripper_state"])
    target = finite_vector(sample["joint_target"], 14)
    return q[:7]+[g[0]]+q[7:]+[g[1]], target[:7]+[g[2]]+target[7:]+[g[3]]


class StreamCache:
    def __init__(self):
        self.items = {}

    def update(self, name, values, received_at, source_time=None):
        # Invalid messages invalidate the old value rather than extending its lifetime.
        self.items.pop(name, None)
        size = REQUIRED.get(name, 6)
        value = binary_states(values) if name == "gripper_state" else finite_vector(values, size)
        self.items[name] = (value, received_at, source_time)

    def snapshot(self, now, max_age, max_skew):
        missing = set(REQUIRED) - self.items.keys()
        if missing:
            raise ValueError(f"Missing streams: {sorted(missing)}")
        stamps = [self.items[key][1] for key in REQUIRED]
        if any(now - stamp > max_age or stamp > now for stamp in stamps):
            raise ValueError("Robot feedback/command is stale")
        if max(stamps) - min(stamps) > max_skew:
            raise ValueError("Robot stream receive-time skew exceeds limit")
        data = {key: value[0] for key, value in self.items.items()
                if 0 <= now - value[1] <= max_age}
        data["stream_timestamps"] = {key: {"receive_monotonic_s": value[1],
                                            "source_ros_s": value[2]}
                                     for key, value in self.items.items() if key in data}
        return data


class EpisodeWriter:
    """Write to a partial directory; only explicit stop commits a complete episode.

    A recorder failure keeps partial evidence but it is excluded from export.
    A discard deletes only the current uncommitted episode.
    """
    def __init__(self, root, metadata, image_write):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.name = f"episode_{time.time_ns()}_{uuid.uuid4().hex[:6]}"
        self.path = self.root / (self.name + ".partial")
        self.path.mkdir()
        self.metadata = dict(metadata, schema_version=SCHEMA_VERSION)
        self.image_write = image_write
        self.frames = 0
        self.closed = False
        self.file = (self.path / "frames.jsonl").open("w")
        self._write_metadata("recording")

    def _write_metadata(self, status, **extra):
        (self.path / "episode.json").write_text(json.dumps(
            dict(self.metadata, frames=self.frames, status=status, **extra),
            ensure_ascii=False, indent=2, allow_nan=False) + "\n")

    def add(self, sample, images):
        if self.closed:
            raise RuntimeError("Episode is closed")
        paths = {}
        for name, img in images.items():
            relative = Path("images") / name / f"{self.frames:06d}.png"
            path = self.path / relative
            path.parent.mkdir(exist_ok=True, parents=True)
            if not self.image_write(str(path), img):
                raise IOError(f"Could not save {path}")
            paths[name] = str(relative)
        self.file.write(json.dumps(dict(sample, frame_index=self.frames, images=paths),
                                   allow_nan=False) + "\n")
        self.file.flush()
        self.frames += 1

    def finish(self, min_frames=1):
        if self.closed:
            raise RuntimeError("Episode is closed")
        self.file.close()
        self.closed = True
        if self.frames < min_frames:
            self._write_metadata("too_short")
            return None
        self._write_metadata("complete")
        target = self.root / self.name
        self.path.rename(target)
        return target

    def abort(self, reason):
        if not self.closed:
            self.file.close()
            self.closed = True
            self._write_metadata("aborted", reason=str(reason))

    def discard(self):
        if self.closed:
            raise RuntimeError("Episode is closed")
        self.file.close()
        self.closed = True
        shutil.rmtree(self.path)
