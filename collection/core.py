"""Hardware-independent schema, stream validation and atomic episode storage."""
import json
import math
import shutil
import time
import uuid
import copy
import threading
from collections import deque
from pathlib import Path

SCHEMA_VERSION = 2
JOINT_NAMES = [f"{side}_joint_{i}" for side in ("left", "right") for i in range(1, 8)]
TCP_NAMES = [f"{side}_tcp_{axis}" for side in ("left", "right")
             for axis in ("x", "y", "z", "roll", "pitch", "yaw")]
STATE_NAMES = JOINT_NAMES[:7] + ["left_gripper_closed"] + JOINT_NAMES[7:] + ["right_gripper_closed"]
ACTION_NAMES = JOINT_NAMES[:7] + ["left_gripper_closed"] + JOINT_NAMES[7:] + ["right_gripper_closed"]
REQUIRED = {"q": 14, "dq": 14, "joint_target": 14, "tcp": 12,
            "tcp_target": 12, "gripper_state": 4}

# 检查向量的长度和数据是否合法
def finite_vector(values, size):
    result = [float(x) for x in values]
    if len(result) != size or not all(math.isfinite(x) for x in result):
        raise ValueError(f"Expected {size} finite values")
    return result

# 检查数组是不是只有0和1
def binary_states(values):
    result = finite_vector(values, 4)
    if any(value not in (0, 1) for value in result):
        raise ValueError("Gripper states must be binary and calibrated (0=open, 1=closed)")
    return [int(value) for value in result]

# 连接机器人原始数据和学习数据
def vectors(sample):
    """Joint angles plus measured/commanded binary gripper states."""
    q = finite_vector(sample["q"], 14)
    g = binary_states(sample["gripper_state"])
    target = finite_vector(sample["joint_target"], 14)
    return q[:7]+[g[0]]+q[7:]+[g[1]], target[:7]+[g[2]]+target[7:]+[g[3]]

# 缓存机器人不同 ROS topic / 数据流中最新的一条数据
class StreamCache:
    def __init__(self):
        self.items = {}
        self.lock = threading.RLock()

    def update(self, name, values, received_at, source_time=None):
        with self.lock:
            self.items.pop(name, None)
            size = REQUIRED.get(name, 6)
            value = binary_states(values) if name == "gripper_state" else finite_vector(values, size)
            self.items[name] = (value, received_at, source_time)

    def snapshot(self, now, max_age, max_skew, *, received_after=None):
        with self.lock:
            items = dict(self.items)
        missing = set(REQUIRED) - items.keys()
        if missing:
            raise ValueError(f"Missing streams: {sorted(missing)}")
        if received_after is not None:
            pending = [key for key in REQUIRED if items[key][1] <= received_after]
            if pending:
                raise ValueError(f"Waiting for post-startup streams: {pending}")
        stamps = [items[key][1] for key in REQUIRED]
        stale = [f"{key}(age_s={now-items[key][1]:.4f})" for key in REQUIRED
                 if now - items[key][1] > max_age or items[key][1] > now]
        if stale:
            raise ValueError(f"Robot feedback/command is stale: {', '.join(stale)}; max_age_s={max_age:g}")
        if max(stamps) - min(stamps) > max_skew:
            raise ValueError("Robot stream receive-time skew exceeds limit")
        data = {key: list(value[0]) for key, value in items.items()
                if 0 <= now - value[1] <= max_age}
        data["stream_timestamps"] = {key: {"receive_monotonic_s": value[1],
                                            "source_ros_s": value[2]}
                                     for key, value in items.items() if key in data}
        return data

# 用来保存数据
class EpisodeWriter:
    
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
            relative = Path("images") / name / f"{self.frames:06d}.jpg"
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


class AsyncEpisodeWriter:
    """One ordered disk writer; bounded snapshots decouple capture from JPEG I/O."""

    def __init__(self, root, metadata, image_write, *, capacity=16, finish_timeout=10.0):
        if isinstance(capacity, bool) or not isinstance(capacity, int) or capacity < 1:
            raise ValueError("writer_queue_frames must be a positive integer")
        if not math.isfinite(finish_timeout) or finish_timeout <= 0:
            raise ValueError("writer_finish_timeout_s must be finite and positive")
        self.capacity, self.finish_timeout = capacity, finish_timeout
        self._image_write = image_write
        self._condition = threading.Condition()
        self._queue = deque()
        self._terminal = None
        self._error = None
        self._result = None
        self._min_frames = 1
        self.frames = 0  # Accepted captures; the underlying writer counts committed frames.
        self._written = self._high_water = 0
        self._write_last = self._write_total = self._write_max = 0.0
        self._camera_times = {}
        self._frame_camera_times = {}
        self.done = threading.Event()
        self._writer = EpisodeWriter(root, metadata, self._write_image)
        self.path = self._writer.path
        self._thread = threading.Thread(target=self._run, name="episode-writer", daemon=True)
        self._thread.start()

    def _write_image(self, path, image):
        start = time.monotonic()
        try:
            return self._image_write(path, image)
        finally:
            self._frame_camera_times[Path(path).parent.name] = time.monotonic() - start

    def check_error(self):
        with self._condition:
            if self._error is not None:
                raise RuntimeError(self._error)

    def add(self, sample, images, *, start_metadata=None):
        with self._condition:
            self.check_error()
            if self._terminal is not None:
                raise RuntimeError("Episode is closing")
            if len(self._queue) >= self.capacity:
                self.abort(f"Writer queue full (capacity={self.capacity}); episode is incomplete")
                self.check_error()
            # Own the queued data: camera/cache updates cannot change a captured frame.
            if start_metadata is not None and self.frames:
                raise RuntimeError("Capture start metadata must accompany the first frame")
            self._queue.append((copy.deepcopy(sample), {name: img.copy() for name, img in images.items()},
                                copy.deepcopy(start_metadata)))
            self.frames += 1
            self._high_water = max(self._high_water, len(self._queue))
            self._condition.notify()

    def stats(self):
        with self._condition:
            return dict(accepted=self.frames, written=self._written, queued=len(self._queue),
                        capacity=self.capacity, high_water=self._high_water,
                        write_last=self._write_last, write_total=self._write_total,
                        write_max=self._write_max, camera_times=dict(self._camera_times))

    def abort(self, reason):
        with self._condition:
            if self.done.is_set():
                return
            if self._error is None:
                self._error = str(reason)
            self._terminal = "abort"
            self._queue.clear()
            self._condition.notify()

    def _wait(self):
        if not self.done.wait(self.finish_timeout):
            self.abort(f"Writer drain timed out after {self.finish_timeout:g}s; episode is incomplete")
            if not self.done.is_set():
                raise TimeoutError(self._error)
        self._thread.join()
        self.check_error()
        return self._result

    def finish(self, min_frames=1):
        with self._condition:
            self.check_error()
            if self._terminal is not None:
                raise RuntimeError("Episode is closing")
            self._terminal, self._min_frames = "finish", min_frames
            self._condition.notify()
        return self._wait()

    def discard(self):
        with self._condition:
            self._terminal = "discard"
            self._queue.clear()
            self._condition.notify()
        return self._wait()

    def _run(self):
        try:
            while True:
                with self._condition:
                    self._condition.wait_for(lambda: self._queue or self._terminal is not None)
                    if self._terminal in ("abort", "discard"):
                        break
                    if not self._queue:
                        break  # Finish only after every accepted frame was written.
                    sample, images, start_metadata = self._queue.popleft()
                self._frame_camera_times = {}
                started = time.monotonic()
                if start_metadata is not None:
                    self._writer.metadata.update(start_metadata)
                    self._writer._write_metadata("recording")
                self._writer.add(sample, images)
                elapsed = time.monotonic() - started
                with self._condition:
                    self._written = self._writer.frames
                    self._write_last = elapsed
                    self._write_total += elapsed
                    self._write_max = max(self._write_max, elapsed)
                    self._camera_times = dict(self._frame_camera_times)
            # Serialize finalization against abort/timeout so an aborted episode
            # cannot subsequently be renamed as a complete recording.
            with self._condition:
                if self._terminal == "abort":
                    self._writer.abort(self._error)
                elif self._terminal == "discard":
                    self._writer.discard()
                else:
                    self._result = self._writer.finish(self._min_frames)
                self.done.set()
        except Exception as exc:
            with self._condition:
                self._error = self._error or f"Episode write failed: {exc}"
                self._terminal = "abort"
                self._queue.clear()
            try:
                self._writer.abort(self._error)
            except Exception as abort_error:
                with self._condition:
                    self._error += f"; abort metadata write failed: {abort_error}"
        finally:
            self.done.set()
