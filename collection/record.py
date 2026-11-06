#!/usr/bin/env python3
"""Passive dual-arm recorder. Does not publish any robot motion commands."""
import argparse
import hashlib
import json
import os
import threading
import time
from time import thread_time
from pathlib import Path

import cv2
import numpy as np
import yaml

from core import AsyncEpisodeWriter, StreamCache

ROOT = Path(__file__).resolve().parents[1]

WINDOW_TITLES = {
    "head": "Head camera",
    "left_wrist": "Left wrist camera",
    "right_wrist": "Right wrist camera",
}


class RecordingDiagnostics:
    """Bounded timing counters; emit summaries rather than logging every frame."""

    def __init__(self, now):
        self.started_at = self.last_report = now
        self.last_tick = None
        self.tick_count = self.tick_total = self.tick_max = 0
        self.frame_gap = self.frame_gap_max = 0.0
        self.write_count = self.write_total = self.write_max = self.last_write = 0
        self.camera_write_s = {}
        self.camera_meta = {}
        self.camera_observed_at = None
        self.previous_ids = {}
        self.duplicate_skips = 0
        self.motion_skips = 0
        self.exec_count = self.exec_total = self.exec_max = self.exec_last = 0
        self.cpu_total = self.cpu_max = self.cpu_last = 0
        self.body_last = 0.

    def finished(self, wall, cpu):
        self.exec_count += 1
        self.exec_last, self.cpu_last = wall, cpu
        self.exec_total += wall
        self.cpu_total += cpu
        self.exec_max = max(self.exec_max, wall)
        self.cpu_max = max(self.cpu_max, cpu)

    def summary(self):
        return dict(callback_interval_max_ms=self.tick_max * 1000,
                    sample_body_last_ms=self.body_last * 1000,
                    callback_exec_last_ms=self.exec_last * 1000,
                    callback_exec_max_ms=self.exec_max * 1000,
                    callback_thread_cpu_last_ms=self.cpu_last * 1000,
                    frame_gap_max_ms=self.frame_gap_max * 1000,
                    duplicate_skips=self.duplicate_skips, cameras=self.camera_meta)

    def tick(self, now, last_frame_time):
        if self.last_tick is not None:
            interval = now - self.last_tick
            self.tick_count += 1
            self.tick_total += interval
            self.tick_max = max(self.tick_max, interval)
        self.last_tick = now
        self.frame_gap = now - last_frame_time
        self.frame_gap_max = max(self.frame_gap_max, self.frame_gap)

    def wrote(self, elapsed):
        self.last_write = elapsed
        self.write_count += 1
        self.write_total += elapsed
        self.write_max = max(self.write_max, elapsed)

    def report(self, now, frames, *, force=False):
        if not force and now - self.last_report < 2.0:
            return None
        self.last_report = now
        camera_details = []
        for name, meta in sorted(self.camera_meta.items()):
            frame = meta['frame_number']
            previous = self.previous_ids.get(name)
            step = frame - previous if previous is not None else 'n/a'
            received = meta.get('receive_monotonic_s')
            observed_at = self.camera_observed_at if self.camera_observed_at is not None else now
            age = f"{(observed_at - received) * 1000:.1f}" if received is not None else 'n/a'
            arrival = meta.get('arrival_interval_ms')
            arrival_text = (f", arrival_ms(last/max)={arrival:.1f}/{meta['arrival_interval_max_ms']:.1f}"
                            f", device_interval_ms={meta['device_interval_ms']:.1f}"
                            f", receiver_frame_gaps={meta['receiver_frame_gaps']}" if arrival is not None else '')
            camera_details.append(f"{name}(frame={frame}, previous_saved={previous}, "
                                  f"step={step}, age_ms={age}{arrival_text})")
        per_camera = ', '.join(f"{name}={elapsed * 1000:.1f}"
                               for name, elapsed in sorted(self.camera_write_s.items()))
        duration = now - self.started_at
        return (
            f"Capture diagnostics: frames={frames}, fps={frames / duration if duration > 0 else 0:.1f}, "
            f"callback_interval_ms(avg/max)={self.tick_total / max(1, self.tick_count) * 1000:.1f}/"
            f"{self.tick_max * 1000:.1f}, frame_gap_ms(current/max)={self.frame_gap * 1000:.1f}/"
            f"{self.frame_gap_max * 1000:.1f}, "
            f"sample_body_ms={self.body_last * 1000:.1f}, "
            f"callback_exec_ms(last/avg/max)={self.exec_last * 1000:.1f}/"
            f"{self.exec_total / max(1, self.exec_count) * 1000:.1f}/{self.exec_max * 1000:.1f}, "
            f"callback_thread_cpu_ms(last/avg/max)={self.cpu_last * 1000:.1f}/"
            f"{self.cpu_total / max(1, self.exec_count) * 1000:.1f}/{self.cpu_max * 1000:.1f}, "
            f"write_ms(last/avg/max)={self.last_write * 1000:.1f}/"
            f"{self.write_total / max(1, self.write_count) * 1000:.1f}/{self.write_max * 1000:.1f}, "
            f"jpeg_write_ms=[{per_camera}], duplicate_skips={self.duplicate_skips}, "
            f"motion_skips={self.motion_skips}, "
            f"cameras=[{'; '.join(camera_details)}]"
        )


class OpenCVPreview:
    """Display the recorder's latest frames in one window per configured camera."""

    def __init__(self, camera_names, *, cv2_module=cv2):
        self.camera_names = tuple(camera_names)
        self.cv2 = cv2_module
        self.visible = True
        self._opened = False

    def show(self, images):
        if not self.visible:
            return False
        if not self._opened:
            for name in self.camera_names:
                self.cv2.namedWindow(WINDOW_TITLES.get(name, name), self.cv2.WINDOW_NORMAL)
            self._opened = True
        for name in self.camera_names:
            self.cv2.imshow(WINDOW_TITLES.get(name, name), images[name])
        key = self.cv2.waitKey(1) & 0xFF
        if key in (27, ord("q"), ord("Q")):
            self.close()
        return self.visible

    def close(self):
        if self.visible:
            self.visible = False
            self.cv2.destroyAllWindows()


def run_opencv_preview(cameras, camera_names, *, startup_timeout_s=5.0):
    """Run a standalone preview until q/Esc, Ctrl+C, or a camera failure."""
    preview = OpenCVPreview(camera_names)
    deadline = time.monotonic() + startup_timeout_s
    try:
        while preview.visible:
            try:
                images, _ = cameras.snapshot(time.monotonic())
            except ValueError as error:
                if str(error) == "Waiting for all cameras" and time.monotonic() < deadline:
                    time.sleep(0.02)
                    continue
                raise
            preview.show(images)
    except KeyboardInterrupt:
        pass
    finally:
        preview.close()

# 为每一个配置好的RealSense相机创建一个独立线程来不断接受图像
class Cameras:
    # 根据序列号创建摄像头并给每一个摄像头开一个接受线程
    def __init__(self, config):
        import pyrealsense2 as rs
        self.rs, self.config = rs, config
        self.pipelines, self.threads, self.latest = {}, [], {}
        self.lock, self.stop = threading.Lock(), threading.Event()
        self.error = None
        serials = list(config["cameras"].values())
        if len(set(serials)) != len(serials) or not all(serials):
            raise ValueError("Camera serials must be nonempty and distinct")
        try:
            for name, serial in config["cameras"].items():
                pipeline, rs_config = rs.pipeline(), rs.config()
                rs_config.enable_device(str(serial))
                rs_config.enable_stream(rs.stream.color, config["width"], config["height"],
                                        rs.format.bgr8, config["fps"])
                pipeline.start(rs_config)
                self.pipelines[name] = pipeline
                thread = threading.Thread(target=self._receive, args=(name, pipeline), daemon=True)
                thread.start()
                self.threads.append(thread)
        except Exception:
            self.close()
            raise
    # 不断接收图像并保存帧数、时间等信息
    def _receive(self, name, pipeline):
        previous = None
        arrival_max = 0.
        frame_gaps = 0
        try:
            while not self.stop.is_set():
                frames = pipeline.wait_for_frames(1000)
                color = frames.get_color_frame()
                if not color:
                    continue
                received = time.monotonic()
                metadata = {
                    "frame_number": color.get_frame_number(),
                    "device_timestamp_ms": color.get_timestamp(),
                    "timestamp_domain": str(color.get_frame_timestamp_domain()),
                    "receive_monotonic_s": received,
                }
                if previous is not None:
                    arrival = (received - previous['receive_monotonic_s']) * 1000
                    arrival_max = max(arrival_max, arrival)
                    frame_gaps += max(0, metadata['frame_number'] - previous['frame_number'] - 1)
                    metadata.update(arrival_interval_ms=arrival, arrival_interval_max_ms=arrival_max,
                                    device_interval_ms=metadata['device_timestamp_ms'] - previous['device_timestamp_ms'],
                                    receiver_frame_gaps=frame_gaps)
                previous = metadata
                entry = (np.asanyarray(color.get_data()).copy(), metadata)
                with self.lock:
                    self.latest[name] = entry
        except Exception as exc:
            if not self.stop.is_set():
                self.error = str(exc)
    def frame_metadata(self):
        """Read camera progress even when freshness checks reject a snapshot."""
        with self.lock:
            return {name: dict(entry[1]) for name, entry in self.latest.items()}

    # 从所有相机中取出最新的数据，检查后返回
    def snapshot(self, now):
        with self.lock:
            latest = dict(self.latest)
        if self.error:
            raise ValueError(f"Camera receive failure: {self.error}")
        if set(latest) != set(self.pipelines):
            raise ValueError("Waiting for all cameras")
        stamps = [entry[1]["receive_monotonic_s"] for entry in latest.values()]
        if self.config.get("camera_timing_checks_enabled", True):
            if any(now - stamp > self.config["max_camera_age_s"] for stamp in stamps):
                raise ValueError("Camera frame is stale")
            if max(stamps) - min(stamps) > self.config["max_camera_skew_s"]:
                raise ValueError("Camera receive-time skew exceeds limit")
        return ({name: entry[0] for name, entry in latest.items()},
                {name: entry[1] for name, entry in latest.items()})
    # 退出接口
    def close(self):
        self.stop.set()
        for thread in self.threads:
            thread.join(timeout=1.5)
        for pipeline in self.pipelines.values():
            pipeline.stop()

# 用于处理与ROS的交互，将各个topic的信息整合为一帧
def make_node(config, cameras, *, show_opencv=False, status_callback=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.qos import qos_profile_sensor_data
    from ctl_msgs_hw.msg import CtlInformation, FkDataPub, MidNodeCtlMsgRet, MidNodeCtlMsgSend
    from std_msgs.msg import Int8MultiArray, Bool
    from sensor_msgs.msg import Image
    from std_srvs.srv import SetBool, Trigger

    class Recorder(Node):
        def __init__(self):
            super().__init__("towel_recorder")
            self.cache, self.episode = StreamCache(), None
            self.retiring_episode = None
            self.writer_finish_timeout_s = float(config.get("writer_finish_timeout_s", 10.0))
            self.startup_timeout_s = float(config.get("startup_timeout_s", 3.0))
            if not np.isfinite(self.startup_timeout_s) or self.startup_timeout_s <= 0:
                raise ValueError("startup_timeout_s must be finite and positive")
            self.feedback_group = MutuallyExclusiveCallbackGroup()
            self.prepared_at = None
            self.startup_wait_reason = "Waiting for first capture"
            self.last_startup_log = -float("inf")
            self.last_recording_error = ""
            self.start_time, self.last_frame_time = None, None
            self.last_saved_q = None
            self.last_saved_gripper_state = None
            self.motion_skip_pending = False
            self.camera_ids = {}
            self.diagnostics = None
            self.status = self.create_publisher(Bool, "/towel/recording", 10)
            self.publish_rviz = bool(config.get("publish_rviz", False))
            self.opencv_preview = OpenCVPreview(config["cameras"]) if show_opencv else None
            self.image_publishers = {}
            self._last_visualization_error = ""
            self._last_opencv_error = ""
            if self.publish_rviz:
                self.image_publishers = {
                    name: self.create_publisher(
                        Image, f"/towel/camera/{name}/image_raw", qos_profile_sensor_data
                    )
                    for name in config["cameras"]
                }
            self.create_timer(0.2, self.publish_status)
            subs = [
                (MidNodeCtlMsgRet, "/dexbotCtl/midMonitor_msg_send", self.joints),
                (CtlInformation, "/dexbotCtl/ctl_data_pub", lambda m: self.put(
                    "joint_target", list(m.l_position) + list(m.r_position), m)),
                (FkDataPub, "/dexbotCtl/DescPub/Fk_ret_data_pub", lambda m: self.put(
                    "tcp", [getattr(m, axis + "_" + side) for side in ("l", "r")
                            for axis in ("x", "y", "z", "roll", "pitch", "yaw")], m)),
                (MidNodeCtlMsgSend, "/dexbotCtl/upperMonitor_msg_send", lambda m: self.put(
                    "tcp_target", list(m.l_arm_des_pos_descartes) + list(m.r_arm_des_pos_descartes), m)),
                (Int8MultiArray, "/towel/gripper_state", lambda m: self.put("gripper_state", m.data, m)),
            ]
            for typ, topic, callback in subs:
                self.create_subscription(typ, topic, callback, qos_profile_sensor_data,
                                         callback_group=self.feedback_group)
            self.create_service(SetBool, "/towel/record", self.record)
            self.create_service(Trigger, "/towel/discard", self.discard)
            self.create_timer(1.0 / config["fps"], self.sample)
            self.get_logger().info("Ready: /towel/record, /towel/discard. No episode starts automatically.")
            if self.publish_rviz:
                self.get_logger().info("Publishing live camera images for RViz; this does not change recording labels.")

        def publish_status(self):
            msg = Bool()
            msg.data = self.episode is not None
            self.status.publish(msg)
            if status_callback is not None:
                status_callback(self.recording_status())

        def recording_status(self):
            episode = self.episode or self.retiring_episode
            return {
                "session": episode.path.name if episode is not None else None,
                "active": self.episode is not None,
                "ready": self.episode is not None and self.start_time is not None,
                "error": self.last_recording_error,
                "frames": episode.frames if episode is not None else 0,
                "diagnostics": self.diagnostics.summary() if self.diagnostics is not None else None,
            }

        def put(self, key, values, msg):
            stamp = getattr(getattr(msg, "header", None), "stamp", None)
            source = stamp.sec + stamp.nanosec * 1e-9 if stamp else None
            try:
                self.cache.update(key, values, time.monotonic(), source)
            except ValueError as exc:
                self.get_logger().warning(f"Invalid {key}: {exc}")

        def joints(self, msg):
            with self.cache.lock:
                self.put("q", list(msg.l_arm_pos_fdback) + list(msg.r_arm_pos_fdback), msg)
                self.put("dq", list(msg.l_arm_spd_fdback) + list(msg.r_arm_spd_fdback), msg)

        def snapshot(self, *, received_after=None):
            # Read time and cache atomically relative to feedback callbacks.
            with self.cache.lock:
                now = time.monotonic()
                sample = self.cache.snapshot(now, config["max_stream_age_s"], config["max_stream_skew_s"],
                                             received_after=received_after)
            images, camera_meta = cameras.snapshot(now)
            if received_after is not None:
                pending = [name for name in config["cameras"]
                           if camera_meta.get(name, {}).get("receive_monotonic_s", -float("inf")) <= received_after]
                if pending:
                    raise ValueError(f"Waiting for post-startup camera frames: {pending}")
            return now, sample, images, camera_meta

        def record(self, request, response):
            try:
                if request.data and self.episode is None:
                    if self.retiring_episode is not None and not self.retiring_episode.done.is_set():
                        raise RuntimeError("Previous episode writer is still stopping")
                    self.retiring_episode = None
                    requested_at = time.monotonic()
                    requested_wall_time_ns = time.time_ns()
                    calibration = {}
                    for folder in (ROOT / "ros2_ws/src/ctl_msgs_hw/config", ROOT / "teleop/VR_controller/config"):
                        for path in folder.glob("*.yaml"):
                            calibration[str(path.relative_to(ROOT))] = {
                                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                                "content": path.read_text(),
                            }
                    self.episode = AsyncEpisodeWriter(config["dataset_root"], {
                        **config.get("episode_metadata", {}),
                        "config": config, "calibration": calibration,
                        "requested_wall_time_ns": requested_wall_time_ns, "requested_monotonic_s": requested_at,
                        "start_wall_time_ns": None, "start_monotonic_s": None,
                    }, self.write_image, capacity=config.get("writer_queue_frames", 16),
                        finish_timeout=self.writer_finish_timeout_s)
                    self.prepared_at = time.monotonic()
                    self.start_time = self.last_frame_time = None
                    self.startup_wait_reason = "Waiting for post-startup feedback and cameras"
                    self.last_startup_log = -float("inf")
                    self.last_recording_error = ""
                    self.last_saved_q = None
                    self.last_saved_gripper_state = None
                    self.motion_skip_pending = False
                    self.camera_ids = {}
                    self.diagnostics = RecordingDiagnostics(self.prepared_at)
                    response.message = "preparing; waiting for first fresh capture"
                elif not request.data and self.episode is not None:
                    self.log_diagnostics(force=True)
                    episode, self.episode = self.episode, None
                    self.retiring_episode = episode
                    path = episode.finish(config["min_episode_frames"])
                    response.message = f"Saved {path}" if path else "Too short; kept as .partial, excluded from export"
                else:
                    response.message = "already in requested state"
                response.success = True
            except Exception as exc:
                response.success, response.message = False, str(exc)
            self.publish_status()
            self.get_logger().info(response.message)
            return response

        def discard(self, request, response):
            response.success = self.episode is not None
            response.message = "discarded" if response.success else "not recording"
            if self.episode:
                episode, self.episode = self.episode, None
                self.retiring_episode = episode
                try:
                    episode.discard()
                except Exception as exc:
                    response.success, response.message = False, str(exc)
            self.publish_status()
            return response

        def publish_images_for_rviz(self, images):
            """Publish the exact BGR frames used by this recorder; this is read-only visualization."""
            stamp = self.get_clock().now().to_msg()
            for name, image in images.items():
                if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
                    raise ValueError(f"Unexpected {name} image layout for RViz: {image.shape} {image.dtype}")
                msg = Image()
                msg.header.stamp = stamp
                msg.header.frame_id = f"towel_{name}_camera"
                msg.height, msg.width = image.shape[:2]
                msg.encoding = "bgr8"
                msg.is_bigendian = False
                msg.step = int(image.strides[0])
                msg.data = image.tobytes()
                self.image_publishers[name].publish(msg)

        def publish_preview(self):
            try:
                images, _ = cameras.snapshot(time.monotonic())
                self.publish_images_for_rviz(images)
                self._last_visualization_error = ""
            except Exception as exc:
                detail = str(exc)
                if detail != self._last_visualization_error:
                    self._last_visualization_error = detail
                    self.get_logger().warning(f"RViz camera preview unavailable: {detail}")

        def publish_opencv_preview(self):
            if self.opencv_preview is None:
                return
            try:
                images, _ = cameras.snapshot(time.monotonic())
                if not self.opencv_preview.show(images):
                    self.opencv_preview = None
                self._last_opencv_error = ""
            except Exception as exc:
                detail = str(exc)
                if detail != self._last_opencv_error:
                    self._last_opencv_error = detail
                    self.get_logger().warning(f"OpenCV camera preview unavailable: {detail}")

        def write_image(self, path, image):
            return cv2.imwrite(path, image, [cv2.IMWRITE_JPEG_QUALITY, 90])

        def log_diagnostics(self, *, force=False, failure=False):
            episode = self.episode or self.retiring_episode
            stats = episode.stats()
            self.diagnostics.write_count = stats["written"]
            self.diagnostics.last_write = stats["write_last"]
            self.diagnostics.write_total = stats["write_total"]
            self.diagnostics.write_max = stats["write_max"]
            self.diagnostics.camera_write_s = stats["camera_times"]
            if failure and hasattr(cameras, 'frame_metadata'):
                self.diagnostics.camera_meta = cameras.frame_metadata()
                self.diagnostics.camera_observed_at = time.monotonic()
            detail = self.diagnostics.report(time.monotonic(), episode.frames, force=force)
            if detail:
                detail += (f", written_frames={stats['written']}, writer_queue={stats['queued']}/{stats['capacity']}, "
                           f"writer_queue_high_water={stats['high_water']}")
                logger = self.get_logger()
                # rclpy binds severity to the call site, so these must be separate calls.
                if failure:
                    logger.warning(detail)
                else:
                    logger.info(detail)

        def sample(self):
            active = self.episode is not None
            started, cpu_started = time.monotonic(), thread_time()
            self._capture_failure = None
            try:
                self._sample()
            finally:
                if active:
                    self.diagnostics.body_last = time.monotonic() - started
                    failure = self._capture_failure
                    try:
                        try:
                            self.log_diagnostics(force=bool(failure), failure=bool(failure))
                        except Exception as diagnostic_error:
                            self.get_logger().warning(f"Capture diagnostics unavailable: {diagnostic_error}")
                        if failure:
                            self.get_logger().error(f"Recording aborted; retained .partial: {failure}")
                    finally:
                        # Completed callback statistics include diagnostic/log output.
                        self.diagnostics.finished(time.monotonic() - started, thread_time() - cpu_started)

        def _sample(self):
            if self.episode is not None and self.last_frame_time is not None:
                self.diagnostics.tick(time.monotonic(), self.last_frame_time)
                self.diagnostics.previous_ids = dict(self.camera_ids)
            if self.publish_rviz:
                self.publish_preview()
            self.publish_opencv_preview()
            if self.episode is None:
                return
            try:
                self.episode.check_error()
                first_frame = self.start_time is None
                if first_frame and time.monotonic() - self.prepared_at >= self.startup_timeout_s:
                    raise TimeoutError(f"Capture startup timed out after {self.startup_timeout_s:g}s: {self.startup_wait_reason}")
                try:
                    now, sample, images, camera_meta = self.snapshot(
                        received_after=self.prepared_at if first_frame else None)
                except ValueError as startup_error:
                    if not first_frame:
                        raise
                    self.startup_wait_reason = str(startup_error)
                    if time.monotonic() - self.last_startup_log >= 1.0:
                        self.last_startup_log = time.monotonic()
                        self.get_logger().info(f"Capture preparing: {self.startup_wait_reason}")
                    return
                self.diagnostics.camera_meta = camera_meta
                self.diagnostics.camera_observed_at = now
                gap = 0.0 if first_frame else now - self.last_frame_time
                self.diagnostics.frame_gap = gap
                self.diagnostics.frame_gap_max = max(self.diagnostics.frame_gap_max, gap)
                if not self.motion_skip_pending and gap > config["max_frame_gap_s"]:
                    raise ValueError(
                        f"Capture frame gap exceeds limit; episode is incomplete "
                        f"(gap={gap * 1000:.1f} ms, limit={config['max_frame_gap_s'] * 1000:.1f} ms)"
                    )
                ids = {name: meta["frame_number"] for name, meta in camera_meta.items()}
                if any(self.camera_ids.get(name) == value for name, value in ids.items()):
                    self.diagnostics.duplicate_skips += 1
                    return  # Never label the same camera exposure as a fresh image.
                min_joint_delta = float(config.get("min_joint_delta_rad", 0.0))
                if min_joint_delta < 0:
                    raise ValueError("min_joint_delta_rad must be nonnegative")
                joint_delta = (
                    max(abs(current - previous) for current, previous in zip(sample["q"], self.last_saved_q))
                    if self.last_saved_q is not None else float("inf")
                )
                gripper_changed = (
                    self.last_saved_gripper_state is not None
                    and sample["gripper_state"] != self.last_saved_gripper_state
                )
                if (self.last_saved_q is not None and min_joint_delta > 0
                        and joint_delta < min_joint_delta and not gripper_changed):
                    self.diagnostics.motion_skips += 1
                    self.motion_skip_pending = True
                    return  # Keep sampling until the next meaningful motion.
                sample.update(timestamp_s=0.0 if first_frame else now-self.start_time, monotonic_s=now,
                              ros_time_s=self.get_clock().now().nanoseconds * 1e-9,
                              camera_timestamps=camera_meta)
                self.episode.add(sample, images, start_metadata=(
                    {"start_monotonic_s": now, "start_wall_time_ns": time.time_ns()}
                    if first_frame else None))
                if first_frame:
                    self.start_time = now
                    self.diagnostics = RecordingDiagnostics(now)
                    self.get_logger().info("Capture started: first fresh frame queued; recording clock started")
                    self.publish_status()
                self.last_frame_time, self.camera_ids = now, ids
                self.last_saved_q = list(sample["q"])
                self.last_saved_gripper_state = list(sample["gripper_state"])
                self.motion_skip_pending = False
            except Exception as exc:
                self.last_recording_error = str(exc)
                self._capture_failure = str(exc)
                self.episode.abort(exc)
                self.retiring_episode = self.episode
                self.episode = None
                self.publish_status()

        def close(self):
            if self.opencv_preview is not None:
                self.opencv_preview.close()
            if self.episode:
                self.episode.abort("Recorder stopped without explicit save")
                self.retiring_episode = self.episode
                self.episode = None
            if self.retiring_episode and not self.retiring_episode.done.wait(self.writer_finish_timeout_s):
                self.get_logger().error("Episode writer is still blocked; .partial retained")
            self.destroy_node()

    return Recorder()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/recording.yaml")
    parser.add_argument("--dataset-root", type=Path)
    parser.add_argument("--task")
    parser.add_argument(
        "--publish-rviz", action="store_true", default=None,
        help="Publish every configured camera frame to its RViz image topic.",
    )
    parser.add_argument(
        "--preview-opencv", action="store_true",
        help="Show three OpenCV camera windows without ROS or recording.",
    )
    parser.add_argument(
        "--show-opencv", action="store_true",
        help="Show OpenCV camera windows while the recorder remains active.",
    )
    args = parser.parse_args()
    if args.preview_opencv and args.show_opencv:
        parser.error("--preview-opencv and --show-opencv cannot be used together")
    config = yaml.safe_load(args.config.read_text())
    if args.task:
        config["task"] = args.task
    if args.publish_rviz is None:
        config["publish_rviz"] = os.environ.get("TOWEL_PUBLISH_RVIZ", "").lower() in {"1", "true", "yes"} or bool(
            config.get("publish_rviz", False)
        )
    else:
        config["publish_rviz"] = args.publish_rviz
    path = args.dataset_root or Path(config["dataset_root"])
    config["dataset_root"] = str(path if path.is_absolute() else ROOT / path)
    cameras, node, rclpy = None, None, None
    try:
        cameras = Cameras(config)
        if args.preview_opencv:
            run_opencv_preview(cameras, config["cameras"])
            return
        import rclpy as rclpy_module
        rclpy = rclpy_module
        rclpy.init()
        node = make_node(config, cameras, show_opencv=args.show_opencv)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
        if cameras:
            cameras.close()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
