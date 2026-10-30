#!/usr/bin/env python3
"""Passive dual-arm recorder. Does not publish any robot motion commands."""
import argparse
import hashlib
import json
import os
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

from core import EpisodeWriter, StreamCache

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
            camera_details.append(f"{name}(frame={frame}, previous_saved={previous}, "
                                  f"step={step}, age_ms={age})")
        per_camera = ', '.join(f"{name}={elapsed * 1000:.1f}"
                               for name, elapsed in sorted(self.camera_write_s.items()))
        duration = now - self.started_at
        return (
            f"Capture diagnostics: frames={frames}, fps={frames / duration if duration > 0 else 0:.1f}, "
            f"callback_ms(avg/max)={self.tick_total / max(1, self.tick_count) * 1000:.1f}/"
            f"{self.tick_max * 1000:.1f}, frame_gap_ms(current/max)={self.frame_gap * 1000:.1f}/"
            f"{self.frame_gap_max * 1000:.1f}, "
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
        try:
            while not self.stop.is_set():
                frames = pipeline.wait_for_frames(1000)
                color = frames.get_color_frame()
                if not color:
                    continue
                received = time.monotonic()
                entry = (np.asanyarray(color.get_data()).copy(), {
                    "frame_number": color.get_frame_number(),
                    "device_timestamp_ms": color.get_timestamp(),
                    "timestamp_domain": str(color.get_frame_timestamp_domain()),
                    "receive_monotonic_s": received,
                })
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
def make_node(config, cameras, *, show_opencv=False):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from ctl_msgs_hw.msg import CtlInformation, FkDataPub, MidNodeCtlMsgRet, MidNodeCtlMsgSend
    from std_msgs.msg import Int8MultiArray, Bool
    from sensor_msgs.msg import Image
    from std_srvs.srv import SetBool, Trigger

    class Recorder(Node):
        def __init__(self):
            super().__init__("towel_recorder")
            self.cache, self.episode = StreamCache(), None
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
                self.create_subscription(typ, topic, callback, qos_profile_sensor_data)
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

        def put(self, key, values, msg):
            stamp = getattr(getattr(msg, "header", None), "stamp", None)
            source = stamp.sec + stamp.nanosec * 1e-9 if stamp else None
            try:
                self.cache.update(key, values, time.monotonic(), source)
            except ValueError as exc:
                self.get_logger().warning(f"Invalid {key}: {exc}")

        def joints(self, msg):
            self.put("q", list(msg.l_arm_pos_fdback) + list(msg.r_arm_pos_fdback), msg)
            self.put("dq", list(msg.l_arm_spd_fdback) + list(msg.r_arm_spd_fdback), msg)

        def snapshot(self):
            now = time.monotonic()
            sample = self.cache.snapshot(now, config["max_stream_age_s"], config["max_stream_skew_s"])
            images, camera_meta = cameras.snapshot(now)
            return now, sample, images, camera_meta

        def record(self, request, response):
            try:
                if request.data and self.episode is None:
                    now, _, _, _ = self.snapshot()  # Refuse an empty/unready episode.
                    calibration = {}
                    for folder in (ROOT / "ros2_ws/src/ctl_msgs_hw/config", ROOT / "teleop/VR_controller/config"):
                        for path in folder.glob("*.yaml"):
                            calibration[str(path.relative_to(ROOT))] = {
                                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                                "content": path.read_text(),
                            }
                    self.episode = EpisodeWriter(config["dataset_root"], {
                        "config": config, "calibration": calibration,
                        "start_wall_time_ns": time.time_ns(), "start_monotonic_s": now,
                    }, self.write_image)
                    self.start_time, self.last_frame_time = now, now
                    self.last_saved_q = None
                    self.last_saved_gripper_state = None
                    self.motion_skip_pending = False
                    self.camera_ids = {}
                    self.diagnostics = RecordingDiagnostics(now)
                    response.message = "recording"
                elif not request.data and self.episode is not None:
                    self.log_diagnostics(force=True)
                    episode, self.episode = self.episode, None
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
                self.episode.discard()
                self.episode = None
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
            start = time.monotonic()
            try:
                return cv2.imwrite(path, image, [cv2.IMWRITE_JPEG_QUALITY, 90])
            finally:
                self.diagnostics.camera_write_s[Path(path).parent.name] = time.monotonic() - start

        def log_diagnostics(self, *, force=False, failure=False):
            if failure and hasattr(cameras, 'frame_metadata'):
                self.diagnostics.camera_meta = cameras.frame_metadata()
                self.diagnostics.camera_observed_at = time.monotonic()
            detail = self.diagnostics.report(time.monotonic(), self.episode.frames, force=force)
            if detail:
                logger = self.get_logger()
                (logger.warning if failure else logger.info)(detail)

        def sample(self):
            if self.episode is not None:
                self.diagnostics.tick(time.monotonic(), self.last_frame_time)
                self.diagnostics.previous_ids = dict(self.camera_ids)
            if self.publish_rviz:
                self.publish_preview()
            self.publish_opencv_preview()
            if self.episode is None:
                return
            try:
                now, sample, images, camera_meta = self.snapshot()
                self.diagnostics.camera_meta = camera_meta
                self.diagnostics.camera_observed_at = now
                gap = now - self.last_frame_time
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
                    self.log_diagnostics()
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
                    self.log_diagnostics()
                    return  # Keep sampling until the next meaningful motion.
                sample.update(timestamp_s=now-self.start_time, monotonic_s=now,
                              ros_time_s=self.get_clock().now().nanoseconds * 1e-9,
                              camera_timestamps=camera_meta)
                write_start = time.monotonic()
                self.diagnostics.camera_write_s = {}
                try:
                    self.episode.add(sample, images)
                finally:
                    self.diagnostics.wrote(time.monotonic() - write_start)
                self.last_frame_time, self.camera_ids = now, ids
                self.last_saved_q = list(sample["q"])
                self.last_saved_gripper_state = list(sample["gripper_state"])
                self.motion_skip_pending = False
                self.log_diagnostics()
            except Exception as exc:
                self.log_diagnostics(force=True, failure=True)
                self.episode.abort(exc)
                self.episode = None
                self.publish_status()
                self.get_logger().error(f"Recording aborted; retained .partial: {exc}")

        def close(self):
            if self.opencv_preview is not None:
                self.opencv_preview.close()
            if self.episode:
                self.episode.abort("Recorder stopped without explicit save")
                self.episode = None
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
