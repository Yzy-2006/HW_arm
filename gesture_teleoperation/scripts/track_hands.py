"""Track depth-measured or calibrated RGB/PnP palm poses and send them to ROS."""

import argparse
import json
import os
from pathlib import Path
import sys
import socket
import time

import cv2
import mediapipe as mp
import numpy as np
import pyrealsense2 as rs
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from scipy.spatial.transform import Rotation

from hand_pose import PalmTracker, estimate_palm
from hand_facing import BACK, PALM, FACING_WINDOW_NAME, FacingTracker, classify_palm_facing, draw_facing_flags
from hand_gestures import (
    DEFAULT_OK_DISTANCE, DEFAULT_PINCH_DISTANCE, GESTURE_NAMES,
    GestureTracker, classify_gesture, fingertip_distances, gesture_features,
)
from pose_protocol import DEFAULT_PORT, FRAME_ID
from pose_comparison import compare_hands
from rgb_hand_pose import (
    PalmCalibrator, RGBPalmEstimator, camera_parameters, check_camera, load_calibration,
)


WINDOW_NAME = "D435i Hand Landmarks"
FINGER_COLORS = (
    (80, 180, 255),
    (100, 230, 100),
    (255, 210, 70),
    (255, 120, 200),
    (80, 230, 240),
)


def draw_hands(frame, result, mirror=False, show_indices=False):
    """Draw image-space landmarks; mirroring affects only the displayed image."""
    output = cv2.flip(frame, 1) if mirror else frame.copy()
    height, width = output.shape[:2]
    for landmarks, handedness in zip(result.hand_landmarks, result.handedness):
        points = []
        for landmark in landmarks:
            x = int(round(landmark.x * width))
            y = int(round(landmark.y * height))
            if mirror:
                x = width - 1 - x
            points.append((x, y))

        for connection in vision.HandLandmarksConnections.HAND_CONNECTIONS:
            color = FINGER_COLORS[(connection.end - 1) // 4]
            cv2.line(
                output, points[connection.start], points[connection.end],
                color, 2, cv2.LINE_AA,
            )
        for index, point in enumerate(points):
            color = (255, 255, 255) if index == 0 else FINGER_COLORS[(index - 1) // 4]
            cv2.circle(output, point, 4, (25, 25, 25), -1, cv2.LINE_AA)
            cv2.circle(output, point, 3, color, -1, cv2.LINE_AA)
            if show_indices:
                label = str(index)
                (text_width, _), baseline = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, 0.35, 1,
                )
                origin = (
                    max(0, min(width - text_width - 2, point[0] + 5)),
                    max(46, min(height - baseline - 2, point[1] - 5)),
                )
                cv2.putText(
                    output, label, origin, cv2.FONT_HERSHEY_SIMPLEX,
                    0.35, (255, 255, 255), 1, cv2.LINE_AA,
                )

        category = handedness[0]
        label = f"{category.category_name} {category.score:.2f}"
        (text_width, _), baseline = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2,
        )
        origin = (
            max(4, min(width - text_width - 4, min(point[0] for point in points))),
            max(58, min(height - baseline - 4, min(point[1] for point in points) - 12)),
        )
        cv2.putText(
            output, label, origin, cv2.FONT_HERSHEY_SIMPLEX,
            0.55, (20, 20, 20), 4, cv2.LINE_AA,
        )
        cv2.putText(
            output, label, origin, cv2.FONT_HERSHEY_SIMPLEX,
            0.55, (255, 255, 255), 2, cv2.LINE_AA,
        )
    return output


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("rgbd", "rgb", "compare"), default="rgbd")
    parser.add_argument("--serial", help="D435i serial; RGB/compare default to the calibrated camera")
    parser.add_argument("--calibration", type=Path,
                        default=Path(__file__).resolve().parents[1] / "calibration/hand_geometry.json")
    parser.add_argument("--calibrate", action="store_true", help="Collect RGB-D palm geometry and exit after saving")
    parser.add_argument("--calibration-hands", nargs="+", choices=("Left", "Right"), default=["Left", "Right"])
    parser.add_argument("--calibration-samples", type=int, default=60, help="Accepted samples per hand")
    parser.add_argument("--pnp-max-error", type=float, default=4.0, help="Maximum RGB PnP RMS reprojection error in pixels")
    parser.add_argument("--pnp-orientation-hint", action=argparse.BooleanOptionalAction, default=True,
                        help="Resolve planar PnP tilt using MediaPipe's RGB-only palm orientation")
    parser.add_argument("--delegate", choices=("gpu", "cpu"), default="gpu")
    parser.add_argument("--num-hands", type=int, choices=(1, 2), default=2)
    parser.add_argument("--mirror", action=argparse.BooleanOptionalAction, default=True,
                        help="Mirror the display only")
    parser.add_argument("--show-indices", action=argparse.BooleanOptionalAction, default=True,
                        help="Label landmark IDs 0-20")
    parser.add_argument("--udp-port", type=int, default=DEFAULT_PORT,
                        help="Local ROS bridge port; 0 disables pose transmission")
    parser.add_argument("--comparison-udp-port", type=int, default=DEFAULT_PORT + 1,
                        help="RGB/PnP bridge port in comparison mode")
    parser.add_argument("--min-depth", type=float, default=0.2, help="Minimum depth in meters")
    parser.add_argument("--max-depth", type=float, default=1.5, help="Maximum depth in meters")
    parser.add_argument("--ok-distance", type=float, default=DEFAULT_OK_DISTANCE,
                        help="Thumb/index 3D contact threshold in palm-size units")
    parser.add_argument("--pinch-distance", type=float, default=DEFAULT_PINCH_DISTANCE,
                        help="Maximum thumb-to-fingertip 3D distance in palm-size units")
    parser.add_argument("--gesture-debug", action="store_true",
                        help="Print normalized fingertip distances once per second")
    parser.add_argument("--no-display", action="store_true", help="Run without a GUI window")
    parser.add_argument("--max-frames", type=int, default=0, help="Stop after N frames; 0 runs until exit")
    parser.add_argument(
        "--model", type=Path,
        default=Path(__file__).resolve().parents[1] / "models/hand_landmarker.task",
    )
    args = parser.parse_args()
    if args.calibrate and args.mode != "rgbd":
        parser.error("--calibrate requires --mode rgbd")
    if not 20 <= args.calibration_samples <= 1000:
        parser.error("--calibration-samples must be between 20 and 1000")
    if not 0.1 <= args.pnp_max_error <= 20:
        parser.error("--pnp-max-error must be between 0.1 and 20 pixels")
    if args.max_frames < 0:
        parser.error("--max-frames must be >= 0")
    if not (0 <= args.udp_port <= 65535):
        parser.error("--udp-port must be between 0 and 65535")
    if not 1 <= args.comparison_udp_port <= 65535 or (
        args.mode == "compare" and args.comparison_udp_port == args.udp_port
    ):
        parser.error("Comparison port must be between 1 and 65535 and differ from --udp-port")
    if not (0 < args.min_depth < args.max_depth <= 10):
        parser.error("Expected 0 < --min-depth < --max-depth <= 10 meters")
    if not (0 < args.ok_distance <= 1.5 and 0 < args.pinch_distance <= 1.5):
        parser.error("Gesture distance thresholds must be between 0 and 1.5 palm-size units")
    if not args.model.is_file():
        parser.error(f"Model missing: {args.model}. See README.md for download instructions.")
    if (
        not args.no_display and sys.platform.startswith("linux")
        and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    ):
        parser.error("No desktop display found. Run in a desktop terminal or use --no-display.")
    return args


def draw_poses(output, hands, intrinsics, mirror):
    for hand in hands:
        position = np.asarray(hand["position"])
        matrix = Rotation.from_quat(hand["orientation"]).as_matrix()
        points = [position] + [position + 0.07 * matrix[:, axis] for axis in range(3)]
        pixels = []
        for point in points:
            pixel = rs.rs2_project_point_to_pixel(intrinsics, point.tolist())
            x, y = int(round(pixel[0])), int(round(pixel[1]))
            pixels.append((output.shape[1] - 1 - x if mirror else x, y))
        for endpoint, color in zip(pixels[1:], ((40, 40, 255), (40, 255, 40), (255, 100, 40))):
            cv2.arrowedLine(output, pixels[0], endpoint, color, 2, cv2.LINE_AA, tipLength=0.2)
    for index, hand in enumerate(sorted(hands, key=lambda item: item["label"])):
        x, y, z = hand["position"]
        label = f"{hand['label']}: x {x:+.3f}  y {y:+.3f}  z {z:+.3f} m"
        baseline = output.shape[0] - 9 - 23 * (len(hands) - index - 1)
        cv2.rectangle(output, (0, baseline - 18), (output.shape[1] - 1, baseline + 5), (30, 30, 30), -1)
        cv2.putText(output, label, (10, baseline), cv2.FONT_HERSHEY_SIMPLEX,
                    0.48, (240, 240, 240), 1, cv2.LINE_AA)


def main():
    args = parse_args()
    calibration = load_calibration(args.calibration) if args.mode in ("rgb", "compare") else None
    rgb_estimator = RGBPalmEstimator(calibration, args.pnp_max_error) if calibration is not None else None
    context = rs.context()
    selected_serial = args.serial or (calibration["camera"]["serial"] if calibration else None)
    camera = next(
        (
            device for device in context.query_devices()
            if "D435I" in device.get_info(rs.camera_info.name).upper()
            and (selected_serial is None or device.get_info(rs.camera_info.serial_number) == selected_serial)
        ),
        None,
    )
    if camera is None:
        raise RuntimeError(f"D435i {selected_serial or '(any serial)'} not detected. Check its USB connection and permissions.")

    delegate = (
        python.BaseOptions.Delegate.GPU if args.delegate == "gpu"
        else python.BaseOptions.Delegate.CPU
    )
    options = vision.HandLandmarkerOptions(
        base_options=python.BaseOptions(model_asset_path=str(args.model), delegate=delegate),
        running_mode=vision.RunningMode.VIDEO,
        num_hands=args.num_hands,
    )
    config = rs.config()
    config.disable_all_streams()
    config.enable_device(camera.get_info(rs.camera_info.serial_number))
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    if args.mode != "rgb":
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline = rs.pipeline(context)
    tracker = PalmTracker()
    comparison_tracker = PalmTracker()
    rgb_facing_tracker = FacingTracker()
    facing_tracker = FacingTracker()
    gesture_tracker = GestureTracker()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    align = rs.align(rs.stream.color) if args.mode != "rgb" else None
    calibrator = None
    processed = detected_frames = pose_frames = dual_pose_frames = 0
    rejected = {}
    facing_counts = {"palm": 0, "back": 0}
    gesture_counts = {name: 0 for name in GESTURE_NAMES.values()}
    fps = 0.0
    last_timestamp = -1
    last_gesture_debug = 0.0
    last_calibration_log = 0.0
    last_comparison_log = 0.0

    with sender, vision.HandLandmarker.create_from_options(options) as detector:
        profile = pipeline.start(config)
        started = previous = time.perf_counter()
        try:
            intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            if calibration is not None or args.calibrate:
                current_camera = camera_parameters(intrinsics, camera.get_info(rs.camera_info.serial_number))
                if calibration is not None:
                    check_camera(calibration, current_camera)
                else:
                    calibrator = PalmCalibrator(current_camera, args.calibration_hands, args.calibration_samples)
                    print(
                        "Calibration: show open palms toward the camera at 0.3-0.7 m; "
                        "keep fingers naturally extended and vary the tilt gently. "
                        "Hands may be collected one at a time. Incomplete collection does not replace the file.",
                        flush=True,
                    )
            depth_scale = profile.get_device().first_depth_sensor().get_depth_scale() if align is not None else None
            if not args.no_display:
                font_dir = os.environ.get("QT_QPA_FONTDIR")
                system_fonts = Path("/usr/share/fonts/truetype/dejavu")
                if (not font_dir or not Path(font_dir).is_dir()) and system_fonts.is_dir():
                    os.environ["QT_QPA_FONTDIR"] = str(system_fonts)
                cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE | cv2.WINDOW_GUI_NORMAL)
                cv2.namedWindow(FACING_WINDOW_NAME, cv2.WINDOW_AUTOSIZE | cv2.WINDOW_GUI_NORMAL)
                cv2.moveWindow(FACING_WINDOW_NAME, 700, 545)
            mode_name = {
                "rgbd": "RGB-D", "rgb": "RGB/PnP (color stream only)",
                "compare": "RGB-D + RGB/PnP comparison",
            }[args.mode]
            print(
                f"D435i {mode_name}: "
                f"640x480 @ 30 FPS | {args.delegate.upper()} | "
                f"up to {args.num_hands} hands. Press Q/Esc or close either OpenCV window to exit.",
                flush=True,
            )
            while args.max_frames == 0 or processed < args.max_frames:
                frames = pipeline.wait_for_frames(timeout_ms=5000)
                capture_time_ns = time.time_ns()
                capture_monotonic_ns = time.monotonic_ns()
                if align is not None:
                    frames = align.process(frames)
                color = frames.get_color_frame()
                if not color:
                    raise RuntimeError("Camera returned a frameset without color.")
                frame = np.asanyarray(color.get_data())
                depth_m = None
                if align is not None:
                    depth = frames.get_depth_frame()
                    if not depth:
                        raise RuntimeError("Camera returned a frameset without depth.")
                    depth_m = np.asanyarray(depth.get_data()).astype(np.float32) * depth_scale
                    intrinsics = depth.profile.as_video_stream_profile().get_intrinsics()
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                timestamp = max(last_timestamp + 1, time.perf_counter_ns() // 1_000_000)
                last_timestamp = timestamp
                result = detector.detect_for_video(image, timestamp)
                processed += 1
                detected_frames += bool(result.hand_landmarks)
                observations = []
                rgb_observations = []
                calibration_status = {label: "not detected" for label in args.calibration_hands}
                for detection_index, (landmarks, categories) in enumerate(zip(result.hand_landmarks, result.handedness)):
                    category = categories[0]
                    if category.score < 0.6:
                        calibration_status[category.category_name] = "uncertain handedness"
                        continue
                    world_points = np.asarray([
                        (point.x, point.y, point.z)
                        for point in result.hand_world_landmarks[detection_index]
                    ]) if detection_index < len(result.hand_world_landmarks) else None
                    pnp_hint = world_points if args.pnp_orientation_hint else None
                    if args.mode == "compare":
                        rgb_pose, rgb_reason = rgb_estimator.estimate(
                            category.category_name, landmarks, capture_monotonic_ns / 1e9,
                            args.min_depth, args.max_depth, world_landmarks=pnp_hint,
                        )
                        if rgb_pose is not None:
                            rgb_pose.update(detected_label=category.category_name, score=float(category.score),
                                            detection_index=detection_index)
                            rgb_observations.append(rgb_pose)
                        else:
                            key = f"RGB: {rgb_reason}"
                            rejected[key] = rejected.get(key, 0) + 1
                    if args.mode == "rgb":
                        pose, reason = rgb_estimator.estimate(
                            category.category_name, landmarks, capture_monotonic_ns / 1e9,
                            args.min_depth, args.max_depth, world_landmarks=pnp_hint,
                        )
                    else:
                        pose, reason = estimate_palm(
                            landmarks, depth_m, intrinsics, args.min_depth, args.max_depth,
                        )
                    if pose is None:
                        rejected[reason] = rejected.get(reason, 0) + 1
                        calibration_status[category.category_name] = reason
                        continue
                    pose.update(detected_label=category.category_name, score=float(category.score),
                                detection_index=detection_index)
                    pose.setdefault("pose_source", "rgbd")
                    observations.append(pose)
                    if calibrator is not None:
                        label = category.category_name
                        unique = sum(item[0].category_name == label for item in result.handedness) == 1
                        features = gesture_features(world_points)
                        facing = classify_palm_facing(dict(pose, label=label, handedness_score=category.score))
                        if label in calibrator.models:
                            calibration_status[label] = "complete"
                        elif category.score < 0.9 or not unique:
                            calibration_status[label] = "uncertain handedness"
                        elif facing != PALM:
                            calibration_status[label] = "palm not facing camera"
                        elif features is None or not (
                            np.all(features["pip_angles"] > 150) and np.all(features["extension"] > 0.85)
                        ):
                            calibration_status[label] = "fingers not extended"
                        else:
                            calibrator.add(label, pose, capture_monotonic_ns / 1e9)
                            calibration_status[label] = calibrator.status.get(label, "hand not requested")
                if calibrator is not None and time.monotonic() - last_calibration_log >= 1.0:
                    details = "; ".join(f"{label}: {calibration_status[label]}" for label in calibrator.samples)
                    print(f"Calibration: {calibrator.progress()} | {details}", flush=True)
                    last_calibration_log = time.monotonic()
                hands = tracker.update(observations, capture_monotonic_ns / 1e9)
                rgb_hands = comparison_tracker.update(rgb_observations, capture_monotonic_ns / 1e9)
                comparison = compare_hands(hands, rgb_hands) if args.mode == "compare" else None
                rgb_facing_flags = rgb_facing_tracker.update(rgb_hands, capture_monotonic_ns / 1e9)
                if comparison is not None and time.monotonic() - last_comparison_log >= 1.0:
                    for label, difference in comparison.items():
                        text = (
                            f"{difference['position_difference_m'] * 1000:.1f} mm, "
                            f"{difference['orientation_difference_deg']:.1f} deg"
                            if difference is not None else "no matched valid pose pair"
                        )
                        print(f"Compare {label}: {text}", flush=True)
                    last_comparison_log = time.monotonic()
                facing_flags = facing_tracker.update(hands, capture_monotonic_ns / 1e9)
                tracked_detections = {hand["detection_index"]: hand for hand in hands}
                gesture_detections = []
                debug_gestures = args.gesture_debug and time.monotonic() - last_gesture_debug >= 1.0
                for detection_index, categories in enumerate(result.handedness):
                    category = categories[0]
                    tracked = tracked_detections.get(detection_index)
                    points = np.array([
                        (point.x, point.y, point.z)
                        for point in result.hand_world_landmarks[detection_index]
                    ]) if detection_index < len(result.hand_world_landmarks) else np.empty((0, 3))
                    image_points = np.array([
                        (point.x * frame.shape[1], point.y * frame.shape[0])
                        for point in result.hand_landmarks[detection_index]
                    ])
                    gesture = classify_gesture(points, image_points, args.ok_distance, args.pinch_distance)
                    label = tracked["label"] if tracked else category.category_name
                    gesture_detections.append({
                        "label": label,
                        "score": tracked["handedness_score"] if tracked else float(category.score),
                        "gesture": gesture,
                    })
                    if debug_gestures:
                        world_distances = fingertip_distances(points)
                        image_distances = fingertip_distances(image_points)
                        if world_distances is not None and image_distances is not None:
                            print(
                                f"Gesture {label}: {GESTURE_NAMES[gesture]} | "
                                f"3D thumb-index={world_distances['thumb_index']:.2f}, "
                                f"thumb-max={world_distances['thumb_max']:.2f}, "
                                f"thumb-mean={world_distances['thumb_mean']:.2f} | "
                                f"2D thumb-index={image_distances['thumb_index']:.2f}, "
                                f"thumb-max={image_distances['thumb_max']:.2f}",
                                flush=True,
                            )
                if debug_gestures:
                    last_gesture_debug = time.monotonic()
                gestures = gesture_tracker.update(gesture_detections, capture_monotonic_ns / 1e9)
                for gesture in gestures.values():
                    gesture_counts[GESTURE_NAMES[gesture]] += 1
                for hand in hands:
                    hand["palm_facing"] = facing_flags[hand["label"]]
                    hand["gesture_id"] = gestures[hand["label"]]
                for hand in rgb_hands:
                    hand["palm_facing"] = rgb_facing_flags[hand["label"]]
                    hand["gesture_id"] = gestures[hand["label"]]
                facing_counts["palm"] += sum(flag == PALM for flag in facing_flags.values())
                facing_counts["back"] += sum(flag == BACK for flag in facing_flags.values())
                pose_frames += bool(hands)
                dual_pose_frames += len(hands) == 2
                if args.udp_port:
                    packet = {
                        "version": 1, "sequence": processed, "frame_id": FRAME_ID,
                        "capture_time_ns": capture_time_ns,
                        "capture_monotonic_ns": capture_monotonic_ns,
                        "hands": hands,
                        "gestures": gestures,
                    }
                    if comparison is not None:
                        packet["comparison"] = comparison
                    sender.sendto(json.dumps(packet, allow_nan=False).encode(), ("127.0.0.1", args.udp_port))
                    if comparison is not None:
                        rgb_packet = dict(packet, hands=rgb_hands)
                        rgb_packet.pop("comparison")
                        sender.sendto(
                            json.dumps(rgb_packet, allow_nan=False).encode(),
                            ("127.0.0.1", args.comparison_udp_port),
                        )

                now = time.perf_counter()
                instantaneous_fps = 1.0 / max(now - previous, 1e-6)
                fps = instantaneous_fps if processed == 1 else 0.9 * fps + 0.1 * instantaneous_fps
                previous = now
                if calibrator is not None and calibrator.complete:
                    calibrator.save(args.calibration)
                    print(f"Calibration saved: {args.calibration}", flush=True)
                    for label, model in calibrator.models.items():
                        print(
                            f"{label}: width {model['palm_width_m'] * 1000:.1f} mm, "
                            f"length {model['palm_length_m'] * 1000:.1f} mm, "
                            f"shape RMS {model['shape_rms_m'] * 1000:.1f} mm", flush=True,
                        )
                if not args.no_display:
                    output = draw_hands(frame, result, args.mirror, args.show_indices)
                    draw_poses(output, hands, intrinsics, args.mirror)
                    cv2.rectangle(output, (0, 0), (output.shape[1] - 1, 33), (30, 30, 30), -1)
                    status = (
                        f"CAL {calibrator.progress()} | {args.delegate.upper()}" if calibrator is not None
                        else f"FPS: {fps:.1f}   Hands: {len(result.hand_landmarks)}   "
                        f"Poses: {len(hands)}   {args.mode.upper()} {args.delegate.upper()}"
                    )
                    cv2.putText(
                        output,
                        status,
                        (10, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (240, 240, 240), 1, cv2.LINE_AA,
                    )
                    cv2.imshow(WINDOW_NAME, output)
                    gesture_labels = {
                        label: f"{GESTURE_NAMES[gesture]} [{gesture}]"
                        for label, gesture in gestures.items()
                    }
                    cv2.imshow(FACING_WINDOW_NAME, draw_facing_flags(facing_flags, gesture_labels))
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), ord("Q"), 27):
                        break
                    if any(
                        cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE) < 1
                        for name in (WINDOW_NAME, FACING_WINDOW_NAME)
                    ):
                        break
                if calibrator is not None and calibrator.complete:
                    break
        finally:
            pipeline.stop()
            if not args.no_display:
                cv2.destroyAllWindows()
            elapsed = time.perf_counter() - started
            print(
                f"Stopped: {processed} frames, {processed / max(elapsed, 1e-6):.1f} FPS average, "
                f"{detected_frames} frames with hands, {pose_frames} with valid palm poses, "
                f"{dual_pose_frames} with both palm poses. Facing: {facing_counts}. "
                f"Gestures: {gesture_counts}. Rejected: {rejected}",
                flush=True,
            )
            if calibrator is not None and not calibrator.complete:
                print("Calibration incomplete; no calibration file was written.", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Stopped.", flush=True)
    except (ImportError, OSError, RuntimeError, ValueError, cv2.error) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        if any(term in str(error).lower() for term in ("egl", "gpu", "gl context")):
            print("For GPU initialization errors, retry with --delegate cpu.", file=sys.stderr)
        sys.exit(1)
