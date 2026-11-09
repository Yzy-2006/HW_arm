"""Check MediaPipe inference and optionally acquire aligned D435i frames."""

import argparse
from importlib.metadata import version
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", action="store_true", help="Test D435i RGB-D capture")
    parser.add_argument("--delegate", choices=("cpu", "gpu"), default="cpu")
    parser.add_argument(
        "--model",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "models/hand_landmarker.task",
    )
    args = parser.parse_args()

    import cv2
    import mediapipe as mp
    import numpy as np
    import pyrealsense2 as rs
    from mediapipe.tasks import python
    from mediapipe.tasks.python import vision
    from scipy.spatial.transform import Rotation

    print(f"Python: {sys.version.split()[0]}", flush=True)
    for package in ("mediapipe", "pyrealsense2", "opencv-contrib-python", "numpy", "scipy"):
        print(f"{package}: {version(package)}", flush=True)
    converted = cv2.cvtColor(np.zeros((8, 8, 3), dtype=np.uint8), cv2.COLOR_BGR2RGB)
    if converted.shape != (8, 8, 3):
        raise RuntimeError("OpenCV color conversion returned an unexpected shape.")
    if not np.allclose(Rotation.identity().as_quat(), [0, 0, 0, 1]):
        raise RuntimeError("SciPy quaternion conversion failed.")

    if not args.model.is_file():
        raise FileNotFoundError(
            f"Model missing: {args.model}. See README.md for download instructions."
        )
    delegate = (
        python.BaseOptions.Delegate.CPU
        if args.delegate == "cpu"
        else python.BaseOptions.Delegate.GPU
    )
    options = vision.HandLandmarkerOptions(
        base_options=python.BaseOptions(model_asset_path=str(args.model), delegate=delegate),
        running_mode=vision.RunningMode.IMAGE,
        num_hands=2,
    )
    with vision.HandLandmarker.create_from_options(options) as detector:
        image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=np.zeros((480, 640, 3), dtype=np.uint8),
        )
        result = detector.detect(image)
        print(
            f"PASS: Hand Landmarker {args.delegate.upper()} inference; "
            f"{len(result.hand_landmarks)} hand(s) on a blank image (expected: 0).",
            flush=True,
        )

    if args.camera:
        context = rs.context()
        devices = list(context.query_devices())
        print(f"RealSense devices: {len(devices)}", flush=True)
        camera = next(
            (
                device
                for device in devices
                if "D435I" in device.get_info(rs.camera_info.name).upper()
            ),
            None,
        )
        if camera is None:
            raise RuntimeError("D435i not detected. Check USB connection and device permissions.")
        print(f"Camera: {camera.get_info(rs.camera_info.name)}", flush=True)
        config = rs.config()
        config.enable_device(camera.get_info(rs.camera_info.serial_number))
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline = rs.pipeline(context)
        profile = pipeline.start(config)
        try:
            scale = profile.get_device().first_depth_sensor().get_depth_scale()
            print(f"Depth scale: {scale} meters/unit", flush=True)
            align = rs.align(rs.stream.color)
            valid_ratios = []
            for index in range(15):
                frames = align.process(pipeline.wait_for_frames(timeout_ms=5000))
                depth, color = frames.get_depth_frame(), frames.get_color_frame()
                if not depth or not color:
                    raise RuntimeError("A frameset is missing depth or color data.")
                depth_data = np.asanyarray(depth.get_data())
                color_data = np.asanyarray(color.get_data())
                if depth_data.shape != color_data.shape[:2]:
                    raise RuntimeError("Aligned depth and color dimensions differ.")
                if index >= 5:
                    valid_ratios.append(float(np.count_nonzero(depth_data) / depth_data.size))
            valid_ratio = float(np.mean(valid_ratios))
            if valid_ratio == 0:
                raise RuntimeError("All measured depth values are invalid.")
            print(
                f"PASS: D435i RGB-D capture and alignment, 640x480 @ 30 FPS configured; "
                f"10 frames checked, mean valid depth: {valid_ratio:.1%}.",
                flush=True,
            )
        finally:
            pipeline.stop()


if __name__ == "__main__":
    try:
        main()
    except (ImportError, OSError, RuntimeError, ValueError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        sys.exit(1)
