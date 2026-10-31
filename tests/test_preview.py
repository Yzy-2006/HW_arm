import sys
import subprocess
from pathlib import Path

import numpy as np
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "collection"))


def make_dry_run(target):
    return subprocess.run(
        ["make", "-n", target],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )


def test_make_preview_uses_opencv_without_starting_ros_launch():
    result = make_dry_run("preview")

    assert result.returncode == 0
    assert "collection/record.py" in result.stdout
    assert "--preview-opencv" in result.stdout
    assert "ros2 launch" not in result.stdout


def test_make_record_starts_the_recorder_with_opencv_windows():
    result = make_dry_run("record")

    assert result.returncode == 0
    assert "collection/record.py" in result.stdout
    assert "--show-opencv" in result.stdout


def test_make_record_opencv_keeps_the_recorder_and_enables_windows():
    result = make_dry_run("record-opencv")

    assert result.returncode == 0
    assert "collection/record.py" in result.stdout
    assert "--show-opencv" in result.stdout


def test_opencv_preview_renders_three_cameras_and_closes_on_q():
    import record

    class FakeOpenCV:
        WINDOW_NORMAL = 0

        def __init__(self):
            self.windows = []
            self.frames = []
            self.waits = []
            self.destroyed = False

        def namedWindow(self, title, mode):
            self.windows.append((title, mode))

        def imshow(self, title, image):
            self.frames.append((title, image))

        def waitKey(self, timeout_ms):
            self.waits.append(timeout_ms)
            return ord("q")

        def destroyAllWindows(self):
            self.destroyed = True

    cv = FakeOpenCV()
    preview = record.OpenCVPreview(("head", "left_wrist", "right_wrist"), cv2_module=cv)
    images = {
        "head": np.zeros((2, 3, 3), dtype=np.uint8),
        "left_wrist": np.ones((2, 3, 3), dtype=np.uint8),
        "right_wrist": np.full((2, 3, 3), 2, dtype=np.uint8),
    }

    assert preview.show(images) is False
    assert [title for title, _ in cv.windows] == [
        "Head camera",
        "Left wrist camera",
        "Right wrist camera",
    ]
    assert [title for title, _ in cv.frames] == [
        "Head camera",
        "Left wrist camera",
        "Right wrist camera",
    ]
    assert cv.waits == [1]
    preview.close()
    assert cv.destroyed


def test_rviz_image_displays_use_the_current_topic_property_to_subscribe_to_camera_topics():
    config = yaml.safe_load(
        (ROOT / "ros2_ws/src/towel_bringup/rviz/towel_experiment.rviz").read_text()
    )
    displays = config["Visualization Manager"]["Displays"]
    image_displays = [display for display in displays if display["Class"] == "rviz_default_plugins/Image"]

    assert [display.get("Topic", {}).get("Value") for display in image_displays] == [
        "/towel/camera/head/image_raw",
        "/towel/camera/left_wrist/image_raw",
        "/towel/camera/right_wrist/image_raw",
    ]
    assert all(display.get("Enabled") is True and display.get("Value") is True for display in image_displays)


def test_visualizer_does_not_shutdown_an_already_stopped_ros_context(monkeypatch):
    import towel_state_visualizer

    class FakeNode:
        destroyed = False

        def destroy_node(self):
            self.destroyed = True

    node = FakeNode()

    monkeypatch.setattr(towel_state_visualizer, "TowelStateVisualizer", lambda: node)
    monkeypatch.setattr(towel_state_visualizer.rclpy, "init", lambda: None)
    monkeypatch.setattr(
        towel_state_visualizer.rclpy,
        "spin",
        lambda _: (_ for _ in ()).throw(KeyboardInterrupt()),
    )
    monkeypatch.setattr(towel_state_visualizer.rclpy, "ok", lambda: False)
    monkeypatch.setattr(
        towel_state_visualizer.rclpy,
        "shutdown",
        lambda: pytest.fail("shutdown must not run after launch stopped the ROS context"),
    )

    towel_state_visualizer.main()

    assert node.destroyed
