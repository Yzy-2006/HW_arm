import sys
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "collection"))


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
