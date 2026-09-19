import os
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SERVO_NODE = ROOT / "ros2_ws/install/servo_control_hw/lib/servo_control_hw/servo_node"


def test_installed_servo_node_embeds_workspace_and_ros_library_paths():
    """A capability-enabled servo must not depend on LD_LIBRARY_PATH."""
    result = subprocess.run(
        ["readelf", "-d", str(SERVO_NODE)],
        check=True,
        capture_output=True,
        text=True,
    )

    dynamic_entries = result.stdout
    ros_distro = os.environ.get("ROS_DISTRO", "jazzy")

    ctl_msgs_library_dir = ROOT / "ros2_ws/install/ctl_msgs_hw/lib"

    assert str(ctl_msgs_library_dir) in dynamic_entries
    assert f"/opt/ros/{ros_distro}/lib" in dynamic_entries


def test_installed_servo_node_resolves_transitive_libraries_without_ld_library_path():
    """RPATH must apply to the ROS libraries required by direct dependencies."""
    environment = os.environ.copy()
    environment.pop("LD_LIBRARY_PATH", None)
    result = subprocess.run(
        ["ldd", str(SERVO_NODE)],
        check=True,
        capture_output=True,
        text=True,
        env=environment,
    )

    assert "not found" not in result.stdout
