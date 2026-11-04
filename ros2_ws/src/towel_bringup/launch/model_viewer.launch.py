"""Display the dual-arm model with local joint states, without robot hardware."""

from pathlib import Path
import os
import sys
import xml.etree.ElementTree as ET
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(os.environ["TOWEL_ROOT"])
    sys.path.insert(0, str(root / "simulation"))
    from robot_model import robot_description

    rviz_config = Path(__file__).resolve().parents[1] / "rviz/model_viewer.rviz"
    model = robot_description(mesh_uris=True)
    description = ET.tostring(model, encoding="unicode")
    gui = LaunchConfiguration("gui")
    joint_publisher = LaunchConfiguration("joint_publisher")
    initial = yaml.safe_load((root / "simulation/scene.yaml").read_text())["robot"]
    joint_defaults = {}
    for side, key in (("l", "left"), ("r", "right")):
        for i, angle in enumerate(initial[key], 1):
            joint_defaults[f"zeros.arm_{side}_joint_{i}"] = float(angle)
        limit = model.find(f"joint[@name='arm_{side}_joint_finger']/limit")
        joint_defaults[f"zeros.arm_{side}_joint_finger"] = initial["gripper_opening"] * float(
            limit.get("lower" if side == "l" else "upper"))

    # Keep preview joint states and transforms separate from live robot topics.
    common = {
        "namespace": "model_viewer",
        "output": "screen",
        "remappings": [("/tf", "tf"), ("/tf_static", "tf_static")],
    }
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", str(rviz_config)],
        condition=IfCondition(LaunchConfiguration("rviz")),
        **common,
    )
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("joint_publisher", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": description}],
            **common,
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            parameters=[joint_defaults],
            condition=IfCondition(PythonExpression(["'", joint_publisher, "' == 'true' and '", gui, "' == 'true'"])),
            **common,
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            parameters=[joint_defaults],
            condition=IfCondition(PythonExpression(["'", joint_publisher, "' == 'true' and '", gui, "' == 'false'"])),
            **common,
        ),
        RegisterEventHandler(OnProcessExit(
            target_action=rviz_node,
            on_exit=[EmitEvent(event=Shutdown(reason="RViz closed or failed"))],
        )),
        rviz_node,
    ])
