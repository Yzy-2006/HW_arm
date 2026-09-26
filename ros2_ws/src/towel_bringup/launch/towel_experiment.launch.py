"""Start selected towel-task processes in a controlled order.

The launch file intentionally defaults every hardware-affecting process to false.
The normal real-robot workflow starts servo, arms, robot, recorder and VR in
separate terminals; this launch entry remains for individual visualization or
diagnostic components selected explicitly through launch arguments.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def towel_root():
    root = os.environ.get("TOWEL_ROOT")
    if not root:
        raise RuntimeError("TOWEL_ROOT is unset. Run `source scripts/env.sh` before ros2 launch.")
    path = Path(root).resolve()
    if not (path / "collection/record.py").is_file():
        raise RuntimeError(f"TOWEL_ROOT does not identify a towel_task directory: {path}")
    return path


def timed_node(delay_s, package, executable, enabled, *, name=None, parameters=None, remappings=None):
    return TimerAction(
        period=delay_s,
        actions=[Node(
            package=package,
            executable=executable,
            name=name,
            output="screen",
            condition=IfCondition(enabled),
            parameters=parameters or [],
            remappings=remappings or [],
        )],
    )


def generate_launch_description():
    root = towel_root()
    bringup_share = Path(get_package_share_directory("towel_bringup"))
    urdf_path = Path(get_package_share_directory("dual_arm_hw")) / "urdf/dual_arm.urdf"
    record_config = LaunchConfiguration("record_config")
    visualize = LaunchConfiguration("visualize")

    arguments = [
        DeclareLaunchArgument("start_servo", default_value="false"),
        DeclareLaunchArgument("start_arms", default_value="false"),
        DeclareLaunchArgument("start_robot", default_value="false"),
        DeclareLaunchArgument("start_record", default_value="false"),
        DeclareLaunchArgument("start_vr", default_value="false"),
        DeclareLaunchArgument("visualize", default_value="false"),
        DeclareLaunchArgument("record_config", default_value=str(root / "config/recording.yaml")),
    ]

    # `rviz_` creates an independent TF tree, avoiding collisions with the
    # original middle-layer TF broadcaster on /tf.
    rviz_robot = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="towel_rviz_robot_state_publisher",
        output="screen",
        condition=IfCondition(visualize),
        parameters=[{"robot_description": urdf_path.read_text(), "frame_prefix": "rviz_"}],
        remappings=[("/joint_states", "/towel/joint_states")],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="towel_rviz",
        output="screen",
        condition=IfCondition(visualize),
        arguments=["-d", str(bringup_share / "rviz/towel_experiment.rviz")],
    )

    return LaunchDescription([
        *arguments,
        SetEnvironmentVariable("TOWEL_PUBLISH_RVIZ", visualize),
        timed_node(0.0, "servo_control_hw", "servo_node", LaunchConfiguration("start_servo")),
        timed_node(2.0, "dual_arm_control_hw", "dual_arm_control_node", LaunchConfiguration("start_arms")),
        timed_node(4.0, "robot_control_hw", "robot_control_node", LaunchConfiguration("start_robot")),
        timed_node(5.0, "towel_bringup", "towel_state_visualizer.py", visualize),
        rviz_robot,
        rviz,
        TimerAction(
            period=5.0,
            actions=[ExecuteProcess(
                cmd=["python3", str(root / "collection/record.py"), "--config", record_config],
                cwd=str(root), output="screen", condition=IfCondition(LaunchConfiguration("start_record")),
            )],
        ),
        TimerAction(
            period=6.0,
            actions=[ExecuteProcess(
                cmd=["python3", str(root / "teleop/VR_controller/vr_pose_increment_publisher.py")],
                cwd=str(root), output="screen", condition=IfCondition(LaunchConfiguration("start_vr")),
            )],
        ),
    ])
