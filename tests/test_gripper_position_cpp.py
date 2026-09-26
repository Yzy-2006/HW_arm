import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SERVO = ROOT / 'ros2_ws/src/servo_control_hw'


def test_motor_position_limits_direction_temperature_and_interpolation(tmp_path):
    executable = tmp_path / 'gripper_position_test'
    subprocess.run([
        'g++', '-std=c++17', '-ffunction-sections', '-fdata-sections',
        '-I', str(SERVO/'include'), str(ROOT/'tests/gripper_position_test.cpp'),
        str(SERVO/'src/JntController/ForceComp.cpp'),
        str(SERVO/'src/JntController/runinController.cpp'),
        str(SERVO/'src/ros_controller/ctlConfig.cpp'),
        '-Wl,--gc-sections', '-lyaml-cpp', '-o', str(executable),
    ], check=True, capture_output=True, text=True)
    subprocess.run([
        str(executable), str(ROOT/'ros2_ws/src/ctl_msgs_hw/config/controllerConfig.yaml'),
    ], check=True, capture_output=True, text=True)
