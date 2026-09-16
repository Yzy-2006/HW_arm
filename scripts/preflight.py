#!/usr/bin/env python3
"""Read-only environment checks; never starts robot nodes or opens a CAN bus."""
import argparse
import importlib
import json
import platform
import shutil
import sys
from pathlib import Path

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--scope', choices=['all', 'control', 'record', 'visualize'], default='all')
args = parser.parse_args()
checks = []
def check(name, fn):
    try:
        detail = fn()
        checks.append({'name': name, 'ok': True, 'detail': str(detail or 'available')})
    except Exception as exc:
        checks.append({'name': name, 'ok': False, 'detail': str(exc)})

def module(name):
    m = importlib.import_module(name)
    return getattr(m, '__file__', name)

check('Python 3.12 / ROS 2 Jazzy', lambda: None if sys.version_info[:2] == (3,12) else
      (_ for _ in ()).throw(RuntimeError('Use a Python 3.12 environment with system ROS packages')))
for name in ['rclpy','ctl_msgs_hw.msg','numpy','yaml']:
    check(name, lambda name=name: module(name))
if args.scope in ['all','record']:
    for name in ['cv2','pyrealsense2']:
        check(name, lambda name=name: module(name))
if args.scope in ['all','control']:
    for name in ['casadi', 'pinocchio', 'pinocchio.casadi', 'meshcat', 'scipy', 'televuer']:
        check(name, lambda name=name: module(name))
    from ament_index_python.packages import get_package_share_directory
    for name in ['ctl_msgs_hw','dual_arm_hw','chassis_control_hw','servo_control_hw','dual_arm_control_hw','robot_control_hw']:
        check(name+' ROS package', lambda name=name: get_package_share_directory(name))
    for name in ['cert.pem','key.pem']:
        check(name, lambda name=name: (root/'teleop'/name).stat().st_size)
if args.scope in ['all','visualize']:
    from ament_index_python.packages import get_package_share_directory
    check('towel_bringup ROS package', lambda: get_package_share_directory('towel_bringup'))
    check('rviz2 executable', lambda: shutil.which('rviz2') or (_ for _ in ()).throw(RuntimeError('rviz2 not on PATH')))
print(json.dumps({'architecture': platform.machine(), 'checks': checks}, ensure_ascii=False, indent=2))
sys.exit(0 if all(item['ok'] for item in checks) else 1)
