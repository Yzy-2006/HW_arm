#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
component="${1:-help}"
if [[ $# -gt 0 ]]; then shift; fi
case "$component" in
  servo) exec ros2 run servo_control_hw servo_node "$@" ;;
  arms) exec ros2 run dual_arm_control_hw dual_arm_control_node "$@" ;;
  robot) exec ros2 run robot_control_hw robot_control_node "$@" ;;
  force) exec ros2 run encos_force_sensor force_sensor_node "$@" ;;
  vr) exec python3 "$TOWEL_ROOT/teleop/VR_controller/vr_pose_increment_publisher.py" "$@" ;;
  record) exec python3 "$TOWEL_ROOT/collection/record.py" "$@" ;;
  *) echo 'Usage: scripts/run.sh {servo|arms|robot|force|vr|record} [args]'; exit 2 ;;
esac
