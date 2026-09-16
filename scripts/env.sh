#!/usr/bin/env bash
# Source this in each terminal. Does not source the old workspace implicitly.
TOWEL_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export TOWEL_ROOT
source /opt/ros/"${ROS_DISTRO:-jazzy}"/setup.bash
# Optional compatible vendor prefix, supplied by the operator on the robot host.
if [[ -n "${TOWEL_OPENROBOTS_PREFIX:-}" ]]; then
  export CMAKE_PREFIX_PATH="$TOWEL_OPENROBOTS_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
  export LD_LIBRARY_PATH="$TOWEL_OPENROBOTS_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  export OPENROBOTS_PYTHON_PATH="$TOWEL_OPENROBOTS_PREFIX/lib/python3.12/site-packages"
  export PYTHONPATH="$OPENROBOTS_PYTHON_PATH${PYTHONPATH:+:$PYTHONPATH}"
  export CASADIPATH="$TOWEL_OPENROBOTS_PREFIX/lib"
fi
if [[ -f "$TOWEL_ROOT/ros2_ws/install/local_setup.bash" ]]; then
  source "$TOWEL_ROOT/ros2_ws/install/local_setup.bash"
fi
export PYTHONPATH="$TOWEL_ROOT/teleop/src${PYTHONPATH:+:$PYTHONPATH}"
export DESC_ZEROPOS_YAML="$TOWEL_ROOT/ros2_ws/src/ctl_msgs_hw/config/desc_zeropos.yaml"
export GRIPPER_PD_YAML="$TOWEL_ROOT/teleop/VR_controller/config/gripper_pd.yaml"
