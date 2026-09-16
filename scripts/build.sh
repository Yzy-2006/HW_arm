#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
cd "$TOWEL_ROOT/ros2_ws"
colcon build --executor sequential --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3.12 "$@"
