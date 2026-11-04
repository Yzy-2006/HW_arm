#!/usr/bin/env bash
set -eu
cd -- "$(dirname -- "$0")/.."
mkdir -p reproduction_output
g++ -std=c++17 -O3 -DNDEBUG -ffunction-sections -fdata-sections \
 -I incident_20261104_122027/dynamics_audit/stubs \
 -I source_context/ros2_ws/src/dual_arm_control_hw/include \
 incident_20261104_122027/dynamics_audit/filter_probe.cpp \
 source_context/ros2_ws/src/dual_arm_control_hw/src/controller/JntController.cpp \
 -Wl,--gc-sections -o reproduction_output/filter_probe
reproduction_output/filter_probe reproduction_output/native_filter.csv
g++ -std=c++17 -O3 -DNDEBUG -shared -fPIC \
 -I incident_20261104_122027/dynamics_audit/stubs \
 -I source_context/ros2_ws/src/dual_arm_control_hw/include \
 incident_20261104_122027/dynamics_audit/filter_native.cpp \
 source_context/ros2_ws/src/dual_arm_control_hw/src/controller/JntController.cpp \
 -o reproduction_output/filter_native.so
