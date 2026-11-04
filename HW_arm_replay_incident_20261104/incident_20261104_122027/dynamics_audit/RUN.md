# 离线复算命令

在当前 HW_arm 工程根目录执行。以下命令只构建隔离诊断程序和读取证据，
不调用 ROS、CAN 或 EtherCAT。原始控制源码只作为编译输入。

```bash
g++ -std=c++17 -O3 -DNDEBUG -ffunction-sections -fdata-sections \
  -I outputs/diagnostics/incident_20261104_122027/dynamics_audit/stubs \
  -I ros2_ws/src/dual_arm_control_hw/include \
  outputs/diagnostics/incident_20261104_122027/dynamics_audit/filter_probe.cpp \
  ros2_ws/src/dual_arm_control_hw/src/controller/JntController.cpp \
  -Wl,--gc-sections \
  -o outputs/diagnostics/incident_20261104_122027/dynamics_audit/filter_probe

outputs/diagnostics/incident_20261104_122027/dynamics_audit/filter_probe \
  outputs/diagnostics/incident_20261104_122027/dynamics_audit/native_filter.csv

g++ -std=c++17 -O3 -DNDEBUG -shared -fPIC \
  -I outputs/diagnostics/incident_20261104_122027/dynamics_audit/stubs \
  -I ros2_ws/src/dual_arm_control_hw/include \
  outputs/diagnostics/incident_20261104_122027/dynamics_audit/filter_native.cpp \
  ros2_ws/src/dual_arm_control_hw/src/controller/JntController.cpp \
  -o outputs/diagnostics/incident_20261104_122027/dynamics_audit/filter_native.so

PYTHONPATH=/opt/openrobots/lib/python3.12/site-packages \
LD_LIBRARY_PATH=/opt/openrobots/lib \
.venv-robot/bin/python \
  outputs/diagnostics/incident_20261104_122027/dynamics_audit/audit.py
```

这里使用本机已安装的 Pinocchio 4.1.0。开环实验的近似条件和边界见 report.md。
