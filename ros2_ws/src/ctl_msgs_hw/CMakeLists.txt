cmake_minimum_required(VERSION 3.8)
project(ctl_msgs_hw)

# 强制使用系统 Python 3.12（匹配 ROS2 Jazzy），避免 rosidl_generate_interfaces
# 生成的 Python 绑定链接到 conda 解释器（如 ros_env 的 py3.10）
set(Python3_EXECUTABLE "/usr/bin/python3.12" CACHE FILEPATH "Force system Python 3.12" FORCE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
set(ctl_msgs
  "msg/HandoffStatus.msg"
  "msg/CtlInformation.msg"
  "msg/RetInformation.msg"
  "msg/DbgInformation.msg"
  "msg/TestMsg.msg"
  "msg/EndDescPub.msg"
  "msg/MidNodeCtlMsgSend.msg"
  "msg/MidNodeCtlMsgRet.msg"
  "msg/FkDataPub.msg"

  "msg/GripperCtl.msg"

  "msg/BridgeCtlMsg.msg"

  "msg/JntCompResVisu.msg"
  "msg/JntCtlResVisu.msg"

  "msg/VrTeleopMsg.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HandoffStatus.msg"
  "msg/CtlInformation.msg"
  "msg/RetInformation.msg"
  "msg/DbgInformation.msg"
  "msg/TestMsg.msg"
  "msg/EndDescPub.msg"
  "msg/MidNodeCtlMsgSend.msg"
  "msg/MidNodeCtlMsgRet.msg"
  "msg/FkDataPub.msg"

  "msg/GripperCtl.msg"

  "msg/BridgeCtlMsg.msg"

  "msg/JntCompResVisu.msg"
  "msg/JntCtlResVisu.msg"

  "msg/VrTeleopMsg.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
