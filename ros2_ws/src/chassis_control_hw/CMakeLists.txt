cmake_minimum_required(VERSION 3.16)
project(chassis_control_hw)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BMSAlarmWarning.msg"
  "msg/BMSStatus.msg"
  "msg/BackWheelOdometry.msg"
  "msg/ChassisMotion.msg"
  "msg/FaultBit.msg"
  "msg/FrontWheelOdometry.msg"
  "msg/LifterCmd.msg"
  "msg/LightControlStatus.msg"
  "msg/MotionModeFeedback.msg"
  "msg/MotorHigh.msg"
  "msg/MotorHighArray.msg"
  "msg/MotorLow.msg"
  "msg/MotorLowArray.msg"
  "msg/RemoteControlStatus.msg"
  "msg/SteeringAngles.msg"
  "msg/SystemStatus.msg"
  "msg/SystemStatusArray.msg"
  "msg/WheelSpeeds.msg"
  "srv/SavePreset.srv"
  "action/MoveToPreset.action"
  DEPENDENCIES std_msgs action_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
