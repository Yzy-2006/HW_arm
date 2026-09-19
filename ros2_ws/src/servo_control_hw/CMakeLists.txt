cmake_minimum_required(VERSION 3.16)
project(servo_control_hw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# find dependenciess
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(ctl_msgs_hw REQUIRED)
find_package(dual_arm_control_hw REQUIRED)


find_package(std_srvs REQUIRED)          # 提供 SetBool 服务
find_package(robot_state_publisher REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)


# 头文件路径通过 target_include_directories 配置（不再硬编码系统路径）

add_compile_definitions(
    __GLIBC_USE_IEC_60559_BFP_EXT=1
    __GLIBC_USE_ISOC2X=0
    _GNU_SOURCE=1
)


# 编译SOEM库
add_library(soem STATIC
  thirdparty/soem/soem/ethercatbase.c
  thirdparty/soem/soem/ethercatcoe.c
  thirdparty/soem/soem/ethercatconfig.c
  thirdparty/soem/soem/ethercatdc.c
  thirdparty/soem/soem/ethercateoe.c
  thirdparty/soem/soem/ethercatfoe.c
  thirdparty/soem/soem/ethercatmain.c
  thirdparty/soem/soem/ethercatprint.c
  thirdparty/soem/soem/ethercatsoe.c
  thirdparty/soem/osal/linux/osal.c
  thirdparty/soem/oshw/linux/nicdrv.c
  thirdparty/soem/oshw/linux/oshw.c
)
# SOEM 头文件路径（PUBLIC：链接方也能找到 ethercat.h）
target_include_directories(soem PUBLIC
  thirdparty/soem/soem
  thirdparty/soem/osal
  thirdparty/soem/osal/linux
  thirdparty/soem/oshw/linux
)

# 完整版本的主程序
add_executable(servo_node
  src/main.cpp

  src/servo_control/zero_pos_config.cpp
  src/servo_control/command.cpp
  src/servo_control/math_ops.c
  src/servo_control/motor_control.c
  src/servo_control/transmit.cpp

  src/ros_controller/rosLoop.cpp
  src/ros_controller/rosLoopMsgDefine.cpp
  src/ros_controller/ctlConfig.cpp

  src/debugPublish/debugPublish.cpp

  src/JntController/runinController.cpp
  src/JntController/JntTestCtl.cpp
  src/JntController/ForceComp.cpp
  src/JntController/retZeroControl.cpp
)

target_include_directories(servo_node PRIVATE
  include
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

ament_target_dependencies(servo_node rclcpp std_msgs ctl_msgs_hw Boost yaml-cpp
 rosidl_typesupport_cpp
 std_srvs
 robot_state_publisher
 ament_index_cpp
)
target_link_libraries(servo_node soem pthread rt yaml-cpp)

# EtherCAT needs CAP_NET_RAW/CAP_NET_ADMIN when servo_node is run without sudo.
# A capability-enabled ELF ignores LD_LIBRARY_PATH and rejects an $ORIGIN path
# outside trusted system directories.  Preserve absolute lookup locations for
# the interface package and the ROS prefix found by CMake.
get_filename_component(servo_ctl_msgs_prefix "${ctl_msgs_hw_DIR}/../../.." ABSOLUTE)
get_filename_component(servo_ros_prefix "${rclcpp_DIR}/../../.." ABSOLUTE)
set_target_properties(servo_node PROPERTIES
  INSTALL_RPATH "${servo_ctl_msgs_prefix}/lib;${servo_ros_prefix}/lib"
)
# DT_RUNPATH only applies to direct dependencies.  Use DT_RPATH so ROS
# libraries loaded by ctl_msgs_hw inherit the same lookup locations.
target_link_options(servo_node PRIVATE "-Wl,--disable-new-dtags")

# 安装规则
install(TARGETS servo_node
  DESTINATION lib/${PROJECT_NAME})

# 安装 yamlpkg 文件
install(DIRECTORY yamlpkg
  DESTINATION share/${PROJECT_NAME}/
)

# 告诉其他依赖这个包的工程去哪里找头文件
ament_export_include_directories(include)

ament_package()
