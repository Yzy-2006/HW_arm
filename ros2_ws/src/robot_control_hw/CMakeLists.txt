cmake_minimum_required(VERSION 3.8)
project(robot_control_hw)

# 强制使用系统 Python 3.12（匹配 ROS2 Jazzy 与 openrobots_libs_hw 的 cpython-312），
# 避免被激活的 conda 环境（如 py3.10 的 ros_env）劫持 Python3 变量
set(Python3_EXECUTABLE "/usr/bin/python3.12" CACHE FILEPATH "Force system Python 3.12" FORCE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(ctl_msgs_hw REQUIRED)
find_package(chassis_control_hw REQUIRED)
find_package(pinocchio REQUIRED)


find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(std_srvs REQUIRED)          # 提供 SetBool 服务
find_package(robot_state_publisher REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(chassis_control_hw REQUIRED)

# 包含头文件路径
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${dual_arm_hw_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}

  # 添加python支持
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}

  /usr/include
  /opt/ros/$ENV{ROS_DISTRO}/include
  /usr/include/opencv4
  /usr/include/eigen3
  /usr/include/gazebo-11
  /usr/include/ignition/math6
  /usr/include/sdformat-9.9
  /usr/include/ignition/common3
  /usr/include/ignition/fuel_tools4
  /usr/include/ignition/msgs5
  /usr/include/ignition/transport8
)

add_compile_definitions(
    __GLIBC_USE_IEC_60559_BFP_EXT=1
    __GLIBC_USE_ISOC2X=0
    _GNU_SOURCE=1
)

add_definitions(-DEIGEN_AVOID_COMPLEX_VECTORIZATION)
link_directories(
    "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}"
    "/opt/ros/$ENV{ROS_DISTRO}/lib"
)

add_executable(robot_control_node 

  #主循环
  src/robot_control_node.cpp

  src/controller/demoTraj.cpp
  src/controller/upper_Transmit.cpp
  src/controller/xrTelepercationCom.cpp
  src/controller/ctlTransmit.cpp

  src/messageType/upper_msgType.cpp

  src/debugPublish/debugPublish.cpp
  
  src/robot_control/msgPublisher.cpp
  src/robot_control/msgType.cpp
  src/robot_control/trajProcessWrap.cpp
  src/robot_control/config.cpp

)

target_include_directories(robot_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(robot_control_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  robot_control_node
  "rclcpp"
  "std_msgs"
)

ament_target_dependencies(robot_control_node rclcpp std_msgs
 ctl_msgs_hw
 geometry_msgs
 Boost
 rosidl_typesupport_cpp
 std_srvs sensor_msgs
 robot_state_publisher
 ament_index_cpp
 kdl_parser
 tf2_kdl

 chassis_control_hw

)

target_link_libraries(robot_control_node pthread rt yaml-cpp
  orocos-kdl
  kdl_parser
  urdf
  tbb
  pinocchio::pinocchio
  ${Python3_LIBRARIES}    #python库添加
 ${orocos_kdl_LIBRARIES}  # 确保链接 KDL
"${cpp_typesupport_target}"  # 链接服务接口支持

  ${chassis_control_hw_LIBRARIES}

)

target_compile_definitions(robot_control_node PRIVATE
    "EIGEN_USE_STD_ABS=1"  # 告诉 Eigen 优先使用 std::abs
)

install(TARGETS robot_control_node
  DESTINATION lib/${PROJECT_NAME})

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
