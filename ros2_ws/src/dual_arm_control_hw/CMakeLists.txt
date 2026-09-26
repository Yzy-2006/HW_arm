cmake_minimum_required(VERSION 3.8)
project(dual_arm_control_hw)

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
find_package(dual_arm_hw REQUIRED)
find_package(pinocchio REQUIRED)
# 显式指定库路径（可选，如果 find_package 未自动处理）


find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(std_srvs REQUIRED)          # 提供 SetBool 服务
find_package(robot_state_publisher REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(openrobots_libs_hw QUIET)

#添加python支持
# 查找Python和NumPy
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)

# 获取 NumPy 头文件路径（cython 生成的 C 代码需要）
execute_process(
    COMMAND ${Python3_EXECUTABLE} -c "import numpy; print(numpy.get_include())"
    OUTPUT_VARIABLE PYTHON3_NUMPY_INCLUDE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(
    COMMAND ${Python3_EXECUTABLE} -c "import cython; print(cython.__version__)"
    OUTPUT_VARIABLE CYTHON_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE)
message(STATUS "Cython version: ${CYTHON_VERSION}")
message(STATUS "NumPy include: ${PYTHON3_NUMPY_INCLUDE_DIR}")

# 安装环境钩子
install(
  DIRECTORY env_hooks/
  DESTINATION share/${PROJECT_NAME}/env_hooks
)

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

  # openrobots_libs_hw 第三方库


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

#完整版的主程序
add_executable(dual_arm_control_node 

src/dual_arm_control_node.cpp

src/dual_arm_control/msgPublish.cpp

src/robot_config/config.cpp

src/debugPublish/debugPublish.cpp

#添加python支持
src/controller/ik_solver_wrapper.cpp
src/controller/JntController.cpp
src/controller/ArmController.cpp
src/controller/ArmInterface.cpp
src/controller/IKFKsolver.cpp
)

target_include_directories(dual_arm_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(dual_arm_control_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  dual_arm_control_node
  "rclcpp"
  "std_msgs"
)

ament_target_dependencies(dual_arm_control_node rclcpp std_msgs ctl_msgs_hw Boost
 rosidl_typesupport_cpp
 std_srvs sensor_msgs geometry_msgs
 robot_state_publisher
 ament_index_cpp
 kdl_parser
 tf2_kdl
)
target_link_libraries(dual_arm_control_node pthread rt yaml-cpp
  orocos-kdl
  kdl_parser
  urdf
  tbb
  pinocchio::pinocchio
  ${Python3_LIBRARIES}    #python库添加
 ${orocos_kdl_LIBRARIES}  # 确保链接 KDL
"${cpp_typesupport_target}"  # 链接服务接口支持
)
target_compile_definitions(dual_arm_control_node PRIVATE
    "EIGEN_USE_STD_ABS=1"  # 告诉 Eigen 优先使用 std::abs
)

install(TARGETS dual_arm_control_node
  DESTINATION lib/${PROJECT_NAME})

# Install readable Python IK modules; no Cython or copied architecture-specific extensions.
install(DIRECTORY scripts/ DESTINATION share/${PROJECT_NAME}/scripts
  FILES_MATCHING PATTERN "*.py")

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
