cmake_minimum_required(VERSION 3.8)
project(encos_force_sensor)

# 跨发行版/跨架构可移植：
#   - 仅依赖标准 ROS2 包，无硬编码 /opt/ros 路径
#   - 不依赖发行版相关的 Python 版本
#   - ZLG 官方库 libcontrolcanfd.so 仅提供 x86_64 版本，故仅在该架构编译 ZLG 后端
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ---------------------------------------------------------------------------
# 核心库：传输抽象层 + SocketCAN 后端 + 解码 + 去皮（跨平台，x86 与 aarch64 均编译）
# ---------------------------------------------------------------------------
add_library(encos_force_sensor_core
  src/can_transport.cpp
  src/ft_decoder.cpp
  src/tare_manager.cpp
  src/socketcan_transport.cpp
)
target_include_directories(encos_force_sensor_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(encos_force_sensor_core
  ament_index_cpp)

# ---------------------------------------------------------------------------
# ZLG 后端（可选，仅 x86_64）
# 官方 libcontrolcanfd.so 为 x86_64 only，aarch64(Thor) 上不编译、运行时自动跳过
# ---------------------------------------------------------------------------
set(ENCOS_ENABLE_ZLG OFF)
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(ENCOS_ENABLE_ZLG ON)
endif()

if(ENCOS_ENABLE_ZLG)
  target_compile_definitions(encos_force_sensor_core PUBLIC ENCOS_FORCE_SENSOR_HAS_ZLG=1)
  target_sources(encos_force_sensor_core PRIVATE src/zlg_transport.cpp)
  target_link_libraries(encos_force_sensor_core dl)
endif()

# ---------------------------------------------------------------------------
# 主节点可执行文件
# ---------------------------------------------------------------------------
add_executable(force_sensor_node src/force_sensor_node.cpp)
target_link_libraries(force_sensor_node encos_force_sensor_core)
ament_target_dependencies(force_sensor_node
  rclcpp geometry_msgs std_srvs ament_index_cpp)
# yaml-cpp 非 ament 包，直接链接其导入目标（与 robot_control_hw 用法一致）
target_link_libraries(force_sensor_node yaml-cpp)

# ---------------------------------------------------------------------------
# 安装规则
# ---------------------------------------------------------------------------
install(TARGETS force_sensor_node encos_force_sensor_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY docs DESTINATION share/${PROJECT_NAME})

# ZLG 官方库随包安装（仅 x86_64）
if(ENCOS_ENABLE_ZLG)
  install(FILES lib/libcontrolcanfd.so DESTINATION lib/${PROJECT_NAME})
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
