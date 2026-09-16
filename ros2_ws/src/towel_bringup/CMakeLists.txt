cmake_minimum_required(VERSION 3.16)
project(towel_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../collection/towel_state_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
