cmake_minimum_required(VERSION 3.5)
project(darm_ros2)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config description launch meshes rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/teleop_ps5.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()