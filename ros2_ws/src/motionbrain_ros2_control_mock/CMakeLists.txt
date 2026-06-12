cmake_minimum_required(VERSION 3.8)
project(motionbrain_ros2_control_mock)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
