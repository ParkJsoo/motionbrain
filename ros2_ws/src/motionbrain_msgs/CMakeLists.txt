cmake_minimum_required(VERSION 3.8)
project(motionbrain_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CameraDetection.msg"
  "msg/ControlGuard.msg"
  "msg/KinematicsState.msg"
  "msg/LightCommand.msg"
  "msg/LightResult.msg"
  "msg/MissionCommand.msg"
  "msg/MissionState.msg"
  "msg/MotionEvent.msg"
  "msg/MotionStatus.msg"
  "msg/RoutineCommand.msg"
  "msg/RoutineResult.msg"
  "msg/RoutineStatus.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
