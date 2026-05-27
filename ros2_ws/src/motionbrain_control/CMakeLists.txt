cmake_minimum_required(VERSION 3.8)
project(motionbrain_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(motionbrain_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(motionbrain_control_guard_node src/control_guard_node.cpp)
target_compile_features(motionbrain_control_guard_node PUBLIC cxx_std_17)
ament_target_dependencies(
  motionbrain_control_guard_node
  motionbrain_msgs
  rclcpp
  std_msgs
)

install(
  TARGETS motionbrain_control_guard_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
