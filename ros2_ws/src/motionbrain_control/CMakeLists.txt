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
target_include_directories(
  motionbrain_control_guard_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
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

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_control_guard_logic test/test_control_guard_logic.cpp)
  target_include_directories(
    test_control_guard_logic
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()

ament_export_include_directories(include)

ament_package()
