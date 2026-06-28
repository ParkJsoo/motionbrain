cmake_minimum_required(VERSION 3.8)
project(motionbrain_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(motionbrain_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(
  motionbrain_hardware_interface
  SHARED
  src/motionbrain_hardware_interface.cpp
)
target_compile_features(motionbrain_hardware_interface PUBLIC cxx_std_17)
target_include_directories(
  motionbrain_hardware_interface
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  motionbrain_hardware_interface
  hardware_interface
  motionbrain_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(
  hardware_interface
  motionbrain_hardware_interface.xml
)

install(
  TARGETS motionbrain_hardware_interface
  EXPORT export_motionbrain_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(
    test_load_motionbrain_hardware_interface
    test/test_load_motionbrain_hardware_interface.cpp
  )
  target_include_directories(
    test_load_motionbrain_hardware_interface
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(
    test_load_motionbrain_hardware_interface
    hardware_interface
    pluginlib
  )
  target_link_libraries(
    test_load_motionbrain_hardware_interface
    motionbrain_hardware_interface
  )
endif()

ament_export_include_directories(include)
ament_export_targets(export_motionbrain_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  motionbrain_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
