#include <memory>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/system_interface.hpp"
#include "motionbrain_hardware_interface/motionbrain_hardware_interface.hpp"
#include "pluginlib/class_loader.hpp"

namespace
{

hardware_interface::HardwareInfo make_hardware_info(const std::string & transport_mode)
{
  const std::string urdf = std::string(R"URDF(
<robot name="motionbrain_hardware_interface_test">
  <link name="base_link"/>
  <link name="shoulder_link"/>
  <joint name="base_yaw_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.1416" upper="3.1416" effort="1.0" velocity="1.0"/>
  </joint>
  <link name="upper_arm_link"/>
  <joint name="shoulder_pitch_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="upper_arm_link"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5708" upper="1.5708" effort="1.0" velocity="1.0"/>
  </joint>
  <link name="forearm_link"/>
  <joint name="elbow_pitch_joint" type="revolute">
    <parent link="upper_arm_link"/>
    <child link="forearm_link"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5708" upper="1.5708" effort="1.0" velocity="1.0"/>
  </joint>
  <link name="wrist_link"/>
  <joint name="wrist_pitch_joint" type="revolute">
    <parent link="forearm_link"/>
    <child link="wrist_link"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5708" upper="1.5708" effort="1.0" velocity="1.0"/>
  </joint>
  <link name="gripper_link"/>
  <joint name="gripper_joint" type="revolute">
    <parent link="wrist_link"/>
    <child link="gripper_link"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.7854" upper="0.7854" effort="1.0" velocity="1.0"/>
  </joint>
  <ros2_control name="MotionBrainOpenLoopSystem" type="system">
    <hardware>
      <plugin>motionbrain_hardware_interface/MotionBrainHardwareInterface</plugin>
      <param name="controller_url">http://motionbrain.local</param>
      <param name="transport_mode">)URDF") + transport_mode + R"URDF(</param>
      <param name="command_timeout_sec">1.0</param>
      <param name="max_state_step_rad">0.1</param>
    </hardware>
    <joint name="base_yaw_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="shoulder_pitch_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="elbow_pitch_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="wrist_pitch_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="gripper_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
</robot>
)URDF";

  const auto hardware_infos = hardware_interface::parse_control_resources_from_urdf(urdf);
  if (hardware_infos.size() != 1) {
    throw std::runtime_error("expected one hardware component from test URDF");
  }
  return hardware_infos.front();
}

}  // namespace

TEST(MotionBrainHardwareInterfacePlugin, LoadsThroughPluginlib)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface",
    "hardware_interface::SystemInterface");

  const auto hardware = loader.createUniqueInstance(
    "motionbrain_hardware_interface/MotionBrainHardwareInterface");

  ASSERT_NE(hardware, nullptr);
}

TEST(MotionBrainHardwareInterfaceConfig, AcceptsDryRunTransport)
{
  motionbrain_hardware_interface::MotionBrainHardwareInterface hardware;
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = make_hardware_info("dry_run");

  EXPECT_EQ(
    hardware_interface::CallbackReturn::SUCCESS,
    hardware.on_init(params));
}

TEST(MotionBrainHardwareInterfaceConfig, RejectsNonDryRunTransport)
{
  motionbrain_hardware_interface::MotionBrainHardwareInterface hardware;
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = make_hardware_info("http");

  EXPECT_EQ(
    hardware_interface::CallbackReturn::ERROR,
    hardware.on_init(params));
}
