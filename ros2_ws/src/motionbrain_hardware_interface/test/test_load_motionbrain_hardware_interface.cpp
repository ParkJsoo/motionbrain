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

hardware_interface::HardwareInfo parse_hardware_info(const std::string & urdf)
{
  const auto hardware_infos = hardware_interface::parse_control_resources_from_urdf(urdf);
  if (hardware_infos.size() != 1) {
    throw std::runtime_error("expected one hardware component from test URDF");
  }
  return hardware_infos.front();
}

hardware_interface::HardwareInfo make_dry_run_hardware_info(const std::string & transport_mode)
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

  return parse_hardware_info(urdf);
}

hardware_interface::HardwareInfo make_m4_measured_hardware_info(
  const std::string & calibration_enabled = "true",
  const std::string & direction_sign = "1",
  const std::string & extra_joint_xml = "",
  const std::string & transport_mode = "m4_state")
{
  const std::string urdf = std::string(R"URDF(
<robot name="motionbrain_m4_measured_test">
  <link name="shoulder_link"/>
  <link name="upper_arm_link"/>
  <joint name="shoulder_pitch_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="upper_arm_link"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5708" upper="1.5708" effort="1.0" velocity="1.0"/>
  </joint>
  <ros2_control name="MotionBrainM4MeasuredStateSystem" type="system">
    <hardware>
      <plugin>motionbrain_hardware_interface/MotionBrainHardwareInterface</plugin>
      <param name="transport_mode">)URDF" + transport_mode + R"URDF(</param>
      <param name="status_topic">/motionbrain/status_typed</param>
      <param name="feedback_source">m4_as5600</param>
      <param name="shoulder_feedback_calibration_enabled">)URDF") + calibration_enabled +
    R"URDF(</param>
      <param name="shoulder_sensor_zero_deg">234.5</param>
      <param name="shoulder_direction_sign">)URDF" + direction_sign + R"URDF(</param>
      <param name="shoulder_ros_joint_zero_rad">0.0</param>
      <param name="state_stale_timeout_sec">2.0</param>
    </hardware>
    <joint name="shoulder_pitch_joint">
)URDF" + extra_joint_xml + R"URDF(
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
</robot>
)URDF";

  return parse_hardware_info(urdf);
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
  params.hardware_info = make_dry_run_hardware_info("dry_run");

  ASSERT_EQ(
    hardware_interface::CallbackReturn::SUCCESS,
    hardware.on_init(params));

  EXPECT_EQ(10u, hardware.export_state_interfaces().size());
  EXPECT_EQ(5u, hardware.export_command_interfaces().size());
}

TEST(MotionBrainHardwareInterfaceConfig, AcceptsMeasuredM4StateOnlyTransport)
{
  motionbrain_hardware_interface::MotionBrainHardwareInterface hardware;
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = make_m4_measured_hardware_info();

  ASSERT_EQ(
    hardware_interface::CallbackReturn::SUCCESS,
    hardware.on_init(params));

  EXPECT_EQ(2u, hardware.export_state_interfaces().size());
  EXPECT_TRUE(hardware.export_command_interfaces().empty());
}

TEST(MotionBrainHardwareInterfaceConfig, AcceptsUncalibratedM4StateAsUnavailable)
{
  motionbrain_hardware_interface::MotionBrainHardwareInterface hardware;
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = make_m4_measured_hardware_info("false");

  EXPECT_EQ(
    hardware_interface::CallbackReturn::SUCCESS,
    hardware.on_init(params));
  EXPECT_TRUE(hardware.export_command_interfaces().empty());
}

TEST(MotionBrainHardwareInterfaceConfig, AcceptsM4ProposalWithoutPhysicalForwarding)
{
  motionbrain_hardware_interface::MotionBrainHardwareInterface hardware;
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = make_m4_measured_hardware_info(
    "true", "1", "      <command_interface name=\"position\"/>\n", "m4_proposal");

  ASSERT_EQ(
    hardware_interface::CallbackReturn::SUCCESS,
    hardware.on_init(params));
  EXPECT_EQ(2u, hardware.export_state_interfaces().size());
  EXPECT_EQ(1u, hardware.export_command_interfaces().size());
}

TEST(MotionBrainHardwareInterfaceConfig, RejectsInvalidM4DirectionSign)
{
  motionbrain_hardware_interface::MotionBrainHardwareInterface hardware;
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = make_m4_measured_hardware_info("true", "0");

  EXPECT_EQ(
    hardware_interface::CallbackReturn::ERROR,
    hardware.on_init(params));
}

TEST(MotionBrainHardwareInterfaceConfig, RejectsM4StateOnlyCommandInterface)
{
  motionbrain_hardware_interface::MotionBrainHardwareInterface hardware;
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = make_m4_measured_hardware_info(
    "true",
    "1",
    "      <command_interface name=\"position\"/>\n");

  EXPECT_EQ(
    hardware_interface::CallbackReturn::ERROR,
    hardware.on_init(params));
}

TEST(MotionBrainHardwareInterfaceConfig, RejectsPhysicalWriteTransports)
{
  for (const auto * transport_mode : {"http", "physical"}) {
    motionbrain_hardware_interface::MotionBrainHardwareInterface hardware;
    hardware_interface::HardwareComponentInterfaceParams params;
    params.hardware_info = make_dry_run_hardware_info(transport_mode);

    EXPECT_EQ(
      hardware_interface::CallbackReturn::ERROR,
      hardware.on_init(params))
      << "transport_mode=" << transport_mode;
  }
}
