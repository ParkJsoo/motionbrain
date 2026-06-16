#include <memory>

#include "gtest/gtest.h"
#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"

TEST(MotionBrainHardwareInterfacePlugin, LoadsThroughPluginlib)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface",
    "hardware_interface::SystemInterface");

  const auto hardware = loader.createUniqueInstance(
    "motionbrain_hardware_interface/MotionBrainHardwareInterface");

  ASSERT_NE(hardware, nullptr);
}
