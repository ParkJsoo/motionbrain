#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace motionbrain_hardware_interface
{

class MotionBrainHardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  static double parse_double_parameter(
    const hardware_interface::HardwareInfo & info,
    const std::string & name,
    double fallback);

  bool validate_joint_contract() const;
  void hold_current_position();
  void deactivate_and_hold();
  void advance_open_loop_state(double period_seconds);

  std::vector<std::string> joint_names_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> commands_;
  std::vector<double> accepted_commands_;
  std::vector<double> previous_positions_;
  std::chrono::steady_clock::time_point last_command_change_time_;
  double command_timeout_sec_{1.0};
  double max_state_step_rad_{0.1};
  std::string controller_url_{"http://motionbrain.local"};
  std::string transport_mode_{"dry_run"};
  bool configured_{false};
  bool active_{false};
};

}  // namespace motionbrain_hardware_interface
