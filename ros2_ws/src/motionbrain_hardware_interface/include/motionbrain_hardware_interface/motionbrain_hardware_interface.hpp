#pragma once

#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "motionbrain_msgs/msg/motion_status.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace motionbrain_hardware_interface
{

class MotionBrainHardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

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
  bool validate_dry_run_joint_contract() const;
  bool validate_state_only_joint_contract() const;
  bool state_only_mode() const;
  void configure_state_only_subscription();
  void reset_state_only_subscription();
  void handle_motion_status(const motionbrain_msgs::msg::MotionStatus & message);
  void set_state_only_interfaces_unavailable();
  double map_shoulder_sensor_to_joint(double sensor_angle_deg) const;
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
  std::string controller_url_;
  std::string transport_mode_{"dry_run"};
  std::string status_topic_{"/motionbrain/status_typed"};
  std::string feedback_source_{"m4_as5600"};
  bool shoulder_feedback_calibration_enabled_{false};
  double shoulder_sensor_zero_deg_{0.0};
  int shoulder_direction_sign_{1};
  double shoulder_ros_joint_zero_rad_{0.0};
  double state_stale_timeout_sec_{0.25};
  rclcpp::Subscription<motionbrain_msgs::msg::MotionStatus>::SharedPtr status_subscription_;
  std::mutex state_cache_mutex_;
  bool state_cache_has_sample_{false};
  double cached_shoulder_position_{0.0};
  double cached_shoulder_velocity_{0.0};
  std::chrono::steady_clock::time_point cached_shoulder_time_;
  bool configured_{false};
  bool active_{false};
};

}  // namespace motionbrain_hardware_interface
