#include "motionbrain_hardware_interface/motionbrain_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace motionbrain_hardware_interface
{

namespace
{
constexpr std::size_t kExpectedJointCount = 5;

bool is_finite_vector(const std::vector<double> & values)
{
  return std::all_of(values.begin(), values.end(), [](const double value) {
    return std::isfinite(value);
  });
}
}  // namespace

hardware_interface::CallbackReturn MotionBrainHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!validate_joint_contract()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.clear();
  joint_names_.reserve(info_.joints.size());
  for (const auto & joint : info_.joints) {
    joint_names_.push_back(joint.name);
  }

  positions_.assign(joint_names_.size(), 0.0);
  velocities_.assign(joint_names_.size(), 0.0);
  commands_.assign(joint_names_.size(), 0.0);
  accepted_commands_.assign(joint_names_.size(), 0.0);
  previous_positions_.assign(joint_names_.size(), 0.0);

  command_timeout_sec_ = parse_double_parameter(info_, "command_timeout_sec", 1.0);
  max_state_step_rad_ = parse_double_parameter(info_, "max_state_step_rad", 0.1);

  const auto controller_url = info_.hardware_parameters.find("controller_url");
  if (controller_url != info_.hardware_parameters.end() && !controller_url->second.empty()) {
    controller_url_ = controller_url->second;
  }

  const auto transport_mode = info_.hardware_parameters.find("transport_mode");
  if (transport_mode != info_.hardware_parameters.end() && !transport_mode->second.empty()) {
    transport_mode_ = transport_mode->second;
  }

  last_command_change_time_ = std::chrono::steady_clock::now();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MotionBrainHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(joint_names_.size() * 2);

  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_POSITION,
      &positions_[i]);
    state_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_VELOCITY,
      &velocities_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MotionBrainHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(joint_names_.size());

  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_POSITION,
      &commands_[i]);
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn MotionBrainHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  std::fill(positions_.begin(), positions_.end(), 0.0);
  std::fill(velocities_.begin(), velocities_.end(), 0.0);
  hold_current_position();
  configured_ = true;
  active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotionBrainHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!configured_) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  hold_current_position();
  last_command_change_time_ = std::chrono::steady_clock::now();
  active_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotionBrainHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  hold_current_position();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotionBrainHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  deactivate_and_hold();
  configured_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotionBrainHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  deactivate_and_hold();
  configured_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotionBrainHardwareInterface::on_error(
  const rclcpp_lifecycle::State &)
{
  deactivate_and_hold();
  configured_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MotionBrainHardwareInterface::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  const double period_seconds = std::max(period.seconds(), 0.0);
  advance_open_loop_state(period_seconds);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MotionBrainHardwareInterface::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!active_) {
    return hardware_interface::return_type::OK;
  }

  if (!is_finite_vector(commands_)) {
    deactivate_and_hold();
    return hardware_interface::return_type::ERROR;
  }

  // This scaffold intentionally does not POST to the ESP32 motion controller.
  // Physical actuation remains behind the firmware SafetyGate and operator UI.
  (void)controller_url_;
  (void)transport_mode_;
  if (commands_ != accepted_commands_) {
    accepted_commands_ = commands_;
    last_command_change_time_ = std::chrono::steady_clock::now();
  }
  return hardware_interface::return_type::OK;
}

double MotionBrainHardwareInterface::parse_double_parameter(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  const double fallback)
{
  const auto iter = info.hardware_parameters.find(name);
  if (iter == info.hardware_parameters.end() || iter->second.empty()) {
    return fallback;
  }

  try {
    const double parsed = std::stod(iter->second);
    return std::isfinite(parsed) && parsed > 0.0 ? parsed : fallback;
  } catch (const std::invalid_argument &) {
    return fallback;
  } catch (const std::out_of_range &) {
    return fallback;
  }
}

bool MotionBrainHardwareInterface::validate_joint_contract() const
{
  if (info_.joints.size() != kExpectedJointCount) {
    return false;
  }

  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      return false;
    }

    const bool has_position_state = std::any_of(
      joint.state_interfaces.begin(),
      joint.state_interfaces.end(),
      [](const auto & state_interface) {
        return state_interface.name == hardware_interface::HW_IF_POSITION;
      });
    const bool has_velocity_state = std::any_of(
      joint.state_interfaces.begin(),
      joint.state_interfaces.end(),
      [](const auto & state_interface) {
        return state_interface.name == hardware_interface::HW_IF_VELOCITY;
      });

    if (!has_position_state || !has_velocity_state) {
      return false;
    }
  }

  return true;
}

void MotionBrainHardwareInterface::hold_current_position()
{
  commands_ = positions_;
  accepted_commands_ = positions_;
  previous_positions_ = positions_;
  std::fill(velocities_.begin(), velocities_.end(), 0.0);
}

void MotionBrainHardwareInterface::deactivate_and_hold()
{
  active_ = false;
  hold_current_position();
}

void MotionBrainHardwareInterface::advance_open_loop_state(const double period_seconds)
{
  if (!active_) {
    std::fill(velocities_.begin(), velocities_.end(), 0.0);
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const std::chrono::duration<double> age = now - last_command_change_time_;
  if (age.count() > command_timeout_sec_) {
    hold_current_position();
    return;
  }

  const double max_step = std::max(max_state_step_rad_, 0.0);
  for (std::size_t i = 0; i < positions_.size(); ++i) {
    previous_positions_[i] = positions_[i];
    const double delta = std::clamp(accepted_commands_[i] - positions_[i], -max_step, max_step);
    positions_[i] += delta;
    if (period_seconds > std::numeric_limits<double>::epsilon()) {
      velocities_[i] = (positions_[i] - previous_positions_[i]) / period_seconds;
    } else {
      velocities_[i] = 0.0;
    }
  }
}

}  // namespace motionbrain_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  motionbrain_hardware_interface::MotionBrainHardwareInterface,
  hardware_interface::SystemInterface)
