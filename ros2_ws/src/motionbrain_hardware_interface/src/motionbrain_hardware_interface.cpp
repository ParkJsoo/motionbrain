#include "motionbrain_hardware_interface/motionbrain_hardware_interface.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace motionbrain_hardware_interface
{

namespace
{
constexpr std::size_t kDryRunJointCount = 5;
constexpr std::size_t kStateOnlyJointCount = 1;
constexpr const char * kDryRunTransportMode = "dry_run";
constexpr const char * kStateTransportMode = "m4_state";
constexpr const char * kLegacyStateTransportMode = "state";
constexpr const char * kM4StateJointName = "shoulder_pitch_joint";
constexpr const char * kM4FeedbackSourceParam = "feedback_source";
constexpr const char * kM4FeedbackSource = "m4_as5600";
constexpr double kPi = 3.14159265358979323846;

bool is_finite_vector(const std::vector<double> & values)
{
  return std::all_of(values.begin(), values.end(), [](const double value) {
    return std::isfinite(value);
  });
}

double unavailable_state()
{
  return std::numeric_limits<double>::quiet_NaN();
}

std::string lower_copy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value;
}

bool parse_finite_parameter(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  const double fallback,
  double & output)
{
  const auto iter = info.hardware_parameters.find(name);
  if (iter == info.hardware_parameters.end() || iter->second.empty()) {
    output = fallback;
    return true;
  }

  try {
    output = std::stod(iter->second);
  } catch (const std::invalid_argument &) {
    return false;
  } catch (const std::out_of_range &) {
    return false;
  }
  return std::isfinite(output);
}

bool parse_positive_parameter(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  const double fallback,
  double & output)
{
  if (!parse_finite_parameter(info, name, fallback, output)) {
    return false;
  }
  return output > 0.0;
}

bool parse_bool_parameter(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  const bool fallback,
  bool & output)
{
  const auto iter = info.hardware_parameters.find(name);
  if (iter == info.hardware_parameters.end() || iter->second.empty()) {
    output = fallback;
    return true;
  }

  const auto value = lower_copy(iter->second);
  if (value == "1" || value == "true" || value == "yes" || value == "on") {
    output = true;
    return true;
  }
  if (value == "0" || value == "false" || value == "no" || value == "off") {
    output = false;
    return true;
  }
  return false;
}

bool parse_direction_sign_parameter(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  const int fallback,
  int & output)
{
  const auto iter = info.hardware_parameters.find(name);
  if (iter == info.hardware_parameters.end() || iter->second.empty()) {
    output = fallback;
    return output == -1 || output == 1;
  }

  try {
    output = std::stoi(iter->second);
  } catch (const std::invalid_argument &) {
    return false;
  } catch (const std::out_of_range &) {
    return false;
  }
  return output == -1 || output == 1;
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

  controller_url_.clear();
  transport_mode_ = kDryRunTransportMode;
  status_topic_ = "/motionbrain/status_typed";
  feedback_source_ = kM4FeedbackSource;
  shoulder_feedback_calibration_enabled_ = false;
  shoulder_sensor_zero_deg_ = 0.0;
  shoulder_direction_sign_ = 1;
  shoulder_ros_joint_zero_rad_ = 0.0;
  state_stale_timeout_sec_ = 0.25;

  const auto controller_url = info_.hardware_parameters.find("controller_url");
  if (controller_url != info_.hardware_parameters.end() && !controller_url->second.empty()) {
    controller_url_ = controller_url->second;
  }

  const auto transport_mode = info_.hardware_parameters.find("transport_mode");
  if (transport_mode != info_.hardware_parameters.end() && !transport_mode->second.empty()) {
    transport_mode_ = transport_mode->second;
  }
  if (transport_mode_ == kLegacyStateTransportMode) {
    transport_mode_ = kStateTransportMode;
  }
  if (transport_mode_ != kDryRunTransportMode && transport_mode_ != kStateTransportMode) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  command_timeout_sec_ = parse_double_parameter(info_, "command_timeout_sec", 1.0);
  max_state_step_rad_ = parse_double_parameter(info_, "max_state_step_rad", 0.1);

  if (state_only_mode()) {
    const auto status_topic = info_.hardware_parameters.find("status_topic");
    if (status_topic != info_.hardware_parameters.end() && !status_topic->second.empty()) {
      status_topic_ = status_topic->second;
    }

    const auto feedback_source = info_.hardware_parameters.find(kM4FeedbackSourceParam);
    if (feedback_source != info_.hardware_parameters.end() && !feedback_source->second.empty()) {
      feedback_source_ = feedback_source->second;
    }

    if (!parse_bool_parameter(
        info_,
        "shoulder_feedback_calibration_enabled",
        false,
        shoulder_feedback_calibration_enabled_))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!parse_finite_parameter(
        info_,
        "shoulder_sensor_zero_deg",
        0.0,
        shoulder_sensor_zero_deg_))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!parse_direction_sign_parameter(
        info_,
        "shoulder_direction_sign",
        1,
        shoulder_direction_sign_))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!parse_finite_parameter(
        info_,
        "shoulder_ros_joint_zero_rad",
        0.0,
        shoulder_ros_joint_zero_rad_))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!parse_positive_parameter(
        info_,
        "state_stale_timeout_sec",
        0.25,
        state_stale_timeout_sec_))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
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
  previous_positions_.assign(joint_names_.size(), 0.0);
  if (state_only_mode()) {
    commands_.clear();
    accepted_commands_.clear();
  } else {
    commands_.assign(joint_names_.size(), 0.0);
    accepted_commands_.assign(joint_names_.size(), 0.0);
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
  if (state_only_mode()) {
    return {};
  }

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
  if (state_only_mode()) {
    set_state_only_interfaces_unavailable();
    try {
      configure_state_only_subscription();
    } catch (const std::runtime_error &) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else {
    std::fill(positions_.begin(), positions_.end(), 0.0);
    std::fill(velocities_.begin(), velocities_.end(), 0.0);
    hold_current_position();
  }
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
  if (state_only_mode()) {
    set_state_only_interfaces_unavailable();
  } else {
    hold_current_position();
  }
  last_command_change_time_ = std::chrono::steady_clock::now();
  active_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotionBrainHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  if (state_only_mode()) {
    set_state_only_interfaces_unavailable();
  } else {
    hold_current_position();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotionBrainHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  reset_state_only_subscription();
  deactivate_and_hold();
  configured_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotionBrainHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  reset_state_only_subscription();
  deactivate_and_hold();
  configured_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotionBrainHardwareInterface::on_error(
  const rclcpp_lifecycle::State &)
{
  reset_state_only_subscription();
  deactivate_and_hold();
  configured_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MotionBrainHardwareInterface::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  if (state_only_mode()) {
    if (!active_) {
      set_state_only_interfaces_unavailable();
      return hardware_interface::return_type::OK;
    }

    std::lock_guard<std::mutex> lock(state_cache_mutex_);
    const auto now = std::chrono::steady_clock::now();
    const std::chrono::duration<double> age = now - cached_shoulder_time_;
    const bool stale =
      !state_cache_has_sample_ || age.count() > state_stale_timeout_sec_;
    previous_positions_ = positions_;
    positions_[0] = stale ? unavailable_state() : cached_shoulder_position_;
    velocities_[0] = stale ? unavailable_state() : cached_shoulder_velocity_;
    return hardware_interface::return_type::OK;
  }

  const double period_seconds = std::max(period.seconds(), 0.0);
  advance_open_loop_state(period_seconds);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MotionBrainHardwareInterface::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (state_only_mode()) {
    return hardware_interface::return_type::OK;
  }

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
  if (transport_mode_ == kDryRunTransportMode) {
    return validate_dry_run_joint_contract();
  }
  if (transport_mode_ == kStateTransportMode) {
    return validate_state_only_joint_contract();
  }
  return false;
}

bool MotionBrainHardwareInterface::validate_dry_run_joint_contract() const
{
  if (info_.joints.size() != kDryRunJointCount) {
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

bool MotionBrainHardwareInterface::validate_state_only_joint_contract() const
{
  if (info_.joints.size() != kStateOnlyJointCount) {
    return false;
  }

  const auto feedback_source = info_.hardware_parameters.find(kM4FeedbackSourceParam);
  if (feedback_source == info_.hardware_parameters.end() ||
    feedback_source->second != kM4FeedbackSource)
  {
    return false;
  }

  const auto & joint = info_.joints.front();
  if (joint.name != kM4StateJointName || !joint.command_interfaces.empty()) {
    return false;
  }

  if (feedback_source_ != kM4FeedbackSource || status_topic_.empty()) {
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

  return has_position_state && has_velocity_state;
}

bool MotionBrainHardwareInterface::state_only_mode() const
{
  return transport_mode_ == kStateTransportMode;
}

void MotionBrainHardwareInterface::configure_state_only_subscription()
{
  auto node = get_node();
  if (!node) {
    throw std::runtime_error("framework-managed hardware node is unavailable");
  }
  status_subscription_ = node->create_subscription<motionbrain_msgs::msg::MotionStatus>(
    status_topic_,
    rclcpp::QoS(10),
    [this](const motionbrain_msgs::msg::MotionStatus::SharedPtr message) {
      handle_motion_status(*message);
    });
}

void MotionBrainHardwareInterface::reset_state_only_subscription()
{
  status_subscription_.reset();
  std::lock_guard<std::mutex> lock(state_cache_mutex_);
  state_cache_has_sample_ = false;
  cached_shoulder_position_ = unavailable_state();
  cached_shoulder_velocity_ = unavailable_state();
  cached_shoulder_time_ = std::chrono::steady_clock::time_point{};
}

void MotionBrainHardwareInterface::handle_motion_status(
  const motionbrain_msgs::msg::MotionStatus & message)
{
  const auto now = std::chrono::steady_clock::now();
  double position = unavailable_state();
  double velocity = unavailable_state();

  const bool ready =
    shoulder_feedback_calibration_enabled_ &&
    message.shoulder_feedback_available &&
    message.shoulder_sensor_connected &&
    message.shoulder_sensor_fresh &&
    message.shoulder_sensor_ready &&
    std::isfinite(static_cast<double>(message.shoulder_angle_deg));

  std::lock_guard<std::mutex> lock(state_cache_mutex_);
  if (ready) {
    position = map_shoulder_sensor_to_joint(
      static_cast<double>(message.shoulder_angle_deg));
    if (state_cache_has_sample_ && std::isfinite(cached_shoulder_position_)) {
      const std::chrono::duration<double> dt = now - cached_shoulder_time_;
      if (dt.count() > std::numeric_limits<double>::epsilon()) {
        velocity = (position - cached_shoulder_position_) / dt.count();
      } else {
        velocity = 0.0;
      }
    } else {
      velocity = 0.0;
    }
  }

  cached_shoulder_position_ = position;
  cached_shoulder_velocity_ = velocity;
  cached_shoulder_time_ = now;
  state_cache_has_sample_ = true;
}

void MotionBrainHardwareInterface::set_state_only_interfaces_unavailable()
{
  std::fill(positions_.begin(), positions_.end(), unavailable_state());
  std::fill(velocities_.begin(), velocities_.end(), unavailable_state());
  previous_positions_ = positions_;
  std::lock_guard<std::mutex> lock(state_cache_mutex_);
  state_cache_has_sample_ = false;
  cached_shoulder_position_ = unavailable_state();
  cached_shoulder_velocity_ = unavailable_state();
  cached_shoulder_time_ = std::chrono::steady_clock::time_point{};
}

double MotionBrainHardwareInterface::map_shoulder_sensor_to_joint(
  const double sensor_angle_deg) const
{
  return shoulder_ros_joint_zero_rad_ +
    (static_cast<double>(shoulder_direction_sign_) *
    (sensor_angle_deg - shoulder_sensor_zero_deg_) * kPi / 180.0);
}

void MotionBrainHardwareInterface::hold_current_position()
{
  if (state_only_mode()) {
    commands_.clear();
    accepted_commands_.clear();
  } else {
    commands_ = positions_;
    accepted_commands_ = positions_;
  }
  previous_positions_ = positions_;
  std::fill(velocities_.begin(), velocities_.end(), 0.0);
}

void MotionBrainHardwareInterface::deactivate_and_hold()
{
  active_ = false;
  if (state_only_mode()) {
    set_state_only_interfaces_unavailable();
  } else {
    hold_current_position();
  }
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
