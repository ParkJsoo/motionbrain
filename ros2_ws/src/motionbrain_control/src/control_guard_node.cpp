#include <algorithm>
#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "motionbrain_control/control_guard_logic.hpp"
#include "motionbrain_msgs/msg/camera_detection.hpp"
#include "motionbrain_msgs/msg/motion_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace
{
std::string escape_json(const std::string & value)
{
  std::ostringstream out;
  for (const char ch : value) {
    switch (ch) {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << ch;
        break;
    }
  }
  return out.str();
}

std::string json_bool(const bool value)
{
  return value ? "true" : "false";
}
}  // namespace

class MotionBrainControlGuardNode : public rclcpp::Node
{
public:
  MotionBrainControlGuardNode()
  : Node("motionbrain_control_guard_node")
  {
    status_topic_ = declare_parameter<std::string>("status_topic", "/motionbrain/status_typed");
    detection_topic_ = declare_parameter<std::string>("detection_topic", "/camera/detection_typed");
    output_topic_ = declare_parameter<std::string>("output_topic", "/motionbrain/control_guard");
    stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 3.0);
    require_armed_ = declare_parameter<bool>("require_armed", false);
    require_detection_ = declare_parameter<bool>("require_detection", false);
    const double publish_rate_hz = std::max(declare_parameter<double>("publish_rate_hz", 2.0), 0.1);

    status_sub_ = create_subscription<motionbrain_msgs::msg::MotionStatus>(
      status_topic_,
      10,
      [this](motionbrain_msgs::msg::MotionStatus::SharedPtr msg) {
        latest_status_ = msg;
        last_status_time_ = now();
      });

    detection_sub_ = create_subscription<motionbrain_msgs::msg::CameraDetection>(
      detection_topic_,
      10,
      [this](motionbrain_msgs::msg::CameraDetection::SharedPtr msg) {
        latest_detection_ = msg;
        last_detection_time_ = now();
      });

    guard_pub_ = create_publisher<std_msgs::msg::String>(output_topic_, 10);
    const auto publish_period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz));
    timer_ = create_wall_timer(
      publish_period,
      [this]() { publish_guard_state(); });

    RCLCPP_INFO(
      get_logger(),
      "Publishing C++ control guard on %s from %s and %s",
      output_topic_.c_str(),
      status_topic_.c_str(),
      detection_topic_.c_str());
  }

private:
  double age_seconds(const rclcpp::Time & stamp) const
  {
    if (stamp.nanoseconds() == 0) {
      return stale_timeout_sec_ + 1.0;
    }
    return std::max((now() - stamp).seconds(), 0.0);
  }

  void publish_guard_state()
  {
    const double status_age = age_seconds(last_status_time_);
    const double detection_age = age_seconds(last_detection_time_);
    const bool status_fresh = latest_status_ && status_age <= stale_timeout_sec_;
    const bool detection_fresh = latest_detection_ && detection_age <= stale_timeout_sec_;

    motionbrain_control::MotionStatusSnapshot status;
    if (latest_status_) {
      status.available = latest_status_->available;
      status.armed = latest_status_->armed;
      status.moving = latest_status_->moving;
      status.faulted = latest_status_->faulted;
      status.state = latest_status_->state;
    }

    motionbrain_control::CameraDetectionSnapshot detection;
    if (latest_detection_) {
      detection.available = latest_detection_->available;
      detection.detected = latest_detection_->detected;
      detection.alignment = latest_detection_->alignment;
      detection.command_suggestion = latest_detection_->command_suggestion;
    }

    motionbrain_control::ControlGuardConfig config;
    config.require_armed = require_armed_;
    config.require_detection = require_detection_;
    const auto decision = motionbrain_control::evaluate_control_guard(
      status,
      detection,
      status_fresh,
      detection_fresh,
      config);

    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "{"
        << "\"ready\":" << json_bool(decision.ready) << ","
        << "\"reason\":\"" << escape_json(decision.reason) << "\","
        << "\"suggestedAction\":\"" << escape_json(decision.suggested_action) << "\","
        << "\"statusFresh\":" << json_bool(status_fresh) << ","
        << "\"detectionFresh\":" << json_bool(detection_fresh) << ","
        << "\"statusAgeSec\":" << status_age << ","
        << "\"detectionAgeSec\":" << detection_age << ","
        << "\"state\":\"" << escape_json(status.state) << "\","
        << "\"armed\":" << json_bool(latest_status_ && latest_status_->armed) << ","
        << "\"moving\":" << json_bool(latest_status_ && latest_status_->moving) << ","
        << "\"faulted\":" << json_bool(latest_status_ && latest_status_->faulted) << ","
        << "\"cameraAvailable\":" << json_bool(latest_detection_ && latest_detection_->available) << ","
        << "\"targetDetected\":" << json_bool(latest_detection_ && latest_detection_->detected) << ","
        << "\"alignment\":\"" << escape_json(detection.alignment) << "\""
        << "}";

    std_msgs::msg::String message;
    message.data = out.str();
    guard_pub_->publish(message);
  }

  std::string status_topic_;
  std::string detection_topic_;
  std::string output_topic_;
  double stale_timeout_sec_{3.0};
  bool require_armed_{false};
  bool require_detection_{false};
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_detection_time_{0, 0, RCL_ROS_TIME};
  motionbrain_msgs::msg::MotionStatus::SharedPtr latest_status_;
  motionbrain_msgs::msg::CameraDetection::SharedPtr latest_detection_;
  rclcpp::Subscription<motionbrain_msgs::msg::MotionStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<motionbrain_msgs::msg::CameraDetection>::SharedPtr detection_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr guard_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionBrainControlGuardNode>());
  rclcpp::shutdown();
  return 0;
}
