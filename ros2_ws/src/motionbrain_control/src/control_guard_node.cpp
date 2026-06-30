#include <algorithm>
#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "motionbrain_control/control_guard_logic.hpp"
#include "motionbrain_msgs/msg/camera_detection.hpp"
#include "motionbrain_msgs/msg/control_guard.hpp"
#include "motionbrain_msgs/msg/motion_status.hpp"
#include "motionbrain_msgs/msg/node_lifecycle_status.hpp"
#include "rclcpp/create_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
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

class MotionBrainControlGuardNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  MotionBrainControlGuardNode()
  : rclcpp_lifecycle::LifecycleNode("motionbrain_control_guard_node")
  {
    declare_parameter<std::string>("status_topic", "/motionbrain/status_typed");
    declare_parameter<std::string>("detection_topic", "/camera/detection_typed");
    declare_parameter<std::string>("output_topic", "/motionbrain/control_guard_typed");
    declare_parameter<std::string>("json_output_topic", "/motionbrain/control_guard");
    declare_parameter<double>("stale_timeout_sec", 3.0);
    declare_parameter<bool>("require_armed", false);
    declare_parameter<bool>("require_detection", false);
    declare_parameter<double>("publish_rate_hz", 2.0);
    declare_parameter<bool>("autostart", true);

    lifecycle_pub_ = rclcpp::create_publisher<motionbrain_msgs::msg::NodeLifecycleStatus>(
      *this,
      "/motionbrain/lifecycle_typed",
      10);
    lifecycle_json_pub_ = rclcpp::create_publisher<std_msgs::msg::String>(
      *this,
      "/motionbrain/lifecycle",
      10);
    lifecycle_timer_ = create_wall_timer(5s, [this]() { publish_lifecycle_status(); });
    set_lifecycle_state(
      motionbrain_msgs::msg::NodeLifecycleStatus::PRIMARY_STATE_UNCONFIGURED,
      "unconfigured",
      false,
      false,
      "unconfigured control guard");
    publish_lifecycle_status();

    if (get_parameter("autostart").as_bool()) {
      configure();
      activate();
    }
  }

private:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using State = rclcpp_lifecycle::State;

  CallbackReturn on_configure(const State & previous_state) override
  {
    (void) previous_state;
    try {
      read_configuration();
      create_configured_entities();
      set_lifecycle_state(
        motionbrain_msgs::msg::NodeLifecycleStatus::PRIMARY_STATE_INACTIVE,
        "inactive",
        false,
        false,
        "configured guard from " + status_topic_ + " and " + detection_topic_ +
        "; waiting for activation");
      RCLCPP_INFO(
        get_logger(),
        "MotionBrain control guard configured from %s and %s; waiting for lifecycle activation",
        status_topic_.c_str(),
        detection_topic_.c_str());
      return CallbackReturn::SUCCESS;
    } catch (const std::exception & exc) {
      set_lifecycle_state(
        motionbrain_msgs::msg::NodeLifecycleStatus::TRANSITION_STATE_ERRORPROCESSING,
        "errorprocessing",
        false,
        true,
        std::string("configure failed: ") + exc.what());
      RCLCPP_ERROR(get_logger(), "MotionBrain control guard configure failed: %s", exc.what());
      return CallbackReturn::FAILURE;
    }
  }

  CallbackReturn on_activate(const State & previous_state) override
  {
    try {
      if (!configured_) {
        read_configuration();
        create_configured_entities();
      }
      rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
      create_publish_timer();
      processing_active_ = true;
      set_lifecycle_state(
        motionbrain_msgs::msg::NodeLifecycleStatus::PRIMARY_STATE_ACTIVE,
        "active",
        true,
        false,
        "evaluating guard from " + status_topic_ + " and " + detection_topic_);
      RCLCPP_INFO(
        get_logger(),
        "Publishing C++ control guard on %s and compatibility JSON on %s from %s and %s",
        output_topic_.c_str(),
        json_output_topic_.c_str(),
        status_topic_.c_str(),
        detection_topic_.c_str());
      return CallbackReturn::SUCCESS;
    } catch (const std::exception & exc) {
      set_lifecycle_state(
        motionbrain_msgs::msg::NodeLifecycleStatus::TRANSITION_STATE_ERRORPROCESSING,
        "errorprocessing",
        false,
        true,
        std::string("activate failed: ") + exc.what());
      RCLCPP_ERROR(get_logger(), "MotionBrain control guard activate failed: %s", exc.what());
      return CallbackReturn::FAILURE;
    }
  }

  CallbackReturn on_deactivate(const State & previous_state) override
  {
    processing_active_ = false;
    destroy_publish_timer();
    rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
    set_lifecycle_state(
      motionbrain_msgs::msg::NodeLifecycleStatus::PRIMARY_STATE_INACTIVE,
      "inactive",
      false,
      false,
      "control guard publishing stopped for " + output_topic_);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const State & previous_state) override
  {
    processing_active_ = false;
    destroy_publish_timer();
    destroy_configured_entities();
    rclcpp_lifecycle::LifecycleNode::on_cleanup(previous_state);
    set_lifecycle_state(
      motionbrain_msgs::msg::NodeLifecycleStatus::PRIMARY_STATE_UNCONFIGURED,
      "unconfigured",
      false,
      false,
      "unconfigured control guard");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const State & previous_state) override
  {
    processing_active_ = false;
    destroy_publish_timer();
    rclcpp_lifecycle::LifecycleNode::on_shutdown(previous_state);
    set_lifecycle_state(
      motionbrain_msgs::msg::NodeLifecycleStatus::PRIMARY_STATE_FINALIZED,
      "finalized",
      false,
      false,
      "control guard shutdown requested");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const State & previous_state) override
  {
    (void) previous_state;
    processing_active_ = false;
    destroy_publish_timer();
    set_lifecycle_state(
      motionbrain_msgs::msg::NodeLifecycleStatus::TRANSITION_STATE_ERRORPROCESSING,
      "errorprocessing",
      false,
      true,
      "control guard lifecycle error");
    return CallbackReturn::SUCCESS;
  }

  void read_configuration()
  {
    status_topic_ = get_parameter("status_topic").as_string();
    detection_topic_ = get_parameter("detection_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    json_output_topic_ = get_parameter("json_output_topic").as_string();
    stale_timeout_sec_ = get_parameter("stale_timeout_sec").as_double();
    require_armed_ = get_parameter("require_armed").as_bool();
    require_detection_ = get_parameter("require_detection").as_bool();
    publish_rate_hz_ = std::max(get_parameter("publish_rate_hz").as_double(), 0.1);
  }

  void create_configured_entities()
  {
    if (configured_) {
      return;
    }

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

    guard_pub_ =
      create_publisher<motionbrain_msgs::msg::ControlGuard>(output_topic_, 10);
    guard_json_pub_ = create_publisher<std_msgs::msg::String>(json_output_topic_, 10);
    configured_ = true;
  }

  void destroy_configured_entities()
  {
    status_sub_.reset();
    detection_sub_.reset();
    guard_pub_.reset();
    guard_json_pub_.reset();
    latest_status_.reset();
    latest_detection_.reset();
    last_status_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_detection_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    configured_ = false;
  }

  void create_publish_timer()
  {
    destroy_publish_timer();
    const auto publish_period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    timer_ = create_wall_timer(
      publish_period,
      [this]() { publish_guard_state(); });
  }

  void destroy_publish_timer()
  {
    if (timer_) {
      timer_->cancel();
      timer_.reset();
    }
  }

  double age_seconds(const rclcpp::Time & stamp) const
  {
    if (stamp.nanoseconds() == 0) {
      return stale_timeout_sec_ + 1.0;
    }
    return std::max((now() - stamp).seconds(), 0.0);
  }

  void publish_guard_state()
  {
    if (!processing_active_ || !guard_pub_ || !guard_json_pub_) {
      return;
    }

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

    const bool armed = latest_status_ && latest_status_->armed;
    const bool moving = latest_status_ && latest_status_->moving;
    const bool faulted = latest_status_ && latest_status_->faulted;
    const bool camera_available = latest_detection_ && latest_detection_->available;
    const bool target_detected = latest_detection_ && latest_detection_->detected;

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
        << "\"armed\":" << json_bool(armed) << ","
        << "\"moving\":" << json_bool(moving) << ","
        << "\"faulted\":" << json_bool(faulted) << ","
        << "\"cameraAvailable\":" << json_bool(camera_available) << ","
        << "\"targetDetected\":" << json_bool(target_detected) << ","
        << "\"alignment\":\"" << escape_json(detection.alignment) << "\""
        << "}";
    const std::string raw_json = out.str();

    motionbrain_msgs::msg::ControlGuard typed_message;
    typed_message.stamp = now();
    typed_message.ready = decision.ready;
    typed_message.reason = decision.reason;
    typed_message.suggested_action = decision.suggested_action;
    typed_message.status_fresh = status_fresh;
    typed_message.detection_fresh = detection_fresh;
    typed_message.status_age_sec = static_cast<float>(status_age);
    typed_message.detection_age_sec = static_cast<float>(detection_age);
    typed_message.state = status.state;
    typed_message.armed = armed;
    typed_message.moving = moving;
    typed_message.faulted = faulted;
    typed_message.camera_available = camera_available;
    typed_message.target_detected = target_detected;
    typed_message.alignment = detection.alignment;
    typed_message.raw_json = raw_json;
    guard_pub_->publish(typed_message);

    std_msgs::msg::String message;
    message.data = raw_json;
    guard_json_pub_->publish(message);
  }

  void set_lifecycle_state(
    const uint8_t state_id,
    const std::string & state_label,
    const bool active,
    const bool error,
    const std::string & detail)
  {
    lifecycle_state_id_ = state_id;
    lifecycle_state_label_ = state_label;
    lifecycle_active_ = active;
    lifecycle_error_ = error;
    lifecycle_detail_ = detail;
    publish_lifecycle_status();
  }

  void publish_lifecycle_status()
  {
    if (!lifecycle_pub_ || !lifecycle_json_pub_) {
      return;
    }

    motionbrain_msgs::msg::NodeLifecycleStatus message;
    message.stamp = now();
    message.node_name = get_name();
    message.state_id = lifecycle_state_id_;
    message.state_label = lifecycle_state_label_;
    message.active = lifecycle_active_;
    message.error = lifecycle_error_;
    message.detail = lifecycle_detail_;
    std::ostringstream out;
    out << "{"
        << "\"active\":" << json_bool(message.active) << ","
        << "\"detail\":\"" << escape_json(lifecycle_detail_) << "\","
        << "\"error\":" << json_bool(message.error) << ","
        << "\"nodeName\":\"" << escape_json(message.node_name) << "\","
        << "\"stateId\":" << static_cast<int>(message.state_id) << ","
        << "\"stateLabel\":\"" << escape_json(message.state_label) << "\""
        << "}";
    message.raw_json = out.str();
    lifecycle_pub_->publish(message);

    std_msgs::msg::String json_message;
    json_message.data = message.raw_json;
    lifecycle_json_pub_->publish(json_message);
  }

  std::string status_topic_;
  std::string detection_topic_;
  std::string output_topic_;
  std::string json_output_topic_;
  double stale_timeout_sec_{3.0};
  double publish_rate_hz_{2.0};
  bool require_armed_{false};
  bool require_detection_{false};
  bool configured_{false};
  bool processing_active_{false};
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_detection_time_{0, 0, RCL_ROS_TIME};
  motionbrain_msgs::msg::MotionStatus::SharedPtr latest_status_;
  motionbrain_msgs::msg::CameraDetection::SharedPtr latest_detection_;
  rclcpp::Subscription<motionbrain_msgs::msg::MotionStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<motionbrain_msgs::msg::CameraDetection>::SharedPtr detection_sub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<motionbrain_msgs::msg::ControlGuard>>
  guard_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>>
  guard_json_pub_;
  rclcpp::Publisher<motionbrain_msgs::msg::NodeLifecycleStatus>::SharedPtr lifecycle_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lifecycle_json_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr lifecycle_timer_;
  uint8_t lifecycle_state_id_{
    motionbrain_msgs::msg::NodeLifecycleStatus::PRIMARY_STATE_UNCONFIGURED};
  std::string lifecycle_state_label_{"unconfigured"};
  bool lifecycle_active_{false};
  bool lifecycle_error_{false};
  std::string lifecycle_detail_{"unconfigured control guard"};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionBrainControlGuardNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
