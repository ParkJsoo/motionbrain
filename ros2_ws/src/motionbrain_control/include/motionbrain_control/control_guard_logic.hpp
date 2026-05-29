#pragma once

#include <string>

namespace motionbrain_control
{

struct ControlGuardConfig
{
  bool require_armed{false};
  bool require_detection{false};
};

struct MotionStatusSnapshot
{
  bool available{false};
  bool armed{false};
  bool moving{false};
  bool faulted{false};
  std::string state{"UNKNOWN"};
};

struct CameraDetectionSnapshot
{
  bool available{false};
  bool detected{false};
  std::string alignment{"LOST"};
  std::string command_suggestion;
};

struct ControlGuardDecision
{
  bool ready{false};
  std::string reason{"status_stale"};
  std::string suggested_action{"none"};
};

inline std::string detection_suggestion(const CameraDetectionSnapshot & detection)
{
  if (!detection.available || !detection.detected) {
    return "none";
  }
  if (!detection.command_suggestion.empty()) {
    return detection.command_suggestion;
  }
  if (detection.alignment == "LEFT") {
    return "base_left";
  }
  if (detection.alignment == "RIGHT") {
    return "base_right";
  }
  if (detection.alignment == "CENTER") {
    return "hold";
  }
  return "none";
}

inline ControlGuardDecision evaluate_control_guard(
  const MotionStatusSnapshot & status,
  const CameraDetectionSnapshot & detection,
  const bool status_fresh,
  const bool detection_fresh,
  const ControlGuardConfig & config)
{
  ControlGuardDecision decision;
  decision.ready = true;
  decision.reason = "ready";
  decision.suggested_action = detection_suggestion(detection);

  if (!status_fresh) {
    decision.ready = false;
    decision.reason = "status_stale";
  } else if (!status.available) {
    decision.ready = false;
    decision.reason = "status_unavailable";
  } else if (status.faulted) {
    decision.ready = false;
    decision.reason = "faulted";
  } else if (status.moving) {
    decision.ready = false;
    decision.reason = "already_moving";
  } else if (config.require_armed && !status.armed) {
    decision.ready = false;
    decision.reason = "not_armed";
  } else if (config.require_detection && !detection_fresh) {
    decision.ready = false;
    decision.reason = "detection_stale";
  } else if (config.require_detection && (!detection.available || !detection.detected)) {
    decision.ready = false;
    decision.reason = "target_not_detected";
  }

  return decision;
}

}  // namespace motionbrain_control

