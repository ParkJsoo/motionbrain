#include "control/angle_controller.h"

#include <math.h>
#include <stdio.h>
#include "control/event_log.h"
#include "debug/debug_log.h"
#include "motion/robot_arm.h"
#include "safety/safety_monitor.h"
#include "system/system_init.h"

namespace {

const char* directionToString(MotionDirection direction) {
  switch (direction) {
    case MotionDirection::LEFT:  return "left";
    case MotionDirection::RIGHT: return "right";
    default:                     return "unknown";
  }
}

} // namespace

extern EventLog eventLog;

AngleController::AngleController()
  : systemState_(nullptr)
  , robotArm_(nullptr)
  , safetyMonitor_(nullptr)
  , active_(false)
  , direction_(MotionDirection::LEFT)
  , targetDegrees_(0.0f)
  , accumulatedSignedDegrees_(0.0f)
  , percent_(DEFAULT_SPEED)
  , startedAtMs_(0)
  , timeoutMs_(0)
  , lastTransitionMs_(0)
  , lastSampleStampMs_(0)
  , processedSamples_(0)
  , hasSample_(false)
  , lastRateDegreesPerSecond_(0.0f)
  , lastStopReason_(AngleControllerStopReason::NONE) {
}

void AngleController::init(SystemStateManager* systemState, RobotArm* robotArm, SafetyMonitor* safetyMonitor) {
  systemState_ = systemState;
  robotArm_ = robotArm;
  safetyMonitor_ = safetyMonitor;
  DebugLog::info("Angle controller initialized (base only, tolerance=%.1fdeg, max_target=%.1fdeg)",
                 TARGET_TOLERANCE_DEGREES, MAX_TARGET_DEGREES);
}

void AngleController::update(const SensorSnapshot& snapshot) {
  if (!active_) {
    return;
  }

  if (systemState_ == nullptr || robotArm_ == nullptr || safetyMonitor_ == nullptr) {
    stopInternal(AngleControllerStopReason::START_FAILED, "dependencies missing");
    return;
  }

  if (safetyMonitor_->isMotionBlocked()) {
    stopInternal(AngleControllerStopReason::SENSOR_BLOCK, safetyMonitor_->getBlockReasonString());
    return;
  }

  if (systemState_->getState() != SystemState::ARMED) {
    stopInternal(AngleControllerStopReason::STATE_CHANGED, systemState_->getStateString());
    return;
  }

  uint32_t now = millis();
  if ((now - startedAtMs_) >= FEEDBACK_CHECK_DELAY_MS && !hasRotationFeedback()) {
    stopInternal(AngleControllerStopReason::NO_ROTATION_FEEDBACK, "imu not moving with base?");
    return;
  }

  if ((now - startedAtMs_) >= timeoutMs_) {
    stopInternal(AngleControllerStopReason::TIMEOUT, nullptr);
    return;
  }

  uint32_t sampleStampMs = 0;
  if (!extractSampleStampMs(snapshot, sampleStampMs)) {
    return;
  }

  if (!hasSample_) {
    hasSample_ = true;
    lastSampleStampMs_ = sampleStampMs;
    processedSamples_ = 0;
    lastRateDegreesPerSecond_ = getSignedRateDegreesPerSecond(snapshot);
    systemState_->resetTimeout();
    return;
  }

  if (sampleStampMs == lastSampleStampMs_) {
    return;
  }

  float dtSeconds = static_cast<float>(sampleStampMs - lastSampleStampMs_) / 1000.0f;
  lastSampleStampMs_ = sampleStampMs;

  if (dtSeconds <= 0.0f) {
    return;
  }

  lastRateDegreesPerSecond_ = getSignedRateDegreesPerSecond(snapshot);
  accumulatedSignedDegrees_ += lastRateDegreesPerSecond_ * dtSeconds;
  processedSamples_++;
  systemState_->resetTimeout();

  if (hasReachedTarget()) {
    stopInternal(AngleControllerStopReason::TARGET_REACHED, nullptr);
  }
}

bool AngleController::isReady() const {
  return systemState_ != nullptr && robotArm_ != nullptr && safetyMonitor_ != nullptr;
}

bool AngleController::isActive() const {
  return active_;
}

bool AngleController::startRelative(MotionDirection direction, float targetDegrees, uint8_t percent,
                                    char* message, size_t messageSize) {
  if (message != nullptr && messageSize > 0) {
    message[0] = '\0';
  }

  if (!isReady()) {
    if (message != nullptr && messageSize > 0) {
      snprintf(message, messageSize, "AngleController not initialized");
    }
    return false;
  }

  if (active_) {
    if (message != nullptr && messageSize > 0) {
      snprintf(message, messageSize, "Base angle control already active");
    }
    return false;
  }

  if (direction != MotionDirection::LEFT && direction != MotionDirection::RIGHT) {
    if (message != nullptr && messageSize > 0) {
      snprintf(message, messageSize, "Base angle direction must be left or right");
    }
    return false;
  }

  if (targetDegrees < MIN_TARGET_DEGREES || targetDegrees > MAX_TARGET_DEGREES) {
    if (message != nullptr && messageSize > 0) {
      snprintf(message, messageSize, "Target angle must be %.0f-%.0f deg",
               MIN_TARGET_DEGREES, MAX_TARGET_DEGREES);
    }
    return false;
  }

  if (percent == 0 || percent > 100) {
    if (message != nullptr && messageSize > 0) {
      snprintf(message, messageSize, "Base angle speed must be 1-100%%");
    }
    return false;
  }

  bool started = (direction == MotionDirection::LEFT)
    ? robotArm_->baseLeft(percent)
    : robotArm_->baseRight(percent);
  if (!started) {
    if (message != nullptr && messageSize > 0) {
      snprintf(message, messageSize, "Failed to start base motor");
    }
    return false;
  }

  active_ = true;
  direction_ = direction;
  targetDegrees_ = targetDegrees;
  accumulatedSignedDegrees_ = 0.0f;
  percent_ = percent;
  startedAtMs_ = millis();
  timeoutMs_ = computeTimeoutMs(targetDegrees);
  lastTransitionMs_ = startedAtMs_;
  lastSampleStampMs_ = 0;
  processedSamples_ = 0;
  hasSample_ = false;
  lastRateDegreesPerSecond_ = 0.0f;
  lastStopReason_ = AngleControllerStopReason::NONE;

  DebugLog::info("[ANGLE] Base relative start: dir=%s target=%.1fdeg speed=%u%% timeout=%lums",
                 directionToString(direction_), targetDegrees_, percent_, timeoutMs_);
  String startDetail = "dir=" + String(directionToString(direction_)) +
                       " target_deg=" + String(targetDegrees_, 1) +
                       " speed_pct=" + String(percent_);
  eventLog.push("base_angle", "BASE_ANGLE_START", EventSeverity::INFO, startDetail.c_str());

  if (message != nullptr && messageSize > 0) {
    snprintf(message, messageSize, "Base angle %s %.1f deg at %u%%",
             directionToString(direction_), targetDegrees_, percent_);
  }
  return true;
}

bool AngleController::cancel(AngleControllerStopReason reason, const char* detail) {
  if (!active_) {
    return false;
  }

  stopInternal(reason, detail);
  return true;
}

MotionDirection AngleController::getDirection() const {
  return direction_;
}

const char* AngleController::getDirectionString() const {
  return directionToString(direction_);
}

float AngleController::getTargetDegrees() const {
  return targetDegrees_;
}

float AngleController::getAccumulatedDegrees() const {
  float signedCurrent = getSignedCurrentDegrees();
  return signedCurrent < 0.0f ? -signedCurrent : signedCurrent;
}

float AngleController::getRemainingDegrees() const {
  float remaining = targetDegrees_ - getAccumulatedDegrees();
  return remaining > 0.0f ? remaining : 0.0f;
}

uint8_t AngleController::getPercent() const {
  return percent_;
}

uint32_t AngleController::getElapsedMs() const {
  if (!active_) {
    return 0;
  }
  return millis() - startedAtMs_;
}

uint32_t AngleController::getTimeoutMs() const {
  return timeoutMs_;
}

uint32_t AngleController::getLastTransitionMs() const {
  return lastTransitionMs_;
}

uint32_t AngleController::getProcessedSamples() const {
  return processedSamples_;
}

float AngleController::getLastRateDegreesPerSecond() const {
  return lastRateDegreesPerSecond_;
}

AngleControllerStopReason AngleController::getLastStopReason() const {
  return lastStopReason_;
}

const char* AngleController::getLastStopReasonString() const {
  return stopReasonToString(lastStopReason_);
}

const char* AngleController::stopReasonToString(AngleControllerStopReason reason) {
  switch (reason) {
    case AngleControllerStopReason::NONE:           return "NONE";
    case AngleControllerStopReason::TARGET_REACHED: return "TARGET_REACHED";
    case AngleControllerStopReason::TIMEOUT:        return "TIMEOUT";
    case AngleControllerStopReason::NO_ROTATION_FEEDBACK: return "NO_ROTATION_FEEDBACK";
    case AngleControllerStopReason::SENSOR_BLOCK:   return "SENSOR_BLOCK";
    case AngleControllerStopReason::STATE_CHANGED:  return "STATE_CHANGED";
    case AngleControllerStopReason::MANUAL_STOP:    return "MANUAL_STOP";
    case AngleControllerStopReason::OVERRIDDEN:     return "OVERRIDDEN";
    case AngleControllerStopReason::START_FAILED:   return "START_FAILED";
    default:                                        return "UNKNOWN";
  }
}

bool AngleController::extractSampleStampMs(const SensorSnapshot& snapshot, uint32_t& stampMs) const {
  if (!snapshot.connected || snapshot.lastUpdateMs == 0) {
    return false;
  }

  stampMs = snapshot.sourceTimestampMs != 0 ? snapshot.sourceTimestampMs : snapshot.lastUpdateMs;
  return true;
}

float AngleController::getSignedTargetDegrees() const {
  return direction_ == MotionDirection::LEFT ? targetDegrees_ : -targetDegrees_;
}

float AngleController::getSignedCurrentDegrees() const {
  return accumulatedSignedDegrees_;
}

float AngleController::getSignedRateDegreesPerSecond(const SensorSnapshot& snapshot) const {
  float leftPositiveRate = GYRO_Z_LEFT_IS_POSITIVE ? snapshot.gyroZ : -snapshot.gyroZ;
  return leftPositiveRate;
}

uint32_t AngleController::computeTimeoutMs(float targetDegrees) const {
  return BASE_TIMEOUT_MS + static_cast<uint32_t>(targetDegrees * TIMEOUT_PER_DEGREE_MS);
}

bool AngleController::hasReachedTarget() const {
  float signedTarget = getSignedTargetDegrees();
  float signedCurrent = getSignedCurrentDegrees();
  if (signedTarget >= 0.0f) {
    return signedCurrent >= (signedTarget - TARGET_TOLERANCE_DEGREES);
  }
  return signedCurrent <= (signedTarget + TARGET_TOLERANCE_DEGREES);
}

bool AngleController::hasRotationFeedback() const {
  float absoluteDegrees = getAccumulatedDegrees();
  float absoluteRate = lastRateDegreesPerSecond_ < 0.0f
    ? -lastRateDegreesPerSecond_
    : lastRateDegreesPerSecond_;
  return absoluteDegrees >= MIN_PROGRESS_FOR_FEEDBACK_DEGREES ||
         absoluteRate >= MIN_ROTATION_RATE_DPS;
}

void AngleController::stopInternal(AngleControllerStopReason reason, const char* detail) {
  if (robotArm_ != nullptr) {
    robotArm_->baseStop();
  }

  float currentDegrees = getAccumulatedDegrees();
  uint32_t elapsedMs = active_ ? millis() - startedAtMs_ : 0;

  active_ = false;
  lastStopReason_ = reason;
  lastTransitionMs_ = millis();
  hasSample_ = false;

  char suffix[48] = {0};
  if (detail != nullptr && detail[0] != '\0') {
    snprintf(suffix, sizeof(suffix), " detail=%s", detail);
  }

  DebugLog::info("[ANGLE] Base relative stop: reason=%s current=%.1fdeg target=%.1fdeg elapsed=%lums samples=%lu last_rate=%.2fdps%s",
                 stopReasonToString(reason), currentDegrees, targetDegrees_, elapsedMs,
                 processedSamples_, lastRateDegreesPerSecond_, suffix);
  String stopDetail = "reason=" + String(stopReasonToString(reason)) +
                      " current_deg=" + String(currentDegrees, 1) +
                      " target_deg=" + String(targetDegrees_, 1);
  if (detail != nullptr && detail[0] != '\0') {
    stopDetail += " ";
    stopDetail += detail;
  }
  eventLog.push("base_angle",
                reason == AngleControllerStopReason::TARGET_REACHED ? "BASE_ANGLE_TARGET_REACHED" : "BASE_ANGLE_STOP",
                reason == AngleControllerStopReason::TARGET_REACHED ? EventSeverity::INFO : EventSeverity::WARN,
                stopDetail.c_str());
}
