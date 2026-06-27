#include "control/shoulder_angle_controller.h"

#include <math.h>
#include <stdio.h>
#include "control/event_log.h"
#include "debug/debug_log.h"
#include "motion/robot_arm.h"
#include "motor/motor_driver.h"
#include "peripheral/shoulder_angle_sensor.h"
#include "safety/safety_monitor.h"
#include "system/system_init.h"

extern EventLog eventLog;

ShoulderAngleController::ShoulderAngleController()
  : systemState_(nullptr)
  , robotArm_(nullptr)
  , motorControl_(nullptr)
  , safetyMonitor_(nullptr)
  , sensor_(nullptr)
  , active_(false)
  , settling_(false)
  , directionUp_(true)
  , targetDegrees_(0.0f)
  , stopThresholdDegrees_(0.0f)
  , startDegrees_(0.0f)
  , currentDegrees_(0.0f)
  , finalErrorDegrees_(0.0f)
  , progressReferenceDegrees_(0.0f)
  , requestedPercent_(DEFAULT_PERCENT)
  , appliedPercent_(0)
  , startedAtMs_(0)
  , settleStartedAtMs_(0)
  , lastProgressMs_(0)
  , lastSensorUpdateMs_(0)
  , processedSamples_(0)
  , lastStopReason_(ShoulderAngleStopReason::NONE) {}

void ShoulderAngleController::init(SystemStateManager* systemState,
                                   RobotArm* robotArm,
                                   MotorControl* motorControl,
                                   SafetyMonitor* safetyMonitor,
                                   ShoulderAngleSensor* sensor) {
  systemState_ = systemState;
  robotArm_ = robotArm;
  motorControl_ = motorControl;
  safetyMonitor_ = safetyMonitor;
  sensor_ = sensor;
  DebugLog::info(
    "Shoulder angle controller initialized (M4 absolute %.1f-%.1fdeg tolerance=%.2fdeg)",
    SOFT_MIN_DEGREES, SOFT_MAX_DEGREES, TARGET_TOLERANCE_DEGREES);
}

void ShoulderAngleController::update() {
  if (!active_) {
    return;
  }

  if (!isReady()) {
    stopInternal(ShoulderAngleStopReason::START_FAILED, "dependencies missing");
    return;
  }
  if (systemState_->getState() != SystemState::ARMED) {
    stopInternal(ShoulderAngleStopReason::STATE_CHANGED, systemState_->getStateString());
    return;
  }
  if (safetyMonitor_->isMotionBlocked()) {
    stopInternal(ShoulderAngleStopReason::SAFETY_BLOCK,
                 safetyMonitor_->getBlockReasonString());
    return;
  }
  if (!sensorAllowsMotion()) {
    stopInternal(ShoulderAngleStopReason::SENSOR_FAULT,
                 "AS5600 disconnected/stale/magnet invalid");
    return;
  }

  const uint32_t now = millis();
  if ((now - startedAtMs_) >= COMMAND_TIMEOUT_MS) {
    stopInternal(ShoulderAngleStopReason::TIMEOUT, nullptr);
    return;
  }

  const uint32_t sensorUpdateMs = sensor_->getI2cLastUpdateMs();
  if (sensorUpdateMs == lastSensorUpdateMs_) {
    return;
  }
  lastSensorUpdateMs_ = sensorUpdateMs;
  currentDegrees_ = sensor_->getI2cDegrees();
  processedSamples_++;
  systemState_->resetTimeout();

  if (currentDegrees_ < SOFT_MIN_DEGREES || currentDegrees_ > SOFT_MAX_DEGREES) {
    stopInternal(ShoulderAngleStopReason::SOFT_LIMIT, "measured angle outside calibrated range");
    return;
  }

  if (settling_) {
    if ((now - settleStartedAtMs_) >= SETTLE_TIME_MS) {
      stopInternal(ShoulderAngleStopReason::TARGET_REACHED, "settled feedback");
    }
    return;
  }

  const float error = targetDegrees_ - currentDegrees_;
  const float stopError = stopThresholdDegrees_ - currentDegrees_;
  const bool reachedStopThreshold = directionUp_
    ? stopError <= TARGET_TOLERANCE_DEGREES
    : stopError >= -TARGET_TOLERANCE_DEGREES;
  if (reachedStopThreshold) {
    beginSettling();
    return;
  }

  if (fabsf(currentDegrees_ - progressReferenceDegrees_) >= MIN_PROGRESS_DEGREES) {
    progressReferenceDegrees_ = currentDegrees_;
    lastProgressMs_ = now;
  } else if ((now - startedAtMs_) >= PROGRESS_GRACE_MS &&
             (now - lastProgressMs_) >= PROGRESS_TIMEOUT_MS) {
    stopInternal(ShoulderAngleStopReason::NO_PROGRESS, nullptr);
    return;
  }

  uint8_t desiredPercent = requestedPercent_;
  if (fabsf(error) <= SLOW_ZONE_DEGREES && desiredPercent > SLOW_PERCENT) {
    desiredPercent = SLOW_PERCENT;
  }
  if (desiredPercent != appliedPercent_ && !applyDrive(desiredPercent)) {
    stopInternal(ShoulderAngleStopReason::START_FAILED, "failed to update M4 output");
  }
}

bool ShoulderAngleController::isReady() const {
  return systemState_ != nullptr && robotArm_ != nullptr && motorControl_ != nullptr &&
         safetyMonitor_ != nullptr && sensor_ != nullptr;
}

bool ShoulderAngleController::isActive() const {
  return active_;
}

bool ShoulderAngleController::startAbsolute(float targetDegrees, uint8_t percent,
                                            char* message, size_t messageSize) {
  if (message != nullptr && messageSize > 0) {
    message[0] = '\0';
  }
  if (!isReady()) {
    if (message != nullptr && messageSize > 0) {
      snprintf(message, messageSize, "ShoulderAngleController not initialized");
    }
    return false;
  }
  if (active_) {
    if (message != nullptr && messageSize > 0) {
      snprintf(message, messageSize, "Shoulder angle control already active");
    }
    return false;
  }
  if (systemState_->getState() != SystemState::ARMED) {
    if (message != nullptr && messageSize > 0) {
      snprintf(message, messageSize, "System must be ARMED");
    }
    return false;
  }
  if (targetDegrees < SOFT_MIN_DEGREES || targetDegrees > SOFT_MAX_DEGREES) {
    if (message != nullptr && messageSize > 0) {
      snprintf(message, messageSize, "Shoulder target must be %.1f-%.1f deg",
               SOFT_MIN_DEGREES, SOFT_MAX_DEGREES);
    }
    return false;
  }
  if (percent < MIN_DRIVE_PERCENT || percent > 100) {
    if (message != nullptr && messageSize > 0) {
      snprintf(message, messageSize, "Shoulder speed must be %u-100%%", MIN_DRIVE_PERCENT);
    }
    return false;
  }
  if (!sensorAllowsMotion()) {
    if (message != nullptr && messageSize > 0) {
      snprintf(message, messageSize, "AS5600 not ready (connection/magnet/strength)");
    }
    return false;
  }

  currentDegrees_ = sensor_->getI2cDegrees();
  if (currentDegrees_ < SOFT_MIN_DEGREES || currentDegrees_ > SOFT_MAX_DEGREES) {
    if (message != nullptr && messageSize > 0) {
      snprintf(message, messageSize, "Current shoulder angle %.2f outside soft limits",
               currentDegrees_);
    }
    return false;
  }

  const float initialError = targetDegrees - currentDegrees_;
  if (fabsf(initialError) <= TARGET_TOLERANCE_DEGREES) {
    targetDegrees_ = targetDegrees;
    startDegrees_ = currentDegrees_;
    finalErrorDegrees_ = initialError;
    lastStopReason_ = ShoulderAngleStopReason::TARGET_REACHED;
    if (message != nullptr && messageSize > 0) {
      snprintf(message, messageSize, "Shoulder already at %.2f deg", currentDegrees_);
    }
    return true;
  }

  directionUp_ = initialError > 0.0f;
  targetDegrees_ = targetDegrees;
  const float requestedLead = directionUp_
    ? UP_STOP_LEAD_DEGREES
    : DOWN_STOP_LEAD_DEGREES;
  const float usableLead = fminf(requestedLead,
                                 fmaxf(0.0f, fabsf(initialError) -
                                                   TARGET_TOLERANCE_DEGREES));
  stopThresholdDegrees_ = targetDegrees_ + (directionUp_ ? -usableLead : usableLead);
  startDegrees_ = currentDegrees_;
  finalErrorDegrees_ = initialError;
  progressReferenceDegrees_ = currentDegrees_;
  requestedPercent_ = percent;
  appliedPercent_ = 0;
  startedAtMs_ = millis();
  settleStartedAtMs_ = 0;
  lastProgressMs_ = startedAtMs_;
  lastSensorUpdateMs_ = sensor_->getI2cLastUpdateMs();
  processedSamples_ = 0;
  lastStopReason_ = ShoulderAngleStopReason::NONE;
  settling_ = false;

  if (!applyDrive(requestedPercent_)) {
    lastStopReason_ = ShoulderAngleStopReason::START_FAILED;
    if (message != nullptr && messageSize > 0) {
      snprintf(message, messageSize, "Failed to start M4 shoulder motor");
    }
    return false;
  }

  active_ = true;
  DebugLog::info(
    "[SHOULDER_ANGLE] start current=%.2f target=%.2f stop_at=%.2f dir=%s speed=%u%%",
    startDegrees_, targetDegrees_, stopThresholdDegrees_,
    directionUp_ ? "up" : "down", requestedPercent_);
  String detail = "start_deg=" + String(startDegrees_, 2) +
                  " target_deg=" + String(targetDegrees_, 2) +
                  " speed_pct=" + String(requestedPercent_);
  eventLog.push("shoulder_angle", "SHOULDER_ANGLE_START", EventSeverity::INFO,
                detail.c_str());

  if (message != nullptr && messageSize > 0) {
    snprintf(message, messageSize, "Shoulder %.2f -> %.2f deg at %u%%",
             startDegrees_, targetDegrees_, requestedPercent_);
  }
  return true;
}

bool ShoulderAngleController::cancel(ShoulderAngleStopReason reason,
                                     const char* detail) {
  if (!active_) {
    return false;
  }
  stopInternal(reason, detail);
  return true;
}

float ShoulderAngleController::getTargetDegrees() const { return targetDegrees_; }
float ShoulderAngleController::getCurrentDegrees() const { return currentDegrees_; }
float ShoulderAngleController::getErrorDegrees() const { return targetDegrees_ - currentDegrees_; }
float ShoulderAngleController::getStartDegrees() const { return startDegrees_; }
float ShoulderAngleController::getFinalErrorDegrees() const { return finalErrorDegrees_; }
uint8_t ShoulderAngleController::getRequestedPercent() const { return requestedPercent_; }
uint8_t ShoulderAngleController::getAppliedPercent() const { return appliedPercent_; }
uint32_t ShoulderAngleController::getElapsedMs() const {
  return active_ ? millis() - startedAtMs_ : 0;
}
uint32_t ShoulderAngleController::getProcessedSamples() const { return processedSamples_; }
ShoulderAngleStopReason ShoulderAngleController::getLastStopReason() const {
  return lastStopReason_;
}
const char* ShoulderAngleController::getLastStopReasonString() const {
  return stopReasonToString(lastStopReason_);
}

const char* ShoulderAngleController::stopReasonToString(ShoulderAngleStopReason reason) {
  switch (reason) {
    case ShoulderAngleStopReason::NONE:           return "NONE";
    case ShoulderAngleStopReason::TARGET_REACHED: return "TARGET_REACHED";
    case ShoulderAngleStopReason::TIMEOUT:        return "TIMEOUT";
    case ShoulderAngleStopReason::SENSOR_FAULT:   return "SENSOR_FAULT";
    case ShoulderAngleStopReason::SAFETY_BLOCK:   return "SAFETY_BLOCK";
    case ShoulderAngleStopReason::STATE_CHANGED:  return "STATE_CHANGED";
    case ShoulderAngleStopReason::NO_PROGRESS:    return "NO_PROGRESS";
    case ShoulderAngleStopReason::MANUAL_STOP:    return "MANUAL_STOP";
    case ShoulderAngleStopReason::OVERRIDDEN:     return "OVERRIDDEN";
    case ShoulderAngleStopReason::START_FAILED:   return "START_FAILED";
    case ShoulderAngleStopReason::SOFT_LIMIT:     return "SOFT_LIMIT";
    default:                                      return "UNKNOWN";
  }
}

bool ShoulderAngleController::sensorAllowsMotion() const {
  return sensor_ != nullptr && sensor_->isReadyForMotion(SENSOR_STALE_MS);
}

bool ShoulderAngleController::applyDrive(uint8_t percent) {
  const bool success = directionUp_
    ? robotArm_->shoulderUp(percent)
    : robotArm_->shoulderDown(percent);
  if (success) {
    appliedPercent_ = percent;
  }
  return success;
}

void ShoulderAngleController::beginSettling() {
  if (motorControl_ != nullptr) {
    motorControl_->hardStop(MotorControl::MOTOR_4);
  }
  settling_ = true;
  settleStartedAtMs_ = millis();
  appliedPercent_ = 0;
  DebugLog::info(
    "[SHOULDER_ANGLE] coast compensation stop current=%.2f stop_at=%.2f target=%.2f settle=%lums",
    currentDegrees_, stopThresholdDegrees_, targetDegrees_, SETTLE_TIME_MS);
}

void ShoulderAngleController::stopInternal(ShoulderAngleStopReason reason,
                                           const char* detail) {
  if (motorControl_ != nullptr) {
    motorControl_->hardStop(MotorControl::MOTOR_4);
  }

  if (sensor_ != nullptr && sensor_->isI2cFresh(SENSOR_STALE_MS)) {
    currentDegrees_ = sensor_->getI2cDegrees();
  }
  finalErrorDegrees_ = targetDegrees_ - currentDegrees_;
  const uint32_t elapsedMs = active_ ? millis() - startedAtMs_ : 0;

  active_ = false;
  settling_ = false;
  appliedPercent_ = 0;
  lastStopReason_ = reason;

  DebugLog::info(
    "[SHOULDER_ANGLE] stop reason=%s current=%.2f target=%.2f error=%.2f elapsed=%lums samples=%lu%s%s",
    stopReasonToString(reason), currentDegrees_, targetDegrees_, finalErrorDegrees_,
    elapsedMs, processedSamples_, detail != nullptr ? " detail=" : "",
    detail != nullptr ? detail : "");
  String eventDetail = "reason=" + String(stopReasonToString(reason)) +
                       " current_deg=" + String(currentDegrees_, 2) +
                       " target_deg=" + String(targetDegrees_, 2) +
                       " error_deg=" + String(finalErrorDegrees_, 2);
  eventLog.push("shoulder_angle",
                reason == ShoulderAngleStopReason::TARGET_REACHED
                  ? "SHOULDER_ANGLE_TARGET_REACHED"
                  : "SHOULDER_ANGLE_STOP",
                reason == ShoulderAngleStopReason::TARGET_REACHED
                  ? EventSeverity::INFO
                  : EventSeverity::WARN,
                eventDetail.c_str());
}
