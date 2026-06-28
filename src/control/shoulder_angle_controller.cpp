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
  , correcting_(false)
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
  , correctionStartedAtMs_(0)
  , lastProgressMs_(0)
  , lastSensorUpdateMs_(0)
  , processedSamples_(0)
  , correctionAttempts_(0)
  , lastStopReason_(ShoulderAngleStopReason::NONE)
  , manualGuardBlocked_(false) {}

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
    "Shoulder angle controller initialized (M4 absolute %.1f-%.1fdeg acceptance=%.2fdeg success=%.2fdeg)",
    SOFT_MIN_DEGREES, SOFT_MAX_DEGREES, TARGET_TOLERANCE_DEGREES,
    SETTLED_SUCCESS_TOLERANCE_DEGREES);
}

void ShoulderAngleController::update() {
  if (!active_) {
    enforceManualDriveLimits();
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
      finalErrorDegrees_ = targetDegrees_ - currentDegrees_;
      if (fabsf(finalErrorDegrees_) <= SETTLED_SUCCESS_TOLERANCE_DEGREES) {
        stopInternal(ShoulderAngleStopReason::TARGET_REACHED, "settled within tolerance");
      } else if (correctionAttempts_ >= MAX_CORRECTION_ATTEMPTS) {
        stopInternal(ShoulderAngleStopReason::TARGET_MISSED,
                     "settled outside tolerance after corrections");
      } else if (!beginCorrection(now)) {
        stopInternal(ShoulderAngleStopReason::START_FAILED,
                     "failed to start bounded correction");
      }
    }
    return;
  }

  const float error = targetDegrees_ - currentDegrees_;
  const float stopError = stopThresholdDegrees_ - currentDegrees_;
  const bool reachedCorrectionTarget = directionUp_
    ? error <= CORRECTION_CUTOFF_ERROR_DEGREES
    : error >= -CORRECTION_CUTOFF_ERROR_DEGREES;
  const bool correctionPulseExpired = correcting_ &&
    (now - correctionStartedAtMs_) >= correctionPulseMs();
  const bool reachedStopThreshold = correcting_
    ? (reachedCorrectionTarget || correctionPulseExpired)
    : (directionUp_
         ? stopError <= STOP_THRESHOLD_WINDOW_DEGREES
         : stopError >= -STOP_THRESHOLD_WINDOW_DEGREES);
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

  uint8_t desiredPercent = correcting_ ? correctionPercent() : requestedPercent_;
  if (!correcting_ && fabsf(error) <= SLOW_ZONE_DEGREES && desiredPercent > SLOW_PERCENT) {
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

  settling_ = false;
  correcting_ = false;
  correctionAttempts_ = 0;
  const float initialError = targetDegrees - currentDegrees_;
  if (fabsf(initialError) <= SETTLED_SUCCESS_TOLERANCE_DEGREES) {
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
                                                   STOP_THRESHOLD_WINDOW_DEGREES));
  stopThresholdDegrees_ = targetDegrees_ + (directionUp_ ? -usableLead : usableLead);
  startDegrees_ = currentDegrees_;
  finalErrorDegrees_ = initialError;
  progressReferenceDegrees_ = currentDegrees_;
  requestedPercent_ = percent;
  appliedPercent_ = 0;
  startedAtMs_ = millis();
  settleStartedAtMs_ = 0;
  correctionStartedAtMs_ = 0;
  lastProgressMs_ = startedAtMs_;
  lastSensorUpdateMs_ = sensor_->getI2cLastUpdateMs();
  processedSamples_ = 0;
  lastStopReason_ = ShoulderAngleStopReason::NONE;

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

bool ShoulderAngleController::manualDirectionAllowed(bool directionUp,
                                                      char* message,
                                                      size_t messageSize) const {
  if (message != nullptr && messageSize > 0) {
    message[0] = '\0';
  }
  if (!isReady() || !sensorAllowsMotion()) {
    if (message != nullptr && messageSize > 0) {
      snprintf(message, messageSize, "M4 AS5600 not ready");
    }
    return false;
  }

  const float currentDegrees = sensor_->getI2cDegrees();
  const float boundaryDegrees = directionUp
    ? SOFT_MAX_DEGREES - UP_STOP_LEAD_DEGREES
    : SOFT_MIN_DEGREES + DOWN_STOP_LEAD_DEGREES;
  const bool allowed = directionUp
    ? currentDegrees < boundaryDegrees
    : currentDegrees > boundaryDegrees;
  if (!allowed && message != nullptr && messageSize > 0) {
    snprintf(message, messageSize, "M4 %s blocked at %.2f deg (manual boundary %.2f deg)",
             directionUp ? "up" : "down", currentDegrees, boundaryDegrees);
  }
  return allowed;
}

void ShoulderAngleController::appendShoulderStatusJson(String& json) const {
  const bool available = isReady();
  const bool connected = available && sensor_->isI2cConnected();
  const bool fresh = available && sensor_->isI2cFresh(SENSOR_STALE_MS);
  const bool sensorReady = available && sensor_->isReadyForMotion(SENSOR_STALE_MS);
  const float angleDegrees = connected ? sensor_->getI2cDegrees() : 0.0f;
  const float errorDegrees = active_ ? targetDegrees_ - angleDegrees : finalErrorDegrees_;

  json += "\"shoulderAngle\":{";
  json += "\"available\":";
  json += available ? "true" : "false";
  json += ",\"sensorConnected\":";
  json += connected ? "true" : "false";
  json += ",\"sensorFresh\":";
  json += fresh ? "true" : "false";
  json += ",\"sensorReady\":";
  json += sensorReady ? "true" : "false";
  json += ",\"i2cAddress\":";
  json += String(ShoulderAngleSensor::I2C_ADDRESS);
  json += ",\"sdaPin\":";
  json += String(ShoulderAngleSensor::I2C_SDA_PIN);
  json += ",\"sclPin\":";
  json += String(ShoulderAngleSensor::I2C_SCL_PIN);
  json += ",\"adcPin\":";
  json += String(ShoulderAngleSensor::ADC_PIN);
  json += ",\"raw\":";
  json += String(connected ? sensor_->getI2cRawAngle() : 0);
  json += ",\"rawDeg\":";
  json += String(connected ? sensor_->getI2cRawDegrees() : 0.0f, 2);
  json += ",\"angleDeg\":";
  json += String(angleDegrees, 2);
  json += ",\"mountOffsetDeg\":";
  json += String(ShoulderAngleSensor::MOUNT_OFFSET_DEGREES, 2);
  json += ",\"magnetDetected\":";
  json += available && sensor_->isMagnetDetected() ? "true" : "false";
  json += ",\"magnetTooWeak\":";
  json += available && sensor_->isMagnetTooWeak() ? "true" : "false";
  json += ",\"magnetTooStrong\":";
  json += available && sensor_->isMagnetTooStrong() ? "true" : "false";
  json += ",\"agc\":";
  json += String(connected ? sensor_->getAgc() : 0);
  json += ",\"magnitude\":";
  json += String(connected ? sensor_->getMagnitude() : 0);
  json += ",\"ageMs\":";
  json += String(available ? sensor_->getI2cAgeMs() : 0);
  json += ",\"active\":";
  json += active_ ? "true" : "false";
  json += ",\"correctionActive\":";
  json += correcting_ && !settling_ ? "true" : "false";
  json += ",\"correctionAttempts\":";
  json += String(correctionAttempts_);
  json += ",\"maxCorrectionAttempts\":";
  json += String(MAX_CORRECTION_ATTEMPTS);
  json += ",\"targetDeg\":";
  json += String(targetDegrees_, 2);
  json += ",\"errorDeg\":";
  json += String(errorDegrees, 2);
  json += ",\"requestedPercent\":";
  json += String(requestedPercent_);
  json += ",\"appliedPercent\":";
  json += String(appliedPercent_);
  json += ",\"softMinDeg\":";
  json += String(SOFT_MIN_DEGREES, 2);
  json += ",\"softMaxDeg\":";
  json += String(SOFT_MAX_DEGREES, 2);
  json += ",\"targetToleranceDeg\":";
  json += String(TARGET_TOLERANCE_DEGREES, 2);
  json += ",\"settledSuccessToleranceDeg\":";
  json += String(SETTLED_SUCCESS_TOLERANCE_DEGREES, 2);
  json += ",\"stopThresholdWindowDeg\":";
  json += String(STOP_THRESHOLD_WINDOW_DEGREES, 2);
  json += ",\"upCorrectionPercent\":";
  json += String(UP_CORRECTION_PERCENT);
  json += ",\"downCorrectionPercent\":";
  json += String(DOWN_CORRECTION_PERCENT);
  json += ",\"upCorrectionPulseMs\":";
  json += String(UP_CORRECTION_PULSE_MS);
  json += ",\"downCorrectionPulseMs\":";
  json += String(DOWN_CORRECTION_PULSE_MS);
  json += ",\"manualDownBoundaryDeg\":";
  json += String(SOFT_MIN_DEGREES + DOWN_STOP_LEAD_DEGREES, 2);
  json += ",\"manualUpBoundaryDeg\":";
  json += String(SOFT_MAX_DEGREES - UP_STOP_LEAD_DEGREES, 2);
  json += ",\"manualGuardBlocked\":";
  json += manualGuardBlocked_ ? "true" : "false";
  json += ",\"lastStopReason\":\"";
  json += getLastStopReasonString();
  json += "\"}";
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
    case ShoulderAngleStopReason::TARGET_MISSED:  return "TARGET_MISSED";
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

uint8_t ShoulderAngleController::correctionPercent() const {
  return directionUp_ ? UP_CORRECTION_PERCENT : DOWN_CORRECTION_PERCENT;
}

uint32_t ShoulderAngleController::correctionPulseMs() const {
  return directionUp_ ? UP_CORRECTION_PULSE_MS : DOWN_CORRECTION_PULSE_MS;
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

void ShoulderAngleController::enforceManualDriveLimits() {
  if (!isReady()) {
    return;
  }

  const int16_t speed = motorControl_->getSpeed(MotorControl::MOTOR_4);
  if (speed == 0) {
    manualGuardBlocked_ = false;
    return;
  }

  const bool rawForward = speed > 0;
  const bool directionUp = rawForward == RobotArm::SHOULDER_UP_IS_FORWARD;
  char message[96] = {0};
  if (manualDirectionAllowed(directionUp, message, sizeof(message))) {
    manualGuardBlocked_ = false;
    return;
  }

  motorControl_->hardStop(MotorControl::MOTOR_4);
  if (!manualGuardBlocked_) {
    manualGuardBlocked_ = true;
    DebugLog::warn("[SHOULDER_GUARD] %s", message);
    eventLog.push("shoulder_angle", "SHOULDER_MANUAL_GUARD",
                  EventSeverity::WARN, message);
  }
}

void ShoulderAngleController::beginSettling() {
  if (motorControl_ != nullptr) {
    motorControl_->hardStop(MotorControl::MOTOR_4);
  }
  settling_ = true;
  settleStartedAtMs_ = millis();
  appliedPercent_ = 0;
  DebugLog::info(
    "[SHOULDER_ANGLE] settle phase=%s current=%.2f stop_at=%.2f target=%.2f settle=%lums",
    correcting_ ? "correction" : "coast", currentDegrees_,
    correcting_ ? targetDegrees_ : stopThresholdDegrees_, targetDegrees_,
    SETTLE_TIME_MS);
}

bool ShoulderAngleController::beginCorrection(uint32_t now) {
  const float error = targetDegrees_ - currentDegrees_;
  if (fabsf(error) <= SETTLED_SUCCESS_TOLERANCE_DEGREES) {
    return false;
  }

  correctionAttempts_++;
  correcting_ = true;
  settling_ = false;
  directionUp_ = error > 0.0f;
  progressReferenceDegrees_ = currentDegrees_;
  lastProgressMs_ = now;
  settleStartedAtMs_ = 0;
  correctionStartedAtMs_ = now;

  const uint8_t percent = correctionPercent();
  const uint32_t pulseMs = correctionPulseMs();
  if (!applyDrive(percent)) {
    return false;
  }

  DebugLog::info(
    "[SHOULDER_ANGLE] correction attempt=%u/%u current=%.2f target=%.2f error=%.2f dir=%s speed=%u%% pulse=%lums",
    correctionAttempts_, MAX_CORRECTION_ATTEMPTS, currentDegrees_, targetDegrees_,
    error, directionUp_ ? "up" : "down", percent,
    pulseMs);
  String detail = "attempt=" + String(correctionAttempts_) +
                  " current_deg=" + String(currentDegrees_, 2) +
                  " target_deg=" + String(targetDegrees_, 2) +
                  " error_deg=" + String(error, 2) +
                  " pulse_ms=" + String(pulseMs);
  eventLog.push("shoulder_angle", "SHOULDER_ANGLE_CORRECTION",
                EventSeverity::INFO, detail.c_str());
  return true;
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
  correcting_ = false;
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
