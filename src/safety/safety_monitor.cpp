#include "safety/safety_monitor.h"
#include "control/event_log.h"
#include "debug/debug_log.h"
#include "motor/motor_driver.h"
#include "motion/motion_sequence.h"
#include "system/system_init.h"

SafetyMonitor::SafetyMonitor()
  : systemState_(nullptr)
  , motorControl_(nullptr)
  , motionSequence_(nullptr)
  , blocked_(false)
  , blockReason_(SafetyBlockReason::NONE)
  , latchedFault_(false)
  , latchedFaultReason_(SafetyBlockReason::NONE)
  , vibrationActive_(false)
  , vibrationHighSamples_(0)
  , vibrationLowSamples_(0)
  , lastSafetyEventMs_(0) {}

extern EventLog eventLog;

void SafetyMonitor::init(SystemStateManager* systemState, MotorControl* motorControl, MotionSequence* motionSequence) {
  systemState_ = systemState;
  motorControl_ = motorControl;
  motionSequence_ = motionSequence;
  DebugLog::info("Safety monitor initialized (stale=%lums, obstacle=%.1fcm, vibe_enter=%.2f x%u, vibe_clear=%.2f x%u)",
                 SENSOR_STALE_MS, OBSTACLE_STOP_CM,
                 VIBRATION_FAULT_ENTER_THRESHOLD, VIBRATION_ENTER_SAMPLES,
                 VIBRATION_FAULT_CLEAR_THRESHOLD, VIBRATION_CLEAR_SAMPLES);
}

void SafetyMonitor::update(const SensorSnapshot& snapshot) {
  uint32_t now = millis();
  SafetyBlockReason nextReason = SafetyBlockReason::NONE;
  const bool faultLatchedNow = latchedFault_ &&
                               latchedFaultReason_ == SafetyBlockReason::VIBRATION &&
                               systemState_ != nullptr &&
                               systemState_->getState() == SystemState::FAULT;

  if (latchedFault_ && systemState_ != nullptr && systemState_->getState() != SystemState::FAULT) {
    DebugLog::safety("FAULT_CLEARED", reasonToString(latchedFaultReason_));
    eventLog.push("safety", "FAULT_CLEARED", EventSeverity::INFO, reasonToString(latchedFaultReason_));
    latchedFault_ = false;
    latchedFaultReason_ = SafetyBlockReason::NONE;
  }

  if (snapshot.vibe >= VIBRATION_FAULT_ENTER_THRESHOLD) {
    if (vibrationHighSamples_ < 255) {
      vibrationHighSamples_++;
    }
    vibrationLowSamples_ = 0;
  } else if (snapshot.vibe <= VIBRATION_FAULT_CLEAR_THRESHOLD) {
    if (vibrationLowSamples_ < 255) {
      vibrationLowSamples_++;
    }
    vibrationHighSamples_ = 0;
  } else {
    vibrationHighSamples_ = 0;
    vibrationLowSamples_ = 0;
  }

  if (!vibrationActive_ && vibrationHighSamples_ >= VIBRATION_ENTER_SAMPLES) {
    vibrationActive_ = true;
    vibrationLowSamples_ = 0;
  } else if (vibrationActive_ && vibrationLowSamples_ >= VIBRATION_CLEAR_SAMPLES) {
    vibrationActive_ = false;
    vibrationHighSamples_ = 0;
  }

  if (!snapshot.connected || snapshot.lastUpdateMs == 0 || (now - snapshot.lastUpdateMs) > SENSOR_STALE_MS) {
    nextReason = SafetyBlockReason::SENSOR_STALE;
  } else if (!snapshot.imuOk) {
    nextReason = SafetyBlockReason::IMU_FAULT;
  } else if (!snapshot.rangeOk) {
    nextReason = SafetyBlockReason::RANGE_FAULT;
  } else if (vibrationActive_) {
    nextReason = SafetyBlockReason::VIBRATION;
  } else if (snapshot.distanceCm > 0.0f && snapshot.distanceCm < OBSTACLE_STOP_CM) {
    nextReason = SafetyBlockReason::OBSTACLE;
  }

  if (nextReason == SafetyBlockReason::VIBRATION) {
    if (!faultLatchedNow && blockReason_ != nextReason) {
      String details = "vibe=" + String(snapshot.vibe, 2);
      triggerFault("VIBRATION_FAULT", details.c_str());
    }
    blocked_ = true;
    blockReason_ = nextReason;
    lastSafetyEventMs_ = now;
    return;
  }

  if (nextReason == SafetyBlockReason::NONE) {
    const bool suppressVibrationClearLog = blocked_ &&
                                           blockReason_ == SafetyBlockReason::VIBRATION &&
                                           latchedFault_ &&
                                           latchedFaultReason_ == SafetyBlockReason::VIBRATION;
    if (blocked_ && !suppressVibrationClearLog) {
      DebugLog::safety("BLOCK_CLEARED", reasonToString(blockReason_));
      eventLog.push("safety", "BLOCK_CLEARED", EventSeverity::INFO, reasonToString(blockReason_));
    }
    blocked_ = false;
    blockReason_ = SafetyBlockReason::NONE;
    return;
  }

  if (blockReason_ != nextReason) {
    String details;
    if (nextReason == SafetyBlockReason::OBSTACLE) {
      details = "to=OBSTACLE dist_cm=" + String(snapshot.distanceCm, 1);
    } else if (nextReason == SafetyBlockReason::SENSOR_STALE) {
      details = "to=SENSOR_STALE age_ms=" + String(snapshot.lastUpdateMs == 0 ? 0 : now - snapshot.lastUpdateMs);
    } else {
      details = "to=" + String(reasonToString(nextReason));
    }
    if (!blocked_) {
      triggerStop(reasonToString(nextReason), details.c_str());
    } else {
      DebugLog::safety("BLOCK_CHANGED", details.c_str());
      eventLog.push("safety", "BLOCK_CHANGED", EventSeverity::WARN, details.c_str());
    }
    lastSafetyEventMs_ = now;
  }

  blocked_ = true;
  blockReason_ = nextReason;
}

bool SafetyMonitor::isMotionBlocked() const {
  return blocked_;
}

SafetyBlockReason SafetyMonitor::getBlockReason() const {
  return blockReason_;
}

const char* SafetyMonitor::getBlockReasonString() const {
  return reasonToString(blockReason_);
}

bool SafetyMonitor::hasLatchedFault() const {
  return latchedFault_;
}

SafetyBlockReason SafetyMonitor::getLatchedFaultReason() const {
  return latchedFaultReason_;
}

const char* SafetyMonitor::getLatchedFaultReasonString() const {
  return reasonToString(latchedFaultReason_);
}

uint32_t SafetyMonitor::getLastSafetyEventMs() const {
  return lastSafetyEventMs_;
}

const char* SafetyMonitor::reasonToString(SafetyBlockReason reason) {
  switch (reason) {
    case SafetyBlockReason::NONE:         return "NONE";
    case SafetyBlockReason::SENSOR_STALE: return "SENSOR_STALE";
    case SafetyBlockReason::IMU_FAULT:    return "IMU_FAULT";
    case SafetyBlockReason::RANGE_FAULT:  return "RANGE_FAULT";
    case SafetyBlockReason::OBSTACLE:     return "OBSTACLE";
    case SafetyBlockReason::VIBRATION:    return "VIBRATION";
    default:                              return "UNKNOWN";
  }
}

bool SafetyMonitor::shouldForceImmediateStop() const {
  if (systemState_ != nullptr && systemState_->getState() == SystemState::ARMED) {
    return true;
  }

  if (motorControl_ != nullptr && motorControl_->isEnabled()) {
    return true;
  }

  return false;
}

void SafetyMonitor::triggerStop(const char* eventName, const char* details) {
  if (motionSequence_ != nullptr) {
    motionSequence_->stop();
  }
  if (shouldForceImmediateStop()) {
    if (systemState_ != nullptr &&
        systemState_->getState() == SystemState::ARMED &&
        !systemState_->enterSafe()) {
      DebugLog::warn("SafetyMonitor: enterSafe() failed during %s", eventName);
    }
    if (motorControl_ != nullptr) {
      motorControl_->emergencyStop();
    }
  }
  DebugLog::safety(eventName, details);
  eventLog.push("safety", eventName, EventSeverity::WARN, details);
}

void SafetyMonitor::triggerFault(const char* eventName, const char* details) {
  if (motionSequence_ != nullptr) {
    motionSequence_->stop();
  }
  if (motorControl_ != nullptr) {
    motorControl_->emergencyStop();
  }
  if (systemState_ != nullptr) {
    systemState_->transitionTo(SystemState::FAULT);
  }
  latchedFault_ = true;
  latchedFaultReason_ = SafetyBlockReason::VIBRATION;
  DebugLog::safety(eventName, details);
  eventLog.push("safety", eventName, EventSeverity::ERROR, details);
}
