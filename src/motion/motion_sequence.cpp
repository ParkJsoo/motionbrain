#include "motion_sequence.h"
#include "control/angle_controller.h"
#include "control/shoulder_angle_controller.h"
#include "safety/safety_monitor.h"
#include "motion/robot_arm.h"
#include "system/system_init.h"
#include "debug/debug_log.h"

extern SafetyMonitor safetyMonitor;

/**
 * 생성자
 */
MotionSequence::MotionSequence()
  : count_(0)
  , currentIndex_(0)
  , state_(SequenceState::IDLE)
  , stepStartMs_(0)
  , robotArm_(nullptr)
  , systemState_(nullptr)
  , angleController_(nullptr)
  , shoulderAngleController_(nullptr)
{
}

/**
 * 초기화
 */
void MotionSequence::init(RobotArm* robotArm, SystemStateManager* systemState,
                          AngleController* angleController,
                          ShoulderAngleController* shoulderAngleController) {
  robotArm_    = robotArm;
  systemState_ = systemState;
  angleController_ = angleController;
  shoulderAngleController_ = shoulderAngleController;
  DebugLog::info("MotionSequence: initialized (max %d commands)", MAX_COMMANDS);
}

/**
 * 업데이트 — loop()에서 주기 호출
 */
void MotionSequence::update() {
  if (state_ != SequenceState::RUNNING) {
    return;
  }

  // ARMED 상태가 아니면 즉시 중단
  if (systemState_ == nullptr || systemState_->getState() != SystemState::ARMED) {
    DebugLog::warn("MotionSequence: system no longer ARMED — stopping sequence");
    stop();
    return;
  }

  if (currentIndex_ >= count_) {
    state_ = SequenceState::COMPLETED;
    DebugLog::info("MotionSequence: COMPLETED (%d commands executed)", count_);
    return;
  }

  const MotionCommand& cmd = queue_[currentIndex_];
  if (isBaseAngleCommand(cmd)) {
    if (angleController_ == nullptr) {
      stopWithReason("base angle step requires AngleController");
      return;
    }

    if (angleController_->isActive()) {
      return;
    }

    if (angleController_->getLastStopReason() == AngleControllerStopReason::TARGET_REACHED) {
      advanceToNextStep();
    } else {
      char reason[96];
      snprintf(reason, sizeof(reason), "base angle step ended: %s",
               angleController_->getLastStopReasonString());
      stopWithReason(reason);
    }
    return;
  }

  if (cmd.joint == MotionJoint::SHOULDER && shoulderAngleController_ != nullptr) {
    char message[96] = {0};
    const bool directionUp = cmd.direction == MotionDirection::UP;
    if (!shoulderAngleController_->manualDirectionAllowed(directionUp,
                                                          message,
                                                          sizeof(message))) {
      stopCurrentJoint(cmd);
      stopWithReason(message);
      return;
    }
  }

  uint32_t elapsed = millis() - stepStartMs_;

  if (elapsed >= cmd.durationMs) {
    // 현재 스텝 완료 → 관절 정지 → 다음 스텝으로
    stopCurrentJoint(cmd);
    advanceToNextStep();
  }
}

/**
 * 명령 추가
 */
bool MotionSequence::addCommand(MotionJoint joint, MotionDirection direction,
                                uint8_t speed, uint32_t durationMs) {
  // COMPLETED 또는 STOPPED 후 첫 add 시 자동 clear — 새 시퀀스 시작 의도
  if (state_ == SequenceState::COMPLETED || state_ == SequenceState::STOPPED) {
    clear();
  }
  if (isFull()) {
    DebugLog::warn("MotionSequence: queue full (%d/%d)", count_, MAX_COMMANDS);
    return false;
  }
  if (durationMs == 0) {
    DebugLog::warn("MotionSequence: durationMs must be > 0");
    return false;
  }
  if (speed == 0 || speed > 100) {
    DebugLog::warn("MotionSequence: speed must be 1-100 (got %d)", speed);
    return false;
  }

  queue_[count_].joint      = joint;
  queue_[count_].direction  = direction;
  queue_[count_].speed      = speed;
  queue_[count_].durationMs = durationMs;
  queue_[count_].targetDegrees = 0.0f;
  count_++;

  DebugLog::debug("MotionSequence: command added (%d/%d), duration=%lums",
                  count_, MAX_COMMANDS, durationMs);
  return true;
}

bool MotionSequence::addBaseAngleCommand(MotionDirection direction, uint8_t speed, float targetDegrees) {
  if (state_ == SequenceState::COMPLETED || state_ == SequenceState::STOPPED) {
    clear();
  }
  if (isFull()) {
    DebugLog::warn("MotionSequence: queue full (%d/%d)", count_, MAX_COMMANDS);
    return false;
  }
  if (direction != MotionDirection::LEFT && direction != MotionDirection::RIGHT) {
    DebugLog::warn("MotionSequence: base angle direction must be left/right");
    return false;
  }
  if (speed < 1 || speed > 100) {
    DebugLog::warn("MotionSequence: base angle speed must be 1-100 (got %d)", speed);
    return false;
  }
  if (targetDegrees < AngleController::MIN_TARGET_DEGREES ||
      targetDegrees > AngleController::MAX_TARGET_DEGREES) {
    DebugLog::warn("MotionSequence: base angle target must be %.0f-%.0f (got %.1f)",
                   AngleController::MIN_TARGET_DEGREES,
                   AngleController::MAX_TARGET_DEGREES,
                   targetDegrees);
    return false;
  }

  queue_[count_].joint = MotionJoint::BASE;
  queue_[count_].direction = direction;
  queue_[count_].speed = speed;
  queue_[count_].durationMs = 0;
  queue_[count_].targetDegrees = targetDegrees;
  count_++;

  DebugLog::debug("MotionSequence: base angle step added (%d/%d), target=%.1fdeg",
                  count_, MAX_COMMANDS, targetDegrees);
  return true;
}

/**
 * 시퀀스 실행 시작
 */
bool MotionSequence::run() {
  if (count_ == 0) {
    DebugLog::warn("MotionSequence: no commands to run");
    return false;
  }
  if (state_ == SequenceState::RUNNING) {
    DebugLog::warn("MotionSequence: already running");
    return false;
  }
  if (systemState_ == nullptr || systemState_->getState() != SystemState::ARMED) {
    DebugLog::warn("MotionSequence: system must be ARMED to run");
    return false;
  }
  if (safetyMonitor.isMotionBlocked()) {
    DebugLog::warn("MotionSequence: blocked by safety (%s)", safetyMonitor.getBlockReasonString());
    return false;
  }

  currentIndex_ = 0;
  state_        = SequenceState::RUNNING;
  stepStartMs_  = millis();
  systemState_->resetTimeout();

  if (!startCurrentCommand()) {
    state_ = SequenceState::STOPPED;
    return false;
  }

  DebugLog::info("MotionSequence: RUNNING - step 1/%d started", count_);
  return true;
}

/**
 * 시퀀스 중단
 */
void MotionSequence::stop() {
  if (state_ != SequenceState::RUNNING) {
    return;
  }
  if (currentIndex_ < count_) {
    const MotionCommand& cmd = queue_[currentIndex_];
    if (isBaseAngleCommand(cmd)) {
      if (angleController_ != nullptr) {
        angleController_->cancel(AngleControllerStopReason::MANUAL_STOP, "sequence stop");
      }
    } else {
      stopCurrentJoint(cmd);
    }
  }
  state_ = SequenceState::STOPPED;
  DebugLog::info("MotionSequence: STOPPED at step %d/%d", currentIndex_ + 1, count_);
}

/**
 * 큐 초기화
 */
void MotionSequence::clear() {
  if (state_ == SequenceState::RUNNING) {
    stop();
  }
  count_        = 0;
  currentIndex_ = 0;
  state_        = SequenceState::IDLE;
  DebugLog::info("MotionSequence: cleared");
}

// ===== 상태 조회 =====

SequenceState MotionSequence::getState() const {
  return state_;
}

uint8_t MotionSequence::getCurrentIndex() const {
  return currentIndex_;
}

uint8_t MotionSequence::getTotalCount() const {
  return count_;
}

uint32_t MotionSequence::getRemainingMs() const {
  if (state_ != SequenceState::RUNNING || currentIndex_ >= count_) {
    return 0;
  }
  if (isBaseAngleCommand(queue_[currentIndex_])) {
    if (angleController_ == nullptr || !angleController_->isActive()) {
      return 0;
    }
    uint32_t elapsed = angleController_->getElapsedMs();
    uint32_t timeout = angleController_->getTimeoutMs();
    return elapsed < timeout ? (timeout - elapsed) : 0;
  }
  uint32_t elapsed = millis() - stepStartMs_;
  uint32_t dur     = queue_[currentIndex_].durationMs;
  return (elapsed < dur) ? (dur - elapsed) : 0;
}

bool MotionSequence::isFull() const {
  return count_ >= MAX_COMMANDS;
}

// ===== 파싱 유틸 =====

const char* MotionSequence::stateToString(SequenceState state) {
  switch (state) {
    case SequenceState::IDLE:      return "IDLE";
    case SequenceState::RUNNING:   return "RUNNING";
    case SequenceState::COMPLETED: return "COMPLETED";
    case SequenceState::STOPPED:   return "STOPPED";
    default:                       return "UNKNOWN";
  }
}

bool MotionSequence::parseJoint(const char* str, MotionJoint& joint) {
  if (str == nullptr) return false;
  if (strcasecmp(str, "gripper")  == 0) { joint = MotionJoint::GRIPPER;  return true; }
  if (strcasecmp(str, "wrist")    == 0) { joint = MotionJoint::WRIST;    return true; }
  if (strcasecmp(str, "elbow")    == 0) { joint = MotionJoint::ELBOW;    return true; }
  if (strcasecmp(str, "shoulder") == 0) { joint = MotionJoint::SHOULDER; return true; }
  if (strcasecmp(str, "base")     == 0) { joint = MotionJoint::BASE;     return true; }
  return false;
}

bool MotionSequence::parseDirection(MotionJoint joint, const char* str, MotionDirection& direction) {
  if (str == nullptr) return false;

  switch (joint) {
    case MotionJoint::GRIPPER:
      if (strcasecmp(str, "open")  == 0) { direction = MotionDirection::OPEN;  return true; }
      if (strcasecmp(str, "close") == 0) { direction = MotionDirection::CLOSE; return true; }
      break;
    case MotionJoint::WRIST:
    case MotionJoint::ELBOW:
    case MotionJoint::SHOULDER:
      if (strcasecmp(str, "up")   == 0) { direction = MotionDirection::UP;   return true; }
      if (strcasecmp(str, "down") == 0) { direction = MotionDirection::DOWN; return true; }
      break;
    case MotionJoint::BASE:
      if (strcasecmp(str, "left")  == 0) { direction = MotionDirection::LEFT;  return true; }
      if (strcasecmp(str, "right") == 0) { direction = MotionDirection::RIGHT; return true; }
      break;
  }
  return false;
}

// ===== private =====

/**
 * 명령 실행 (관절 구동 시작)
 */
bool MotionSequence::isBaseAngleCommand(const MotionCommand& cmd) const {
  return cmd.joint == MotionJoint::BASE && cmd.targetDegrees > 0.0f;
}

bool MotionSequence::executeCommand(const MotionCommand& cmd, char* errorMessage, size_t errorMessageSize) {
  if (errorMessage != nullptr && errorMessageSize > 0) {
    errorMessage[0] = '\0';
  }

  if (isBaseAngleCommand(cmd)) {
    if (angleController_ == nullptr) {
      if (errorMessage != nullptr && errorMessageSize > 0) {
        strlcpy(errorMessage, "AngleController not initialized", errorMessageSize);
      }
      return false;
    }
    return angleController_->startRelative(cmd.direction, cmd.targetDegrees, cmd.speed,
                                           errorMessage, errorMessageSize);
  }

  if (robotArm_ == nullptr) {
    if (errorMessage != nullptr && errorMessageSize > 0) {
      strlcpy(errorMessage, "RobotArm not initialized", errorMessageSize);
    }
    return false;
  }

  switch (cmd.joint) {
    case MotionJoint::GRIPPER:
      if      (cmd.direction == MotionDirection::OPEN)  return robotArm_->gripperOpen(cmd.speed);
      else if (cmd.direction == MotionDirection::CLOSE) return robotArm_->gripperClose(cmd.speed);
      return false;
    case MotionJoint::WRIST:
      if      (cmd.direction == MotionDirection::UP)   return robotArm_->wristUp(cmd.speed);
      else if (cmd.direction == MotionDirection::DOWN) return robotArm_->wristDown(cmd.speed);
      return false;
    case MotionJoint::ELBOW:
      if      (cmd.direction == MotionDirection::UP)   return robotArm_->elbowUp(cmd.speed);
      else if (cmd.direction == MotionDirection::DOWN) return robotArm_->elbowDown(cmd.speed);
      return false;
    case MotionJoint::SHOULDER:
      if (shoulderAngleController_ == nullptr) {
        if (errorMessage != nullptr && errorMessageSize > 0) {
          strlcpy(errorMessage, "ShoulderAngleController not initialized", errorMessageSize);
        }
        return false;
      }
      if (cmd.direction == MotionDirection::UP || cmd.direction == MotionDirection::DOWN) {
        const bool directionUp = cmd.direction == MotionDirection::UP;
        if (!shoulderAngleController_->manualDirectionAllowed(directionUp,
                                                              errorMessage,
                                                              errorMessageSize)) {
          return false;
        }
        return directionUp ? robotArm_->shoulderUp(cmd.speed)
                           : robotArm_->shoulderDown(cmd.speed);
      }
      return false;
    case MotionJoint::BASE:
      if      (cmd.direction == MotionDirection::LEFT)  return robotArm_->baseLeft(cmd.speed);
      else if (cmd.direction == MotionDirection::RIGHT) return robotArm_->baseRight(cmd.speed);
      return false;
  }

  return false;
}

bool MotionSequence::startCurrentCommand() {
  if (currentIndex_ >= count_) {
    state_ = SequenceState::COMPLETED;
    return false;
  }

  char errorMessage[96] = {0};
  if (!executeCommand(queue_[currentIndex_], errorMessage, sizeof(errorMessage))) {
    stopWithReason(errorMessage[0] != '\0' ? errorMessage : "failed to start step");
    return false;
  }

  stepStartMs_ = millis();
  DebugLog::debug("MotionSequence: step %d/%d started", currentIndex_ + 1, count_);
  return true;
}

void MotionSequence::advanceToNextStep() {
  currentIndex_++;

  if (currentIndex_ >= count_) {
    state_ = SequenceState::COMPLETED;
    DebugLog::info("MotionSequence: COMPLETED (%d commands executed)", count_);
    return;
  }

  if (systemState_ != nullptr) {
    systemState_->resetTimeout();
  }
  startCurrentCommand();
}

void MotionSequence::stopWithReason(const char* reason) {
  state_ = SequenceState::STOPPED;
  DebugLog::warn("MotionSequence: STOPPED at step %d/%d (%s)",
                 currentIndex_ + 1, count_, reason != nullptr ? reason : "unknown");
}

/**
 * 현재 관절 정지
 */
void MotionSequence::stopCurrentJoint(const MotionCommand& cmd) {
  if (robotArm_ == nullptr) return;

  switch (cmd.joint) {
    case MotionJoint::GRIPPER:  robotArm_->gripperStop();  break;
    case MotionJoint::WRIST:    robotArm_->wristStop();    break;
    case MotionJoint::ELBOW:    robotArm_->elbowStop();    break;
    case MotionJoint::SHOULDER: robotArm_->shoulderStop(); break;
    case MotionJoint::BASE:     robotArm_->baseStop();     break;
  }
}
