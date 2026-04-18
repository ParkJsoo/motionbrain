#include "motion_sequence.h"
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
{
}

/**
 * 초기화
 */
void MotionSequence::init(RobotArm* robotArm, SystemStateManager* systemState) {
  robotArm_    = robotArm;
  systemState_ = systemState;
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
    // 모든 명령 완료
    stopCurrentJoint(queue_[currentIndex_ - 1]);
    state_ = SequenceState::COMPLETED;
    DebugLog::info("MotionSequence: COMPLETED (%d commands executed)", count_);
    return;
  }

  const MotionCommand& cmd = queue_[currentIndex_];
  uint32_t elapsed = millis() - stepStartMs_;

  if (elapsed >= cmd.durationMs) {
    // 현재 스텝 완료 → 관절 정지 → 다음 스텝으로
    stopCurrentJoint(cmd);
    currentIndex_++;

    if (currentIndex_ >= count_) {
      // 마지막 스텝 완료
      state_ = SequenceState::COMPLETED;
      DebugLog::info("MotionSequence: COMPLETED (%d commands executed)", count_);
    } else {
      // 다음 스텝 실행
      systemState_->resetTimeout();  // 30초 타임아웃 리셋
      stepStartMs_ = millis();
      executeCommand(queue_[currentIndex_]);
      DebugLog::debug("MotionSequence: step %d/%d started", currentIndex_ + 1, count_);
    }
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
  count_++;

  DebugLog::debug("MotionSequence: command added (%d/%d), duration=%lums",
                  count_, MAX_COMMANDS, durationMs);
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

  executeCommand(queue_[0]);
  DebugLog::info("MotionSequence: RUNNING — step 1/%d started", count_);
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
    stopCurrentJoint(queue_[currentIndex_]);
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
void MotionSequence::executeCommand(const MotionCommand& cmd) {
  if (robotArm_ == nullptr) return;

  switch (cmd.joint) {
    case MotionJoint::GRIPPER:
      if      (cmd.direction == MotionDirection::OPEN)  robotArm_->gripperOpen(cmd.speed);
      else if (cmd.direction == MotionDirection::CLOSE) robotArm_->gripperClose(cmd.speed);
      break;
    case MotionJoint::WRIST:
      if      (cmd.direction == MotionDirection::UP)   robotArm_->wristUp(cmd.speed);
      else if (cmd.direction == MotionDirection::DOWN) robotArm_->wristDown(cmd.speed);
      break;
    case MotionJoint::ELBOW:
      if      (cmd.direction == MotionDirection::UP)   robotArm_->elbowUp(cmd.speed);
      else if (cmd.direction == MotionDirection::DOWN) robotArm_->elbowDown(cmd.speed);
      break;
    case MotionJoint::SHOULDER:
      if      (cmd.direction == MotionDirection::UP)   robotArm_->shoulderUp(cmd.speed);
      else if (cmd.direction == MotionDirection::DOWN) robotArm_->shoulderDown(cmd.speed);
      break;
    case MotionJoint::BASE:
      if      (cmd.direction == MotionDirection::LEFT)  robotArm_->baseLeft(cmd.speed);
      else if (cmd.direction == MotionDirection::RIGHT) robotArm_->baseRight(cmd.speed);
      break;
  }
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
