#include "robot_arm.h"
#include "debug/debug_log.h"

/**
 * 생성자
 */
RobotArm::RobotArm(MotorControl* motorControl)
  : motorControl_(motorControl)
{
}

// ===== 내부 헬퍼 =====

bool RobotArm::drive(uint8_t motorId, uint8_t percent, bool isForward) {
  if (motorControl_ == nullptr) {
    DebugLog::error("RobotArm: MotorControl not initialized");
    return false;
  }
  return isForward
    ? motorControl_->forward(motorId, percent)
    : motorControl_->reverse(motorId, percent);
}

// ===== 그리퍼 (M1) =====

bool RobotArm::gripperOpen(uint8_t percent) {
  bool result = drive(MotorControl::MOTOR_1, percent, GRIPPER_OPEN_IS_FORWARD);
  if (result) DebugLog::info("[ARM] Gripper: open at %d%%", percent);
  return result;
}

bool RobotArm::gripperClose(uint8_t percent) {
  bool result = drive(MotorControl::MOTOR_1, percent, !GRIPPER_OPEN_IS_FORWARD);
  if (result) DebugLog::info("[ARM] Gripper: close at %d%%", percent);
  return result;
}

bool RobotArm::gripperStop() {
  bool result = motorControl_->stop(MotorControl::MOTOR_1);
  if (result) DebugLog::info("[ARM] Gripper: stop");
  return result;
}

// ===== 손목 (M2) =====

bool RobotArm::wristUp(uint8_t percent) {
  bool result = drive(MotorControl::MOTOR_2, percent, WRIST_UP_IS_FORWARD);
  if (result) DebugLog::info("[ARM] Wrist: up at %d%%", percent);
  return result;
}

bool RobotArm::wristDown(uint8_t percent) {
  bool result = drive(MotorControl::MOTOR_2, percent, !WRIST_UP_IS_FORWARD);
  if (result) DebugLog::info("[ARM] Wrist: down at %d%%", percent);
  return result;
}

bool RobotArm::wristStop() {
  bool result = motorControl_->stop(MotorControl::MOTOR_2);
  if (result) DebugLog::info("[ARM] Wrist: stop");
  return result;
}

// ===== 팔꿈치 (M3) =====

bool RobotArm::elbowUp(uint8_t percent) {
  bool result = drive(MotorControl::MOTOR_3, percent, ELBOW_UP_IS_FORWARD);
  if (result) DebugLog::info("[ARM] Elbow: up at %d%%", percent);
  return result;
}

bool RobotArm::elbowDown(uint8_t percent) {
  bool result = drive(MotorControl::MOTOR_3, percent, !ELBOW_UP_IS_FORWARD);
  if (result) DebugLog::info("[ARM] Elbow: down at %d%%", percent);
  return result;
}

bool RobotArm::elbowStop() {
  bool result = motorControl_->stop(MotorControl::MOTOR_3);
  if (result) DebugLog::info("[ARM] Elbow: stop");
  return result;
}

// ===== 어깨 (M4) =====

bool RobotArm::shoulderUp(uint8_t percent) {
  bool result = drive(MotorControl::MOTOR_4, percent, SHOULDER_UP_IS_FORWARD);
  if (result) DebugLog::info("[ARM] Shoulder: up at %d%%", percent);
  return result;
}

bool RobotArm::shoulderDown(uint8_t percent) {
  bool result = drive(MotorControl::MOTOR_4, percent, !SHOULDER_UP_IS_FORWARD);
  if (result) DebugLog::info("[ARM] Shoulder: down at %d%%", percent);
  return result;
}

bool RobotArm::shoulderStop() {
  bool result = motorControl_->stop(MotorControl::MOTOR_4);
  if (result) DebugLog::info("[ARM] Shoulder: stop");
  return result;
}

// ===== 베이스 (M5) =====

bool RobotArm::baseLeft(uint8_t percent) {
  bool result = drive(MotorControl::MOTOR_5, percent, BASE_LEFT_IS_FORWARD);
  if (result) DebugLog::info("[ARM] Base: left at %d%%", percent);
  return result;
}

bool RobotArm::baseRight(uint8_t percent) {
  bool result = drive(MotorControl::MOTOR_5, percent, !BASE_LEFT_IS_FORWARD);
  if (result) DebugLog::info("[ARM] Base: right at %d%%", percent);
  return result;
}

bool RobotArm::baseStop() {
  bool result = motorControl_->stop(MotorControl::MOTOR_5);
  if (result) DebugLog::info("[ARM] Base: stop");
  return result;
}

// ===== 전체 =====

bool RobotArm::stopAll() {
  if (motorControl_ == nullptr) return false;
  bool result = motorControl_->stopAll();
  if (result) DebugLog::info("[ARM] All joints: stop");
  return result;
}
