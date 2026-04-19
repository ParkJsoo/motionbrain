#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include <Arduino.h>
#include "motor/motor_driver.h"

/**
 * MotionBrain Robot Arm Module
 *
 * Phase 2-A: 관절 추상화 계층
 * - MotorControl 위에 얹는 고수준 API
 * - 모터 번호/방향 대신 관절 동작 이름으로 제어
 * - 방향 상수로 실물 테스트 후 쉽게 보정 가능
 *
 * 관절 구성:
 * - M1: 그리퍼 (Gripper)     — open / close
 * - M2: 손목 (Wrist)         — up / down
 * - M3: 팔꿈치 (Elbow)       — up / down
 * - M4: 어깨 (Shoulder)      — up / down
 * - M5: 베이스 (Base)        — left / right
 *
 * 방향 보정:
 * - 각 관절의 _IS_FORWARD 상수를 true/false로 바꾸면 방향 반전
 * - Step 6 실물 테스트 후 필요한 관절만 수정
 */
class RobotArm {
public:
  // 기본 속도 (0~100, percent 기준)
  static const uint8_t DEFAULT_SPEED = 50;

  // ===== 방향 보정 상수 =====
  // 실물 테스트 후 방향이 반대면 true <-> false 로 변경
  static const bool GRIPPER_OPEN_IS_FORWARD   = true;   // M1: open 방향
  static const bool WRIST_UP_IS_FORWARD       = true;   // M2: up 방향
  static const bool ELBOW_UP_IS_FORWARD       = false;  // M3: up 방향
  static const bool SHOULDER_UP_IS_FORWARD    = true;   // M4: up 방향
  static const bool BASE_LEFT_IS_FORWARD      = true;   // M5: left 방향

  /**
   * 생성자
   * @param motorControl MotorControl 참조
   */
  explicit RobotArm(MotorControl* motorControl);

  // ===== 그리퍼 (M1) =====
  bool gripperOpen(uint8_t percent = DEFAULT_SPEED);
  bool gripperClose(uint8_t percent = DEFAULT_SPEED);
  bool gripperStop();

  // ===== 손목 (M2) =====
  bool wristUp(uint8_t percent = DEFAULT_SPEED);
  bool wristDown(uint8_t percent = DEFAULT_SPEED);
  bool wristStop();

  // ===== 팔꿈치 (M3) =====
  bool elbowUp(uint8_t percent = DEFAULT_SPEED);
  bool elbowDown(uint8_t percent = DEFAULT_SPEED);
  bool elbowStop();

  // ===== 어깨 (M4) =====
  bool shoulderUp(uint8_t percent = DEFAULT_SPEED);
  bool shoulderDown(uint8_t percent = DEFAULT_SPEED);
  bool shoulderStop();

  // ===== 베이스 (M5) =====
  bool baseLeft(uint8_t percent = DEFAULT_SPEED);
  bool baseRight(uint8_t percent = DEFAULT_SPEED);
  bool baseStop();

  // ===== 전체 =====
  bool stopAll();

private:
  MotorControl* motorControl_;

  /**
   * 방향 플래그에 따라 forward/reverse 선택
   * @param motorId   모터 번호 (1~5)
   * @param percent   속도 비율 (0~100)
   * @param isForward true면 forward, false면 reverse
   */
  bool drive(uint8_t motorId, uint8_t percent, bool isForward);
};

#endif // ROBOT_ARM_H
