#ifndef MOTIONBRAIN_MANUAL_COMMAND_DIRECTION_H
#define MOTIONBRAIN_MANUAL_COMMAND_DIRECTION_H
#include "motion/robot_arm.h"

// Raw /motor forward and the semantic positive joint action intentionally share
// the existing RobotArm calibration. The dispatcher and lease admission must use
// the same mapping; direction labels are not direct H-bridge polarity.
namespace ManualCommandDirection {
inline bool positiveIsForward(uint8_t motor) {
  switch (motor) {
    case MotorControl::MOTOR_1: return RobotArm::GRIPPER_OPEN_IS_FORWARD;
    case MotorControl::MOTOR_2: return RobotArm::WRIST_UP_IS_FORWARD;
    case MotorControl::MOTOR_3: return RobotArm::ELBOW_UP_IS_FORWARD;
    case MotorControl::MOTOR_4: return RobotArm::SHOULDER_UP_IS_FORWARD;
    case MotorControl::MOTOR_5: return RobotArm::BASE_LEFT_IS_FORWARD;
    default: return true;
  }
}
inline bool physicalForward(uint8_t motor, bool semanticPositive) {
  const bool positive = positiveIsForward(motor);
  return semanticPositive ? positive : !positive;
}
}
#endif
