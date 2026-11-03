#include "control/finite_profile_legacy_guard.h"

#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER
#include "control/command.h"
#include "motor/motor_driver.h"
extern MotorControl motorControl;

namespace FiniteProfileLegacy {
bool allowed(CommandType type) {
  if (!motorControl.finiteProfileOwned()) return true;
  switch (type) {
    case CommandType::LIGHT_ON:
    case CommandType::LIGHT_OFF:
    case CommandType::LIGHT_TOGGLE:
    case CommandType::ROUTINE_DRY_RUN:
      return true;
    default:
      break;
  }
  motorControl.invalidateFiniteProfileLease();
  switch (type) {
    case CommandType::STOP:
    case CommandType::DISARM:
    case CommandType::MOTOR_STOP:
    case CommandType::MOTOR_STOP_ALL:
    case CommandType::JOINT_STOP:
    case CommandType::JOINT_STOP_ALL:
    case CommandType::SEQUENCE_STOP:
    case CommandType::SEQUENCE_CLEAR:
    case CommandType::ROUTINE_ABORT:
      return true;
    default:
      return false;
  }
}
}
#endif
