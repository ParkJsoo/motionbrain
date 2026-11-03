#include "finite_profile_service_environment.h"

#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER
#include "control/angle_controller.h"
#include "control/bounded_action_service.h"
#include "control/command_bus.h"
#include "control/dispatcher.h"
#include "control/guarded_routine_executor.h"
#include "control/safety_gate.h"
#include "control/shoulder_angle_controller.h"
#include "input/teleop_adapter.h"
#include "motion/motion_sequence.h"
#include "network/web_server.h"

extern SafetyGate safetyGate;
extern Dispatcher dispatcher;
extern AngleController angleController;
extern ShoulderAngleController shoulderAngleController;
extern TeleopAdapter teleopAdapter;
extern MotionSequence motionSequence;
extern MotionBrainWebServer webServer;
extern CommandBus commandBus;

namespace finite_profile {
bool ServiceEnvironment::ready() const {
  return safetyGate.isReady() && dispatcher.isReady() && angleController.isReady() &&
      shoulderAngleController.isReady() && teleopAdapter.isReady() && webServer.isActive();
}
bool ServiceEnvironment::allowed() const {
  if (!issuer_ || !ready()) return false;
  Command command;
  command.type = CommandType::MOTOR_RUN;
  CommandResult result;
  return safetyGate.allows(command, result);
}
bool ServiceEnvironment::controllersIdle() const {
  if (!ready() || BoundedActions::active() || commandBus.hasPendingControlCommand() ||
      motionSequence.getState() == SequenceState::RUNNING || motionSequence.getTotalCount() != 0 ||
      angleController.isActive() || shoulderAngleController.isActive() ||
      teleopAdapter.isControlActive() || teleopAdapter.isDeadmanHeld() ||
      webServer.hasManualControlLeases()) return false;
  // Prepared/blocked routines may retain work even with zero motor PWM.
  // Stored sequence programs are also rejected; never clear them implicitly.
  switch (GuardedRoutineExecutor::status().state) {
    case GuardedRoutineExecutorState::IDLE:
    case GuardedRoutineExecutorState::ABORTED:
    case GuardedRoutineExecutorState::TIMED_OUT:
    case GuardedRoutineExecutorState::COMPLETED:
      return true;
    default:
      return false;
  }
}
bool ServiceEnvironment::mintToken(Token& token) {
  token = Token();
  if (!allowed() || !controllersIdle()) return false;
  if (!issuer_(token)) { token = Token(); return false; }
  return true; // The core rejects zero/reused tokens without issuing a grant.
}
} // namespace finite_profile
#endif
