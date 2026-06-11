#include "control/dispatcher.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "control/angle_controller.h"
#include "control/command_bus.h"
#include "control/event_log.h"
#include "control/guarded_routine.h"
#include "control/guarded_routine_executor.h"
#include "control/safety_gate.h"
#include "debug/debug_log.h"
#include "motion/robot_arm.h"
#include "motion/motion_sequence.h"
#include "motor/motor_driver.h"
#include "peripheral/search_light.h"
#include "safety/safety_monitor.h"
#include "system/system_init.h"

extern SafetyMonitor safetyMonitor;
extern EventLog eventLog;

namespace {

const char* commandTypeToString(CommandType type) {
  switch (type) {
    case CommandType::ARM:                     return "arm";
    case CommandType::DISARM:                  return "disarm";
    case CommandType::STOP:                    return "stop";
    case CommandType::MOTOR_RUN:               return "motor run";
    case CommandType::MOTOR_STOP:              return "motor stop";
    case CommandType::MOTOR_STOP_ALL:          return "motor stop all";
    case CommandType::MOTOR_SET_DEFAULT_SPEED: return "motor default";
    case CommandType::JOINT_RUN:               return "joint run";
    case CommandType::JOINT_STOP:              return "joint stop";
    case CommandType::JOINT_STOP_ALL:          return "joint stop all";
    case CommandType::BASE_ANGLE_RUN:          return "base angle";
    case CommandType::SEQUENCE_ADD:            return "sequence add";
    case CommandType::SEQUENCE_RUN:            return "sequence run";
    case CommandType::SEQUENCE_STOP:           return "sequence stop";
    case CommandType::SEQUENCE_CLEAR:          return "sequence clear";
    case CommandType::ROUTINE_DRY_RUN:         return "routine dry-run";
    case CommandType::ROUTINE_RUN:             return "routine run";
    case CommandType::ROUTINE_ABORT:           return "routine abort";
    case CommandType::LIGHT_ON:                return "light on";
    case CommandType::LIGHT_OFF:               return "light off";
    case CommandType::LIGHT_TOGGLE:            return "light toggle";
    default:                                   return "unknown";
  }
}

const char* commandSourceToString(CommandSource source) {
  switch (source) {
    case CommandSource::SERIAL_INPUT: return "serial";
    case CommandSource::WEB_INPUT:    return "web";
    case CommandSource::INTERNAL:     return "internal";
    default:                          return "unknown";
  }
}

void appendEscaped(String& json, const char* raw) {
  const char* text = raw != nullptr ? raw : "";
  while (*text != '\0') {
    switch (*text) {
      case '\\': json += "\\\\"; break;
      case '"':  json += "\\\""; break;
      case '\n': json += "\\n"; break;
      case '\r': json += "\\r"; break;
      case '\t': json += "\\t"; break;
      default:   json += *text; break;
    }
    text++;
  }
}

const char* jointToString(MotionJoint joint) {
  switch (joint) {
    case MotionJoint::GRIPPER:  return "gripper";
    case MotionJoint::WRIST:    return "wrist";
    case MotionJoint::ELBOW:    return "elbow";
    case MotionJoint::SHOULDER: return "shoulder";
    case MotionJoint::BASE:     return "base";
    default:                    return "unknown";
  }
}

const char* directionToString(MotionDirection direction) {
  switch (direction) {
    case MotionDirection::OPEN:  return "open";
    case MotionDirection::CLOSE: return "close";
    case MotionDirection::UP:    return "up";
    case MotionDirection::DOWN:  return "down";
    case MotionDirection::LEFT:  return "left";
    case MotionDirection::RIGHT: return "right";
    default:                     return "unknown";
  }
}

bool semanticPositiveIsForward(uint8_t motorId) {
  switch (motorId) {
    case MotorControl::MOTOR_1: return RobotArm::GRIPPER_OPEN_IS_FORWARD;
    case MotorControl::MOTOR_2: return RobotArm::WRIST_UP_IS_FORWARD;
    case MotorControl::MOTOR_3: return RobotArm::ELBOW_UP_IS_FORWARD;
    case MotorControl::MOTOR_4: return RobotArm::SHOULDER_UP_IS_FORWARD;
    case MotorControl::MOTOR_5: return RobotArm::BASE_LEFT_IS_FORWARD;
    default:                    return true;
  }
}

bool resolveMotorRunForward(uint8_t motorId, bool commandForward) {
  return commandForward ? semanticPositiveIsForward(motorId)
                        : !semanticPositiveIsForward(motorId);
}

} // namespace

Dispatcher::Dispatcher()
  : systemState_(nullptr)
  , motorControl_(nullptr)
  , robotArm_(nullptr)
  , motionSequence_(nullptr)
  , searchLight_(nullptr) {
  safetyGate_ = nullptr;
  angleController_ = nullptr;
}

void Dispatcher::init(SystemStateManager* systemState,
                      MotorControl* motorControl,
                      RobotArm* robotArm,
                      MotionSequence* motionSequence,
                      SearchLight* searchLight,
                      SafetyGate* safetyGate,
                      AngleController* angleController) {
  systemState_ = systemState;
  motorControl_ = motorControl;
  robotArm_ = robotArm;
  motionSequence_ = motionSequence;
  searchLight_ = searchLight;
  safetyGate_ = safetyGate;
  angleController_ = angleController;
}

bool Dispatcher::isReady() const {
  return hasCoreDependencies();
}

bool Dispatcher::hasCoreDependencies() const {
  return systemState_ != nullptr && motorControl_ != nullptr && safetyGate_ != nullptr;
}

bool Dispatcher::hasDependenciesFor(CommandType type, const char** missingDependency) const {
  if (!hasCoreDependencies()) {
    if (missingDependency != nullptr) {
      *missingDependency = "core services";
    }
    return false;
  }

  switch (type) {
    case CommandType::JOINT_RUN:
    case CommandType::JOINT_STOP:
    case CommandType::JOINT_STOP_ALL:
      if (robotArm_ == nullptr) {
        if (missingDependency != nullptr) {
          *missingDependency = "robot arm";
        }
        return false;
      }
      break;

    case CommandType::BASE_ANGLE_RUN:
      if (angleController_ == nullptr) {
        if (missingDependency != nullptr) {
          *missingDependency = "angle controller";
        }
        return false;
      }
      break;

    case CommandType::SEQUENCE_ADD:
    case CommandType::SEQUENCE_RUN:
    case CommandType::SEQUENCE_STOP:
    case CommandType::SEQUENCE_CLEAR:
      if (motionSequence_ == nullptr) {
        if (missingDependency != nullptr) {
          *missingDependency = "motion sequence";
        }
        return false;
      }
      break;

    case CommandType::LIGHT_ON:
    case CommandType::LIGHT_OFF:
    case CommandType::LIGHT_TOGGLE:
      if (searchLight_ == nullptr) {
        if (missingDependency != nullptr) {
          *missingDependency = "search light";
        }
        return false;
      }
      break;

    default:
      break;
  }

  return true;
}

bool Dispatcher::commandExtendsTimeout(CommandType type) const {
  switch (type) {
    case CommandType::ARM:
    case CommandType::MOTOR_RUN:
    case CommandType::MOTOR_STOP:
    case CommandType::MOTOR_STOP_ALL:
    case CommandType::JOINT_RUN:
    case CommandType::JOINT_STOP:
    case CommandType::JOINT_STOP_ALL:
    case CommandType::BASE_ANGLE_RUN:
    case CommandType::SEQUENCE_RUN:
    case CommandType::SEQUENCE_STOP:
      return true;
    default:
      return false;
  }
}

bool Dispatcher::execute(const Command& command, CommandResult& result) {
  const char* missingDependency = nullptr;
  if (!hasDependenciesFor(command.type, &missingDependency)) {
    setResult(result, command.id, false, "Dispatcher missing %s", missingDependency);
    recordCommandResult(command, result);
    return false;
  }

  if (!safetyGate_->allows(command, result)) {
    DebugLog::command(commandTypeToString(command.type), false, result.message);
    recordCommandResult(command, result);
    return false;
  }

  cancelBaseAngleIfNeeded(command);

  bool success = false;

  switch (command.type) {
    case CommandType::ARM:
      success = systemState_->arm();
      setResult(result, command.id, success, success
        ? "System armed successfully"
        : "Failed to arm - check current state");
      break;

    case CommandType::DISARM:
      if (angleController_ != nullptr) {
        angleController_->cancel(AngleControllerStopReason::STATE_CHANGED, "disarm");
      }
      success = systemState_->disarm();
      setResult(result, command.id, success, success
        ? "System disarmed successfully"
        : "Failed to disarm - check current state");
      break;

    case CommandType::STOP: {
      SystemState previousState = systemState_->getState();
      if (angleController_ != nullptr) {
        angleController_->cancel(AngleControllerStopReason::STATE_CHANGED, "stop");
      }
      if (motionSequence_ != nullptr) {
        motionSequence_->stop();
      }
      if (!systemState_->enterSafe()) {
        DebugLog::warn("Dispatcher: enterSafe() failed during STOP - forcing motor emergency stop");
      }
      motorControl_->emergencyStop();
      success = true;
      setResult(result, command.id, true,
                previousState == SystemState::FAULT ? "Fault cleared to IDLE" : "Emergency stop activated");
      break;
    }

    case CommandType::MOTOR_RUN:
      success = resolveMotorRunForward(command.motorId, command.forward)
        ? motorControl_->forward(command.motorId, command.percent)
        : motorControl_->reverse(command.motorId, command.percent);
      if (success) {
        setResult(result, command.id, true, "Motor M%d %s at %u%%",
                  command.motorId, command.forward ? "forward" : "reverse", command.percent);
      } else if (safetyMonitor.isMotionBlocked()) {
        setResult(result, command.id, false, "Blocked by safety: %s",
                  safetyMonitor.getBlockReasonString());
      } else {
        setResult(result, command.id, false, "Failed to set motor M%d %s",
                  command.motorId, command.forward ? "forward" : "reverse");
      }
      break;

    case CommandType::MOTOR_STOP:
      success = motorControl_->stop(command.motorId);
      setResult(result, command.id, success,
                success ? "Motor M%d stopped" : "Failed to stop motor M%d",
                command.motorId);
      break;

    case CommandType::MOTOR_STOP_ALL:
      success = motorControl_->stopAll();
      setResult(result, command.id, success,
                success ? "All motors stopped" : "Failed to stop all motors");
      break;

    case CommandType::MOTOR_SET_DEFAULT_SPEED:
      success = motorControl_->setDefaultSpeed(command.speed);
      setResult(result, command.id, success,
                success ? "Default speed set to %u" : "Failed to set default speed",
                command.speed);
      break;

    case CommandType::JOINT_RUN:
      success = executeJointRun(command.joint, command.direction, command.percent);
      if (success) {
        setResult(result, command.id, true, "%s %s at %u%%",
                  jointToString(command.joint), directionToString(command.direction), command.percent);
      } else if (safetyMonitor.isMotionBlocked()) {
        setResult(result, command.id, false, "Blocked by safety: %s",
                  safetyMonitor.getBlockReasonString());
      } else {
        setResult(result, command.id, false, "Failed to run %s %s",
                  jointToString(command.joint), directionToString(command.direction));
      }
      break;

    case CommandType::JOINT_STOP:
      success = executeJointStop(command.joint);
      setResult(result, command.id, success,
                success ? "%s stop" : "Failed to stop %s",
                jointToString(command.joint));
      break;

    case CommandType::JOINT_STOP_ALL:
      success = robotArm_->stopAll();
      setResult(result, command.id, success,
                success ? "All joints stopped" : "Failed to stop all joints");
      break;

    case CommandType::BASE_ANGLE_RUN: {
      char message[sizeof(result.message)] = {0};
      success = angleController_->startRelative(command.direction, command.targetDegrees,
                                                command.percent, message, sizeof(message));
      if (success) {
        setResult(result, command.id, true, "%s", message);
      } else if (message[0] != '\0') {
        setResult(result, command.id, false, "%s", message);
      } else {
        setResult(result, command.id, false, "Failed to start base angle control");
      }
      break;
    }

    case CommandType::SEQUENCE_ADD:
      if (command.joint == MotionJoint::BASE && command.targetDegrees > 0.0f) {
        success = motionSequence_->addBaseAngleCommand(command.direction, command.percent,
                                                       command.targetDegrees);
        if (success) {
          setResult(result, command.id, true, "Base angle step added [%.1fdeg] [%u/%u]",
                    command.targetDegrees,
                    motionSequence_->getTotalCount(), MotionSequence::MAX_COMMANDS);
        } else {
          setResult(result, command.id, false, "Base angle add failed (queue full or invalid params)");
        }
      } else {
        success = motionSequence_->addCommand(command.joint, command.direction, command.percent,
                                              command.durationMs);
        if (success) {
          setResult(result, command.id, true, "Command added [%u/%u]",
                    motionSequence_->getTotalCount(), MotionSequence::MAX_COMMANDS);
        } else {
          setResult(result, command.id, false, "Add failed (queue full or invalid params)");
        }
      }
      break;

    case CommandType::SEQUENCE_RUN:
      success = motionSequence_->run();
      if (success) {
        setResult(result, command.id, true, "Sequence started");
      } else if (safetyMonitor.isMotionBlocked()) {
        setResult(result, command.id, false, "Blocked by safety: %s",
                  safetyMonitor.getBlockReasonString());
      } else {
        setResult(result, command.id, false, "Sequence run failed (ARMED? commands queued?)");
      }
      break;

    case CommandType::SEQUENCE_STOP:
      motionSequence_->stop();
      success = true;
      setResult(result, command.id, true, "Sequence stopped");
      break;

    case CommandType::SEQUENCE_CLEAR:
      motionSequence_->clear();
      success = true;
      setResult(result, command.id, true, "Sequence cleared");
      break;

    case CommandType::ROUTINE_DRY_RUN: {
      GuardedRoutinePlan plan;
      if (!GuardedRoutine::getPlan(command.routineName, plan)) {
        setResult(result, command.id, false, "Unknown routine '%s'", command.routineName);
        break;
      }

      char detail[96] = {0};
      snprintf(detail, sizeof(detail), "name=%s steps=%u state=%s blocked=%s",
               plan.name,
               plan.stepCount,
               systemState_ != nullptr ? systemState_->getStateString() : "UNKNOWN",
               safetyMonitor.isMotionBlocked() ? safetyMonitor.getBlockReasonString() : "NONE");
      eventLog.push("routine", "ROUTINE_DRY_RUN", safetyMonitor.isMotionBlocked()
                      ? EventSeverity::WARN
                      : EventSeverity::INFO,
                    detail);
      success = true;
      setResult(result, command.id, true, "Routine '%s' dry-run plan ready (%u steps)",
                plan.name, plan.stepCount);
      break;
    }

    case CommandType::ROUTINE_RUN: {
      GuardedRoutinePlan plan;
      if (!GuardedRoutine::getPlan(command.routineName, plan)) {
        setResult(result, command.id, false, "Unknown routine '%s'", command.routineName);
        break;
      }

      const bool operatorConfirmed =
        !plan.requiresOperatorConfirm ||
        strcmp(command.routineConfirmCode, plan.confirmationCode) == 0;
      const bool stateAllowsExecute =
        systemState_ != nullptr && systemState_->getState() == SystemState::ARMED;
      const bool motionClear = !safetyMonitor.isMotionBlocked();
      const bool faultClear = !safetyMonitor.hasLatchedFault();
      const bool noActiveSequence =
        motionSequence_ == nullptr || motionSequence_->getState() != SequenceState::RUNNING;
      const bool perceptionReady = !plan.perceptionRequired;
      const GuardedRoutineExecutePreflight preflight =
        GuardedRoutine::evaluateExecutePreflight(plan,
                                                 operatorConfirmed,
                                                 stateAllowsExecute,
                                                 motionClear,
                                                 faultClear,
                                                 noActiveSequence,
                                                 perceptionReady);
      const char* preflightResult = GuardedRoutine::preflightResultToString(preflight.result);

      if (preflight.result == GuardedRoutinePreflightResult::CONFIRM_REQUIRED) {
        char detail[96] = {0};
        snprintf(detail, sizeof(detail), "name=%s confirm=missing_or_mismatch", plan.name);
        eventLog.push("routine", "ROUTINE_CONFIRM_REQ", EventSeverity::WARN, detail);
        success = false;
        setResult(result, command.id, false,
                  "Routine '%s' requires operator confirmation code", plan.name);
        break;
      }

      if (!preflight.executeReady) {
        char detail[96] = {0};
        snprintf(detail, sizeof(detail), "name=%s result=%s state=%s",
                 plan.name,
                 preflightResult,
                 systemState_ != nullptr ? systemState_->getStateString() : "UNKNOWN");
        eventLog.push("routine", "ROUTINE_PREFLIGHT_BLOCK", EventSeverity::WARN, detail);
        success = false;
        setResult(result, command.id, false,
                  "Routine '%s' preflight blocked: %s", plan.name, preflightResult);
        break;
      }

      GuardedRoutineExecutorReport executorReport;
      GuardedRoutineExecutor::begin(plan, executorReport, motionSequence_);

      char detail[96] = {0};
      snprintf(detail, sizeof(detail),
               "n=%s e=%s p=%s m=%s a=%u s=%u steps=%u",
               plan.name,
               GuardedRoutineExecutor::resultToString(executorReport.result),
               GuardedRoutineExecutor::prepareResultToString(executorReport.prepareResult),
               GuardedRoutineExecutor::materializeResultToString(executorReport.materializeResult),
               executorReport.preparedSequenceApplied ? 1 : 0,
               executorReport.sequenceStarted ? 1 : 0,
               executorReport.motionStepCount);
      eventLog.push("routine", "ROUTINE_EXECUTE_BLOCKED", EventSeverity::WARN, detail);
      success = false;
      setResult(result, command.id, false,
                "Routine executor blocked: %s", executorReport.detail);
      break;
    }

    case CommandType::ROUTINE_ABORT: {
      GuardedRoutineExecutorReport executorReport;
      success = GuardedRoutineExecutor::abort("operator_request", executorReport);
      eventLog.push("routine",
                    success ? "ROUTINE_ABORT" : "ROUTINE_ABORT_IDLE",
                    success ? EventSeverity::WARN : EventSeverity::INFO,
                    executorReport.detail);
      setResult(result, command.id, success,
                success ? "Routine executor abort requested"
                        : "No active routine executor to abort");
      break;
    }

    case CommandType::LIGHT_ON:
      searchLight_->on();
      success = true;
      setResult(result, command.id, true, "SearchLight: ON");
      break;

    case CommandType::LIGHT_OFF:
      searchLight_->off();
      success = true;
      setResult(result, command.id, true, "SearchLight: OFF");
      break;

    case CommandType::LIGHT_TOGGLE:
      searchLight_->toggle();
      success = true;
      setResult(result, command.id, true, "SearchLight: %s",
                searchLight_->isOn() ? "ON" : "OFF");
      break;
  }

  if (success && commandExtendsTimeout(command.type) && systemState_ != nullptr) {
    systemState_->resetTimeout();
  }

  DebugLog::command(commandTypeToString(command.type), result.success, result.message);
  recordCommandResult(command, result);
  return result.success;
}

DispatcherCommandAudit Dispatcher::lastCommandAudit() const {
  return lastCommand_;
}

void Dispatcher::appendLastCommandJson(String& json) const {
  json += "\"lastCommand\":{";
  json += "\"seen\":";
  json += lastCommand_.seen ? "true" : "false";

  if (lastCommand_.seen) {
    json += ",\"id\":";
    json += String(lastCommand_.commandId);
    json += ",\"type\":\"";
    json += commandTypeToString(lastCommand_.type);
    json += "\",\"source\":\"";
    json += commandSourceToString(lastCommand_.source);
    json += "\",\"executedAtMs\":";
    json += String(lastCommand_.executedAtMs);
    json += ",\"ageMs\":";
    json += String(millis() - lastCommand_.executedAtMs);
    json += ",\"success\":";
    json += lastCommand_.success ? "true" : "false";
    json += ",\"message\":\"";
    appendEscaped(json, lastCommand_.message);
    json += "\"";
  }

  json += "}";
}

void Dispatcher::cancelBaseAngleIfNeeded(const Command& command) {
  if (angleController_ == nullptr || !angleController_->isActive()) {
    return;
  }

  switch (command.type) {
    case CommandType::STOP:
    case CommandType::DISARM:
      angleController_->cancel(AngleControllerStopReason::STATE_CHANGED, commandTypeToString(command.type));
      return;

    case CommandType::SEQUENCE_RUN:
    case CommandType::SEQUENCE_STOP:
    case CommandType::SEQUENCE_CLEAR:
      angleController_->cancel(AngleControllerStopReason::OVERRIDDEN, commandTypeToString(command.type));
      return;

    case CommandType::MOTOR_STOP_ALL:
    case CommandType::JOINT_STOP_ALL:
      angleController_->cancel(AngleControllerStopReason::MANUAL_STOP, commandTypeToString(command.type));
      return;

    case CommandType::MOTOR_RUN:
      if (command.motorId == MotorControl::MOTOR_5) {
        angleController_->cancel(AngleControllerStopReason::OVERRIDDEN, "base motor override");
      }
      return;

    case CommandType::MOTOR_STOP:
      if (command.motorId == MotorControl::MOTOR_5) {
        angleController_->cancel(command.type == CommandType::MOTOR_STOP
                                   ? AngleControllerStopReason::MANUAL_STOP
                                   : AngleControllerStopReason::OVERRIDDEN,
                                 "base motor override");
      }
      return;

    case CommandType::JOINT_RUN:
    case CommandType::JOINT_STOP:
      if (command.joint == MotionJoint::BASE) {
        angleController_->cancel(command.type == CommandType::JOINT_STOP
                                   ? AngleControllerStopReason::MANUAL_STOP
                                   : AngleControllerStopReason::OVERRIDDEN,
                                 "base joint override");
      }
      return;

    default:
      return;
  }
}

void Dispatcher::recordCommandResult(const Command& command, const CommandResult& result) {
  lastCommand_.seen = true;
  lastCommand_.commandId = result.commandId;
  lastCommand_.type = command.type;
  lastCommand_.source = command.source;
  lastCommand_.executedAtMs = millis();
  lastCommand_.success = result.success;
  strlcpy(lastCommand_.message, result.message, sizeof(lastCommand_.message));
}

bool Dispatcher::dispatchNext(CommandBus& commandBus, uint32_t* processedId, CommandResult* result) {
  Command command;
  if (!commandBus.dequeue(command)) {
    return false;
  }

  CommandResult localResult;
  execute(command, localResult);

  if (processedId != nullptr) {
    *processedId = command.id;
  }
  if (result != nullptr) {
    *result = localResult;
  }
  return true;
}

uint8_t Dispatcher::dispatchPending(CommandBus& commandBus, uint8_t maxCommands) {
  uint8_t processed = 0;
  while ((maxCommands == 0 || processed < maxCommands) && !commandBus.isEmpty()) {
    if (!dispatchNext(commandBus, nullptr, nullptr)) {
      break;
    }
    processed++;
  }
  return processed;
}

void Dispatcher::setResult(CommandResult& result, uint32_t commandId, bool success, const char* format, ...) const {
  result.commandId = commandId;
  result.success = success;

  va_list args;
  va_start(args, format);
  vsnprintf(result.message, sizeof(result.message), format, args);
  va_end(args);
}

bool Dispatcher::executeJointRun(MotionJoint joint, MotionDirection direction, uint8_t percent) {
  switch (joint) {
    case MotionJoint::GRIPPER:
      if (direction == MotionDirection::OPEN)  return robotArm_->gripperOpen(percent);
      if (direction == MotionDirection::CLOSE) return robotArm_->gripperClose(percent);
      break;

    case MotionJoint::WRIST:
      if (direction == MotionDirection::UP)   return robotArm_->wristUp(percent);
      if (direction == MotionDirection::DOWN) return robotArm_->wristDown(percent);
      break;

    case MotionJoint::ELBOW:
      if (direction == MotionDirection::UP)   return robotArm_->elbowUp(percent);
      if (direction == MotionDirection::DOWN) return robotArm_->elbowDown(percent);
      break;

    case MotionJoint::SHOULDER:
      if (direction == MotionDirection::UP)   return robotArm_->shoulderUp(percent);
      if (direction == MotionDirection::DOWN) return robotArm_->shoulderDown(percent);
      break;

    case MotionJoint::BASE:
      if (direction == MotionDirection::LEFT)  return robotArm_->baseLeft(percent);
      if (direction == MotionDirection::RIGHT) return robotArm_->baseRight(percent);
      break;
  }

  return false;
}

bool Dispatcher::executeJointStop(MotionJoint joint) {
  switch (joint) {
    case MotionJoint::GRIPPER:  return robotArm_->gripperStop();
    case MotionJoint::WRIST:    return robotArm_->wristStop();
    case MotionJoint::ELBOW:    return robotArm_->elbowStop();
    case MotionJoint::SHOULDER: return robotArm_->shoulderStop();
    case MotionJoint::BASE:     return robotArm_->baseStop();
    default:                    return false;
  }
}
