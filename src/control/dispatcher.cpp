#include "control/dispatcher.h"

#include <stdarg.h>
#include <stdio.h>
#include "control/command_bus.h"
#include "control/safety_gate.h"
#include "debug/debug_log.h"
#include "motion/robot_arm.h"
#include "motion/motion_sequence.h"
#include "motor/motor_driver.h"
#include "peripheral/search_light.h"
#include "safety/safety_monitor.h"
#include "system/system_init.h"

extern SafetyMonitor safetyMonitor;

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
    case CommandType::SEQUENCE_ADD:            return "sequence add";
    case CommandType::SEQUENCE_RUN:            return "sequence run";
    case CommandType::SEQUENCE_STOP:           return "sequence stop";
    case CommandType::SEQUENCE_CLEAR:          return "sequence clear";
    case CommandType::LIGHT_ON:                return "light on";
    case CommandType::LIGHT_OFF:               return "light off";
    case CommandType::LIGHT_TOGGLE:            return "light toggle";
    default:                                   return "unknown";
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

} // namespace

Dispatcher::Dispatcher()
  : systemState_(nullptr)
  , motorControl_(nullptr)
  , robotArm_(nullptr)
  , motionSequence_(nullptr)
  , searchLight_(nullptr) {
  safetyGate_ = nullptr;
}

void Dispatcher::init(SystemStateManager* systemState,
                      MotorControl* motorControl,
                      RobotArm* robotArm,
                      MotionSequence* motionSequence,
                      SearchLight* searchLight,
                      SafetyGate* safetyGate) {
  systemState_ = systemState;
  motorControl_ = motorControl;
  robotArm_ = robotArm;
  motionSequence_ = motionSequence;
  searchLight_ = searchLight;
  safetyGate_ = safetyGate;
}

bool Dispatcher::isReady() const {
  return systemState_ != nullptr && motorControl_ != nullptr &&
         robotArm_ != nullptr && motionSequence_ != nullptr &&
         searchLight_ != nullptr && safetyGate_ != nullptr;
}

bool Dispatcher::execute(const Command& command, CommandResult& result) {
  if (!isReady()) {
    setResult(result, command.id, false, "Dispatcher not initialized");
    return false;
  }

  if (!safetyGate_->allows(command, result)) {
    DebugLog::command(commandTypeToString(command.type), false, result.message);
    return false;
  }

  bool success = false;

  switch (command.type) {
    case CommandType::ARM:
      success = systemState_->arm();
      setResult(result, command.id, success, success
        ? "System armed successfully"
        : "Failed to arm - check current state");
      break;

    case CommandType::DISARM:
      success = systemState_->disarm();
      setResult(result, command.id, success, success
        ? "System disarmed successfully"
        : "Failed to disarm - check current state");
      break;

    case CommandType::STOP: {
      SystemState previousState = systemState_->getState();
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
      success = command.forward
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

    case CommandType::SEQUENCE_ADD:
      success = motionSequence_->addCommand(command.joint, command.direction, command.percent, command.durationMs);
      if (success) {
        setResult(result, command.id, true, "Command added [%u/%u]",
                  motionSequence_->getTotalCount(), MotionSequence::MAX_COMMANDS);
      } else {
        setResult(result, command.id, false, "Add failed (queue full or invalid params)");
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

  if (success &&
      command.type != CommandType::ARM &&
      command.type != CommandType::DISARM &&
      command.type != CommandType::STOP &&
      systemState_ != nullptr) {
    systemState_->resetTimeout();
  }

  DebugLog::command(commandTypeToString(command.type), result.success, result.message);
  return result.success;
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
