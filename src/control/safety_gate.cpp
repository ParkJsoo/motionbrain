#include "control/safety_gate.h"

#include <stdio.h>
#include "safety/safety_monitor.h"
#include "system/system_init.h"

SafetyGate::SafetyGate()
  : systemState_(nullptr)
  , safetyMonitor_(nullptr) {
}

void SafetyGate::init(SystemStateManager* systemState, SafetyMonitor* safetyMonitor) {
  systemState_ = systemState;
  safetyMonitor_ = safetyMonitor;
}

bool SafetyGate::isReady() const {
  return systemState_ != nullptr && safetyMonitor_ != nullptr;
}

bool SafetyGate::allows(const Command& command, CommandResult& result) const {
  if (!isReady()) {
    deny(result, command.id, "SafetyGate not initialized");
    return false;
  }

  if (requiresMotionClearance(command.type) && safetyMonitor_->isMotionBlocked()) {
    char message[sizeof(result.message)];
    snprintf(message, sizeof(message), "Blocked by safety: %s", safetyMonitor_->getBlockReasonString());
    deny(result, command.id, message);
    return false;
  }

  if (requiresArmedState(command.type) && systemState_->getState() != SystemState::ARMED) {
    deny(result, command.id, "System must be ARMED");
    return false;
  }

  return true;
}

bool SafetyGate::requiresArmedState(CommandType type) const {
  switch (type) {
    case CommandType::MOTOR_RUN:
    case CommandType::JOINT_RUN:
    case CommandType::BASE_ANGLE_RUN:
    case CommandType::SHOULDER_ANGLE_RUN:
    case CommandType::SEQUENCE_RUN:
      return true;
    default:
      return false;
  }
}

bool SafetyGate::requiresMotionClearance(CommandType type) const {
  switch (type) {
    case CommandType::ARM:
    case CommandType::MOTOR_RUN:
    case CommandType::JOINT_RUN:
    case CommandType::BASE_ANGLE_RUN:
    case CommandType::SHOULDER_ANGLE_RUN:
    case CommandType::SEQUENCE_RUN:
      return true;
    default:
      return false;
  }
}

void SafetyGate::deny(CommandResult& result, uint32_t commandId, const char* message) const {
  result.commandId = commandId;
  result.success = false;
  strlcpy(result.message, message, sizeof(result.message));
}
