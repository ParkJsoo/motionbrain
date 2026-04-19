#ifndef CONTROL_SAFETY_GATE_H
#define CONTROL_SAFETY_GATE_H

#include <Arduino.h>
#include <stdint.h>
#include "control/command.h"

class SystemStateManager;
class SafetyMonitor;

class SafetyGate {
public:
  SafetyGate();

  void init(SystemStateManager* systemState, SafetyMonitor* safetyMonitor);
  bool isReady() const;
  bool allows(const Command& command, CommandResult& result) const;

private:
  SystemStateManager* systemState_;
  SafetyMonitor* safetyMonitor_;

  bool requiresArmedState(CommandType type) const;
  bool requiresMotionClearance(CommandType type) const;
  void deny(CommandResult& result, uint32_t commandId, const char* message) const;
};

#endif // CONTROL_SAFETY_GATE_H
