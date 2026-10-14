#pragma once
#include "control/command.h"
class SafetyGate {
public:
  bool allowed = true;
  mutable unsigned calls = 0;
  bool allows(const Command& command, CommandResult&) const {
    ++calls;
    return command.type == CommandType::MOTOR_RUN && allowed;
  }
};
