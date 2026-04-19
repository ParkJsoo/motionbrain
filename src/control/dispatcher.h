#ifndef CONTROL_DISPATCHER_H
#define CONTROL_DISPATCHER_H

#include <Arduino.h>
#include <stdint.h>
#include "control/command.h"

class SystemStateManager;
class MotorControl;
class RobotArm;
class MotionSequence;
class SearchLight;
class CommandBus;
class SafetyGate;

class Dispatcher {
public:
  Dispatcher();

  void init(SystemStateManager* systemState,
            MotorControl* motorControl,
            RobotArm* robotArm,
            MotionSequence* motionSequence,
            SearchLight* searchLight,
            SafetyGate* safetyGate);

  bool isReady() const;
  bool execute(const Command& command, CommandResult& result);
  bool dispatchNext(CommandBus& commandBus, uint32_t* processedId = nullptr, CommandResult* result = nullptr);
  uint8_t dispatchPending(CommandBus& commandBus, uint8_t maxCommands = 0);

private:
  SystemStateManager* systemState_;
  MotorControl*       motorControl_;
  RobotArm*           robotArm_;
  MotionSequence*     motionSequence_;
  SearchLight*        searchLight_;
  SafetyGate*         safetyGate_;

  bool hasCoreDependencies() const;
  bool hasDependenciesFor(CommandType type, const char** missingDependency) const;
  bool commandExtendsTimeout(CommandType type) const;
  void setResult(CommandResult& result, uint32_t commandId, bool success, const char* format, ...) const;
  bool executeJointRun(MotionJoint joint, MotionDirection direction, uint8_t percent);
  bool executeJointStop(MotionJoint joint);
};

#endif // CONTROL_DISPATCHER_H
