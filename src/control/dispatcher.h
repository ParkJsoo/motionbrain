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
class AngleController;
class ShoulderAngleController;

struct DispatcherCommandAudit {
  bool seen;
  uint32_t commandId;
  CommandType type;
  CommandSource source;
  uint32_t executedAtMs;
  bool success;
  char message[96];

  DispatcherCommandAudit()
    : seen(false)
    , commandId(0)
    , type(CommandType::STOP)
    , source(CommandSource::INTERNAL)
    , executedAtMs(0)
    , success(false)
    , message{0} {
  }
};

class Dispatcher {
public:
  Dispatcher();

  void init(SystemStateManager* systemState,
            MotorControl* motorControl,
            RobotArm* robotArm,
            MotionSequence* motionSequence,
            SearchLight* searchLight,
            SafetyGate* safetyGate,
            AngleController* angleController,
            ShoulderAngleController* shoulderAngleController);

  bool isReady() const;
  bool execute(const Command& command, CommandResult& result);
  bool dispatchNext(CommandBus& commandBus, uint32_t* processedId = nullptr, CommandResult* result = nullptr);
  uint8_t dispatchPending(CommandBus& commandBus, uint8_t maxCommands = 0);
  DispatcherCommandAudit lastCommandAudit() const;
  void appendLastCommandJson(String& json) const;

private:
  SystemStateManager* systemState_;
  MotorControl*       motorControl_;
  RobotArm*           robotArm_;
  MotionSequence*     motionSequence_;
  SearchLight*        searchLight_;
  SafetyGate*         safetyGate_;
  AngleController*    angleController_;
  ShoulderAngleController* shoulderAngleController_;
  DispatcherCommandAudit lastCommand_;

  bool hasCoreDependencies() const;
  bool hasDependenciesFor(CommandType type, const char** missingDependency) const;
  bool commandExtendsTimeout(CommandType type) const;
  void cancelBaseAngleIfNeeded(const Command& command);
  void cancelShoulderAngleIfNeeded(const Command& command);
  void recordCommandResult(const Command& command, const CommandResult& result);
  void setResult(CommandResult& result, uint32_t commandId, bool success, const char* format, ...) const;
  bool executeJointRun(MotionJoint joint, MotionDirection direction, uint8_t percent);
  bool executeJointStop(MotionJoint joint);
};

#endif // CONTROL_DISPATCHER_H
