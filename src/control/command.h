#ifndef CONTROL_COMMAND_H
#define CONTROL_COMMAND_H

#include <Arduino.h>
#include <stdint.h>
#include "motion/motion_sequence.h"

enum class CommandType : uint8_t {
  ARM = 0,
  DISARM,
  STOP,
  MOTOR_RUN,
  MOTOR_STOP,
  MOTOR_STOP_ALL,
  MOTOR_SET_DEFAULT_SPEED,
  JOINT_RUN,
  JOINT_STOP,
  JOINT_STOP_ALL,
  SEQUENCE_ADD,
  SEQUENCE_RUN,
  SEQUENCE_STOP,
  SEQUENCE_CLEAR,
  LIGHT_ON,
  LIGHT_OFF,
  LIGHT_TOGGLE
};

enum class CommandSource : uint8_t {
  SERIAL_INPUT = 0,
  WEB_INPUT = 1,
  INTERNAL = 2
};

struct Command {
  uint32_t        id;
  CommandType     type;
  CommandSource   source;
  uint32_t        createdAtMs;
  uint8_t         motorId;
  bool            forward;
  uint8_t         percent;
  uint8_t         speed;
  MotionJoint     joint;
  MotionDirection direction;
  uint32_t        durationMs;

  Command()
    : id(0)
    , type(CommandType::STOP)
    , source(CommandSource::INTERNAL)
    , createdAtMs(0)
    , motorId(0)
    , forward(true)
    , percent(0)
    , speed(0)
    , joint(MotionJoint::GRIPPER)
    , direction(MotionDirection::OPEN)
    , durationMs(0) {
  }
};

struct CommandResult {
  uint32_t commandId;
  bool     success;
  char     message[96];

  CommandResult()
    : commandId(0)
    , success(false)
    , message{0} {
  }
};

#endif // CONTROL_COMMAND_H
