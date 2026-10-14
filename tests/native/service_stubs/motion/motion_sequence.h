#pragma once
#include <stdint.h>
enum class MotionJoint : uint8_t { GRIPPER = 1 };
enum class MotionDirection : uint8_t { OPEN = 0 };
enum class SequenceState : uint8_t { IDLE, RUNNING };
class MotionSequence {
public:
  bool running = false;
  SequenceState getState() const { return running ? SequenceState::RUNNING : SequenceState::IDLE; }
};
