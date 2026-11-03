#pragma once
#include <stdint.h>
enum class GuardedRoutineExecutorState : uint8_t {
  IDLE, PREPARED, RUNNING, ABORT_REQUESTED, ABORTED, TIMED_OUT, COMPLETED, BLOCKED
};
struct GuardedRoutineExecutorStatus { GuardedRoutineExecutorState state; };
class GuardedRoutineExecutor {
public:
  static GuardedRoutineExecutorState state;
  static GuardedRoutineExecutorStatus status() { return {state}; }
};
