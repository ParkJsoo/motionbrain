#pragma once
#if defined(MOTIONBRAIN_ENABLE_ELBOW_TIMER) && MOTIONBRAIN_ENABLE_ELBOW_TIMER
#include_next "system/system_init.h"
#else
enum class SystemState { BOOT, IDLE, ARMED, FAULT };
class SystemStateManager {
public:
  SystemState state = SystemState::IDLE;
  unsigned resets = 0;
  SystemState getState() const { return state; }
  const char* getStateString() const { return "fake-state"; }
  void resetTimeout() { ++resets; }
};
#endif
