#pragma once
enum class SystemState { BOOT, IDLE, ARMED, FAULT };
class SystemStateManager {
public:
  SystemState state = SystemState::IDLE;
  unsigned resets = 0;
  SystemState getState() const { return state; }
  const char* getStateString() const { return "fake-state"; }
  void resetTimeout() { ++resets; }
};
