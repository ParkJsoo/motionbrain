#pragma once
class TeleopAdapter {
public:
  bool active = false;
  bool held = false;
  unsigned resets = 0;
  bool isControlActive() const { return active; }
  bool isDeadmanHeld() const { return held; }
  void resetOutputCache() { ++resets; }
};
