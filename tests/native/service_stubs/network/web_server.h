#pragma once
class MotionBrainWebServer {
public:
  bool active = true;
  bool manual = false;
  bool isActive() const { return active; }
  bool hasManualControlLeases() const { return manual; }
  unsigned clears = 0;
  void clearManualControlLeases() { ++clears; }
};
