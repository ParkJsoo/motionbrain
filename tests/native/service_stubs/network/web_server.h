#pragma once
class MotionBrainWebServer {
public:
  unsigned clears = 0;
  void clearManualControlLeases() { ++clears; }
};
