#pragma once
class SafetyMonitor {
public:
  bool blocked = false;
  bool isMotionBlocked() const { return blocked; }
  const char* getBlockReasonString() const { return "fake-safety-block"; }
};
