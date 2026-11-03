#pragma once
class AngleController {
public:
  bool ready = true;
  bool isReady() const { return ready; }
  bool active = false;
  bool isActive() const { return active; }
};
