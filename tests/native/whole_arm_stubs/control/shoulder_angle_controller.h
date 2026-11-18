#pragma once
class ShoulderAngleController {
public:
  bool isActive() const { return false; }
  bool manualDirectionAllowed(bool) const { return true; }
};
