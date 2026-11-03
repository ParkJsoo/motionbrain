#pragma once
#include <stddef.h>
enum class ShoulderAngleStopReason { OVERRIDDEN };
class ShoulderAngleController { public: bool active = false; bool isActive() const { return active; }
  bool cancel(ShoulderAngleStopReason, const char*) { active = false; return true; }
  bool manualDirectionAllowed(bool, char*, size_t) const { return true; } };
