#pragma once
enum class AngleControllerStopReason { OVERRIDDEN };
class AngleController { public: bool active = false; bool isActive() const { return active; }
  bool cancel(AngleControllerStopReason, const char*) { active = false; return true; } };
