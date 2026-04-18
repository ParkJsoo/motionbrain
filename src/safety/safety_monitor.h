#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include <Arduino.h>
#include <stdint.h>
#include "safety/sensor_snapshot.h"

class SystemStateManager;
class MotorControl;
class MotionSequence;

enum class SafetyBlockReason : uint8_t {
  NONE = 0,
  SENSOR_STALE,
  IMU_FAULT,
  RANGE_FAULT,
  OBSTACLE,
  VIBRATION
};

class SafetyMonitor {
public:
  static constexpr uint32_t SENSOR_STALE_MS = 1000;
  static constexpr float OBSTACLE_STOP_CM = 15.0f;
  static constexpr float VIBRATION_FAULT_THRESHOLD = 4.0f;

  SafetyMonitor();

  void init(SystemStateManager* systemState, MotorControl* motorControl, MotionSequence* motionSequence);
  void update(const SensorSnapshot& snapshot);

  bool isMotionBlocked() const;
  SafetyBlockReason getBlockReason() const;
  const char* getBlockReasonString() const;
  uint32_t getLastSafetyEventMs() const;

  static const char* reasonToString(SafetyBlockReason reason);

private:
  SystemStateManager* systemState_;
  MotorControl* motorControl_;
  MotionSequence* motionSequence_;
  bool blocked_;
  SafetyBlockReason blockReason_;
  uint32_t lastSafetyEventMs_;

  void triggerStop(const char* eventName, const char* details);
  void triggerFault(const char* eventName, const char* details);
};

#endif // SAFETY_MONITOR_H
