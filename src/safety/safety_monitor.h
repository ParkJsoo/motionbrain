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
  static constexpr float VIBRATION_FAULT_ENTER_THRESHOLD = 8.0f;
  static constexpr float VIBRATION_FAULT_CLEAR_THRESHOLD = 2.5f;
  static constexpr uint8_t VIBRATION_ENTER_SAMPLES = 2;
  static constexpr uint8_t VIBRATION_CLEAR_SAMPLES = 3;

  SafetyMonitor();

  void init(SystemStateManager* systemState, MotorControl* motorControl, MotionSequence* motionSequence);
  void update(const SensorSnapshot& snapshot);

  bool isMotionBlocked() const;
  SafetyBlockReason getBlockReason() const;
  const char* getBlockReasonString() const;
  bool hasLatchedFault() const;
  SafetyBlockReason getLatchedFaultReason() const;
  const char* getLatchedFaultReasonString() const;
  uint32_t getLastSafetyEventMs() const;

  static const char* reasonToString(SafetyBlockReason reason);

private:
  SystemStateManager* systemState_;
  MotorControl* motorControl_;
  MotionSequence* motionSequence_;
  bool blocked_;
  SafetyBlockReason blockReason_;
  bool latchedFault_;
  SafetyBlockReason latchedFaultReason_;
  bool vibrationActive_;
  uint8_t vibrationHighSamples_;
  uint8_t vibrationLowSamples_;
  uint32_t lastSafetyEventMs_;

  bool shouldForceImmediateStop() const;
  void triggerStop(const char* eventName, const char* details);
  void triggerFault(const char* eventName, const char* details);
};

#endif // SAFETY_MONITOR_H
