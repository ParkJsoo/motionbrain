#ifndef CONTROL_ANGLE_CONTROLLER_H
#define CONTROL_ANGLE_CONTROLLER_H

#include <Arduino.h>
#include <stdint.h>
#include "motion/motion_sequence.h"
#include "safety/sensor_snapshot.h"

class SystemStateManager;
class RobotArm;
class SafetyMonitor;

enum class AngleControllerStopReason : uint8_t {
  NONE = 0,
  TARGET_REACHED,
  TIMEOUT,
  NO_ROTATION_FEEDBACK,
  SENSOR_BLOCK,
  STATE_CHANGED,
  MANUAL_STOP,
  OVERRIDDEN,
  START_FAILED
};

class AngleController {
public:
  static constexpr uint8_t DEFAULT_SPEED = 40;
  static constexpr float MIN_TARGET_DEGREES = 3.0f;
  static constexpr float MAX_TARGET_DEGREES = 180.0f;
  static constexpr float TARGET_TOLERANCE_DEGREES = 3.0f;
  static constexpr uint32_t BASE_TIMEOUT_MS = 3000;
  static constexpr uint32_t TIMEOUT_PER_DEGREE_MS = 100;
  static constexpr uint32_t FEEDBACK_CHECK_DELAY_MS = 1500;
  static constexpr float MIN_PROGRESS_FOR_FEEDBACK_DEGREES = 1.5f;
  static constexpr float MIN_ROTATION_RATE_DPS = 2.0f;
  static constexpr bool GYRO_Z_LEFT_IS_POSITIVE = true;

  AngleController();

  void init(SystemStateManager* systemState, RobotArm* robotArm, SafetyMonitor* safetyMonitor);
  void update(const SensorSnapshot& snapshot);

  bool isReady() const;
  bool isActive() const;

  bool startRelative(MotionDirection direction, float targetDegrees, uint8_t percent,
                     char* message, size_t messageSize);
  bool cancel(AngleControllerStopReason reason, const char* detail = nullptr);

  MotionDirection getDirection() const;
  const char* getDirectionString() const;
  float getTargetDegrees() const;
  float getAccumulatedDegrees() const;
  float getRemainingDegrees() const;
  uint8_t getPercent() const;
  uint32_t getElapsedMs() const;
  uint32_t getTimeoutMs() const;
  uint32_t getLastTransitionMs() const;
  uint32_t getProcessedSamples() const;
  float getLastRateDegreesPerSecond() const;
  AngleControllerStopReason getLastStopReason() const;
  const char* getLastStopReasonString() const;

  static const char* stopReasonToString(AngleControllerStopReason reason);

private:
  SystemStateManager* systemState_;
  RobotArm* robotArm_;
  SafetyMonitor* safetyMonitor_;
  bool active_;
  MotionDirection direction_;
  float targetDegrees_;
  float accumulatedSignedDegrees_;
  uint8_t percent_;
  uint32_t startedAtMs_;
  uint32_t timeoutMs_;
  uint32_t lastTransitionMs_;
  uint32_t lastSampleStampMs_;
  uint32_t processedSamples_;
  bool hasSample_;
  float lastRateDegreesPerSecond_;
  AngleControllerStopReason lastStopReason_;

  bool extractSampleStampMs(const SensorSnapshot& snapshot, uint32_t& stampMs) const;
  float getSignedTargetDegrees() const;
  float getSignedCurrentDegrees() const;
  float getSignedRateDegreesPerSecond(const SensorSnapshot& snapshot) const;
  uint32_t computeTimeoutMs(float targetDegrees) const;
  bool hasReachedTarget() const;
  bool hasRotationFeedback() const;
  void stopInternal(AngleControllerStopReason reason, const char* detail);
};

#endif // CONTROL_ANGLE_CONTROLLER_H
