#ifndef CONTROL_SHOULDER_ANGLE_CONTROLLER_H
#define CONTROL_SHOULDER_ANGLE_CONTROLLER_H

#include <Arduino.h>
#include <stdint.h>

class SystemStateManager;
class RobotArm;
class MotorControl;
class SafetyMonitor;
class ShoulderAngleSensor;

enum class ShoulderAngleStopReason : uint8_t {
  NONE = 0,
  TARGET_REACHED,
  TARGET_MISSED,
  TIMEOUT,
  SENSOR_FAULT,
  SAFETY_BLOCK,
  STATE_CHANGED,
  NO_PROGRESS,
  MANUAL_STOP,
  OVERRIDDEN,
  START_FAILED,
  SOFT_LIMIT
};

/**
 * Conservative absolute-angle controller for the M4 shoulder joint.
 *
 * The initial soft limits intentionally cover only the range proven on the
 * physical arm. They must be expanded only after a supervised full-range
 * calibration.
 */
class ShoulderAngleController {
public:
  static constexpr float SOFT_MIN_DEGREES = 230.0f;
  static constexpr float SOFT_MAX_DEGREES = 245.0f;
  static constexpr float TARGET_TOLERANCE_DEGREES = 0.50f;
  static constexpr float STOP_THRESHOLD_WINDOW_DEGREES = 0.35f;
  static constexpr float SLOW_ZONE_DEGREES = 1.5f;
  static constexpr float UP_STOP_LEAD_DEGREES = 0.90f;
  static constexpr float DOWN_STOP_LEAD_DEGREES = 1.50f;
  static constexpr uint32_t SETTLE_TIME_MS = 600;
  static constexpr float CORRECTION_CUTOFF_ERROR_DEGREES = 0.20f;
  static constexpr uint8_t UP_CORRECTION_PERCENT = 75;
  static constexpr uint8_t DOWN_CORRECTION_PERCENT = 35;
  static constexpr uint32_t UP_CORRECTION_PULSE_MS = 250;
  static constexpr uint32_t DOWN_CORRECTION_PULSE_MS = 250;
  static constexpr uint8_t MAX_CORRECTION_ATTEMPTS = 4;
  static constexpr uint8_t DEFAULT_PERCENT = 100;
  static constexpr uint8_t MIN_DRIVE_PERCENT = 75;
  static constexpr uint8_t SLOW_PERCENT = 75;
  static constexpr uint32_t SENSOR_STALE_MS = 150;
  static constexpr uint32_t COMMAND_TIMEOUT_MS = 7000;
  static constexpr uint32_t PROGRESS_GRACE_MS = 900;
  static constexpr uint32_t PROGRESS_TIMEOUT_MS = 900;
  static constexpr float MIN_PROGRESS_DEGREES = 0.20f;

  ShoulderAngleController();

  void init(SystemStateManager* systemState,
            RobotArm* robotArm,
            MotorControl* motorControl,
            SafetyMonitor* safetyMonitor,
            ShoulderAngleSensor* sensor);
  void update();

  bool isReady() const;
  bool isActive() const;
  bool startAbsolute(float targetDegrees, uint8_t percent,
                     char* message, size_t messageSize);
  bool cancel(ShoulderAngleStopReason reason, const char* detail = nullptr);
  bool manualDirectionAllowed(bool directionUp,
                              char* message = nullptr,
                              size_t messageSize = 0) const;
  void appendShoulderStatusJson(String& json) const;

  float getTargetDegrees() const;
  float getCurrentDegrees() const;
  float getErrorDegrees() const;
  float getStartDegrees() const;
  float getFinalErrorDegrees() const;
  uint8_t getRequestedPercent() const;
  uint8_t getAppliedPercent() const;
  uint32_t getElapsedMs() const;
  uint32_t getProcessedSamples() const;
  ShoulderAngleStopReason getLastStopReason() const;
  const char* getLastStopReasonString() const;

  static const char* stopReasonToString(ShoulderAngleStopReason reason);

private:
  SystemStateManager* systemState_;
  RobotArm* robotArm_;
  MotorControl* motorControl_;
  SafetyMonitor* safetyMonitor_;
  ShoulderAngleSensor* sensor_;
  bool active_;
  bool settling_;
  bool correcting_;
  bool directionUp_;
  float targetDegrees_;
  float stopThresholdDegrees_;
  float startDegrees_;
  float currentDegrees_;
  float finalErrorDegrees_;
  float progressReferenceDegrees_;
  uint8_t requestedPercent_;
  uint8_t appliedPercent_;
  uint32_t startedAtMs_;
  uint32_t settleStartedAtMs_;
  uint32_t correctionStartedAtMs_;
  uint32_t lastProgressMs_;
  uint32_t lastSensorUpdateMs_;
  uint32_t processedSamples_;
  uint8_t correctionAttempts_;
  ShoulderAngleStopReason lastStopReason_;
  bool manualGuardBlocked_;

  bool sensorAllowsMotion() const;
  uint8_t correctionPercent() const;
  uint32_t correctionPulseMs() const;
  bool applyDrive(uint8_t percent);
  void enforceManualDriveLimits();
  void beginSettling();
  bool beginCorrection(uint32_t now);
  void stopInternal(ShoulderAngleStopReason reason, const char* detail);
};

#endif // CONTROL_SHOULDER_ANGLE_CONTROLLER_H
