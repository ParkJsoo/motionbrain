#pragma once
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include "motor/motor_driver.h"
extern MotorControl motorControl;
enum class ShoulderAngleStopReason { NONE, MANUAL_STOP, TARGET_REACHED, SENSOR_ERROR };
class ShoulderAngleController {
public:
  bool active = false;
  unsigned starts = 0;
  unsigned cancels = 0;
  bool failCancel = false;
  float target = 0;
  uint8_t percent = 0;
  ShoulderAngleStopReason reason = ShoulderAngleStopReason::NONE;
  bool isActive() const { return active; }
  bool startAbsolute(float degrees, uint8_t speed, char*, size_t) {
    ++starts;
    target = degrees;
    percent = speed;
    // Fake controller boundary only; real sensor/calibration logic is not exercised.
    if (!isfinite(degrees) || degrees < 0 || degrees > 180) return false;
    active = motorControl.setSpeed(4, 20);
    return active;
  }
  void cancel(ShoulderAngleStopReason why, const char*) {
    ++cancels;
    reason = why;
    if (!failCancel) active = false;
  }
  ShoulderAngleStopReason getLastStopReason() const { return reason; }
};
