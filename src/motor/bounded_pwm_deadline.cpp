// The main loop and the ESP timer task share only the PWM cutoff. State
// reconciliation and output journal publication stay on the main loop.
#if MOTIONBRAIN_DUAL_VIEW_TIMER
#include "motor_driver.h"
#include <esp_timer.h>
#if MOTIONBRAIN_ENABLE_WHOLE_ARM_POLICY || MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER
#error "Dual-view PWM deadline must own the motor driver's PWM write path"
#endif
namespace {
constexpr uint32_t CONTROL_LEASE_MS = 150;
struct Deadline {
  SemaphoreHandle_t mutex = nullptr;
  esp_timer_handle_t timer = nullptr;
  uint32_t start = 0, duration = 0, lastService = 0, zeroAt = 0;
  uint8_t motor = 0, channel = 0;
  bool expired = false, stalled = false, pending = false;
} guard;

void expireLocked() {
  if (!guard.motor || guard.expired) return;
  const uint32_t now = millis();
  const uint32_t age = now - guard.start;
  const uint32_t sinceService = now - guard.lastService;
  if (age < guard.duration && sinceService < CONTROL_LEASE_MS) return;
  // Preserve whichever deadline was first, including after millis rollover.
  guard.stalled = sinceService >= CONTROL_LEASE_MS &&
    (age < guard.duration || sinceService - CONTROL_LEASE_MS > age - guard.duration);
  ledcWrite(guard.channel, 0);
  guard.zeroAt = millis();
  guard.expired = guard.pending = true;
}
void timerTick(void*) {
  xSemaphoreTake(guard.mutex, portMAX_DELAY);
  expireLocked();
  xSemaphoreGive(guard.mutex);
}
}

bool MotorControl::initBoundedDeadline() {
  if (guard.timer) return true;
  if (!guard.mutex) guard.mutex = xSemaphoreCreateMutex();
  if (!guard.mutex) return false;
  esp_timer_create_args_t cfg{};
  cfg.callback = timerTick;
  cfg.name = "mb_dual_cutoff";
  if (esp_timer_create(&cfg, &guard.timer) != ESP_OK) return false;
  if (esp_timer_start_periodic(guard.timer, 1000) == ESP_OK) return true;
  esp_timer_delete(guard.timer);
  guard.timer = nullptr;
  return false;
}

bool MotorControl::armBoundedDeadline(uint8_t motor, uint32_t startMs, uint32_t durationMs) {
  if (!guard.timer || !isValidMotorId(motor)) return false;
  xSemaphoreTake(guard.mutex, portMAX_DELAY);
  const bool available = !guard.motor && (millis() - startMs < durationMs);
  if (available) {
    guard.motor = motor;
    guard.channel = PWM_CHANNELS[motor-1];
    guard.start = guard.lastService = startMs;
    guard.duration = durationMs;
    guard.stalled = guard.expired = guard.pending = false;
  }
  xSemaphoreGive(guard.mutex);
  return available;
}

bool MotorControl::serviceBoundedDeadline() {
  xSemaphoreTake(guard.mutex, portMAX_DELAY);
  expireLocked();
  const bool healthy = !guard.stalled;
  if (!guard.expired) guard.lastService = millis();
  xSemaphoreGive(guard.mutex);
  return healthy;
}

void MotorControl::publishBoundedCutoff() {
  // All callers hold guard.mutex and run on the control thread.
  if (guard.pending) {
    outputLog_.append(guard.zeroAt, actionOrdinal_, guard.motor, 0, bounded::OutputReason::ForcedOff);
    guard.pending = false;
  }
}

bool MotorControl::writeBoundedPwm(uint8_t motor, int16_t speed, bounded::OutputReason reason) {
  if (guard.mutex) xSemaphoreTake(guard.mutex, portMAX_DELAY);
  expireLocked();
  publishBoundedCutoff();
  const bool allowed = !speed || !guard.motor || (motor == guard.motor && !guard.expired);
  if (allowed) {
    ledcWrite(PWM_CHANNELS[motor-1], static_cast<uint8_t>(speed < 0 ? -speed : speed));
    recordActualWrite(motor, speed, reason, actionOrdinal_);
  }
  if (guard.mutex) xSemaphoreGive(guard.mutex);
  return allowed;
}

void MotorControl::releaseBoundedDeadline() {
  xSemaphoreTake(guard.mutex, portMAX_DELAY);
  publishBoundedCutoff();
  guard.motor = 0;
  xSemaphoreGive(guard.mutex);
}

const bounded::AppliedOutputLog& MotorControl::outputLog() const {
  if (guard.mutex) {
    xSemaphoreTake(guard.mutex, portMAX_DELAY);
    const_cast<MotorControl*>(this)->publishBoundedCutoff();
    xSemaphoreGive(guard.mutex);
  }
  return outputLog_;
}
#endif
