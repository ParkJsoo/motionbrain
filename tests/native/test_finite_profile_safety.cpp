// Production system state, safety evaluation and motor driver, fake Arduino IO.
#include "motor/motor_driver.h"
#include "safety/safety_monitor.h"
#include "motion/motion_sequence.h"
#include "control/event_log.h"
#include <assert.h>
#include <string.h>
#include <vector>

SystemStateManager systemState;
MotorControl motorControl;
SafetyMonitor safetyMonitor;
MotionSequence motionSequence;
EventLog eventLog;
static uint32_t nowMs = 100;
static uint8_t pins[40] = {};
struct Write { uint8_t channel; uint32_t duty; SystemState state; };
static std::vector<Write> writes;
uint32_t millis() { return nowMs; }
void pinMode(uint8_t pin, uint8_t mode) { assert(pin < 40 && mode == OUTPUT); }
void digitalWrite(uint8_t pin, uint8_t value) { assert(pin < 40); pins[pin] = value; }
void ledcSetup(uint8_t channel, uint32_t frequency, uint8_t resolution) {
  assert(channel < 5 && frequency == 1000 && resolution == 8);
}
void ledcAttachPin(uint8_t pin, uint8_t channel) { assert(pin < 40 && channel < 5); }
void ledcWrite(uint8_t channel, uint32_t duty) {
  assert(channel < 5 && duty <= 255);
  writes.push_back({channel, duty, systemState.getState()});
}
static void initialize() {
  assert(motorControl.init());
  safetyMonitor.init(&systemState, &motorControl, &motionSequence);
  assert(systemState.transitionTo(SystemState::IDLE));
  assert(systemState.arm());
}
static void legacy() {
  initialize();
  assert(motorControl.setSpeed(2, 30)); motorControl.update();
  assert(motorControl.getSpeed(2) == 10);
  const size_t count = writes.size();
  assert(systemState.disarm());
  // A feature-enabled but unowned driver preserves the existing loop policy.
  assert(writes.size() == count && motorControl.getSpeed(2) == 10);
  motorControl.update();
  assert(!motorControl.hasPendingOutput() && motorControl.getSpeed(2) == 0);
}

#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER
static uint32_t start(bool running = true) {
  initialize();
  uint32_t lease = 0, at = 0;
  assert(!motorControl.finiteProfileOwned());
  assert(motorControl.acquireFiniteProfileLease(17, lease));
  assert(motorControl.finiteProfileOwned());
  if (running) assert(motorControl.applyFiniteProfile(lease, 2, 80, at));
  return lease;
}
static void verifyOldLeaseBlocked(uint32_t lease) {
  uint32_t at = 0;
  assert(motorControl.finiteProfileOwned());
  assert(motorControl.finiteProfileLeaseSealed(lease));
  assert(!motorControl.hasPendingOutput());
  const size_t count = writes.size();
  assert(!motorControl.applyFiniteProfile(lease, 2, 80, at));
  assert(!motorControl.setSpeed(2, 40));
  nowMs += 100; motorControl.update();
  for (size_t i = count; i < writes.size(); ++i) assert(writes[i].duty == 0);
}
static void disarm(bool running) {
  const uint32_t lease = start(running);
  const size_t count = writes.size();
  assert(systemState.disarm());
  assert(writes.size() == count + MotorControl::NUM_MOTORS);
  // Hook executes before the visible state transition, with no intervening tick.
  for (size_t i = count; i < writes.size(); ++i) {
    assert(writes[i].duty == 0 && writes[i].state == SystemState::ARMED);
  }
  assert(motorControl.getSpeed(2) == 0);
  assert(systemState.arm());
  verifyOldLeaseBlocked(lease);
  assert(motorControl.releaseFiniteProfileLease(lease));
  assert(!motorControl.finiteProfileOwned());
  uint32_t fresh = 0, at = 0;
  assert(motorControl.acquireFiniteProfileLease(18, fresh) && fresh > lease);
  assert(!motorControl.applyFiniteProfile(lease, 2, 80, at));
  assert(motorControl.applyFiniteProfile(fresh, 2, 40, at));
}
static void fault() {
  const uint32_t lease = start();
  assert(systemState.transitionTo(SystemState::FAULT));
  assert(motorControl.getSpeed(2) == 0);
  assert(systemState.enterSafe() && systemState.arm());
  verifyOldLeaseBlocked(lease);
}
static void timeout(bool wrap) {
  const uint32_t lease = start();
  if (wrap) nowMs = UINT32_MAX - 20;
  systemState.setTimeout(30); systemState.resetTimeout(); nowMs += 30;
  systemState.update();
  assert(systemState.getState() == SystemState::IDLE);
  assert(motorControl.getSpeed(2) == 0);
  assert(systemState.arm()); verifyOldLeaseBlocked(lease);
}
static SensorSnapshot healthy() {
  SensorSnapshot snapshot;
  snapshot.connected = snapshot.imuOk = snapshot.rangeOk = true;
  snapshot.lastUpdateMs = nowMs;
  snapshot.distanceCm = 50;
  return snapshot;
}
static void blockClear() {
  const uint32_t lease = start();
  SensorSnapshot snapshot = healthy(); snapshot.distanceCm = 10;
  safetyMonitor.update(snapshot);
  assert(safetyMonitor.isMotionBlocked() && motorControl.getSpeed(2) == 0);
  snapshot.distanceCm = 50; safetyMonitor.update(snapshot);
  assert(!safetyMonitor.isMotionBlocked());
  assert(systemState.arm()); verifyOldLeaseBlocked(lease);
}
static void vibration() {
  const uint32_t lease = start();
  SensorSnapshot snapshot = healthy(); snapshot.vibe = 9;
  safetyMonitor.update(snapshot); safetyMonitor.update(snapshot);
  assert(systemState.getState() == SystemState::FAULT && motorControl.getSpeed(2) == 0);
  snapshot.vibe = 0;
  for (unsigned i = 0; i < 3; ++i) safetyMonitor.update(snapshot);
  assert(systemState.enterSafe()); safetyMonitor.update(snapshot);
  assert(!safetyMonitor.isMotionBlocked() && systemState.arm());
  verifyOldLeaseBlocked(lease);
}
static void invalidAndSameState() {
  const uint32_t lease = start();
  const size_t count = writes.size();
  assert(!systemState.transitionTo(SystemState::BOOT));
  assert(systemState.transitionTo(SystemState::ARMED));
  assert(writes.size() == count && motorControl.finiteProfileAllowed(lease));
}
static void foreignManager() {
  const uint32_t lease = start();
  SystemStateManager unrelated;
  const size_t count = writes.size();
  assert(unrelated.transitionTo(SystemState::IDLE));
  assert(unrelated.arm() && unrelated.disarm());
  assert(writes.size() == count && motorControl.finiteProfileAllowed(lease));
}
#endif

int main(int argc, char** argv) {
  assert(argc == 2);
  if (strcmp(argv[1], "legacy") == 0) legacy();
#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER
  else if (strcmp(argv[1], "reserved") == 0) disarm(false);
  else if (strcmp(argv[1], "running") == 0) disarm(true);
  else if (strcmp(argv[1], "fault") == 0) fault();
  else if (strcmp(argv[1], "timeout") == 0) timeout(false);
  else if (strcmp(argv[1], "wrap") == 0) timeout(true);
  else if (strcmp(argv[1], "block_clear") == 0) blockClear();
  else if (strcmp(argv[1], "vibration") == 0) vibration();
  else if (strcmp(argv[1], "invalid_same") == 0) invalidAndSameState();
  else if (strcmp(argv[1], "foreign") == 0) foreignManager();
#endif
  else return 2;
}
