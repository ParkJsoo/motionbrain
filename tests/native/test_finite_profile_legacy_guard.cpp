// Compiles the production motor driver. Only Arduino IO, clock, safety and
// system-state dependencies are substituted; no electrical feedback is claimed.
#include "motor/motor_driver.h"
#include "safety/safety_monitor.h"
#include <assert.h>
#include <string.h>
#include <vector>

MotorControl motorControl;
SystemStateManager systemState;
SafetyMonitor safetyMonitor;
static uint32_t nowMs = 0;
static uint8_t pins[40] = {};
struct Write { uint32_t time; uint8_t motor; int16_t signedPwm; };
static std::vector<Write> writes;
static const uint8_t forwardPins[5] = {16, 19, 23, 27, 4};
static const uint8_t reversePins[5] = {17, 21, 25, 32, 13};
uint32_t millis() { return nowMs; }
void pinMode(uint8_t pin, uint8_t mode) { assert(pin < 40 && mode == OUTPUT); }
void digitalWrite(uint8_t pin, uint8_t value) { assert(pin < 40); pins[pin] = value; }
void ledcSetup(uint8_t channel, uint32_t frequency, uint8_t resolution) {
  assert(channel < 5 && frequency == 1000 && resolution == 8);
}
void ledcAttachPin(uint8_t pin, uint8_t channel) { assert(pin < 40 && channel < 5); }
void ledcWrite(uint8_t channel, uint32_t duty) {
  assert(channel < 5 && duty <= 255);
  int16_t signedPwm = static_cast<int16_t>(duty);
  if (duty) {
    assert(pins[forwardPins[channel]] != pins[reversePins[channel]]);
    if (pins[reversePins[channel]]) signedPwm = -signedPwm;
  }
  writes.push_back({nowMs, static_cast<uint8_t>(channel + 1), signedPwm});
}

static void assertMatches(const MotorControl& driver) {
  const auto& log = driver.outputLog();
  assert(log.count() + log.dropped() == writes.size());
  for (size_t i = 0; i < log.count(); ++i) {
    const auto* entry = log.at(i);
    const auto& write = writes[i + log.dropped()];
    assert(entry && entry->sequence == i + log.dropped() + 1);
    assert(entry->atMs == write.time && entry->motor == write.motor);
    assert(entry->pwm == write.signedPwm);
  }
  assert(log.at(log.count()) == nullptr);
}

#include "control/command.h"
#include "control/command_bus.h"
#include "control/finite_profile_legacy_guard.h"

static void disabled() {
  CommandBus bus;
  for (unsigned i = 0; i <= static_cast<unsigned>(CommandType::LIGHT_TOGGLE); ++i)
    assert(FiniteProfileLegacy::allowed(static_cast<CommandType>(i)));
  Command command; command.type = CommandType::MOTOR_RUN;
  assert(bus.enqueue(command) && bus.size() == 1);
  assert(writes.size() == 5);
  assertMatches(motorControl);
}
#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER
static uint32_t acquire() {
  uint32_t lease = 0;
  assert(motorControl.acquireFiniteProfileLease(42, lease));
  return lease;
}
static void readonly_and_movement() {
  const uint32_t lease = acquire();
  const CommandType safe[] = {CommandType::LIGHT_ON, CommandType::LIGHT_OFF,
      CommandType::LIGHT_TOGGLE, CommandType::ROUTINE_DRY_RUN};
  for (auto type : safe) assert(FiniteProfileLegacy::allowed(type));
  assert(!motorControl.finiteProfileLeaseSealed(lease) && writes.size() == 5);
  uint32_t at = 0; assert(motorControl.applyFiniteProfile(lease, 2, 90, at));
  const CommandType competing[] = {CommandType::ARM, CommandType::MOTOR_RUN,
      CommandType::MOTOR_SET_DEFAULT_SPEED, CommandType::JOINT_RUN,
      CommandType::BASE_ANGLE_RUN, CommandType::SHOULDER_ANGLE_RUN,
      CommandType::SEQUENCE_ADD, CommandType::SEQUENCE_RUN, CommandType::ROUTINE_RUN};
  assert(!FiniteProfileLegacy::allowed(competing[0]));
  assert(motorControl.finiteProfileLeaseSealed(lease) && !motorControl.hasPendingOutput());
  const size_t afterFirst = writes.size();
  for (auto type : competing) assert(!FiniteProfileLegacy::allowed(type));
  assert(writes.size() == afterFirst); // Repeat guards do not repeatedly write OFF.
  for (auto type : safe) assert(FiniteProfileLegacy::allowed(type));
  assert(motorControl.finiteProfileOwned());
  assertMatches(motorControl);
}
static void stop_paths() {
  const CommandType stop[] = {CommandType::STOP, CommandType::DISARM,
      CommandType::MOTOR_STOP, CommandType::MOTOR_STOP_ALL, CommandType::JOINT_STOP,
      CommandType::JOINT_STOP_ALL, CommandType::SEQUENCE_STOP,
      CommandType::SEQUENCE_CLEAR, CommandType::ROUTINE_ABORT};
  for (auto type : stop) {
    const uint32_t lease = acquire(); uint32_t at = 0;
    assert(motorControl.applyFiniteProfile(lease, 2, 90, at));
    assert(FiniteProfileLegacy::allowed(type));
    assert(motorControl.finiteProfileLeaseSealed(lease) && !motorControl.hasPendingOutput());
    assert(motorControl.finiteProfileOwned());
    assert(motorControl.releaseFiniteProfileLease(lease));
  }
  assertMatches(motorControl);
}
static void queue_rejection() {
  const uint32_t lease = acquire(); CommandBus bus;
  Command command; command.type = CommandType::SEQUENCE_RUN;
  assert(!bus.enqueue(command) && bus.isEmpty());
  assert(motorControl.finiteProfileLeaseSealed(lease));
  command.type = CommandType::MOTOR_RUN;
  assert(!bus.enqueue(command) && bus.isEmpty());
  command.type = CommandType::LIGHT_ON; assert(bus.enqueue(command));
  Command queued; assert(bus.dequeue(queued) && queued.type == CommandType::LIGHT_ON);
  // A later empty queue must not revive the invalidated reservation.
  uint32_t at = 0; assert(!motorControl.applyFiniteProfile(lease, 2, 90, at));
  assert(motorControl.getSpeed(2) == 0 && motorControl.finiteProfileOwned());
  assertMatches(motorControl);
}
static void full_queue_stop() {
  CommandBus bus; Command command; command.type = CommandType::LIGHT_ON;
  for (unsigned i = 0; i < CommandBus::MAX_PENDING; ++i) assert(bus.enqueue(command));
  assert(bus.isFull());
  const uint32_t lease = acquire(); uint32_t at = 0;
  assert(motorControl.applyFiniteProfile(lease, 2, 90, at));
  command.type = CommandType::STOP;
  assert(!bus.enqueue(command)); // No room, but stop intent has already forced OFF.
  assert(bus.isFull() && motorControl.finiteProfileLeaseSealed(lease));
  assert(!motorControl.hasPendingOutput() && motorControl.finiteProfileOwned());
  assertMatches(motorControl);
}
#endif
int main(int argc, char** argv) {
  assert(argc == 2); assert(motorControl.init()); systemState.state = SystemState::ARMED;
  if (!strcmp(argv[1], "disabled")) disabled();
#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER
  else if (!strcmp(argv[1], "movement")) readonly_and_movement();
  else if (!strcmp(argv[1], "stop")) stop_paths();
  else if (!strcmp(argv[1], "queue")) queue_rejection();
  else if (!strcmp(argv[1], "full")) full_queue_stop();
#endif
  else return 2;
}
