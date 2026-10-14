// Compiles the production motor driver. Only Arduino IO, clock, safety and
// system-state dependencies are substituted; no electrical feedback is claimed.
#include "motor/motor_driver.h"
#include "safety/safety_monitor.h"
#include <assert.h>
#include <string.h>
#include <vector>

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
static void advance(MotorControl& driver, uint32_t ms) { nowMs += ms; driver.update(); }

static void ramp() {
  MotorControl driver;
  assert(driver.init());
  assert(writes.size() == 5 && !driver.hasPendingOutput());
  for (size_t i = 0; i < 5; ++i) {
    assert(driver.outputLog().at(i)->actionOrdinal == 0);
    assert(driver.outputLog().at(i)->reason == bounded::OutputReason::ForcedOff);
  }
  systemState.state = SystemState::ARMED;
  driver.setActionOrdinal(41);
  assert(driver.setSpeed(4, 25));
  assert(driver.hasPendingOutput() && !driver.isEnabled());
  assert(writes.size() == 5); // target acceptance is not an applied output
  advance(driver, 49);
  assert(writes.size() == 5);
  advance(driver, 1);
  assert(driver.getSpeed(4) == 10);
  advance(driver, 50);
  assert(driver.getSpeed(4) == 20);
  advance(driver, 50);
  assert(driver.getSpeed(4) == 25);
  advance(driver, 1000);
  assert(writes.size() == 8); // unchanged output creates no invented sample
  assert(driver.stop(4));
  assert(driver.hasPendingOutput());
  advance(driver, 50);
  assert(driver.getSpeed(4) == 15);
  advance(driver, 50);
  assert(driver.getSpeed(4) == 0 && !driver.hasPendingOutput());
  for (size_t i = 5; i < writes.size(); ++i) {
    assert(driver.outputLog().at(i)->actionOrdinal == 41);
    assert(driver.outputLog().at(i)->reason == bounded::OutputReason::DriverWrite);
  }
  driver.setActionOrdinal(0);
  assert(driver.setSpeed(1, -20));
  advance(driver, 50);
  advance(driver, 50);
  assert(driver.getSpeed(1) == -20);
  assert(driver.outputLog().at(writes.size() - 1)->actionOrdinal == 0);
  assertMatches(driver);
}

static void forceOff() {
  MotorControl driver;
  driver.init();
  systemState.state = SystemState::ARMED;
  driver.setActionOrdinal(7);
  assert(driver.setSpeedAll(100));
  advance(driver, 50);
  assert(driver.hardStop(4));
  assert(!pins[27] && !pins[32]);
  assert(driver.outputLog().at(10)->reason == bounded::OutputReason::ForcedOff);
  assert(driver.outputLog().at(10)->actionOrdinal == 7);
  driver.emergencyStop();
  assert(!driver.hasPendingOutput());
  for (uint8_t i = 0; i < 5; ++i) {
    assert(!pins[forwardPins[i]] && !pins[reversePins[i]]);
    assert(driver.getSpeed(i + 1) == 0);
  }
  const size_t stoppedCount = writes.size();
  advance(driver, 1000);
  assert(writes.size() == stoppedCount);
  assert(driver.setSpeed(2, 50)); // target only must also be cleared on disarm
  systemState.state = SystemState::IDLE;
  driver.update();
  assert(writes.size() == stoppedCount + 1 && !driver.hasPendingOutput());
  systemState.state = SystemState::ARMED;
  advance(driver, 1000);
  assert(writes.size() == stoppedCount + 1); // no residual target after re-arm
  safetyMonitor.blocked = true;
  assert(!driver.setSpeed(4, 30));
  assert(!driver.forward(4));
  assert(!driver.hasPendingOutput());
  assert(!driver.hardStop(0));
  assert(!driver.hardStop(6));
  assert(writes.size() == stoppedCount + 1);
  assertMatches(driver);
}

static void overflowAndWrap() {
  MotorControl driver;
  driver.init();
  driver.setActionOrdinal(99);
  for (unsigned i = 0; i < 150; ++i) { nowMs = i; driver.hardStop(4); }
  assert(driver.outputLog().count() == 128 && driver.outputLog().dropped() == 27);
  assertMatches(driver);
  systemState.state = SystemState::ARMED;
  nowMs = UINT32_MAX - 20;
  assert(driver.setSpeed(4, -30));
  driver.update();
  assert(driver.getSpeed(4) == -10);
  advance(driver, 49);
  assert(driver.getSpeed(4) == -10);
  advance(driver, 1);
  assert(driver.getSpeed(4) == -20);
  assert(driver.outputLog().at(127)->atMs == 29);
  assertMatches(driver);
}

int main(int argc, char** argv) {
  assert(argc == 2);
  if (!strcmp(argv[1], "ramp")) ramp();
  else if (!strcmp(argv[1], "force")) forceOff();
  else if (!strcmp(argv[1], "overflow")) overflowAndWrap();
  else return 2;
}
