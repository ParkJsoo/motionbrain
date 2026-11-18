#include "control/whole_arm_action.h"
#include "motor/motor_driver.h"
#include "safety/safety_monitor.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

using namespace whole_arm;

static const char TRIAL[] = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
static const char OTHER[] = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";

SystemStateManager systemState;
SafetyMonitor safetyMonitor;
static uint32_t driverClock = 0;
uint32_t millis() { return driverClock; }
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
void ledcSetup(uint8_t, uint32_t, uint8_t) {}
void ledcAttachPin(uint8_t, uint8_t) {}
void ledcWrite(uint8_t, uint32_t duty) { assert(duty <= 70); }

// The timer/HAL is substituted only in this driver-level harness. Production
// MotorControl issues the ramp/zero writes and records the signed output ring.
namespace WholeArmService {
bool owns() { return false; }
bool writePwm(uint8_t, uint8_t channel, int16_t pwm) {
  ledcWrite(channel, static_cast<unsigned>(std::abs(pwm)));
  return true;
}
}

static void motorOutput() {
  MotorControl driver;
  assert(driver.init());
  systemState.state = SystemState::ARMED;
  assert(!driver.beginPolicyOutput(2, 70));
  assert(!driver.beginPolicyOutput(4, 178));
  assert(driver.setDefaultSpeed(255));
  assert(!driver.beginPolicyOutput(4, 70));
  assert(driver.setDefaultSpeed(100));
  const Phase phases[] = {Phase::Approach, Phase::Close, Phase::Lift, Phase::Lower,
                          Phase::Release, Phase::Retreat};
  const uint8_t motors[] = {4, 1, 4, 4, 1, 4};
  const int16_t pwm[] = {-70, -70, 70, -70, 70, 70};
  for (unsigned i = 0; i < 6; ++i) {
    assert(phaseMotor(phases[i]) == motors[i]);
    assert(phasePwm(phases[i]) == pwm[i]);
    assert(driver.beginPolicyOutput(motors[i], pwm[i]));
    for (int j = 0; j < 7; ++j) {
      driverClock += 50;
      driver.update();
    }
    assert(driver.getSpeed(motors[i]) == pwm[i]);
    assert(driver.endPolicyOutput(motors[i]));
    const auto* last = driver.outputLog().at(driver.outputLog().count()-1);
    assert(last->motor == motors[i] && last->pwm == 0);
    assert(last->reason == bounded::OutputReason::DriverWrite);
    assert(!driver.hasPendingOutput() && !driver.isEnabled());
  }
}

struct MockPort : Port {
  uint32_t clock = 100;
  Inputs state{true, true, 220.0f};
  bool acceptStart = true;
  bool confirmOff = true;
  std::vector<Phase> starts, stops;
  uint32_t now() const override { return clock; }
  Inputs inputs() const override { return state; }
  bool start(Phase phase) override {
    assert(state.allOff);
    starts.push_back(phase);
    state.allOff = false;
    return acceptStart;
  }
  uint32_t correctionStartedAt() const override { return clock; }
  bool correctionStep(uint8_t) override { return true; }
  void stop(Phase phase) override {
    stops.push_back(phase);
    state.allOff = confirmOff;
  }
};

struct Fixture {
  MockPort port;
  Controller controller{port};
  bool send(Op op, uint32_t seq, Phase phase = Phase::Approach, const char* trial = TRIAL) {
    return controller.apply({op, phase, trial, seq});
  }
  void begin() {
    assert(send(Op::Start, 1));
    assert(controller.active() && port.starts.size() == 1);
  }
};

static void sequence() {
  Fixture f;
  assert(!f.send(Op::Start, 1, Phase::Release));
  assert(f.port.starts.empty());
  Fixture live;
  live.begin();
  assert(!live.send(Op::Start, 1));
  assert(!live.send(Op::Keep, 2, Phase::Approach, OTHER));
  assert(!live.send(Op::Keep, 3));
  assert(live.port.starts.size() == 1);
}

static void deadline(uint32_t start) {
  Fixture f;
  f.port.clock = start;
  f.begin();
  f.port.clock += 749;
  f.controller.tick();
  assert(f.controller.active());
  f.port.clock++;
  // Check the deadline inside the request handler, even if the periodic tick is late.
  assert(!f.send(Op::Keep, 2));
  assert(f.controller.failed() && !f.controller.active());
  assert(f.port.stops.size() == 1);
  assert(!f.send(Op::Start, 1));
  assert(!f.send(Op::Keep, 2));
  assert(f.port.starts.size() == 1);
}

static void cancel() {
  Fixture f;
  f.begin();
  f.controller.abort();
  assert(!f.controller.active() && f.port.stops.size() == 1);
  assert(!f.send(Op::Keep, 2));
  assert(!f.send(Op::Start, 3, Phase::Close));
  assert(f.port.starts.size() == 1);
}

static void sensor() {
  for (unsigned reason = 0; reason < 3; ++reason) {
    Fixture f;
    f.begin();
    if (reason == 0) f.port.state.ready = false;
    if (reason == 1) f.port.state.shoulderDeg = NAN;
    if (reason == 2) f.port.state.shoulderDeg = 90.0f;
    f.controller.tick();
    assert(f.controller.failed() && !f.controller.active());
    assert(f.port.stops.size() == 1);
  }
}

static void failedOutput() {
  Fixture rejected;
  rejected.port.acceptStart = false;
  assert(!rejected.send(Op::Start, 1));
  assert(rejected.controller.failed() && !rejected.controller.active());
  assert(!rejected.send(Op::Start, 2));
  assert(rejected.port.stops.size() == 1);
  Fixture stop;
  stop.begin();
  stop.port.confirmOff = false;
  assert(!stop.send(Op::Stop, 2));
  assert(stop.controller.failed() && !stop.controller.active());
  assert(!stop.send(Op::Start, 3, Phase::Close));
}

static void phaseAngle() {
  Fixture f;
  f.begin();
  assert(f.send(Op::Stop, 2));
  f.port.state.shoulderDeg = 195.f;
  assert(!f.send(Op::Start, 3, Phase::Close));
  f.port.state.shoulderDeg = 190.f;
  assert(f.send(Op::Start, 3, Phase::Close));
  assert(f.send(Op::Stop, 4, Phase::Close));
  assert(f.send(Op::Start, 5, Phase::Lift));
  f.port.state.shoulderDeg = 209.f;
  assert(!f.send(Op::Keep, 6, Phase::Lift));
  assert(f.controller.failed() && f.port.stops.size() == 3);
}

static void full() {
  Fixture f;
  uint32_t seq = 0;
  const Phase order[] = {Phase::Approach, Phase::Close, Phase::Lift, Phase::Lower,
                         Phase::Release, Phase::Retreat};
  for (const auto phase : order) {
    if (phase != Phase::Approach) f.port.state.shoulderDeg = 190.0f;
    assert(f.send(Op::Start, ++seq, phase));
    f.port.clock += 125;
    assert(f.send(Op::Keep, ++seq, phase));
    assert(f.send(Op::Stop, ++seq, phase));
    assert(!f.controller.active());
    assert(!f.send(Op::Keep, seq, phase));
  }
  assert(!f.send(Op::Start, ++seq));
  assert(f.port.starts.size() == 6 && f.port.stops.size() == 6);
  assert(std::equal(f.port.starts.begin(), f.port.starts.end(), order));
}

static void hardCap() {
  Fixture f;
  f.begin();
  uint32_t seq = 1;
  for (unsigned i = 0; i < 160 && f.controller.active(); ++i) {
    f.port.clock += 250;
    f.send(Op::Keep, ++seq);
    f.controller.tick();
  }
  assert(f.controller.failed() && !f.controller.active());
  assert(f.port.starts.size() == 1 && f.port.stops.size() == 1);
}

int main(int argc, char** argv) {
  assert(argc == 2);
  if (!strcmp(argv[1], "sequence")) sequence();
  else if (!strcmp(argv[1], "deadline")) deadline(100);
  else if (!strcmp(argv[1], "wrap")) deadline(UINT32_MAX - 10);
  else if (!strcmp(argv[1], "abort")) cancel();
  else if (!strcmp(argv[1], "sensor")) sensor();
  else if (!strcmp(argv[1], "output")) failedOutput();
  else if (!strcmp(argv[1], "phase_angle")) phaseAngle();
  else if (!strcmp(argv[1], "full")) full();
  else if (!strcmp(argv[1], "motor")) motorOutput();
  else if (!strcmp(argv[1], "hard_cap")) hardCap();
  else return 2;
}
