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
static uint32_t pwmWriteLatency = 0;
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
  nowMs += pwmWriteLatency;
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


// Finite-profile driver cases follow.

#include "control/finite_profile_driver_bridge.h"

#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER
using namespace finite_profile;

static uint32_t acquire(MotorControl& driver, uint32_t ordinal = 41) {
  uint32_t lease = 0;
  assert(driver.finiteProfileReady());
  assert(driver.acquireFiniteProfileLease(ordinal, lease) && lease != 0);
  assert(driver.finiteProfileLeaseMatches(lease));
  return lease;
}
static void direct() {
  MotorControl driver; assert(driver.init()); systemState.state = SystemState::ARMED;
  const uint32_t lease = acquire(driver); uint32_t at = 0;
  nowMs = 7;
  assert(driver.applyFiniteProfile(lease, 2, 160, at));
  assert(at == 7 && driver.getSpeed(2) == 160 && writes.back().signedPwm == 160);
  assert(driver.outputLog().at(driver.outputLog().count() - 1)->actionOrdinal == 41);
  const size_t count = writes.size(); advance(driver, 1000);
  assert(writes.size() == count && driver.getSpeed(2) == 160);
  nowMs += 3; assert(driver.applyFiniteProfile(lease, 2, 70, at));
  assert(at == nowMs && driver.getSpeed(2) == 70);
  assert(driver.applyFiniteProfile(lease, 2, 0, at));
  assert(driver.finiteProfileLeaseSealed(lease) && driver.finiteProfileSoftwareOff(lease));
  assert(!driver.applyFiniteProfile(lease, 2, 160, at));
  assert(driver.releaseFiniteProfileLease(lease));
  assert(driver.setSpeed(2, 20)); advance(driver, 50);
  assert(driver.getSpeed(2) == 10); // Legacy ramp resumes only after release.
  assertMatches(driver);
}
static void legacy_conflicts() {
  MotorControl driver; driver.init(); systemState.state = SystemState::ARMED;
  const uint32_t lease = acquire(driver, 77); uint32_t at = 0;
  assert(!driver.setSpeed(2, 30) && !driver.setSpeed(1, 30));
  assert(!driver.setSpeedAll(30));
  assert(!driver.forward(2) && !driver.forward(2, 50));
  assert(!driver.reverse(2) && !driver.reverse(2, 50));
  assert(!driver.forward(1) && !driver.reverse(5));
  driver.setActionOrdinal(999);
  assert(driver.applyFiniteProfile(lease, 2, -120, at));
  assert(driver.getSpeed(2) == -120 && writes.back().signedPwm == -120);
  assert(driver.outputLog().at(driver.outputLog().count() - 1)->actionOrdinal == 77);
  const size_t n = writes.size(); advance(driver, 500);
  assert(writes.size() == n && driver.getSpeed(1) == 0 && driver.getSpeed(5) == 0);
  assertMatches(driver);
}
static void stale_lease() {
  MotorControl driver; driver.init(); systemState.state = SystemState::ARMED;
  uint32_t at = 0, unexpected = 0;
  const uint32_t old = acquire(driver);
  assert(!driver.acquireFiniteProfileLease(42, unexpected));
  assert(driver.stopFiniteProfile(old, at));
  assert(driver.releaseFiniteProfileLease(old));
  const uint32_t current = acquire(driver, 42); assert(current != old);
  assert(driver.applyFiniteProfile(current, 2, 90, at));
  const size_t n = writes.size();
  assert(!driver.applyFiniteProfile(old, 2, 200, at));
  assert(!driver.stopFiniteProfile(old, at));
  assert(!driver.releaseFiniteProfileLease(old));
  assert(driver.finiteProfileLeaseMatches(current) && driver.getSpeed(2) == 90);
  assert(writes.size() == n);
  assertMatches(driver);
}
static void ready_requires_idle() {
  MotorControl driver; driver.init(); systemState.state = SystemState::ARMED;
  assert(driver.setSpeed(1, 30)); uint32_t lease = 0;
  assert(!driver.finiteProfileReady() && !driver.acquireFiniteProfileLease(10, lease));
  assert(writes.size() == 5); // Pending legacy target alone prevents acquisition.
  driver.hardStopAll(); assert(driver.finiteProfileReady());
  lease = acquire(driver); uint32_t at = 0;
  assert(!driver.applyFiniteProfile(lease, 1, 100, at));
  assert(!driver.applyFiniteProfile(lease, 2, 256, at));
  assert(!driver.applyFiniteProfile(lease, 2, -256, at));
  assert(driver.getSpeed(1) == 0 && driver.getSpeed(2) == 0);
  assertMatches(driver);
}
static void stop_seals() {
  for (unsigned mode = 0; mode < 8; ++mode) {
    writes.clear(); nowMs = 0; systemState.state = SystemState::ARMED;
    MotorControl driver; driver.init(); const uint32_t lease = acquire(driver); uint32_t at = 0;
    assert(driver.applyFiniteProfile(lease, 2, 100, at));
    switch (mode) {
      case 0: assert(driver.stop(2)); break;
      case 1: assert(driver.stopAll()); break;
      case 2: assert(driver.hardStop(1)); break;
      case 3: driver.hardStopAll(); break;
      case 4: driver.emergencyStop(); break;
      case 5: assert(driver.setSpeed(2, 0)); break;
      case 6: assert(driver.setSpeedAll(0)); break;
      case 7: assert(!driver.init()); break;
    }
    assert(driver.finiteProfileLeaseSealed(lease) && driver.finiteProfileSoftwareOff(lease));
    assert(!driver.hasPendingOutput());
    systemState.state = SystemState::IDLE; driver.update();
    systemState.state = SystemState::ARMED; advance(driver, 1000);
    const size_t n = writes.size();
    assert(!driver.applyFiniteProfile(lease, 2, 200, at));
    advance(driver, 1000);
    for (size_t i = n; i < writes.size(); ++i) assert(writes[i].signedPwm == 0);
    assert(driver.getSpeed(2) == 0 && !driver.setSpeed(2, 30));
    assertMatches(driver);
  }
}
static void safety_changes() {
  for (unsigned mode = 0; mode < 3; ++mode) {
    writes.clear(); nowMs = 0; systemState.state = SystemState::ARMED; safetyMonitor.blocked = false;
    MotorControl driver; driver.init(); const uint32_t lease = acquire(driver); uint32_t at = 0;
    assert(driver.applyFiniteProfile(lease, 2, 80, at));
    if (mode == 0) systemState.state = SystemState::IDLE;
    if (mode == 1) systemState.state = SystemState::FAULT;
    if (mode == 2) safetyMonitor.blocked = true;
    driver.update();
    assert(driver.finiteProfileLeaseSealed(lease) && driver.getSpeed(2) == 0);
    assert(!driver.hasPendingOutput());
    systemState.state = SystemState::ARMED; safetyMonitor.blocked = false;
    assert(!driver.applyFiniteProfile(lease, 2, 80, at));
    assert(!driver.setSpeed(2, 20)); advance(driver, 1000);
    assert(driver.getSpeed(2) == 0);
    assertMatches(driver);
  }
}
static void ring_trace() {
  MotorControl driver; driver.init(); systemState.state = SystemState::ARMED;
  const uint32_t lease = acquire(driver, 91); uint32_t at = 0;
  for (unsigned i = 0; i < 150; ++i) {
    nowMs = i + 1;
    assert(driver.applyFiniteProfile(lease, 2, i % 2 ? 70 : 80, at));
    assert(at == nowMs);
  }
  assert(driver.stopFiniteProfile(lease, at));
  assert(driver.outputLog().count() == 128 && driver.outputLog().dropped() > 0);
  assertMatches(driver);
  for (size_t i = 0; i < driver.outputLog().count(); ++i)
    assert(driver.outputLog().at(i)->actionOrdinal == 91);
}
struct Environment : DriverEnvironment {
  bool safe = true;
  bool otherIdle = true;
  unsigned counter = 0;
  bool allowed() const override { return safe; }
  bool controllersIdle() const override { return otherIdle; }
  bool mintToken(Token& token) override {
    ++counter;
    for (size_t i = 0; i < sizeof(token.bytes); ++i)
      token.bytes[i] = static_cast<uint8_t>(counter + i);
    return true;
  }
};
static Profile bridgeProfile() {
  Profile p; memset(p.catalogHash, 'a', 64); p.catalogHash[64] = 0;
  strcpy(p.profileId, "driver-test"); p.startLeadMs = 20; p.maxLatenessMs = 10;
  p.offConfirmWindowMs = 30; p.admissionWindowMs = 100; p.count = 4;
  p.points[0] = Point(0, 80); p.points[1] = Point(50, 120);
  p.points[2] = Point(100, 40); p.points[3] = Point(150, 0);
  return p;
}
struct BridgeFixture {
  MotorControl driver;
  Environment environment;
  Profile accepted;
  DriverBridge bridge;
  BridgeFixture() : accepted(bridgeProfile()), bridge(driver, environment, "boot-A", &accepted, 1) {
    assert(driver.init()); systemState.state = SystemState::ARMED;
  }
  Request request(const char* id = "command-1") const {
    Request r; strcpy(r.bootId, "boot-A"); strcpy(r.commandId, id);
    r.expectedEpoch = bridge.epoch(); r.profile = accepted; return r;
  }
  PrepareResult prepare(const char* id = "command-1") { return bridge.prepare(request(id)); }
  const Record* start(const char* id = "command-1") {
    const auto p = prepare(id); assert(p.code == Code::Accepted && p.granted && p.record);
    const auto result = bridge.execute("boot-A", id, p.record->request.expectedEpoch, p.token);
    assert(result.code == Code::Accepted && result.record->state == State::Running);
    return result.record;
  }
  void at(uint32_t time) { nowMs = time; driver.update(); bridge.tick(); }
  void finish(const Record* r) {
    for (uint8_t i = r->nextEventIndex; i < accepted.count; ++i)
      at(r->plannedStartAt + accepted.points[i].offsetMs);
  }
};
static void bridge_nominal() {
  BridgeFixture f; const size_t before = writes.size(); const Record* r = f.start();
  const uint32_t lease = f.bridge.leaseId(); assert(lease != 0 && writes.size() == before);
  f.at(r->plannedStartAt); assert(f.driver.getSpeed(2) == 80);
  const auto first = writes.size(); nowMs += 49; f.driver.update(); assert(writes.size() == first);
  f.finish(r);
  assert(r->state == State::Completed && r->traceComplete && r->timingConformant);
  assert(r->quietHeld && f.bridge.leaseId() == lease && !f.bridge.idle());
  assert(f.driver.finiteProfileLeaseSealed(lease) && f.driver.finiteProfileSoftwareOff(lease));
  assert(!f.driver.setSpeed(2, 50));
  for (uint8_t i = 0; i < f.accepted.count; ++i) {
    assert(r->writes[i].present && r->writes[i].pwm == f.accepted.points[i].pwm);
    assert(r->writes[i].atMs == r->plannedStartAt + f.accepted.points[i].offsetMs);
    assert(f.driver.outputLog().at(before + i)->reason == bounded::OutputReason::DriverWrite);
  }
  for (size_t i = before + f.accepted.count; i < f.driver.outputLog().count(); ++i)
    assert(f.driver.outputLog().at(i)->reason == bounded::OutputReason::ForcedOff);
  assert(f.bridge.release("boot-A", "command-1", r->quietEpoch).code == Code::Accepted);
  assert(f.bridge.idle() && f.bridge.leaseId() == 0 && !f.driver.finiteProfileLeaseMatches(lease));
  assert(f.driver.setSpeed(2, 20)); advance(f.driver, 50); assert(f.driver.getSpeed(2) == 10);
  assertMatches(f.driver);
}
static void bridge_old_callback() {
  BridgeFixture f; const auto p = f.prepare("old"); assert(p.code == Code::Accepted);
  const uint32_t oldLease = f.bridge.leaseId();
  assert(f.bridge.cancel("boot-A", "old", p.record->request.expectedEpoch).code == Code::Accepted);
  assert(p.record->state == State::Cancelled && f.bridge.leaseId() == 0);
  const Record* r = f.start("new"); f.at(r->plannedStartAt);
  const uint32_t lease = f.bridge.leaseId(); assert(lease != oldLease);
  const auto n = writes.size();
  assert(f.bridge.execute("boot-A", "old", p.record->request.expectedEpoch, p.token).code != Code::Accepted);
  f.bridge.event("boot-A", "old", p.record->request.expectedEpoch, 0);
  f.bridge.cancel("boot-A", "old", p.record->request.expectedEpoch);
  f.bridge.release("boot-A", "old", p.record->quietEpoch);
  assert(f.bridge.leaseId() == lease && writes.size() == n && f.driver.getSpeed(2) == 80);
  assertMatches(f.driver);
}
static void bridge_external_stop() {
  BridgeFixture f; const Record* r = f.start(); f.at(r->plannedStartAt);
  const uint32_t lease = f.bridge.leaseId(); f.driver.emergencyStop();
  f.bridge.tick();
  assert(r->state == State::Cancelled && r->quietHeld && !r->traceComplete);
  assert(f.bridge.leaseId() == lease && f.driver.finiteProfileLeaseSealed(lease));
  const size_t n = writes.size(); systemState.state = SystemState::IDLE; f.driver.update();
  systemState.state = SystemState::ARMED; nowMs = r->plannedStartAt + 50;
  f.bridge.event("boot-A", "command-1", r->request.expectedEpoch, 1); f.driver.update();
  for (size_t i = n; i < writes.size(); ++i) assert(writes[i].signedPwm == 0);
  assert(f.driver.getSpeed(2) == 0);
  assertMatches(f.driver);
}
static void bridge_reservation_stop() {
  BridgeFixture f; const auto p = f.prepare(); assert(p.code == Code::Accepted);
  f.driver.emergencyStop(); f.bridge.tick();
  assert(p.record->state == State::Cancelled && !p.record->executed);
  assert(f.bridge.leaseId() == 0);
  assert(f.bridge.execute("boot-A", "command-1", p.record->request.expectedEpoch, p.token).code != Code::Accepted);
  for (const auto& w : writes) assert(w.signedPwm == 0);
  assertMatches(f.driver);
}
static void bridge_admission() {
  BridgeFixture f; f.environment.otherIdle = false;
  assert(f.prepare().code != Code::Accepted && f.bridge.leaseId() == 0);
  f.environment.otherIdle = true; f.environment.safe = false;
  assert(f.prepare().code != Code::Accepted && f.bridge.leaseId() == 0);
  f.environment.safe = true;
  uint32_t external = 0; assert(f.driver.acquireFiniteProfileLease(88, external));
  assert(f.prepare().code != Code::Accepted && f.bridge.leaseId() == 0);
  assert(f.driver.finiteProfileLeaseMatches(external));
  assertMatches(f.driver);
}

static void bridge_destroyed() {
  MotorControl driver; driver.init(); systemState.state = SystemState::ARMED;
  Environment environment; Profile p = bridgeProfile(); uint32_t lease = 0;
  {
    DriverBridge bridge(driver, environment, "boot-A", &p, 1);
    Request request; strcpy(request.bootId, "boot-A"); strcpy(request.commandId, "destroyed");
    request.expectedEpoch = bridge.epoch(); request.profile = p;
    const auto prepared = bridge.prepare(request); assert(prepared.granted);
    assert(bridge.execute("boot-A", "destroyed", request.expectedEpoch, prepared.token).code == Code::Accepted);
    lease = bridge.leaseId(); nowMs = prepared.record->plannedStartAt; bridge.tick();
    assert(driver.getSpeed(2) == 80);
  }
  assert(driver.getSpeed(2) == 0 && driver.finiteProfileLeaseMatches(lease));
  assert(driver.finiteProfileLeaseSealed(lease) && !driver.setSpeed(2, 20));
  uint32_t at = 0; assert(!driver.applyFiniteProfile(lease, 2, 80, at));
  assertMatches(driver);
}
static void bridge_owner_replaced() {
  MotorControl driver; driver.init(); systemState.state = SystemState::ARMED;
  Environment environment; Profile p = bridgeProfile(); uint32_t replacement = 0;
  size_t beforeDestruction = 0;
  {
    DriverBridge bridge(driver, environment, "boot-A", &p, 1);
    Request request; strcpy(request.bootId, "boot-A"); strcpy(request.commandId, "replaced");
    request.expectedEpoch = bridge.epoch(); request.profile = p;
    const auto prepared = bridge.prepare(request); assert(prepared.granted);
    const uint32_t old = bridge.leaseId();
    assert(driver.releaseFiniteProfileLease(old));
    assert(driver.acquireFiniteProfileLease(99, replacement) && replacement != old);
    uint32_t at = 0; assert(driver.applyFiniteProfile(replacement, 2, 90, at));
    const size_t n = writes.size();
    assert(bridge.execute("boot-A", "replaced", request.expectedEpoch, prepared.token).code != Code::Accepted);
    bridge.cancel("boot-A", "replaced", request.expectedEpoch); bridge.retryOff("boot-A", "replaced");
    assert(bridge.blocked() && driver.finiteProfileLeaseMatches(replacement));
    assert(driver.getSpeed(2) == 90 && writes.size() == n);
    beforeDestruction = writes.size();
  }
  assert(driver.getSpeed(2) == 90 && driver.finiteProfileLeaseMatches(replacement));
  assert(writes.size() == beforeDestruction);
  assertMatches(driver);
}

static void bridge_cancel_off_timestamp() {
  BridgeFixture f; const Record* r = f.start(); f.at(r->plannedStartAt);
  pwmWriteLatency = 1; nowMs += 5;
  const size_t firstStop = writes.size();
  assert(f.bridge.cancel("boot-A", "command-1", r->request.expectedEpoch).code == Code::Accepted);
  assert(r->state == State::Cancelled && r->softwareOffConfirmed);
  assert(writes.size() == firstStop + MotorControl::NUM_MOTORS);
  uint32_t motor2Off = 0, allOff = 0;
  for (size_t i = firstStop; i < writes.size(); ++i) {
    assert(writes[i].signedPwm == 0);
    if (writes[i].motor == 2) motor2Off = writes[i].time;
    allOff = writes[i].time;
  }
  assert(motor2Off != 0 && motor2Off < allOff);
  assert(r->offWriteAt == motor2Off);
  assert(r->offConfirmedAt >= allOff);
  assertMatches(f.driver);
  pwmWriteLatency = 0;
}

#endif

static void disabled_legacy() {
  MotorControl driver; assert(driver.init()); systemState.state = SystemState::ARMED;
  driver.setActionOrdinal(17); assert(driver.setSpeed(2, 30));
  advance(driver, 50); assert(driver.getSpeed(2) == 10);
  assert(driver.outputLog().at(5)->actionOrdinal == 17);
  assertMatches(driver);
}

int main(int argc, char** argv) {
  assert(argc == 2);
  if (!strcmp(argv[1], "disabled")) disabled_legacy();
#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER
  else if (!strcmp(argv[1], "direct")) direct();
  else if (!strcmp(argv[1], "legacy")) legacy_conflicts();
  else if (!strcmp(argv[1], "stale")) stale_lease();
  else if (!strcmp(argv[1], "ready")) ready_requires_idle();
  else if (!strcmp(argv[1], "stop")) stop_seals();
  else if (!strcmp(argv[1], "safety")) safety_changes();
  else if (!strcmp(argv[1], "ring")) ring_trace();
  else if (!strcmp(argv[1], "bridge_nominal")) bridge_nominal();
  else if (!strcmp(argv[1], "bridge_old")) bridge_old_callback();
  else if (!strcmp(argv[1], "bridge_stop")) bridge_external_stop();
  else if (!strcmp(argv[1], "bridge_reserved")) bridge_reservation_stop();
  else if (!strcmp(argv[1], "bridge_admission")) bridge_admission();
  else if (!strcmp(argv[1], "bridge_destroyed")) bridge_destroyed();
  else if (!strcmp(argv[1], "bridge_replaced")) bridge_owner_replaced();
  else if (!strcmp(argv[1], "bridge_cancel_time")) bridge_cancel_off_timestamp();
#endif
  else return 2;
}
