#include "network/manual_control_lease.h"
#include "motion/manual_command_direction.h"
#include "motor/motor_driver.h"
#include "safety/safety_monitor.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <vector>

SystemStateManager systemState;
SafetyMonitor safetyMonitor;
static uint32_t clockMs = 100;
static std::vector<int> writes;
uint32_t millis() { return clockMs; }
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
void ledcSetup(uint8_t, uint32_t, uint8_t) {}
void ledcAttachPin(uint8_t, uint8_t) {}
void ledcWrite(uint8_t, uint32_t duty) { writes.push_back(duty); }
const char* BOOT = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

class DriverPort : public manual_control::Port {
public:
  MotorControl driver;
  manual_control::Request pending;
  unsigned dispatches = 0, expiries = 0;
  bool reject = false;
  DriverPort() { assert(driver.init()); systemState.state = SystemState::ARMED; }
  uint32_t now() const override { return clockMs; }
  uint64_t controlEpoch(uint8_t motor) const override { return driver.controlEpoch(motor); }
  bool dispatch() override {
    ++dispatches;
    if (reject) return false;
    return pending.forward ? driver.forward(pending.motor, pending.percent)
                           : driver.reverse(pending.motor, pending.percent);
  }
  void expired(uint8_t motor) override { ++expiries; assert(driver.hardStop(motor)); }
};

static manual_control::Request start(manual_control::Leases& leases, DriverPort& port,
                                      uint8_t motor = 1, bool forward = true) {
  leases.synchronize(port);
  const auto* state = leases.state(motor);
  char generation[16]; snprintf(generation, sizeof(generation), "%lu", (unsigned long)state->generation);
  manual_control::Request r;
  assert(manual_control::parseRequest("start", leases.bootId(), generation, "", "", motor, forward, 60, r));
  return r;
}
static manual_control::Request renewal(const char* id, uint32_t sequence,
                                       uint8_t motor = 1, bool forward = true) {
  char seq[16]; snprintf(seq, sizeof(seq), "%lu", (unsigned long)sequence);
  manual_control::Request r;
  assert(manual_control::parseRequest("renew", "", "", id, seq, motor, forward, 60, r));
  return r;
}
static bool run(manual_control::Leases& leases, DriverPort& port,
                const manual_control::Request& request, bool expectedDispatch) {
  port.pending = request;
  bool dispatched = false; const char* reason = nullptr;
  const unsigned before = port.dispatches;
  uint32_t checkedAt = ~clockMs;
  const bool accepted = leases.execute(request, port, dispatched, reason, &checkedAt);
  if (strcmp(reason, "manual_lease_malformed")) assert(checkedAt == clockMs);
  assert(reason && dispatched == expectedDispatch);
  assert(port.dispatches == before + unsigned(expectedDispatch));
  return accepted;
}
static void advance(DriverPort& port, uint32_t ms) { clockMs += ms; port.driver.update(); }

static void user04() {
  // Recorded controller timeline: 90ms is the later nonzero pulse width,
  // not the delay from forced OFF to the stale HTTP request.
  constexpr uint32_t acceptedAt = 12697507;
  constexpr uint32_t forcedOffAt = 12698259;
  constexpr uint32_t lateDispatchAt = 12698811;
  constexpr uint32_t pulseAt = 12698828;
  constexpr uint32_t finalOffAt = 12698918;
  uint32_t legacyWidth = 0;
  {
    clockMs = acceptedAt;
    DriverPort old;
    assert(old.driver.forward(1, 60));
    while (clockMs + 50 < forcedOffAt) advance(old, 50);
    clockMs = forcedOffAt;
    assert(old.driver.hardStop(1));
    const size_t forcedCount = old.driver.outputLog().count();
    assert(old.driver.outputLog().at(forcedCount - 1)->atMs == forcedOffAt);
    clockMs = lateDispatchAt;
    assert(old.driver.forward(1, 60)); // Former untyped refresh admission.
    assert(old.driver.outputLog().count() == forcedCount);
    clockMs = pulseAt;
    old.driver.update();
    const auto* pulse = old.driver.outputLog().at(forcedCount);
    assert(pulse && pulse->atMs == pulseAt && pulse->pwm == 10);
    clockMs = finalOffAt;
    assert(old.driver.stop(1));
    old.driver.update();
    const auto* off = old.driver.outputLog().at(forcedCount + 1);
    assert(off && off->atMs == finalOffAt && off->pwm == 0);
    assert(off->reason == bounded::OutputReason::DriverWrite);
    legacyWidth = off->atMs - pulse->atMs;
    assert(legacyWidth == 90);
  }

  // Independent counterfactual controller, same timestamps and production driver.
  clockMs = acceptedAt;
  DriverPort port;
  manual_control::Leases leases;
  assert(leases.init(BOOT, port));
  assert(run(leases, port, start(leases, port), true));
  char old[manual_control::ID_CAPACITY];
  strcpy(old, leases.state(1)->leaseId);
  while (clockMs + 50 < forcedOffAt) advance(port, 50);
  clockMs = forcedOffAt;
  leases.tick(port); // 752ms since acceptance: actual lease expiry writes OFF.
  assert(port.expiries == 1);
  const size_t forcedCount = port.driver.outputLog().count();
  const auto* forced = port.driver.outputLog().at(forcedCount - 1);
  assert(forced->atMs == forcedOffAt && forced->pwm == 0);
  assert(forced->reason == bounded::OutputReason::ForcedOff);
  clockMs = lateDispatchAt;
  assert(!run(leases, port, renewal(old, 1), false));
  clockMs = pulseAt;
  port.driver.update();
  clockMs = finalOffAt;
  assert(port.driver.stop(1));
  port.driver.update();
  assert(port.driver.getSpeed(1) == 0);
  assert(port.driver.outputLog().count() == forcedCount);
  const uint32_t leasedWidth = 0; // There is no nonzero write in that window.

  assert(run(leases, port, start(leases, port), true));
  char fresh[manual_control::ID_CAPACITY];
  strcpy(fresh, leases.state(1)->leaseId);
  assert(strcmp(old, fresh));
  advance(port, 50);
  const auto freshGeneration = leases.state(1)->generation;
  const unsigned expiryBefore = port.expiries;
  assert(!run(leases, port, renewal(old, 2), false));
  advance(port, 90);
  assert(leases.state(1)->active && leases.state(1)->generation == freshGeneration);
  assert(port.expiries == expiryBefore && port.driver.getSpeed(1) > 0);
  assert(run(leases, port, renewal(fresh, 1), true));
  printf("{\"legacyNonzeroWidthMs\":%lu,\"leasedNonzeroWidthMs\":%lu}\n",
         static_cast<unsigned long>(legacyWidth),
         static_cast<unsigned long>(leasedWidth));
}
static void deadline() {
  DriverPort port; manual_control::Leases leases; assert(leases.init(BOOT, port));
  assert(run(leases, port, start(leases, port), true));
  char id[manual_control::ID_CAPACITY]; strcpy(id, leases.state(1)->leaseId);
  clockMs += 749;
  assert(run(leases, port, renewal(id, 1), true));
  clockMs += 750; // No outer-loop tick: handler execution must enforce the deadline.
  assert(!run(leases, port, renewal(id, 2), false));
  assert(port.expiries == 1 && !leases.state(1)->active && !port.driver.hasPendingOutput());
  const auto writesAfterExpiry = writes.size(); advance(port, 90);
  assert(writes.size() == writesAfterExpiry && port.driver.getSpeed(1) == 0);
}
static void startReplayAndBoot() {
  DriverPort port; manual_control::Leases leases; assert(leases.init(BOOT, port));
  const auto oldStart = start(leases, port);
  leases.invalidate(1, port); assert(port.driver.hardStop(1));
  assert(!run(leases, port, oldStart, false)); // STOP before delayed initial START.
  const auto fresh = start(leases, port); assert(run(leases, port, fresh, true));
  assert(!run(leases, port, fresh, false));
  char id[manual_control::ID_CAPACITY]; strcpy(id, leases.state(1)->leaseId);
  const auto generation = leases.state(1)->generation;
  assert(leases.init(BOOT, port) && leases.state(1)->generation == generation);
  assert(!leases.init("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb", port));
  manual_control::Leases rebooted;
  port.driver.hardStopAll();
  assert(rebooted.init("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb", port));
  assert(!run(rebooted, port, renewal(id, 1), false));
  auto bootMismatch = start(rebooted, port); strcpy(bootMismatch.boot, BOOT);
  assert(!run(rebooted, port, bootMismatch, false));
}
static void sequenceAndDirection() {
  DriverPort port; manual_control::Leases leases; assert(leases.init(BOOT, port));
  for (uint8_t motor = 1; motor <= 5; ++motor) {
    const bool physicalUp = ManualCommandDirection::physicalForward(motor, true);
    assert(physicalUp == ManualCommandDirection::positiveIsForward(motor));
    if (motor == 3) assert(!physicalUp); // M3 UP/raw FORWARD both map to reverse.
    assert(run(leases, port, start(leases, port, motor, physicalUp), true));
    char id[manual_control::ID_CAPACITY]; strcpy(id, leases.state(motor)->leaseId);
    assert(run(leases, port, renewal(id, 1, motor, physicalUp), true));
    assert(!run(leases, port, renewal(id, 1, motor, physicalUp), false));
    assert(run(leases, port, renewal(id, 3, motor, physicalUp), true));
    assert(!run(leases, port, renewal(id, 2, motor, physicalUp), false));
    assert(!run(leases, port, renewal(id, 4, motor, !physicalUp), false));
    assert(leases.state(motor)->active && port.expiries == 0);
  }
}
static void externalStopsAndOverride() {
  DriverPort port; manual_control::Leases leases; assert(leases.init(BOOT, port));
  auto snapshot = start(leases, port); assert(run(leases, port, snapshot, true));
  char id[manual_control::ID_CAPACITY]; strcpy(id, leases.state(1)->leaseId);
  // Simulate serial/OOB stop + rapid re-arm without HTTP clear callbacks.
  assert(port.driver.stop(1)); systemState.state = SystemState::IDLE; systemState.state = SystemState::ARMED;
  assert(!run(leases, port, renewal(id, 1), false));
  auto beforeExternalStop = start(leases, port);
  port.driver.emergencyStop(); systemState.state = SystemState::ARMED;
  assert(!run(leases, port, beforeExternalStop, false));
  assert(run(leases, port, start(leases, port), true));
  strcpy(id, leases.state(1)->leaseId);
  assert(port.driver.forward(1, 80)); // A distinct accepted owner changes the driver epoch.
  const unsigned before = port.expiries; leases.tick(port);
  assert(!leases.state(1)->active && port.expiries == before);
  assert(!run(leases, port, renewal(id, 1), false)); advance(port, 90);
  assert(port.driver.getSpeed(1) > 0); // Old renew rejection must not stop the new owner.
}
static void malformedAndFailedStart() {
  DriverPort port; manual_control::Leases leases; assert(leases.init(BOOT, port));
  const char* modes[] = {"", "begin", "start ", "renew"};
  for (auto mode : modes) {
    manual_control::Request r;
    assert(!manual_control::parseRequest(mode, BOOT, "1", "", "", 1, true, 60, r));
  }
  const char* generations[] = {"", "0", "01", "-1", "1.0", "1x", "4294967296"};
  for (auto generation : generations) {
    manual_control::Request r;
    assert(!manual_control::parseRequest("start", BOOT, generation, "", "", 1, true, 60, r));
  }
  auto begin = start(leases, port); port.reject = true;
  assert(!run(leases, port, begin, true)); assert(!leases.state(1)->active);
  port.reject = false; assert(!run(leases, port, begin, false));
  assert(run(leases, port, start(leases, port), true));
  char id[manual_control::ID_CAPACITY]; strcpy(id, leases.state(1)->leaseId);
  manual_control::Request r;
  for (auto seq : {"0", "01", "-1", "1junk", "4294967296"})
    assert(!manual_control::parseRequest("renew", "", "", id, seq, 1, true, 60, r));
  assert(!manual_control::parseRequest("renew", BOOT, "1", id, "1", 1, true, 60, r));
  const auto count = port.dispatches; auto bad = renewal(id, 1); memset(bad.leaseId, 'a', sizeof(bad.leaseId));
  assert(!run(leases, port, bad, false) && count == port.dispatches);
}
static void clockWrapAndExhaustion() {
  DriverPort port; manual_control::Leases leases; assert(leases.init(BOOT, port));
  clockMs = UINT32_MAX - 100;
  assert(run(leases, port, start(leases, port), true));
  char id[manual_control::ID_CAPACITY]; strcpy(id, leases.state(1)->leaseId);
  clockMs += 200; assert(run(leases, port, renewal(id, 1), true));
  clockMs += 750; assert(!run(leases, port, renewal(id, 2), false));
  auto* state = const_cast<manual_control::State*>(leases.state(1));
  state->generation = UINT32_MAX;
  assert(!run(leases, port, start(leases, port), false));
  assert(!leases.anyActive());
}
int main(int argc, char** argv) {
  assert(argc == 2);
  if (!strcmp(argv[1], "user04")) user04();
  else if (!strcmp(argv[1], "deadline")) deadline();
  else if (!strcmp(argv[1], "start")) startReplayAndBoot();
  else if (!strcmp(argv[1], "direction")) sequenceAndDirection();
  else if (!strcmp(argv[1], "external")) externalStopsAndOverride();
  else if (!strcmp(argv[1], "malformed")) malformedAndFailedStart();
  else if (!strcmp(argv[1], "wrap")) clockWrapAndExhaustion();
  else assert(false);
}
