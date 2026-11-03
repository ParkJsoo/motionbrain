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

#include "control/finite_profile_driver_bridge.h"
#include <stdio.h>
using namespace finite_profile;
struct Environment : DriverEnvironment {
  unsigned tokenNumber = 0;
  bool allowed() const override { return true; }
  bool controllersIdle() const override { return true; }
  bool mintToken(Token& token) override { token.bytes[0] = ++tokenNumber; return true; }
};
static Profile profile() {
  Profile p; memset(p.catalogHash, 'a', 64); p.catalogHash[64] = 0;
  strcpy(p.profileId, "capture-test"); p.startLeadMs = 20; p.maxLatenessMs = 10;
  p.offConfirmWindowMs = 30; p.admissionWindowMs = 100; p.count = 4;
  p.points[0] = Point(0, 80); p.points[1] = Point(50, 80);
  p.points[2] = Point(100, 40); p.points[3] = Point(150, 0); return p;
}
static Request request(DriverBridge& bridge, const Profile& p, const char* id) {
  Request r; strcpy(r.bootId, "boot-capture"); strcpy(r.commandId, id);
  r.expectedEpoch = bridge.epoch(); r.profile = p; return r;
}
struct Fixture {
  MotorControl driver; Environment environment; Profile accepted; DriverBridge bridge;
  Fixture() : accepted(profile()), bridge(driver, environment, "boot-capture", &accepted, 1) {
    assert(driver.init()); systemState.state = SystemState::ARMED;
  }
  PrepareResult prepare(const char* id = "captured") { return bridge.prepare(request(bridge, accepted, id)); }
  const Record* start() {
    const auto p = prepare(); assert(p.granted);
    assert(bridge.execute("boot-capture", "captured", p.record->request.expectedEpoch, p.token).code == Code::Accepted);
    return p.record;
  }
  void finish(const Record* r) {
    for (uint8_t i = r->nextEventIndex; i < accepted.count; ++i) {
      nowMs = r->plannedStartAt + accepted.points[i].offsetMs; bridge.tick();
    }
  }
};
static void matchCapture(const DriverOutputEvidence* c, size_t start) {
  assert(c && c->captureComplete && c->count == writes.size() - start);
  assert(c->totalWrites == c->count && c->dropped == 0);
  for (size_t i = 0; i < c->count; ++i) {
    assert(c->entries[i].sequence == i + 1);
    assert(c->entries[i].atMs == writes[start+i].time);
    assert(c->entries[i].pwm == writes[start+i].signedPwm);
    assert(c->entries[i].motor == writes[start+i].motor);
  }
}
static void repeated() {
  Fixture f; const size_t start = writes.size(); const Record* r = f.start();
  const auto* c = f.bridge.outputEvidence("boot-capture", "captured");
  assert(c && c->opened && !c->closed && c->lease == f.bridge.leaseId());
  assert(c->requestEpoch == r->request.expectedEpoch && c->ordinal == 1);
  f.finish(r); assert(r->state == State::Completed && r->traceComplete);
  assert(c->count == f.accepted.count + 5 && !c->closed);
  assert(c->entries[0].pwm == 80 && c->entries[1].pwm == 80);
  assert(c->entries[0].atMs != c->entries[1].atMs);
  assert(f.driver.outputLog().count() == writes.size() - 1); // Old diagnostic ring deduplicates.
  f.driver.hardStopAll(); assert(c->count == f.accepted.count + 10);
  matchCapture(c, start);
  assert(f.bridge.release("boot-capture", "captured", r->quietEpoch).code == Code::Accepted);
  assert(c->closed && c->closedAt == nowMs && c->captureComplete);
  const size_t count = c->count;
  for (unsigned i = 0; i < 40; ++i) f.driver.hardStopAll();
  assert(f.driver.outputLog().dropped() > 0 && c->count == count && c->captureComplete);
  assert(f.bridge.outputEvidence("boot-capture", "captured") == c);
  assert(!f.bridge.outputEvidence("other-boot", "captured"));
}
static void externalStop() {
  Fixture f; const size_t start = writes.size(); const Record* r = f.start();
  nowMs = r->plannedStartAt; f.bridge.tick(); f.driver.emergencyStop(); f.bridge.tick();
  const auto* c = f.bridge.outputEvidence("boot-capture", "captured");
  assert(r->state == State::Cancelled && !r->traceComplete && r->quietHeld);
  matchCapture(c, start); assert(c->captureComplete && !c->closed);
  for (size_t i = 1; i < c->count; ++i) assert(c->entries[i].reason == bounded::OutputReason::ForcedOff);
  assert(f.bridge.release("boot-capture", "captured", r->quietEpoch).code == Code::Accepted);
  assert(c->closed && c->captureComplete && !r->traceComplete);
}
static void overflow() {
  Fixture f; const Record* r = f.start(); f.finish(r);
  const auto* c = f.bridge.outputEvidence("boot-capture", "captured");
  const DriverOutputEntry first = c->entries[0];
  for (unsigned i = 0; i < 20; ++i) f.driver.hardStopAll();
  assert(c->count == 64 && c->dropped > 0 && !c->captureComplete);
  assert(c->totalWrites == c->count + c->dropped && !f.driver.hasPendingOutput());
  assert(c->entries[0].atMs == first.atMs && c->entries[0].pwm == first.pwm);
  // Control outcome is not retroactively rewritten as a trace-quality result.
  assert(r->state == State::Completed && r->timingConformant && r->traceComplete);
  assert(f.bridge.release("boot-capture", "captured", r->quietEpoch).code == Code::Accepted);
  assert(c->closed && !c->captureComplete);
}
static void retention() {
  MotorControl driver; assert(driver.init()); systemState.state = SystemState::ARMED;
  Environment env; Profile p = profile(); const DriverOutputEvidence* first = nullptr;
  for (unsigned i = 0; i < 32; ++i) {
    DriverBridge bridge(driver, env, "boot-capture", &p, 1);
    char id[32]; snprintf(id, sizeof(id), "kept-%u", i);
    auto prepared = bridge.prepare(request(bridge, p, id)); assert(prepared.granted);
    auto capture = bridge.outputEvidence("boot-capture", id); assert(capture && capture->ordinal == 1);
    if (!i) first = capture;
    assert(bridge.cancel("boot-capture", id, prepared.record->request.expectedEpoch).code == Code::Accepted);
    assert(capture->closed && capture->count == 0 && !driver.finiteProfileOwned());
  }
  assert(driver.finiteProfileEvidenceCount() == 32 && first->closed);
  DriverBridge rejected(driver, env, "boot-capture", &p, 1);
  const size_t before = writes.size();
  auto full = rejected.prepare(request(rejected, p, "capacity-exhausted"));
  assert(!full.granted && full.code == Code::Rejected && !driver.finiteProfileOwned());
  assert(!rejected.outputEvidence("boot-capture", "capacity-exhausted") && writes.size() == before);
  assert(driver.findFiniteProfileEvidence("boot-capture", "kept-0") == first);
}
static void identityReuse() {
  MotorControl driver; assert(driver.init()); systemState.state = SystemState::ARMED;
  Environment env; Profile p = profile(); const DriverOutputEvidence* old = nullptr;
  {
    DriverBridge bridge(driver, env, "boot-capture", &p, 1);
    auto prepared = bridge.prepare(request(bridge, p, "same-id")); assert(prepared.granted);
    old = bridge.outputEvidence("boot-capture", "same-id");
    bridge.cancel("boot-capture", "same-id", prepared.record->request.expectedEpoch);
  }
  assert(old && old->closed);
  DriverBridge recreated(driver, env, "boot-capture", &p, 1);
  auto rejected = recreated.prepare(request(recreated, p, "same-id"));
  assert(!rejected.granted && !recreated.outputEvidence("boot-capture", "same-id"));
  assert(driver.finiteProfileEvidenceCount() == 1 && !driver.finiteProfileOwned());
  assert(driver.findFiniteProfileEvidence("boot-capture", "same-id") == old);
}
static void destruction() {
  MotorControl driver; assert(driver.init()); systemState.state = SystemState::ARMED;
  Environment env; Profile p = profile(); const DriverOutputEvidence* capture = nullptr;
  const size_t start = writes.size(); uint32_t lease = 0;
  {
    DriverBridge bridge(driver, env, "boot-capture", &p, 1);
    auto prepared = bridge.prepare(request(bridge, p, "destroyed")); assert(prepared.granted);
    capture = bridge.outputEvidence("boot-capture", "destroyed"); lease = bridge.leaseId();
    bridge.execute("boot-capture", "destroyed", prepared.record->request.expectedEpoch, prepared.token);
    nowMs = prepared.record->plannedStartAt; bridge.tick();
  }
  assert(driver.finiteProfileLeaseSealed(lease) && !capture->closed && capture->captureComplete);
  driver.hardStopAll(); matchCapture(capture, start);
  assert(driver.releaseFiniteProfileLease(lease)); assert(capture->closed);
}
static void maximum() {
  MotorControl driver; assert(driver.init()); systemState.state = SystemState::ARMED;
  Environment env; Profile p = profile(); p.count = 32;
  for (uint8_t i = 0; i < p.count; ++i) p.points[i] = Point(i*50, i == 31 ? 0 : 80);
  DriverBridge bridge(driver, env, "boot-capture", &p, 1);
  auto prepared = bridge.prepare(request(bridge, p, "maximum")); assert(prepared.granted);
  bridge.execute("boot-capture", "maximum", prepared.record->request.expectedEpoch, prepared.token);
  for (uint8_t i = 0; i < p.count; ++i) { nowMs = prepared.record->plannedStartAt + i*50; bridge.tick(); }
  auto c = bridge.outputEvidence("boot-capture", "maximum");
  assert(prepared.record->state == State::Completed && c->count == 37 && c->captureComplete);
  assert(sizeof(DriverEvidenceStore) <= 40*1024);
  printf("store=%zu driver=%zu bridge=%zu\n", sizeof(DriverEvidenceStore), sizeof(MotorControl), sizeof(DriverBridge));
}
int main(int argc, char** argv) {
  assert(argc == 2);
  if (!strcmp(argv[1], "repeated")) repeated();
  else if (!strcmp(argv[1], "external")) externalStop();
  else if (!strcmp(argv[1], "overflow")) overflow();
  else if (!strcmp(argv[1], "retention")) retention();
  else if (!strcmp(argv[1], "identity")) identityReuse();
  else if (!strcmp(argv[1], "destruction")) destruction();
  else if (!strcmp(argv[1], "maximum")) maximum();
  else return 2;
}
