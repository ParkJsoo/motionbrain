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

#include "input/teleop_adapter.h"
#include "control/angle_controller.h"
#include "control/shoulder_angle_controller.h"
#include "control/command_bus.h"
#include "control/dispatcher.h"
#include "control/event_log.h"
#include "control/finite_profile_driver_bridge.h"
HardwareSerial Serial1;
EventLog eventLog;
AngleController angleController;
ShoulderAngleController shoulderAngleController;
MotionSequence motionSequence;
CommandBus commandBus;
Dispatcher dispatcher;
TeleopAdapter teleop;

static std::string frame(bool deadman, unsigned seq) {
  return std::string("{\"type\":\"teleop\",\"ts_ms\":1,\"seq\":") + std::to_string(seq) +
    ",\"session\":1,\"deadman\":" + (deadman ? "true" : "false") +
    ",\"reach\":0,\"lift\":0,\"twist\":0,\"grip_open\":true,\"grip_close\":false,\"led_toggle_seq\":0}\n";
}
static void init() {
  assert(motorControl.init()); systemState.state = SystemState::ARMED; nowMs = 10;
  assert(teleop.init(&systemState, &motorControl, &motionSequence, &safetyMonitor,
                    &angleController, &shoulderAngleController, &commandBus, &dispatcher));
}
static void batch() {
  const auto previous = teleop.getPacketsReceived();
  Serial1.append(frame(true, 1) + frame(false, 2));
  teleop.processInput();
  assert(Serial1.available() == 0 && teleop.getPacketsReceived() == previous + 2);
  assert(teleop.getParseErrors() == 0 && !teleop.isDeadmanHeld());
}
static void disabled() {
  init(); batch(); teleop.updateControl();
  assert(!teleop.isControlActive() && !motorControl.hasPendingOutput());
  assert(writes.size() == 5);
  assertMatches(motorControl);
}
#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER
using namespace finite_profile;
struct Environment : DriverEnvironment {
  bool allowed() const override { return true; }
  bool controllersIdle() const override { return true; }
  bool mintToken(Token& token) override { for (auto& b : token.bytes) b = 7; return true; }
};
static Profile profile() {
  Profile p; memset(p.catalogHash, 'a', 64); p.catalogHash[64] = 0; strcpy(p.profileId, "teleop-test");
  p.startLeadMs = 20; p.maxLatenessMs = 10; p.offConfirmWindowMs = 30; p.admissionWindowMs = 100;
  p.count = 2; p.points[0] = Point(0, 80); p.points[1] = Point(100, 0); return p;
}
static void transient(const char* mode) {
  init(); Environment environment; Profile p = profile(); DriverBridge bridge(motorControl, environment, "boot-A", &p, 1);
  Request request; strcpy(request.bootId, "boot-A"); strcpy(request.commandId, "teleop-edge");
  request.expectedEpoch = bridge.epoch(); request.profile = p;
  const auto prepared = bridge.prepare(request); assert(prepared.granted);
  const uint32_t lease = bridge.leaseId(); const Record* r = prepared.record;
  if (strcmp(mode, "reserved")) {
    assert(bridge.execute("boot-A", "teleop-edge", request.expectedEpoch, prepared.token).code == Code::Accepted);
    nowMs = r->plannedStartAt; bridge.tick(); assert(motorControl.getSpeed(2) == 80);
  }
  if (!strcmp(mode, "quiet")) {
    nowMs = r->plannedOffAt; bridge.tick(); assert(r->state == State::Completed && r->quietHeld);
  }
  const size_t before = writes.size(); batch();
  // The final received frame says idle, but the earlier deadman edge was retained.
  assert(motorControl.finiteProfileLeaseSealed(lease));
  teleop.updateControl();
  assert(!teleop.isControlActive() && !motorControl.hasPendingOutput());
  for (size_t i = before; i < writes.size(); ++i) assert(writes[i].signedPwm == 0);
  bridge.tick();
  if (!strcmp(mode, "reserved")) {
    assert(r->state == State::Cancelled && !r->executed && bridge.leaseId() == 0);
    assert(bridge.execute("boot-A", "teleop-edge", request.expectedEpoch, prepared.token).code != Code::Accepted);
  } else {
    assert(r->quietHeld && bridge.leaseId() == lease);
    assert(r->state == (!strcmp(mode, "quiet") ? State::Completed : State::Cancelled));
    assert(bridge.release("boot-A", "teleop-edge", r->quietEpoch).code == Code::Accepted);
  }
  // A denied owner-period input must not poison the output cache: after scoped
  // release the same grip input is applied through the real legacy driver.
  Serial1.append(frame(true, 3)); teleop.processInput(); teleop.updateControl();
  assert(teleop.isControlActive() && motorControl.hasPendingOutput());
  nowMs += 50; motorControl.update(); assert(motorControl.getSpeed(1) == 10);
  assertMatches(motorControl);
}
static void blocked_cache() {
  init(); uint32_t lease = 0; assert(motorControl.acquireFiniteProfileLease(41, lease));
  Serial1.append(frame(true, 1)); teleop.processInput(); teleop.updateControl();
  assert(motorControl.finiteProfileLeaseSealed(lease));
  assert(!teleop.isControlActive() && !motorControl.hasPendingOutput());
  assert(motorControl.releaseFiniteProfileLease(lease));
  teleop.updateControl(); // Same parsed frame; no new packet to refresh the cache.
  assert(teleop.isControlActive() && motorControl.hasPendingOutput());
  nowMs += 50; motorControl.update(); assert(motorControl.getSpeed(1) == 10);
  assertMatches(motorControl);
}
#endif
int main(int argc, char** argv) {
  assert(argc == 2);
  if (!strcmp(argv[1], "disabled")) disabled();
#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER
  else if (!strcmp(argv[1], "reserved") || !strcmp(argv[1], "running") || !strcmp(argv[1], "quiet")) transient(argv[1]);
  else if (!strcmp(argv[1], "cache")) blocked_cache();
#endif
  else return 2;
}
