// Real service + executor + motor driver; fake peripherals and other controllers.
#include "control/bounded_action_service.h"
#include "control/safety_gate.h"
#include "control/angle_controller.h"
#include "control/shoulder_angle_controller.h"
#include "input/teleop_adapter.h"
#include "motion/motion_sequence.h"
#include "network/web_server.h"
#include "safety/safety_monitor.h"
#include <assert.h>
#include <string.h>
#include <limits>

MotorControl motorControl;
SystemStateManager systemState;
SafetyMonitor safetyMonitor;
SafetyGate safetyGate;
AngleController angleController;
ShoulderAngleController shoulderAngleController;
MotionSequence motionSequence;
TeleopAdapter teleopAdapter;
MotionBrainWebServer webServer;
static uint32_t nowMs = 0;
static uint32_t offDelay = 0;
static unsigned writes = 0;
uint32_t millis() { return nowMs; }
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
void ledcSetup(uint8_t, uint32_t, uint8_t) {}
void ledcAttachPin(uint8_t, uint8_t) {}
void ledcWrite(uint8_t, uint32_t duty) { ++writes; if (!duty) nowMs += offDelay; }

static bounded::Request request(const char* id) {
  bounded::Request r;
  strcpy(r.bootId, BoundedActions::executor()->bootId());
  strcpy(r.commandId, id);
  r.kind = bounded::Kind::Pulse;
  r.motor = 1;
  r.pwm = 25;
  r.durationMs = 100;
  return r;
}
static void dump(const bounded::Record& record) {
  String status, result, output;
  BoundedActions::appendStatusJson(status);
  BoundedActions::appendRecordJson(result, record);
  BoundedActions::appendOutputJson(output);
  puts(status.c_str()); puts(result.c_str()); puts(output.c_str());
}
static void pulse() {
  auto r = request("pulse-1");
  auto accepted = BoundedActions::submit(r);
  assert(accepted.code == bounded::SubmitCode::Accepted && BoundedActions::active());
  assert(webServer.clears == 1 && teleopAdapter.resets == 1);
  assert(writes == 5 && motorControl.hasPendingOutput());
  nowMs = 50;
  BoundedActions::tick(); motorControl.update();
  assert(motorControl.getSpeed(1) == 10);
  assert(motorControl.outputLog().at(5)->actionOrdinal == accepted.record->ordinal);
  assert(BoundedActions::submit(r).code == bounded::SubmitCode::Duplicate);
  assert(webServer.clears == 1 && writes == 6);
  offDelay = 3;
  nowMs = 100;
  BoundedActions::tick(); motorControl.update();
  assert(!BoundedActions::active() && !motorControl.hasPendingOutput());
  assert(accepted.record->state == bounded::State::TimedOut);
  assert(accepted.record->endedAtMs == 115 && accepted.record->overrunMs == 15);
  assert(motorControl.outputLog().at(6)->actionOrdinal == accepted.record->ordinal);
  motorControl.hardStop(2);
  assert(motorControl.outputLog().at(11)->actionOrdinal == 0);
  assert(BoundedActions::submit(r).code == bounded::SubmitCode::Duplicate);
  dump(*accepted.record);
}
static void guards() {
  auto r = request("guard-1");
  bool* busyFlags[] = {&angleController.active, &shoulderAngleController.active,
    &motionSequence.running, &teleopAdapter.active, &teleopAdapter.held};
  for (bool* flag : busyFlags) {
    *flag = true;
    assert(BoundedActions::submit(r).code == bounded::SubmitCode::Busy);
    *flag = false;
  }
  motorControl.setSpeed(3, 25);
  assert(BoundedActions::submit(r).code == bounded::SubmitCode::Busy);
  motorControl.hardStop(3);
  safetyGate.allowed = false;
  assert(BoundedActions::submit(r).code == bounded::SubmitCode::SafetyBlocked);
  safetyGate.allowed = true;
  assert(webServer.clears == 0 && teleopAdapter.resets == 0);
  auto result = BoundedActions::submit(r);
  assert(result.code == bounded::SubmitCode::Accepted);
  nowMs = 50; motorControl.update();
  teleopAdapter.held = true;
  BoundedActions::tick();
  assert(result.record->state == bounded::State::Cancelled);
  assert(result.record->reason == bounded::Reason::OperatorOverride);
  assert(!motorControl.hasPendingOutput() && !BoundedActions::active());
  dump(*result.record);
}
static void shoulder() {
  auto r = request("shoulder-invalid");
  r.kind = bounded::Kind::Shoulder; r.motor = 4; r.pwm = 0; r.percent = 20;
  r.targetDegrees = std::numeric_limits<float>::quiet_NaN();
  assert(BoundedActions::submit(r).code == bounded::SubmitCode::Invalid);
  assert(shoulderAngleController.starts == 0);
  r.targetDegrees = 181;
  auto rejectedByController = BoundedActions::submit(r);
  assert(rejectedByController.code == bounded::SubmitCode::Accepted);
  assert(rejectedByController.record->state == bounded::State::Failed);
  assert(rejectedByController.record->reason == bounded::Reason::StartFailed);
  assert(!BoundedActions::active() && !motorControl.hasPendingOutput());
  strcpy(r.commandId, "shoulder-reached"); r.targetDegrees = 45;
  auto result = BoundedActions::submit(r);
  assert(result.code == bounded::SubmitCode::Accepted && shoulderAngleController.active);
  assert(shoulderAngleController.target == 45 && shoulderAngleController.percent == 20);
  nowMs = 50; BoundedActions::tick(); motorControl.update();
  assert(motorControl.getSpeed(4) == 10);
  shoulderAngleController.active = false;
  shoulderAngleController.reason = ShoulderAngleStopReason::TARGET_REACHED;
  nowMs = 60; BoundedActions::tick();
  assert(result.record->state == bounded::State::Completed && result.record->targetReached);
  assert(result.record->outputOff && !motorControl.hasPendingOutput());
  dump(*result.record);
  strcpy(r.commandId, "shoulder-deadline");
  auto deadline = BoundedActions::submit(r);
  nowMs = 160; BoundedActions::tick();
  assert(deadline.record->state == bounded::State::TimedOut && shoulderAngleController.cancels == 1);
}
static void disabled() {
  assert(!BoundedActions::enabled());
  auto r = request("disabled");
  assert(BoundedActions::submit(r).code == bounded::SubmitCode::SafetyBlocked);
  assert(!BoundedActions::active() && writes == 5);
  assert(BoundedActions::executor()->registry().count() == 0);
  assert(webServer.clears == 0 && teleopAdapter.resets == 0 && safetyGate.calls == 0);
  String status; BoundedActions::appendStatusJson(status); puts(status.c_str());
}
static void unknownStop() {
  auto r = request("shoulder-unknown");
  r.kind = bounded::Kind::Shoulder; r.motor = 4; r.pwm = 0;
  r.percent = 20; r.targetDegrees = 45;
  const auto result = BoundedActions::submit(r);
  assert(result.code == bounded::SubmitCode::Accepted);
  shoulderAngleController.failCancel = true;
  nowMs = 100; BoundedActions::tick();
  assert(result.record->state == bounded::State::OutcomeUnknown && BoundedActions::active());
  assert(BoundedActions::beforeLegacyCommand(CommandType::STOP));
  assert(BoundedActions::beforeLegacyCommand(CommandType::DISARM));
  assert(!BoundedActions::beforeLegacyCommand(CommandType::MOTOR_RUN));
  assert(!BoundedActions::beforeLegacyCommand(CommandType::ARM));
  assert(BoundedActions::beforeLegacyCommand(CommandType::LIGHT_ON));
  assert(BoundedActions::active() && result.record->state == bounded::State::OutcomeUnknown);
  shoulderAngleController.failCancel = false;
  assert(BoundedActions::beforeLegacyCommand(CommandType::STOP));
  assert(result.record->outputOff && BoundedActions::active()); // unknown latch retained
  assert(!BoundedActions::beforeLegacyCommand(CommandType::MOTOR_RUN));
  dump(*result.record);
}
int main(int argc, char** argv) {
  assert(argc == 2);
  motorControl.init(); systemState.state = SystemState::ARMED;
  BoundedActions::init(); BoundedActions::init();
  if (!strcmp(argv[1], "disabled")) disabled();
  else {
    assert(BoundedActions::enabled());
    if (!strcmp(argv[1], "pulse")) pulse();
    else if (!strcmp(argv[1], "guards")) guards();
    else if (!strcmp(argv[1], "shoulder")) shoulder();
    else if (!strcmp(argv[1], "unknown")) unknownStop();
    else return 2;
  }
}
