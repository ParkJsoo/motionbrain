// Real environment, bridge, driver and bounded service; other services are fakes.
#include "control/finite_profile_service_environment.h"
#include "control/angle_controller.h"
#include "control/bounded_action_service.h"
#include "control/command_bus.h"
#include "control/dispatcher.h"
#include "control/guarded_routine_executor.h"
#include "control/safety_gate.h"
#include "control/shoulder_angle_controller.h"
#include "input/teleop_adapter.h"
#include "motion/motion_sequence.h"
#include "network/web_server.h"
#include "safety/safety_monitor.h"
#include <assert.h>
#include <string.h>

MotorControl motorControl;
SystemStateManager systemState;
SafetyMonitor safetyMonitor;
SafetyGate safetyGate;
Dispatcher dispatcher;
AngleController angleController;
ShoulderAngleController shoulderAngleController;
TeleopAdapter teleopAdapter;
MotionSequence motionSequence;
MotionBrainWebServer webServer;
CommandBus commandBus;
GuardedRoutineExecutorState GuardedRoutineExecutor::state = GuardedRoutineExecutorState::IDLE;
static uint32_t nowMs = 0;
static unsigned nonzeroWrites = 0;
static unsigned tokenCalls = 0;
static bool tokenOk = true;
static bool tokenZero = false;
uint32_t millis() { return nowMs; }
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
void ledcSetup(uint8_t, uint32_t, uint8_t) {}
void ledcAttachPin(uint8_t, uint8_t) {}
void ledcWrite(uint8_t, uint32_t duty) { if (duty) ++nonzeroWrites; }
using namespace finite_profile;
static bool issuer(Token& token) {
  ++tokenCalls; token.bytes[0] = tokenZero ? 0 : static_cast<uint8_t>(tokenCalls);
  return tokenOk;
}
static Profile profile() {
  Profile p; memset(p.catalogHash, 'a', 64); p.catalogHash[64] = 0;
  strcpy(p.profileId, "environment-test"); p.startLeadMs = 20; p.maxLatenessMs = 10;
  p.offConfirmWindowMs = 30; p.admissionWindowMs = 100; p.count = 2;
  p.points[0] = Point(0, 80); p.points[1] = Point(50, 0); return p;
}
struct Fixture {
  ServiceEnvironment environment;
  Profile accepted;
  DriverBridge bridge;
  Fixture() : environment(issuer), accepted(profile()),
      bridge(motorControl, environment, "boot-env", &accepted, 1) {}
  Request request(const char* id = "env-command") const {
    Request r; strcpy(r.bootId, "boot-env"); strcpy(r.commandId, id);
    r.expectedEpoch = bridge.epoch(); r.profile = accepted; return r;
  }
  PrepareResult prepare(const char* id = "env-command") { return bridge.prepare(request(id)); }
};
static void readiness() {
  Fixture f;
  bool* flags[] = {&safetyGate.ready, &dispatcher.ready, &angleController.ready,
      &shoulderAngleController.ready, &teleopAdapter.ready, &webServer.active};
  for (bool* flag : flags) {
    *flag = false;
    assert(!f.environment.allowed() && !f.environment.controllersIdle());
    assert(!f.prepare().granted && !motorControl.finiteProfileOwned());
    *flag = true;
  }
  assert(tokenCalls == 0 && f.bridge.count() == 0 && nonzeroWrites == 0);
  assert(f.prepare().granted);
}
static void admission() {
  Fixture f;
  bool* flags[] = {&angleController.active, &shoulderAngleController.active,
      &teleopAdapter.active, &teleopAdapter.held, &motionSequence.running, &webServer.manual};
  for (bool* flag : flags) {
    *flag = true;
    assert(!f.prepare().granted && !motorControl.finiteProfileOwned());
    *flag = false;
  }
  motionSequence.total = 1;
  assert(!f.prepare().granted && motionSequence.total == 1);
  motionSequence.total = 0;
  Command pending; pending.type = CommandType::MOTOR_RUN;
  assert(commandBus.enqueue(pending));
  assert(!f.prepare().granted && commandBus.size() == 1);
  assert(commandBus.dequeue(pending));
  for (auto state : {GuardedRoutineExecutorState::PREPARED, GuardedRoutineExecutorState::RUNNING,
                    GuardedRoutineExecutorState::ABORT_REQUESTED, GuardedRoutineExecutorState::BLOCKED}) {
    GuardedRoutineExecutor::state = state; assert(!f.prepare().granted);
  }
  GuardedRoutineExecutor::state = GuardedRoutineExecutorState::IDLE;
  assert(tokenCalls == 0 && nonzeroWrites == 0 && f.bridge.count() == 0);
  assert(webServer.clears == 0 && teleopAdapter.resets == 0);
  assert(f.prepare().granted);
}
static void safety() {
  Fixture f; safetyGate.allowed = false;
  assert(f.prepare().code == Code::SafetyBlocked && tokenCalls == 0);
  safetyGate.allowed = true; auto p = f.prepare(); assert(p.granted);
  safetyGate.allowed = false;
  assert(f.bridge.execute("boot-env", "env-command", p.record->request.expectedEpoch, p.token).code != Code::Accepted);
  safetyGate.allowed = true;
  f.bridge.execute("boot-env", "env-command", p.record->request.expectedEpoch, p.token);
  assert(nonzeroWrites == 0 && p.record->state == State::Failed);
}
static void boundedOwner() {
  Fixture f; bounded::Request old;
  strcpy(old.bootId, BoundedActions::executor()->bootId()); strcpy(old.commandId, "bounded");
  old.kind = bounded::Kind::Pulse; old.motor = 2; old.pwm = 30; old.durationMs = 100;
  assert(BoundedActions::submit(old).code == bounded::SubmitCode::Accepted);
  assert(!f.prepare().granted && tokenCalls == 0);
  BoundedActions::cancel(bounded::Reason::Cancelled);
  assert(f.prepare().granted);
}
static void oldBounded() {
  Fixture f; bounded::Request old;
  strcpy(old.bootId, BoundedActions::executor()->bootId()); strcpy(old.commandId, "old-bounded");
  old.kind = bounded::Kind::Pulse; old.motor = 2; old.pwm = 30; old.durationMs = 100;
  auto oldResult = BoundedActions::submit(old); assert(oldResult.code == bounded::SubmitCode::Accepted);
  BoundedActions::cancel(bounded::Reason::Cancelled);
  auto p = f.prepare(); assert(p.granted);
  assert(f.bridge.execute("boot-env", "env-command", p.record->request.expectedEpoch, p.token).code == Code::Accepted);
  nowMs = p.record->plannedStartAt; f.bridge.tick();
  const uint32_t lease = f.bridge.leaseId(); const auto writes = motorControl.outputLog().count();
  bounded::Request invalid = old;
  strcpy(invalid.bootId, "wrong-boot");
  assert(BoundedActions::submit(invalid).code == bounded::SubmitCode::BootMismatch);
  invalid = old; strcpy(invalid.commandId, "invalid-new"); invalid.pwm = 0;
  assert(BoundedActions::submit(invalid).code == bounded::SubmitCode::Invalid);
  assert(BoundedActions::submit(old).code == bounded::SubmitCode::Duplicate);
  old.pwm = 31; assert(BoundedActions::submit(old).code == bounded::SubmitCode::Conflict);
  assert(motorControl.finiteProfileLeaseMatches(lease) && !motorControl.finiteProfileLeaseSealed(lease));
  assert(motorControl.getSpeed(2) == 80 && motorControl.outputLog().count() == writes);
  assert(p.record->state == State::Running && oldResult.record->state == bounded::State::Cancelled);
  strcpy(old.commandId, "new-bounded");
  assert(BoundedActions::submit(old).code == bounded::SubmitCode::Busy);
  assert(motorControl.finiteProfileLeaseSealed(lease) && motorControl.getSpeed(2) == 0);
  f.bridge.tick(); assert(p.record->state == State::Cancelled && p.record->quietHeld);
}
static void quiet() {
  Fixture f; auto p = f.prepare(); assert(p.granted);
  assert(f.bridge.execute("boot-env", "env-command", p.record->request.expectedEpoch, p.token).code == Code::Accepted);
  nowMs = p.record->plannedStartAt; f.bridge.tick(); assert(nonzeroWrites == 1);
  nowMs += 50; f.bridge.tick();
  assert(p.record->state == State::Completed && p.record->quietHeld && motorControl.finiteProfileOwned());
  assert(f.environment.controllersIdle()); // Own finite output/quiet is not a competing controller.
  assert(f.bridge.release("boot-env", "env-command", p.record->quietEpoch).code == Code::Accepted);
  assert(!motorControl.finiteProfileOwned() && f.bridge.idle());
}
static void passiveQueue() {
  Fixture f; auto p = f.prepare(); assert(p.granted);
  for (auto type : {CommandType::LIGHT_ON, CommandType::LIGHT_OFF,
                    CommandType::LIGHT_TOGGLE, CommandType::ROUTINE_DRY_RUN}) {
    Command command; command.type = type; assert(commandBus.enqueue(command));
  }
  assert(!commandBus.isEmpty() && !commandBus.hasPendingControlCommand());
  f.bridge.tick();
  assert(p.record->state == State::Reserved && !f.bridge.blocked());
  assert(f.bridge.execute("boot-env", "env-command", p.record->request.expectedEpoch, p.token).code == Code::Accepted);
  nowMs = p.record->plannedStartAt; f.bridge.tick();
  assert(p.record->state == State::Running && nonzeroWrites == 1);
  nowMs += 50; f.bridge.tick();
  assert(p.record->state == State::Completed && p.record->traceComplete && !f.bridge.blocked());
  assert(f.bridge.release("boot-env", "env-command", p.record->quietEpoch).code == Code::Accepted);
  assert(!motorControl.finiteProfileOwned() && commandBus.size() == 4);
  // Scan a wrapped queue and keep unknown/control commands conservative.
  Command command;
  for (unsigned i = 0; i < 4; ++i) assert(commandBus.dequeue(command));
  for (unsigned i = 0; i < 20; ++i) {
    command.type = CommandType::LIGHT_ON; assert(commandBus.enqueue(command)); assert(commandBus.dequeue(command));
  }
  command.type = CommandType::LIGHT_ON; assert(commandBus.enqueue(command));
  command.type = CommandType::ARM; assert(commandBus.enqueue(command));
  assert(commandBus.hasPendingControlCommand());
  assert(commandBus.dequeue(command) && commandBus.hasPendingControlCommand());
  assert(commandBus.dequeue(command) && !commandBus.hasPendingControlCommand());
}
static void tokens() {
  ServiceEnvironment absent; Token token; token.bytes[0] = 99;
  assert(!absent.allowed() && !absent.mintToken(token) && token.bytes[0] == 0);
  Fixture f; tokenOk = false;
  auto bad = f.prepare("issuer-failed");
  assert(!bad.granted && bad.record->reason == Reason::TokenFailure && !motorControl.finiteProfileOwned());
  tokenOk = true; tokenZero = true;
  auto zero = f.prepare("issuer-zero");
  assert(!zero.granted && zero.record->reason == Reason::TokenFailure);
  tokenZero = false; assert(f.prepare("issuer-good").granted);
  assert(nonzeroWrites == 0);
}
static void lateBusy() {
  Fixture f; auto p = f.prepare(); assert(p.granted);
  shoulderAngleController.active = true; // A violated external-serialization/ownership precondition.
  f.bridge.execute("boot-env", "env-command", p.record->request.expectedEpoch, p.token);
  shoulderAngleController.active = false;
  f.bridge.execute("boot-env", "env-command", p.record->request.expectedEpoch, p.token);
  assert(nonzeroWrites == 0 && p.record->state != State::Running && f.bridge.blocked());
}
int main(int argc, char** argv) {
  assert(argc == 2); assert(motorControl.init()); systemState.state = SystemState::ARMED;
  BoundedActions::init();
  if (!strcmp(argv[1], "readiness")) readiness();
  else if (!strcmp(argv[1], "admission")) admission();
  else if (!strcmp(argv[1], "safety")) safety();
  else if (!strcmp(argv[1], "bounded")) boundedOwner();
  else if (!strcmp(argv[1], "old-bounded")) oldBounded();
  else if (!strcmp(argv[1], "quiet")) quiet();
  else if (!strcmp(argv[1], "tokens")) tokens();
  else if (!strcmp(argv[1], "passive-queue")) passiveQueue();
  else if (!strcmp(argv[1], "late-busy")) lateBusy();
  else return 2;
}
