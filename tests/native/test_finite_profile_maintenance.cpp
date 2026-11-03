// Maintenance never dispatches profile output. Real core, fake time and port.
#include "control/finite_profile_executor.h"
#include <assert.h>
#include <string.h>
#include <vector>

#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_CORE
using namespace finite_profile;
static Profile profile() {
  Profile p; memset(p.catalogHash, 'a', 64); p.catalogHash[64] = 0;
  strcpy(p.profileId, "maintenance-test");
  p.startLeadMs = 20; p.maxLatenessMs = 10;
  p.offConfirmWindowMs = 30; p.admissionWindowMs = 100; p.count = 4;
  p.points[0] = Point(0, 10); p.points[1] = Point(50, 20);
  p.points[2] = Point(100, 10); p.points[3] = Point(150, 0);
  return p;
}
struct FakePort : Port {
  uint32_t clock = 0;
  bool safe = true;
  bool off = true;
  Proof proof = Proof::Confirmed;
  unsigned tokens = 0;
  unsigned applies = 0;
  unsigned stops = 0;
  unsigned probes = 0;
  std::vector<int16_t> writes;
  uint32_t now() const override { return clock; }
  bool allowed() const override { return safe; }
  bool idle() const override { return off; }
  bool apply(uint8_t motor, int16_t pwm, uint32_t& at) override {
    assert(motor == 2); ++applies; at = clock; off = pwm == 0; writes.push_back(pwm); return true;
  }
  bool stop(uint32_t& at) override {
    ++stops; at = clock; off = true; writes.push_back(0); return true;
  }
  Proof offProof() override { ++probes; return proof; }
  bool mintToken(Token& token) override {
    ++tokens;
    for (unsigned i = 0; i < sizeof(token.bytes); ++i) token.bytes[i] = static_cast<uint8_t>(tokens + i);
    return true;
  }
  size_t nonzero() const {
    size_t count = 0; for (auto pwm : writes) if (pwm) ++count; return count;
  }
};
struct Fixture {
  FakePort port;
  Profile accepted;
  Executor executor;
  Fixture() : accepted(profile()), executor(port, "boot", &accepted, 1) {}
  PrepareResult prepare(const char* id = "action", uint32_t now = 0) {
    port.clock = now; Request request;
    strcpy(request.bootId, "boot"); strcpy(request.commandId, id);
    request.expectedEpoch = executor.epoch(); request.profile = accepted;
    PrepareResult result = executor.prepare(request); assert(result.granted); return result;
  }
  PrepareResult start(const char* id = "action", uint32_t now = 0) {
    PrepareResult result = prepare(id, now);
    assert(executor.execute("boot", id, result.record->request.expectedEpoch, result.token).code == Code::Accepted);
    return result;
  }
  void poll(uint32_t now) { port.clock = now; executor.poll(); }
  void event(const Record* record, uint8_t index) {
    port.clock = record->plannedStartAt + accepted.points[index].offsetMs;
    assert(executor.event("boot", record->request.commandId,
                          record->request.expectedEpoch, index).code == Code::Accepted);
  }
  void cancel(const Record* record, uint32_t now) {
    port.clock = now;
    assert(executor.cancel("boot", record->request.commandId,
                           record->request.expectedEpoch).code == Code::Accepted);
  }
};
static void deadline() {
  for (unsigned alreadyStarted = 0; alreadyStarted < 2; ++alreadyStarted) {
    Fixture f; const Record* r = f.start().record;
    if (alreadyStarted) f.event(r, 0);
    const uint32_t due = r->plannedStartAt + f.accepted.points[r->nextEventIndex].offsetMs;
    f.poll(due); f.poll(due + f.accepted.maxLatenessMs);
    assert(r->state == State::Running && f.port.nonzero() == alreadyStarted && f.port.stops == 0);
    assert(f.port.applies == alreadyStarted);
    f.poll(due + f.accepted.maxLatenessMs + 1);
    assert(r->state == State::Failed && r->reason == Reason::DeadlineMissed);
    assert(f.port.nonzero() == alreadyStarted && f.port.stops == 1);
    assert(r->quietHeld == static_cast<bool>(alreadyStarted));
  }
}
static void safety() {
  for (unsigned stage = 0; stage < 3; ++stage) {
    Fixture f; const auto prepared = stage ? f.start() : f.prepare(); const Record* r = prepared.record;
    if (stage == 2) f.event(r, 0);
    f.port.safe = false; f.poll(f.port.clock + 1);
    assert(r->state == State::Failed && r->reason == Reason::SafetyChanged);
    assert(f.port.nonzero() == (stage == 2 ? 1U : 0U));
    assert(f.port.stops == (stage ? 1U : 0U));
    assert(f.port.off);
  }
}
static void expiry() {
  Fixture f; const auto prepared = f.prepare(); const Record* r = prepared.record;
  f.poll(99); assert(r->state == State::Reserved);
  f.poll(100); assert(r->state == State::Expired && r->reason == Reason::ReservationExpired);
  assert(f.port.writes.empty() && f.executor.idle());
  assert(f.executor.execute("boot", "action", r->request.expectedEpoch, prepared.token).code != Code::Accepted);
}
static void failedReserved() {
  Fixture f; const auto prepared = f.prepare(); const Record* r = prepared.record;
  f.port.clock = 5;
  assert(f.executor.scheduleFailed("boot", "action", r->request.expectedEpoch).code == Code::Accepted);
  assert(r->state == State::Failed && r->reason == Reason::ScheduleFailed && r->decisionAt == 5);
  assert(!r->executed && !r->quietHeld && f.port.writes.empty() && f.executor.idle());
  assert(f.executor.execute("boot", "action", r->request.expectedEpoch, prepared.token).code != Code::Accepted);
  const auto duplicate = f.executor.prepare(r->request);
  assert(duplicate.code == Code::Duplicate && !duplicate.granted);
}
static void failedRunning() {
  Fixture f; const auto prepared = f.start(); const Record* r = prepared.record;
  f.event(r, 0); f.port.clock = 25;
  assert(f.executor.scheduleFailed("boot", "action", r->request.expectedEpoch).code == Code::Accepted);
  assert(r->state == State::Failed && r->reason == Reason::ScheduleFailed && r->quietHeld);
  assert(r->executed && r->softwareOffConfirmed && f.port.nonzero() == 1 && f.port.stops == 1);
  const size_t n = f.port.writes.size();
  assert(f.executor.execute("boot", "action", r->request.expectedEpoch, prepared.token).code != Code::Accepted);
  f.port.clock = 70;
  assert(f.executor.event("boot", "action", r->request.expectedEpoch, 1).code != Code::Accepted);
  assert(f.port.writes.size() == n);
  assert(f.executor.release("boot", "action", r->quietEpoch).code == Code::Accepted);
}
static void identityScope() {
  Fixture f; const Record* old = f.prepare("old").record;
  assert(f.executor.scheduleFailed("boot", "old", old->request.expectedEpoch).code == Code::Accepted);
  const Record* current = f.start("current", 100).record; f.event(current, 0);
  const size_t n = f.port.writes.size(); const auto epoch = f.executor.epoch();
  assert(f.executor.scheduleFailed("other-boot", "current", current->request.expectedEpoch).code != Code::Accepted);
  assert(f.executor.scheduleFailed("boot", "unknown", current->request.expectedEpoch).code != Code::Accepted);
  assert(f.executor.scheduleFailed("boot", "current", current->request.expectedEpoch + 1).code != Code::Accepted);
  assert(f.executor.scheduleFailed("boot", "old", old->request.expectedEpoch).code == Code::Duplicate);
  assert(f.executor.scheduleFailed("boot", "old", current->request.expectedEpoch).code != Code::Accepted);
  assert(current->state == State::Running && !f.port.off);
  assert(f.port.writes.size() == n && f.executor.epoch() == epoch);
}
static void pendingFailure() {
  Fixture f; const Record* r = f.start().record; f.event(r, 0);
  f.port.proof = Proof::Pending; f.cancel(r, 25);
  assert(r->state == State::Stopping && r->reason == Reason::Cancelled);
  const auto decision = r->decisionAt; const auto reason = r->reason;
  const auto epoch = f.executor.epoch(); const auto checks = f.port.probes;
  assert(f.executor.scheduleFailed("boot", "action", epoch).code == Code::EpochMismatch);
  assert(r->state == State::Stopping && f.port.probes == checks);
  f.poll(26); assert(r->state == State::Stopping);
  assert(f.executor.scheduleFailed("boot", "action", r->request.expectedEpoch).code == Code::Accepted);
  assert(r->state == State::OutcomeUnknown && f.executor.blocked() && r->quietHeld);
  assert(r->reason == reason && r->decisionAt == decision && r->offWriteAt == 25);
  f.port.proof = Proof::Confirmed; f.poll(27);
  assert(r->state == State::OutcomeUnknown && !r->softwareOffConfirmed);
  assert(f.executor.retryOff("boot", "action").code == Code::Accepted);
  assert(r->state == State::OutcomeUnknown && f.executor.blocked() && r->softwareOffConfirmed);
  assert(r->reason == reason && r->decisionAt == decision && r->offWriteAt == 25);
  assert(f.executor.release("boot", "action", r->quietEpoch).code == Code::Rejected);
}
static void pendingConfirmation() {
  Fixture f; const Record* r = f.start().record; f.event(r, 0);
  f.port.proof = Proof::Pending; f.cancel(r, 25);
  f.poll(26); assert(r->state == State::Stopping);
  f.port.proof = Proof::Confirmed; f.poll(27);
  assert(r->state == State::Cancelled && r->softwareOffConfirmed && r->offConfirmedAt == 27);
  assert(f.port.nonzero() == 1 && f.port.stops == 1 && r->decisionAt == 25);
  assert(f.executor.scheduleFailed("boot", "action", r->request.expectedEpoch).code == Code::Duplicate);
  assert(r->state == State::Cancelled && !f.executor.blocked());
}
static void pendingDeadline() {
  Fixture f; const Record* r = f.start().record; f.event(r, 0);
  f.port.proof = Proof::Pending; f.cancel(r, 25);
  f.poll(54); assert(r->state == State::Stopping);
  const auto checks = f.port.probes; f.poll(55);
  assert(r->state == State::OutcomeUnknown && f.port.probes == checks && f.executor.blocked());
  assert(r->decisionAt == 25 && r->reason == Reason::Cancelled);
  assert(f.port.nonzero() == 1 && f.port.stops == 1);
}
static void wrap() {
  {
    Fixture f; const Record* r = f.start("action", UINT32_MAX - 10).record;
    assert(r->plannedStartAt == 9); f.poll(9); f.poll(19);
    assert(r->state == State::Running && f.port.writes.empty());
    f.poll(20); assert(r->state == State::Failed && r->reason == Reason::DeadlineMissed);
    assert(f.port.nonzero() == 0 && f.port.stops == 1);
  }
  {
    Fixture f; const Record* r = f.prepare("action", UINT32_MAX - 50).record;
    f.poll(48); assert(r->state == State::Reserved);
    f.poll(49); assert(r->state == State::Expired && f.port.writes.empty());
  }
  {
    Fixture f; const Record* r = f.start("action", UINT32_MAX - 30).record;
    f.event(r, 0); f.port.proof = Proof::Pending; f.cancel(r, UINT32_MAX - 5);
    f.port.proof = Proof::Confirmed; f.poll(2);
    assert(r->state == State::Cancelled && r->offConfirmedAt == 2 && r->softwareOffConfirmed);
  }
}
#endif

int main(int argc, char** argv) {
  assert(argc == 2);
#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_CORE
  if (!strcmp(argv[1], "deadline")) deadline();
  else if (!strcmp(argv[1], "safety")) safety();
  else if (!strcmp(argv[1], "expiry")) expiry();
  else if (!strcmp(argv[1], "reserved")) failedReserved();
  else if (!strcmp(argv[1], "running")) failedRunning();
  else if (!strcmp(argv[1], "identity")) identityScope();
  else if (!strcmp(argv[1], "pending_failure")) pendingFailure();
  else if (!strcmp(argv[1], "pending_confirmation")) pendingConfirmation();
  else if (!strcmp(argv[1], "pending_deadline")) pendingDeadline();
  else if (!strcmp(argv[1], "wrap")) wrap();
  else return 2;
#else
  assert(!strcmp(argv[1], "disabled"));
#endif
}
