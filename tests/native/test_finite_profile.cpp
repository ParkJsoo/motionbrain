// Production finite-profile contract/core with fake time and output only.
// These tests do not establish electrical stopping or physical timing guarantees.
#include "control/finite_profile_contract.h"
#include <cassert>
#include <cstdio>
#include <cstring>

#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_CORE
#include "control/finite_profile_executor.h"
#include <climits>
#include <string>
#include <vector>

using namespace finite_profile;

static Profile profile() {
  Profile p;
  std::memset(p.catalogHash, 'a', 64);
  p.catalogHash[64] = 0;
  std::strcpy(p.profileId, "offline-test");
  p.startLeadMs = 20;
  p.maxLatenessMs = 10;
  p.offConfirmWindowMs = 30;
  p.admissionWindowMs = 100;
  p.count = 4;
  p.points[0] = Point(0, 10);
  p.points[1] = Point(50, 20);
  p.points[2] = Point(100, 10);
  p.points[3] = Point(150, 0);
  return p;
}

static void contract() {
  const Profile original = profile();
  assert(validProfile(original));
  const std::string expected =
      "motionbrain.finite_profile.v1\n"
      "contractVersion=1\n"
      "profileVersion=1\n"
      "timingPolicyVersion=1\n"
      "catalogHash=aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n"
      "profileId=offline-test\n"
      "motor=2\n"
      "direction=1\n"
      "startLeadMs=20\n"
      "maxLatenessMs=10\n"
      "offConfirmWindowMs=30\n"
      "admissionWindowMs=100\n"
      "pointCount=4\n"
      "point=0,10\n"
      "point=50,20\n"
      "point=100,10\n"
      "point=150,0\n";
  assert(canonicalProfile(original) == expected);
  assert(sameProfile(original, original));
  std::vector<Profile> invalid;
  Profile p = original; p.contractVersion = 2; invalid.push_back(p);
  p = original; p.profileVersion = 0; invalid.push_back(p);
  p = original; p.timingPolicyVersion = 2; invalid.push_back(p);
  p = original; p.catalogHash[0] = 'A'; invalid.push_back(p);
  p = original; p.catalogHash[63] = 0; invalid.push_back(p);
  p = original; p.catalogHash[64] = 'a'; invalid.push_back(p);
  p = original; p.profileId[0] = 0; invalid.push_back(p);
  p = original; std::memset(p.profileId, 'x', sizeof(p.profileId)); invalid.push_back(p);
  p = original; std::strcpy(p.profileId, "bad id"); invalid.push_back(p);
  p = original; std::strcpy(p.profileId, "bad\nid"); invalid.push_back(p);
  p = original; p.motor = 1; invalid.push_back(p);
  p = original; p.motor = 4; invalid.push_back(p);
  p = original; p.direction = 0; invalid.push_back(p);
  p = original; p.direction = -1; invalid.push_back(p);
  p = original; p.count = 0; invalid.push_back(p);
  p = original; p.count = 1; invalid.push_back(p);
  p = original; p.count = MAX_POINTS + 1; invalid.push_back(p);
  p = original; p.points[0].offsetMs = 1; invalid.push_back(p);
  p = original; p.points[1].offsetMs = 0; invalid.push_back(p);
  p = original; p.points[1].offsetMs = p.maxLatenessMs; invalid.push_back(p);
  p = original; p.points[2].offsetMs = 49; invalid.push_back(p);
  p = original; p.points[2].pwm = 0; invalid.push_back(p);
  p = original; p.points[1].pwm = -20; invalid.push_back(p);
  p = original; p.points[0].pwm = 256; invalid.push_back(p);
  p = original; p.points[3].pwm = 10; invalid.push_back(p);
  p = original; p.points[3].offsetMs = 10001; invalid.push_back(p);
  p = original; p.points[3].offsetMs = UINT32_MAX; invalid.push_back(p);
  p = original; p.startLeadMs = 10001; invalid.push_back(p);
  p = original; p.maxLatenessMs = 10001; invalid.push_back(p);
  p = original; p.offConfirmWindowMs = 0; invalid.push_back(p);
  p = original; p.offConfirmWindowMs = 10001; invalid.push_back(p);
  p = original; p.admissionWindowMs = 0; invalid.push_back(p);
  p = original; p.admissionWindowMs = 10001; invalid.push_back(p);
  for (const auto& bad : invalid) {
    assert(!validProfile(bad));
    assert(canonicalProfile(bad).empty());
    assert(!sameProfile(original, bad));
  }
  // The sign and numeric limits are exact; unused fixed-capacity slots are not
  // part of the wire contract or the duplicate identity.
  p = original; p.direction = -1;
  for (size_t i = 0; i + 1 < p.count; ++i) p.points[i].pwm = -p.points[i].pwm;
  assert(validProfile(p) && !sameProfile(original, p));
  assert(canonicalProfile(p).find("point=50,-20\n") != std::string::npos);
  p = original; p.points[MAX_POINTS - 1] = Point(UINT32_MAX, -32768);
  assert(validProfile(p) && sameProfile(original, p));
  assert(canonicalProfile(p) == expected);
  p = original; p.count = 2; p.points[0].pwm = 255;
  p.points[1] = Point(10000, 0); p.startLeadMs = 10000;
  p.offConfirmWindowMs = 10000; p.admissionWindowMs = 10000;
  assert(validProfile(p));
  p.direction = -1; p.points[0].pwm = -255;
  assert(validProfile(p));
  p = original; p.maxLatenessMs = 0; assert(validProfile(p));
  p = original; std::memset(p.profileId, 'x', 32); p.profileId[32] = 0;
  assert(validProfile(p));
  assert(!validIdentifier(nullptr, 33));
  assert(!validIdentifier("a", 0));
}

struct Output {
  uint32_t at;
  int16_t pwm;
  Output(uint32_t time, int16_t value) : at(time), pwm(value) {}
};

struct FakePort : Port {
  mutable uint32_t clock = 0;
  uint32_t idleLatency = 0;
  uint32_t applyLatency = 0;
  uint32_t stopLatency = 0;
  int32_t proofClockDelta = 0;
  bool safe = true;
  bool externalIdle = true;
  bool off = true;
  bool applyWorks = true;
  bool stopWorks = true;
  bool tokenWorks = true;
  Proof proof = Proof::Confirmed;
  unsigned tokenCount = 0;
  unsigned applyCount = 0;
  unsigned stopCount = 0;
  unsigned proofCount = 0;
  std::vector<Output> writes;
  uint32_t now() const override { return clock; }
  bool allowed() const override { return safe; }
  bool idle() const override { clock += idleLatency; return externalIdle && off; }
  bool apply(uint8_t motor, int16_t pwm, uint32_t& actual) override {
    assert(motor == 2);
    ++applyCount;
    clock += applyLatency;
    actual = clock;
    // A failed write may have partly reached the output: the executor must
    // preserve ownership/quiet handling even when the return value is false.
    off = pwm == 0;
    writes.push_back(Output(actual, pwm));
    return applyWorks;
  }
  bool stop(uint32_t& actual) override {
    ++stopCount;
    clock += stopLatency;
    actual = clock;
    if (stopWorks) { off = true; writes.push_back(Output(actual, 0)); }
    return stopWorks;
  }
  Proof offProof() override {
    ++proofCount;
    clock += static_cast<uint32_t>(proofClockDelta);
    return proof;
  }
  bool mintToken(Token& token) override {
    if (!tokenWorks) return false;
    ++tokenCount;
    for (size_t i = 0; i < sizeof(token.bytes); ++i)
      token.bytes[i] = static_cast<uint8_t>(tokenCount + i);
    return true;
  }
  size_t nonzeroCount() const {
    size_t n = 0;
    for (const auto& w : writes) if (w.pwm) ++n;
    return n;
  }
};

struct Fixture {
  FakePort port;
  Profile accepted;
  Executor executor;
  explicit Fixture(const Profile& p = profile())
      : accepted(p), executor(port, "boot-A", &accepted, 1) {}
  Request request(const char* id = "action-1") const {
    Request r;
    std::strcpy(r.bootId, "boot-A");
    std::strcpy(r.commandId, id);
    r.expectedEpoch = executor.epoch();
    r.profile = accepted;
    return r;
  }
  PrepareResult prepare(const char* id = "action-1", uint32_t now = 0) {
    port.clock = now;
    return executor.prepare(request(id));
  }
  const Record* start(const char* id = "action-1", uint32_t now = 0) {
    const PrepareResult p = prepare(id, now);
    assert(p.code == Code::Accepted && p.granted && p.record);
    const Result r = executor.execute("boot-A", id, p.record->request.expectedEpoch, p.token);
    assert(r.code == Code::Accepted && r.record == p.record);
    assert(r.record->state == State::Running);
    return r.record;
  }
  void tick(uint32_t now) { port.clock = now; executor.tick(); }
  Result event(const Record* r, uint8_t index, uint32_t now) {
    port.clock = now;
    return executor.event("boot-A", r->request.commandId, r->request.expectedEpoch, index);
  }
  void finishPoints(const Record* r) {
    for (uint8_t i = r->nextEventIndex; i < r->request.profile.count; ++i)
      tick(r->plannedStartAt + r->request.profile.points[i].offsetMs);
  }
};

static void prepare() {
  Fixture f;
  assert(f.executor.initialized() && f.executor.idle() && f.executor.epoch() == 1);
  Request wrong = f.request(); std::strcpy(wrong.bootId, "boot-B");
  assert(f.executor.prepare(wrong).code == Code::BootMismatch);
  wrong = f.request(); wrong.commandId[0] = 0;
  assert(f.executor.prepare(wrong).code == Code::Invalid);
  wrong = f.request(); std::memset(wrong.commandId, 'x', sizeof(wrong.commandId));
  assert(f.executor.prepare(wrong).code == Code::Invalid);
  wrong = f.request(); wrong.expectedEpoch = 0;
  assert(f.executor.prepare(wrong).code == Code::EpochMismatch);
  assert(f.executor.count() == 0 && f.port.writes.empty() && f.port.tokenCount == 0);
  const auto p = f.prepare("action-1", 100);
  assert(p.code == Code::Accepted && p.granted && p.record);
  assert(p.record->state == State::Reserved && p.record->preparedAt == 100);
  assert(!p.record->executed && f.port.writes.empty() && f.port.applyCount == 0);
  assert(!f.executor.idle() && f.executor.count() == 1);
  assert(f.executor.find("boot-B", "action-1") == nullptr);
  assert(f.executor.find("boot-A", "missing") == nullptr);
  assert(f.executor.find("boot-A", "action-1") == p.record);
  assert(f.executor.prepare(f.request("other")).code == Code::Busy);
  assert(f.executor.cancel("boot-A", "unknown", f.executor.epoch()).code == Code::Busy);
  assert(f.executor.count() == 1 && f.port.writes.empty());
  Fixture busy; busy.port.externalIdle = false;
  assert(busy.prepare().code == Code::Busy && busy.executor.count() == 0);
  Fixture unsafe; unsafe.port.safe = false;
  assert(unsafe.prepare().code == Code::SafetyBlocked && unsafe.executor.count() == 0);
  Fixture tokenFailure; tokenFailure.port.tokenWorks = false;
  const auto failed = tokenFailure.prepare();
  assert(failed.code == Code::Rejected && !failed.granted);
  assert(failed.record->state == State::Failed && failed.record->reason == Reason::TokenFailure);
  assert(tokenFailure.port.writes.empty() && tokenFailure.executor.idle());
  tokenFailure.port.tokenWorks = true;
  assert(tokenFailure.executor.prepare(failed.record->request).code == Code::Duplicate);
  Profile badCatalog = profile(); badCatalog.motor = 1;
  Fixture invalidCatalog(badCatalog);
  assert(!invalidCatalog.executor.initialized() && invalidCatalog.prepare().code == Code::Invalid);
  assert(invalidCatalog.port.writes.empty());
}

static void identity() {
  Fixture f;
  const auto p = f.prepare();
  const Request original = p.record->request;
  const auto duplicate = f.executor.prepare(original);
  assert(duplicate.code == Code::Duplicate && !duplicate.granted);
  assert(duplicate.record == p.record && f.port.tokenCount == 1);
  std::vector<Profile> changes;
  Profile q = original.profile; q.points[1].pwm = 21; changes.push_back(q);
  q = original.profile; q.points[1].offsetMs = 51; changes.push_back(q);
  q = original.profile; q.startLeadMs += 1; changes.push_back(q);
  q = original.profile; q.maxLatenessMs += 1; changes.push_back(q);
  q = original.profile; q.offConfirmWindowMs += 1; changes.push_back(q);
  q = original.profile; q.admissionWindowMs += 1; changes.push_back(q);
  q = original.profile; q.catalogHash[0] = 'b'; changes.push_back(q);
  q = original.profile; std::strcpy(q.profileId, "other-profile"); changes.push_back(q);
  q = original.profile; q.direction = -1;
  for (uint8_t i = 0; i + 1 < q.count; ++i) q.points[i].pwm = -q.points[i].pwm;
  changes.push_back(q);
  for (const auto& changed : changes) {
    assert(validProfile(changed));
    Request request = original; request.profile = changed;
    assert(f.executor.prepare(request).code == Code::Conflict);
  }
  assert(f.port.writes.empty());
  Token wrong = p.token; wrong.bytes[0] ^= 1;
  assert(f.executor.execute("boot-A", "action-1", original.expectedEpoch, wrong).code == Code::TokenMismatch);
  assert(f.executor.execute("boot-B", "action-1", original.expectedEpoch, p.token).code == Code::BootMismatch);
  assert(f.executor.execute("boot-A", "action-1", original.expectedEpoch + 1, p.token).code == Code::EpochMismatch);
  assert(f.executor.execute("boot-A", "action-1", original.expectedEpoch, p.token).code == Code::Accepted);
  const uint32_t anchor = p.record->plannedStartAt;
  f.port.clock = 5;
  assert(f.executor.execute("boot-A", "action-1", original.expectedEpoch, p.token).code == Code::Duplicate);
  assert(p.record->plannedStartAt == anchor && f.port.writes.empty());
  f.finishPoints(p.record);
  const auto writes = f.port.writes.size();
  assert(f.executor.prepare(original).code == Code::Duplicate);
  assert(f.executor.execute("boot-A", "action-1", original.expectedEpoch, p.token).code == Code::Duplicate);
  assert(f.port.writes.size() == writes);
}

static void cancel_before_first() {
  Fixture reserved;
  const auto p = reserved.prepare();
  const Request old = p.record->request;
  assert(reserved.executor.cancel("boot-A", "action-1", old.expectedEpoch).code == Code::Accepted);
  assert(p.record->state == State::Cancelled && reserved.executor.idle());
  assert(reserved.port.writes.empty() && reserved.executor.epoch() != old.expectedEpoch);
  assert(reserved.executor.execute("boot-A", "action-1", old.expectedEpoch, p.token).code != Code::Accepted);
  reserved.tick(1000); assert(reserved.port.writes.empty());
  Fixture started;
  const Record* r = started.start();
  const uint32_t epoch = r->request.expectedEpoch;
  started.port.clock = r->plannedStartAt - 1;
  assert(started.executor.cancel("boot-A", "action-1", epoch).code == Code::Accepted);
  assert(r->state == State::Cancelled);
  started.event(r, 0, r->plannedStartAt);
  started.tick(r->plannedOffAt);
  assert(started.port.nonzeroCount() == 0);
}

static void unknown_cancel() {
  Fixture f;
  const uint32_t old = f.executor.epoch();
  Request delayed = f.request("late-request");
  assert(f.executor.cancel("boot-A", "late-request", old).code == Code::Accepted);
  assert(f.executor.epoch() != old && f.executor.count() == 0 && f.port.writes.empty());
  assert(f.executor.prepare(delayed).code == Code::EpochMismatch);
  assert(f.executor.cancel("boot-A", "another-unknown", old).code == Code::EpochMismatch);
  assert(f.executor.cancel("boot-B", "unknown", f.executor.epoch()).code == Code::BootMismatch);
  assert(f.executor.prepare(f.request("new-request")).code == Code::Accepted);
  assert(f.port.writes.empty());
}

static void expiry() {
  Fixture f;
  auto p = f.prepare("action-1", 100);
  f.port.clock = 100 + f.accepted.admissionWindowMs;
  const auto expired = f.executor.execute("boot-A", "action-1", p.record->request.expectedEpoch, p.token);
  assert(expired.code == Code::Expired && p.record->state == State::Expired);
  assert(f.port.writes.empty() && f.executor.idle());
  const auto originalPreparedAt = p.record->preparedAt;
  assert(f.executor.prepare(p.record->request).code == Code::Duplicate);
  assert(p.record->preparedAt == originalPreparedAt && f.port.tokenCount == 1);
  Fixture edge;
  p = edge.prepare("action-1", 100);
  edge.port.clock = 100 + edge.accepted.admissionWindowMs - 1;
  assert(edge.executor.execute("boot-A", "action-1", p.record->request.expectedEpoch, p.token).code == Code::Accepted);
  const auto start = p.record->plannedStartAt;
  edge.port.clock += 1;
  assert(edge.executor.execute("boot-A", "action-1", p.record->request.expectedEpoch, p.token).code == Code::Duplicate);
  assert(p.record->plannedStartAt == start && edge.port.writes.empty());
  Fixture checks;
  const auto checked = checks.prepare();
  checks.port.clock = 99; checks.port.idleLatency = 1;
  assert(checks.executor.execute("boot-A", "action-1", checked.record->request.expectedEpoch, checked.token).code == Code::Expired);
  assert(checked.record->state == State::Expired && checks.port.writes.empty());
}

static void capacity() {
  Fixture f;
  for (size_t i = 0; i < MAX_RECORDS; ++i) {
    const std::string id = "record-" + std::to_string(i);
    const auto p = f.prepare(id.c_str(), static_cast<uint32_t>(i));
    assert(p.code == Code::Accepted && p.granted);
    assert(f.executor.cancel("boot-A", id.c_str(), p.record->request.expectedEpoch).code == Code::Accepted);
  }
  assert(f.executor.count() == MAX_RECORDS && f.port.writes.empty());
  assert(f.prepare("overflow").code == Code::Capacity);
  const Record* first = f.executor.find("boot-A", "record-0");
  assert(first && first->state == State::Cancelled);
  const auto duplicate = f.executor.prepare(first->request);
  assert(duplicate.code == Code::Duplicate && !duplicate.granted);
  assert(f.executor.count() == MAX_RECORDS && f.port.writes.empty());
}

static void nominal() {
  Fixture f;
  const Record* r = f.start("action-1", 100);
  assert(r->executeAcceptedAt == 100 && r->plannedStartAt == 120 && r->plannedOffAt == 270);
  f.tick(119); assert(f.port.writes.empty());
  f.finishPoints(r);
  assert(r->state == State::Completed && r->reason == Reason::NominalEnd);
  assert(r->timingConformant && r->traceComplete && r->softwareOffConfirmed);
  assert(r->quietHeld && !f.executor.idle() && r->offWriteAt == 270);
  assert(r->firstNonzeroWriteAt == 120 && r->nextEventIndex == f.accepted.count);
  assert(f.port.writes.size() == static_cast<size_t>(f.accepted.count) + 1);
  assert(f.port.writes.back().pwm == 0 && f.port.stopCount == 1);
  for (uint8_t i = 0; i < f.accepted.count; ++i) {
    assert(f.port.writes[i].pwm == f.accepted.points[i].pwm);
    assert(f.port.writes[i].at == 120 + f.accepted.points[i].offsetMs);
    assert(r->writes[i].present && r->writes[i].pwm == f.accepted.points[i].pwm);
  }
  const auto n = f.port.writes.size();
  f.tick(1000); f.event(r, 0, 1001);
  assert(f.port.writes.size() == n);
  assert(f.executor.prepare(f.request("blocked-by-observation")).code == Code::Busy);
  Profile full = profile(); full.count = MAX_POINTS;
  for (uint8_t i = 0; i < full.count; ++i)
    full.points[i] = Point(static_cast<uint32_t>(i) * 50, i + 1 == full.count ? 0 : 10);
  assert(validProfile(full));
  Fixture capacityEdge(full);
  const Record* e = capacityEdge.start(); capacityEdge.finishPoints(e);
  assert(e->state == State::Completed && e->traceComplete && e->timingConformant);
  assert(e->nextEventIndex == MAX_POINTS && capacityEdge.port.applyCount == MAX_POINTS);
  assert(e->writes[MAX_POINTS - 1].present && e->writes[MAX_POINTS - 1].pwm == 0);
  assert(e->writes[MAX_POINTS - 1].atMs == e->plannedOffAt && e->offWriteAt == e->plannedOffAt);
  assert(e->offAttempts == 1 && capacityEdge.port.stopCount == 1);
  assert(capacityEdge.port.writes.size() == MAX_POINTS + 1 && capacityEdge.port.writes.back().pwm == 0);
}

static void latency() {
  Fixture boundary;
  const Record* r = boundary.start();
  boundary.event(r, 0, r->plannedStartAt + boundary.accepted.maxLatenessMs);
  assert(r->state == State::Running && boundary.port.nonzeroCount() == 1);
  Fixture late;
  r = late.start();
  late.event(r, 0, r->plannedStartAt + late.accepted.maxLatenessMs + 1);
  assert(r->state == State::Failed && r->reason == Reason::DeadlineMissed);
  assert(!r->timingConformant && late.port.nonzeroCount() == 0);
  Fixture during;
  r = during.start();
  during.port.applyLatency = during.accepted.maxLatenessMs + 1;
  during.event(r, 0, r->plannedStartAt);
  assert(during.port.nonzeroCount() == 1); // Write began in time but finished late.
  assert(r->state == State::Failed && r->reason == Reason::DeadlineMissed);
  assert(!r->timingConformant && r->softwareOffConfirmed && r->quietHeld);
  assert(during.port.writes.back().pwm == 0);
  assert(r->writes[0].present && r->writes[0].atMs == r->plannedStartAt + 11);
}

static void missed() {
  Fixture f;
  const Record* r = f.start();
  f.tick(r->plannedStartAt);
  const auto n = f.port.nonzeroCount();
  f.tick(r->plannedStartAt + 120); // Several events due; never replay them together.
  assert(r->state == State::Failed && r->reason == Reason::DeadlineMissed);
  assert(f.port.nonzeroCount() == n && f.port.writes.back().pwm == 0);
  f.tick(r->plannedOffAt + 1000);
  assert(f.port.nonzeroCount() == n);
}

static void event_index() {
  Fixture f;
  const Record* r = f.start();
  assert(f.event(r, 0, r->plannedStartAt - 1).code == Code::NotDue);
  assert(f.port.writes.empty() && r->nextEventIndex == 0);
  f.event(r, 0, r->plannedStartAt);
  const auto n = f.port.writes.size();
  assert(f.event(r, 0, r->plannedStartAt + 1).code == Code::StaleEvent);
  assert(f.port.writes.size() == n && r->duplicateEvents == 1);
  assert(f.event(r, 2, r->plannedStartAt + 2).code == Code::Rejected);
  assert(r->state == State::Failed && r->reason == Reason::EventOrder);
  assert(f.port.nonzeroCount() == 1 && f.port.writes.back().pwm == 0);
  Fixture foreign;
  const Record* q = foreign.start();
  foreign.port.clock = q->plannedStartAt;
  assert(foreign.executor.event("boot-B", "action-1", q->request.expectedEpoch, 0).code == Code::BootMismatch);
  assert(foreign.executor.event("boot-A", "action-1", q->request.expectedEpoch + 1, 0).code == Code::EpochMismatch);
  assert(foreign.port.writes.empty());
}

static void cancel_running() {
  Fixture f;
  const Record* r = f.start();
  f.tick(r->plannedStartAt);
  f.port.clock += 1;
  assert(f.executor.cancel("boot-A", "action-1", r->request.expectedEpoch).code == Code::Accepted);
  assert(r->state == State::Cancelled && r->softwareOffConfirmed && r->quietHeld);
  const auto n = f.port.nonzeroCount();
  f.event(r, 1, r->plannedStartAt + 50); f.tick(r->plannedOffAt);
  assert(f.port.nonzeroCount() == n && f.port.writes.back().pwm == 0);
  const auto epoch = f.executor.epoch();
  f.executor.cancel("boot-A", "action-1", r->request.expectedEpoch);
  assert(f.executor.epoch() == epoch && r->state == State::Cancelled);
  Fixture partial;
  const Record* q = partial.start(); partial.port.applyWorks = false;
  partial.tick(q->plannedStartAt);
  assert(q->state == State::Failed && q->reason == Reason::WriteFailed);
  assert(q->quietHeld && q->softwareOffConfirmed && partial.port.writes.back().pwm == 0);
}

static void late_off() {
  Fixture f;
  const Record* r = f.start();
  for (uint8_t i = 0; i + 1 < f.accepted.count; ++i)
    f.tick(r->plannedStartAt + f.accepted.points[i].offsetMs);
  f.tick(r->plannedOffAt + f.accepted.maxLatenessMs + 1);
  assert(r->state == State::Failed && r->reason == Reason::DeadlineMissed);
  assert(!r->timingConformant && r->softwareOffConfirmed);
  assert(f.port.writes.back().pwm == 0 && r->offWriteAt == r->plannedOffAt + 11);
  Fixture slow;
  const Record* q = slow.start();
  for (uint8_t i = 0; i + 1 < slow.accepted.count; ++i)
    slow.tick(q->plannedStartAt + slow.accepted.points[i].offsetMs);
  slow.port.applyLatency = 11;
  slow.tick(q->plannedOffAt);
  assert(q->state == State::Failed && q->reason == Reason::DeadlineMissed);
  assert(q->offWriteAt == q->plannedOffAt + 11 && !q->timingConformant);
  Fixture confirm;
  const Record* c = confirm.start();
  for (uint8_t i = 0; i + 1 < confirm.accepted.count; ++i)
    confirm.tick(c->plannedStartAt + confirm.accepted.points[i].offsetMs);
  confirm.port.stopLatency = 11;
  confirm.tick(c->plannedOffAt);
  assert(c->state == State::Completed && c->timingConformant);
  assert(c->offWriteAt == c->plannedOffAt && c->offConfirmedAt == c->plannedOffAt + 11);
}

static void safety() {
  Fixture reserved;
  const auto reservation = reserved.prepare();
  reserved.port.safe = false; reserved.tick(1);
  assert(reservation.record->state == State::Failed);
  assert(reservation.record->reason == Reason::SafetyChanged && reserved.port.writes.empty());
  reserved.port.safe = true;
  assert(reserved.executor.execute("boot-A", "action-1", reservation.record->request.expectedEpoch, reservation.token).code != Code::Accepted);
  assert(reserved.port.writes.empty());
  Fixture before;
  const auto p = before.prepare(); before.port.safe = false;
  assert(before.executor.execute("boot-A", "action-1", p.record->request.expectedEpoch, p.token).code != Code::Accepted);
  assert(before.port.nonzeroCount() == 0);
  Fixture running;
  const Record* r = running.start(); running.tick(r->plannedStartAt);
  running.port.safe = false; running.tick(r->plannedStartAt + 1);
  assert(r->state == State::Failed && r->reason == Reason::SafetyChanged);
  assert(running.port.writes.back().pwm == 0 && r->quietHeld);
}

static void quiet_release() {
  Fixture f;
  const Record* first = f.start("first"); f.finishPoints(first);
  const auto oldQuiet = first->quietEpoch;
  assert(f.executor.release("boot-A", "first", oldQuiet + 1).code != Code::Accepted);
  assert(first->quietHeld && !f.executor.idle());
  f.port.externalIdle = false;
  assert(f.executor.release("boot-A", "first", oldQuiet).code == Code::Busy);
  assert(first->quietHeld && !f.executor.idle() && f.executor.epoch() == oldQuiet);
  f.port.externalIdle = true;
  assert(f.executor.release("boot-A", "first", oldQuiet).code == Code::Accepted);
  assert(!first->quietHeld && f.executor.idle());
  const auto epoch = f.executor.epoch();
  f.executor.release("boot-A", "first", oldQuiet);
  assert(f.executor.epoch() == epoch);
  const Record* second = f.start("second", 1000); f.finishPoints(second);
  const auto nextEpoch = f.executor.epoch();
  f.executor.release("boot-A", "first", oldQuiet);
  assert(second->quietHeld && !f.executor.idle() && f.executor.epoch() == nextEpoch);
  assert(f.executor.release("boot-B", "second", second->quietEpoch).code == Code::BootMismatch);
  assert(f.executor.release("boot-A", "second", second->quietEpoch).code == Code::Accepted);
}

static void unknown_off() {
  Fixture schedule;
  const Record* pending = schedule.start(); schedule.tick(pending->plannedStartAt);
  schedule.port.proof = Proof::Pending; schedule.port.clock += 1;
  schedule.executor.cancel("boot-A", "action-1", pending->request.expectedEpoch);
  assert(pending->state == State::Stopping);
  const auto originalDecisionAt = pending->decisionAt;
  const auto originalStopReason = pending->reason;
  assert(schedule.executor.scheduleFailed("boot-A", "action-1", pending->request.expectedEpoch + 1).code == Code::EpochMismatch);
  assert(pending->state == State::Stopping);
  schedule.executor.scheduleFailed("boot-A", "action-1", pending->request.expectedEpoch);
  assert(pending->state == State::OutcomeUnknown && schedule.executor.blocked());
  assert(pending->decisionAt == originalDecisionAt && pending->reason == originalStopReason);
  assert(schedule.executor.scheduleFailed("boot-A", "action-1", pending->request.expectedEpoch).code == Code::Duplicate);
  assert(pending->decisionAt == originalDecisionAt && pending->reason == originalStopReason);
  Fixture f;
  const Record* r = f.start(); f.tick(r->plannedStartAt);
  f.port.proof = Proof::Pending; f.port.clock += 1;
  f.executor.cancel("boot-A", "action-1", r->request.expectedEpoch);
  assert(r->state == State::Stopping && !r->softwareOffConfirmed);
  const auto decision = r->decisionAt;
  f.tick(decision + f.accepted.offConfirmWindowMs - 1);
  assert(r->state == State::Stopping);
  f.tick(decision + f.accepted.offConfirmWindowMs);
  assert(r->state == State::OutcomeUnknown && f.executor.blocked());
  const auto checks = r->offChecks;
  f.tick(decision + 1000); assert(r->offChecks == checks);
  f.port.proof = Proof::Confirmed;
  f.executor.retryOff("boot-A", "action-1");
  assert(r->state == State::OutcomeUnknown && r->softwareOffConfirmed && f.executor.blocked());
  assert(r->decisionAt == decision && r->offChecks <= MAX_OFF_CHECKS);
  assert(f.executor.release("boot-A", "action-1", r->quietEpoch).code != Code::Accepted);
  assert(f.executor.prepare(f.request("blocked")).code != Code::Accepted);
  Fixture failed;
  const Record* q = failed.start(); failed.tick(q->plannedStartAt);
  failed.port.proof = Proof::Failed; failed.port.clock += 1;
  failed.executor.cancel("boot-A", "action-1", q->request.expectedEpoch);
  assert(q->state == State::OutcomeUnknown && failed.executor.blocked());
  failed.port.proof = Proof::Confirmed; failed.executor.retryOff("boot-A", "action-1");
  assert(q->state == State::OutcomeUnknown && failed.executor.blocked());
  const auto oldAttempts = failed.port.stopCount;
  for (unsigned i = 0; i < MAX_OFF_CHECKS + 2; ++i) {
    assert(failed.executor.retryOff("boot-A", "action-1").code == Code::Accepted);
  }
  assert(failed.port.stopCount == oldAttempts + MAX_OFF_CHECKS + 2);
  assert(q->state == State::OutcomeUnknown && failed.executor.blocked());
  Fixture bounded;
  const Record* b = bounded.start(); bounded.tick(b->plannedStartAt);
  bounded.port.proof = Proof::Pending;
  bounded.executor.cancel("boot-A", "action-1", b->request.expectedEpoch);
  for (unsigned i = 0; i < MAX_OFF_CHECKS + 5; ++i) bounded.executor.tick();
  assert(b->state == State::OutcomeUnknown && b->offChecks == MAX_OFF_CHECKS);
  assert(bounded.port.proofCount == MAX_OFF_CHECKS);
  const auto originalDecision = b->decisionAt;
  const auto originalReason = b->reason;
  bounded.port.proof = Proof::Confirmed; bounded.port.clock += 100;
  bounded.executor.retryOff("boot-A", "action-1");
  assert(b->lastRetryConfirmed && b->lastRetryWriteRecorded);
  const auto firstConfirmed = b->offConfirmedAt;
  assert(b->lastRetryConfirmedAt == firstConfirmed);
  bounded.port.clock += 100;
  bounded.executor.retryOff("boot-A", "action-1");
  assert(b->offConfirmedAt == firstConfirmed && b->lastRetryConfirmedAt == firstConfirmed + 100);
  assert(b->decisionAt == originalDecision && b->reason == originalReason);
  assert(b->state == State::OutcomeUnknown && bounded.executor.blocked());
  Fixture stopFailed;
  const Record* sf = stopFailed.start(); stopFailed.tick(sf->plannedStartAt);
  stopFailed.port.stopWorks = false; stopFailed.port.proof = Proof::Failed;
  stopFailed.executor.cancel("boot-A", "action-1", sf->request.expectedEpoch);
  assert(sf->state == State::OutcomeUnknown && !sf->traceComplete && stopFailed.executor.blocked());
}

static void reserved_off_changed() {
  Fixture f;
  const auto p = f.prepare();
  assert(p.record->softwareOffConfirmed);
  f.port.off = false; f.port.externalIdle = false;
  assert(f.executor.execute("boot-A", "action-1", p.record->request.expectedEpoch, p.token).code != Code::Accepted);
  assert(!p.record->softwareOffConfirmed && !f.executor.idle() && f.executor.blocked());
  assert(p.record->state == State::OutcomeUnknown && p.record->quietHeld);
  assert(f.port.nonzeroCount() == 0);
  assert(f.executor.release("boot-A", "action-1", p.record->quietEpoch).code != Code::Accepted);
  assert(f.executor.prepare(f.request("later")).code != Code::Accepted);
  for (unsigned mode = 0; mode < 4; ++mode) {
    Fixture other; const auto reservation = other.prepare();
    other.port.off = false; other.port.externalIdle = false;
    if (mode == 0) other.executor.cancel("boot-A", "action-1", reservation.record->request.expectedEpoch);
    else if (mode == 1) other.tick(1);
    else if (mode == 2) other.tick(other.accepted.admissionWindowMs);
    else {
      other.port.clock = other.accepted.admissionWindowMs;
      other.executor.execute("boot-A", "action-1", reservation.record->request.expectedEpoch, reservation.token);
    }
    assert(reservation.record->state == State::OutcomeUnknown && other.executor.blocked());
    assert(!reservation.record->softwareOffConfirmed && !other.executor.idle());
    assert(other.port.nonzeroCount() == 0);
  }
}

static void proof_clock() {
  Fixture f;
  const Record* r = f.start(); f.tick(r->plannedStartAt);
  f.port.clock = 30; f.port.stopLatency = 10; f.port.proofClockDelta = -5;
  f.executor.cancel("boot-A", "action-1", r->request.expectedEpoch);
  assert(r->decisionAt == 30 && r->offWriteAt == 40 && f.port.clock == 35);
  assert(r->state == State::OutcomeUnknown && f.executor.blocked());
  assert(!r->softwareOffConfirmed && !r->lastRetryConfirmed);
  f.port.stopLatency = 0; f.port.proofClockDelta = 0;
  f.executor.retryOff("boot-A", "action-1");
  assert(f.port.clock == 35 && !r->lastRetryConfirmed && !r->softwareOffConfirmed);
  assert(r->offWriteAt == 40 && r->lastOffWriteAt == 40);
  assert(!r->traceComplete && r->state == State::OutcomeUnknown && f.executor.blocked());
  f.port.stopLatency = 10; f.port.proofClockDelta = -5;
  f.port.clock = 100;
  f.executor.retryOff("boot-A", "action-1");
  assert(r->lastRetryWriteAt == 110 && f.port.clock == 105);
  assert(!r->softwareOffConfirmed && !r->lastRetryConfirmed);
  assert(r->state == State::OutcomeUnknown && r->decisionAt == 30);
  f.port.clock = 200; f.port.proofClockDelta = 0;
  f.executor.retryOff("boot-A", "action-1");
  assert(r->softwareOffConfirmed && r->lastRetryConfirmed && r->offConfirmedAt == 210);
  assert(r->offWriteAt == 40 && r->lastOffWriteAt == 210);
  f.port.clock = 300; f.port.proofClockDelta = -5;
  f.executor.retryOff("boot-A", "action-1");
  assert(!r->lastRetryConfirmed && r->offConfirmedAt == 210);
  assert(r->state == State::OutcomeUnknown && f.executor.blocked());
  Fixture earlier;
  const Record* e = earlier.start(); earlier.tick(e->plannedStartAt);
  earlier.port.clock = 30; earlier.port.stopLatency = 10; earlier.port.proof = Proof::Pending;
  earlier.executor.cancel("boot-A", "action-1", e->request.expectedEpoch);
  assert(e->state == State::Stopping && e->offWriteAt == 40);
  earlier.port.proof = Proof::Confirmed; earlier.tick(35);
  assert(e->state == State::OutcomeUnknown && !e->softwareOffConfirmed && earlier.executor.blocked());
}

static void wrap() {
  Fixture f;
  const uint32_t beginning = UINT32_MAX - 10;
  const Record* r = f.start("action-1", beginning);
  assert(r->plannedStartAt == 9 && r->plannedOffAt == 159);
  f.tick(UINT32_MAX - 1); assert(f.port.writes.empty());
  f.finishPoints(r);
  assert(r->state == State::Completed && r->timingConformant && r->offWriteAt == 159);
  Fixture expiryWrap;
  const uint32_t prepared = UINT32_MAX - 20;
  const auto p = expiryWrap.prepare("expired", prepared);
  expiryWrap.port.clock = prepared + expiryWrap.accepted.admissionWindowMs;
  assert(expiryWrap.executor.execute("boot-A", "expired", p.record->request.expectedEpoch, p.token).code == Code::Expired);
  assert(p.record->state == State::Expired && expiryWrap.port.writes.empty());
}

int main(int argc, char** argv) {
  assert(argc == 2);
  const std::string name = argv[1];
  if (name == "contract") contract();
  else if (name == "prepare") prepare();
  else if (name == "identity") identity();
  else if (name == "cancel_before_first") cancel_before_first();
  else if (name == "unknown_cancel") unknown_cancel();
  else if (name == "expiry") expiry();
  else if (name == "capacity") capacity();
  else if (name == "nominal") nominal();
  else if (name == "latency") latency();
  else if (name == "missed") missed();
  else if (name == "event_index") event_index();
  else if (name == "cancel_running") cancel_running();
  else if (name == "late_off") late_off();
  else if (name == "safety") safety();
  else if (name == "quiet_release") quiet_release();
  else if (name == "unknown_off") unknown_off();
  else if (name == "reserved_off_changed") reserved_off_changed();
  else if (name == "proof_clock") proof_clock();
  else if (name == "wrap") wrap();
  else { std::fprintf(stderr, "Unknown case: %s\n", argv[1]); return 2; }
  std::printf("passed: %s\n", argv[1]);
  return 0;
}
#else
// Core types and ports do not exist in a default build. Both production .cpp
// files still compile/link, with no initialization or hardware operations.
static_assert(MOTIONBRAIN_ENABLE_FINITE_PROFILE_CORE == 0, "default must be disabled");
int main(int argc, char** argv) {
  assert(argc == 2 && std::strcmp(argv[1], "disabled") == 0);
  std::puts("passed: disabled");
  return 0;
}
#endif
