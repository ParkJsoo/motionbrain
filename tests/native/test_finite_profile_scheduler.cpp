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

#include "control/finite_profile_scheduler.h"
#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_SCHEDULER
using namespace finite_profile;
struct Environment : DriverEnvironment {
  unsigned tokens = 0;
  bool allowed() const override { return true; }
  bool controllersIdle() const override { return true; }
  bool mintToken(Token& token) override { ++tokens; for (auto& b : token.bytes) b = static_cast<uint8_t>(tokens); return true; }
};
static bool sameTicket(const ScheduledWake& a, const ScheduledWake& b) {
  return a.lease == b.lease && a.generation == b.generation && a.requestEpoch == b.requestEpoch &&
      a.kind == b.kind && a.eventIndex == b.eventIndex && !strcmp(a.bootId,b.bootId) && !strcmp(a.commandId,b.commandId);
}
struct FakeScheduler : SchedulerPort {
  ScheduledWake ticket;
  uint32_t due = 0;
  uint32_t registrationDelay = 0;
  uint32_t disarmDelay = 0;
  uint32_t failedArmCleanupDelay = 0;
  unsigned registrations = 0;
  unsigned failOn = 0;
  bool active = false;
  bool clearTicketOnDisarm = false;
  bool armOnce(uint32_t at, const ScheduledWake& value) override {
    ++registrations; nowMs += registrationDelay;
    ticket = value; due = at; active = true;
    if (registrations == failOn) disarmDelay = failedArmCleanupDelay;
    return registrations != failOn; // Even failed registration may leave a stale delivery.
  }
  void disarm(const ScheduledWake& value) override {
    nowMs += disarmDelay;
    if (active && sameTicket(value, ticket)) {
      active = false;
      if (clearTicketOnDisarm) ticket = ScheduledWake();
    }
  }
};
static Profile profile() {
  Profile p; memset(p.catalogHash,'a',64); p.catalogHash[64] = 0; strcpy(p.profileId,"scheduler-test");
  p.startLeadMs = 20; p.maxLatenessMs = 10; p.offConfirmWindowMs = 30; p.admissionWindowMs = 100;
  p.count = 4; p.points[0] = Point(0,80); p.points[1] = Point(50,120);
  p.points[2] = Point(100,40); p.points[3] = Point(150,0); return p;
}
struct Fixture {
  MotorControl driver; Environment env; FakeScheduler timer; Profile accepted; ScheduledDriver scheduled;
  Fixture() : accepted(profile()), scheduled(driver,env,timer,"boot-A",&accepted,1) {
    assert(driver.init()); systemState.state = SystemState::ARMED;
  }
  Request request(const char* id = "command-1") const {
    Request r; strcpy(r.bootId,"boot-A"); strcpy(r.commandId,id);r.expectedEpoch=scheduled.epoch();r.profile=accepted;return r;
  }
  PrepareResult prepare(const char* id = "command-1") { return scheduled.prepare(request(id)); }
  const Record* start(const char* id = "command-1") {
    auto p=prepare(id); assert(p.granted);
    auto result=scheduled.execute("boot-A",id,p.record->request.expectedEpoch,p.token);
    assert(result.execution.code==Code::Accepted && result.scheduling==ScheduleStatus::Armed);return p.record;
  }
  Result fire(uint32_t at) { auto key=timer.ticket;nowMs=at;return scheduled.deliver(key); }
  void finish(const Record* r) { while(r->state==State::Running) fire(timer.due); }
};
static void nominal() {
  Fixture f;const Record* r=f.start(); const uint32_t origin=r->executeAcceptedAt;
  const uint32_t lease=f.scheduled.leaseId(); ScheduledWake lastTicket;
  for(uint8_t i=0;i<f.accepted.count;++i) {
    assert(f.timer.ticket.kind==WakeKind::OutputEvent && f.timer.ticket.eventIndex==i);
    assert(f.timer.due==origin+f.accepted.startLeadMs+f.accepted.points[i].offsetMs);
    nowMs=f.timer.due; const size_t before=writes.size();f.scheduled.poll();
    assert(writes.size()==before); // maintenance cannot emit scheduled PWM
    lastTicket=f.timer.ticket;
    assert(f.fire(f.timer.due).code==Code::Accepted);
    assert(r->writes[i].present && r->writes[i].pwm==f.accepted.points[i].pwm);
  }
  assert(r->state==State::Completed && r->quietHeld && !f.timer.active);
  const auto* evidence = f.scheduled.outputEvidence("boot-A", "command-1");
  assert(evidence != nullptr && evidence->opened && !evidence->closed);
  assert(!strcmp(evidence->bootId,"boot-A") && !strcmp(evidence->commandId,"command-1"));
  assert(evidence->requestEpoch==r->request.expectedEpoch && evidence->lease==lease);
  assert(evidence->ordinal!=0 && evidence->openedAt==r->preparedAt);
  assert(evidence->captureComplete && evidence->dropped==0);
  assert(evidence->count==f.accepted.count+5 && evidence->totalWrites==f.accepted.count+5);
  for(uint8_t i=0;i<evidence->count;++i) {
    const auto& actual=evidence->entries[i];
    assert(actual.sequence==static_cast<uint32_t>(i)+1);
    assert(f.driver.outputLog().at(5+i)->actionOrdinal==evidence->ordinal);
    if(i<f.accepted.count) {
      assert(actual.motor==2 && actual.pwm==f.accepted.points[i].pwm);
      assert(actual.atMs==r->writes[i].atMs && actual.reason==bounded::OutputReason::DriverWrite);
    } else {
      assert(actual.motor==i-f.accepted.count+1 && actual.pwm==0);
      assert(actual.atMs==r->plannedOffAt && actual.reason==bounded::OutputReason::ForcedOff);
    }
  }
  const auto rawCount=evidence->totalWrites;
  assert(f.scheduled.deliver(lastTicket).code==Code::StaleEvent && evidence->totalWrites==rawCount);
  assert(r->plannedStartAt==origin+20 && r->plannedOffAt==origin+170);
  assert(f.scheduled.release("boot-A","command-1",r->quietEpoch).code==Code::Accepted);
  assert(f.scheduled.idle() && evidence->closed && evidence->closedAt==nowMs);
  assert(evidence->totalWrites==rawCount && evidence->lease==lease);
  assert(f.scheduled.deliver(lastTicket).code==Code::StaleEvent && evidence->totalWrites==rawCount);
  assertMatches(f.driver);
}
static void early_duplicate() {
  Fixture f;const Record* r=f.start();const auto original=f.timer.ticket;const auto due=f.timer.due;
  assert(f.fire(due-1).code==Code::NotDue);assert(f.timer.due==due);
  assert(f.timer.ticket.generation!=original.generation && writes.size()==5);
  assert(f.scheduled.deliver(original).code==Code::StaleEvent && writes.size()==5);
  const auto current=f.timer.ticket;f.fire(due);const auto count=writes.size();
  assert(f.scheduled.deliver(current).code==Code::StaleEvent && writes.size()==count);
  f.finish(r);assert(r->state==State::Completed);assertMatches(f.driver);
}
static void delay_no_catchup() {
  Fixture f;const Record* r=f.start();f.fire(f.timer.due);
  const auto next=f.timer.ticket;nowMs=f.timer.due+100;const auto n=writes.size();
  f.scheduled.poll();assert(r->state==State::Failed && r->reason==Reason::DeadlineMissed);
  for(size_t i=n;i<writes.size();++i)assert(writes[i].signedPwm==0);
  assert(f.scheduled.deliver(next).code==Code::StaleEvent && r->nextEventIndex==1);
  assertMatches(f.driver);
}
static void registration_failure() {
  Fixture f;auto p=f.prepare();assert(p.granted);f.timer.failOn=f.timer.registrations+1;
  auto result=f.scheduled.execute("boot-A","command-1",p.record->request.expectedEpoch,p.token);
  assert(result.execution.code==Code::Accepted && result.scheduling==ScheduleStatus::Failed);
  assert(p.record->executed && !p.record->outputMayHaveOccurred);
  assert(p.record->state==State::Failed && p.record->reason==Reason::ScheduleFailed);
  const auto stale=f.timer.ticket;assert(!f.timer.active);
  assert(f.scheduled.deliver(stale).code==Code::StaleEvent);
  assert(f.scheduled.execute("boot-A","command-1",p.record->request.expectedEpoch,p.token).execution.code==Code::Duplicate);
  assertMatches(f.driver);
}
static void later_failure() {
  Fixture f;const Record* r=f.start();f.timer.failOn=f.timer.registrations+1;
  f.fire(f.timer.due);
  assert(r->state==State::Failed && r->reason==Reason::ScheduleFailed && r->quietHeld);
  assert(r->nextEventIndex==1 && f.driver.getSpeed(2)==0 && !f.timer.active);
  assertMatches(f.driver);
}
static void cancellation() {
  Fixture f;auto p=f.prepare("old");assert(p.granted);const auto reserved=f.timer.ticket;
  f.scheduled.cancel("boot-A","old",p.record->request.expectedEpoch);
  const Record* r=f.start("new");const auto next=f.timer.ticket;const size_t n=writes.size();
  assert(f.scheduled.deliver(reserved).code==Code::StaleEvent);
  f.scheduled.cancel("boot-A","old",p.record->request.expectedEpoch);
  assert(sameTicket(next,f.timer.ticket) && f.timer.active && writes.size()==n);
  f.finish(r);assertMatches(f.driver);
}
static void wrap() {
  Fixture f;nowMs=UINT32_MAX-10;const Record* r=f.start();assert(f.timer.due==9);
  f.finish(r);assert(r->state==State::Completed && r->offWriteAt==159);assertMatches(f.driver);
}
static void registration_delay() {
  Fixture f;auto p=f.prepare();assert(p.granted);f.timer.registrationDelay=31;
  const auto result=f.scheduled.execute("boot-A","command-1",p.record->request.expectedEpoch,p.token);
  assert(result.scheduling==ScheduleStatus::Failed);
  assert(p.record->state==State::Failed && p.record->reason==Reason::DeadlineMissed);
  assert(p.record->plannedStartAt==20 && p.record->plannedOffAt==170);
  for(const auto&w:writes)assert(w.signedPwm==0);assertMatches(f.driver);
}
static void recreated_wrapper() {
  MotorControl driver;driver.init();systemState.state=SystemState::ARMED;Environment env;FakeScheduler timer;Profile p=profile();
  ScheduledWake stale;uint32_t oldLease=0;
  {
    ScheduledDriver old(driver,env,timer,"boot-A",&p,1);Request req;strcpy(req.bootId,"boot-A");strcpy(req.commandId,"same");req.expectedEpoch=old.epoch();req.profile=p;
    assert(old.prepare(req).granted);stale=timer.ticket;oldLease=old.leaseId();
  }
  assert(driver.finiteProfileOwned() && driver.finiteProfileLeaseSealed(oldLease));
  assert(driver.releaseFiniteProfileLease(oldLease));
  ScheduledDriver current(driver,env,timer,"boot-A",&p,1);Request req;strcpy(req.bootId,"boot-A");strcpy(req.commandId,"same");req.expectedEpoch=current.epoch();req.profile=p;
  assert(!current.prepare(req).granted); // Existing evidence identity cannot be rebound.
  assert(current.outputEvidence("boot-A", "same") == nullptr);
  strcpy(req.commandId,"new"); req.expectedEpoch=current.epoch();
  assert(current.prepare(req).granted);assert(timer.ticket.generation==stale.generation && timer.ticket.lease!=stale.lease);
  const auto valid=timer.ticket;timer.disarm(stale);assert(timer.active);
  assert(current.deliver(stale).code==Code::StaleEvent && sameTicket(valid,timer.ticket));
  ScheduledWake wrongLease = valid; wrongLease.lease = stale.lease;
  assert(current.deliver(wrongLease).code==Code::StaleEvent && sameTicket(valid,timer.ticket));
  assertMatches(driver);
}
static void off_before_cleanup() {
  Fixture f; const Record* r = f.start(); f.fire(f.timer.due);
  const uint32_t cancellationAt = nowMs; f.timer.disarmDelay = 100;
  assert(f.scheduled.cancel("boot-A", "command-1", r->request.expectedEpoch).code == Code::Accepted);
  assert(r->offWriteAt == cancellationAt && r->offConfirmedAt == cancellationAt);
  assert(nowMs == cancellationAt + 100 && !f.timer.active);
  assertMatches(f.driver);
}
static void failure_off_before_cleanup() {
  Fixture f; const Record* r = f.start(); const uint32_t firstDue = f.timer.due;
  f.timer.failOn = f.timer.registrations + 1; f.timer.failedArmCleanupDelay = 100;
  assert(f.fire(firstDue).code == Code::Rejected);
  assert(r->reason == Reason::ScheduleFailed && r->state == State::Failed);
  assert(r->offWriteAt == firstDue && r->offConfirmedAt == firstDue);
  assert(nowMs == firstDue + 100 && f.driver.getSpeed(2) == 0);
  assertMatches(f.driver);
}
static void destruction_off_before_cleanup() {
  MotorControl driver; driver.init(); systemState.state = SystemState::ARMED;
  Environment env; FakeScheduler timer; Profile p = profile(); size_t stopBegin = 0; uint32_t destroyAt = 0;
  {
    ScheduledDriver scheduled(driver,env,timer,"boot-A",&p,1);
    Request req; strcpy(req.bootId,"boot-A"); strcpy(req.commandId,"destroyed"); req.expectedEpoch=scheduled.epoch(); req.profile=p;
    const auto prepared=scheduled.prepare(req); assert(prepared.granted);
    assert(scheduled.execute("boot-A","destroyed",req.expectedEpoch,prepared.token).scheduling==ScheduleStatus::Armed);
    nowMs=timer.due; assert(scheduled.deliver(timer.ticket).code==Code::Accepted);
    stopBegin=writes.size();destroyAt=nowMs;timer.disarmDelay=100;
  }
  assert(!driver.hasPendingOutput() && !timer.active && driver.finiteProfileOwned());
  bool found=false;
  for(size_t i=stopBegin;i<writes.size();++i) {
    if(writes[i].motor==2 && !found) { assert(writes[i].signedPwm==0 && writes[i].time==destroyAt); found=true; }
  }
  assert(found && nowMs==destroyAt+100);assertMatches(driver);
}

static void destruction_keeps_reservation_and_quiet_lease() {
  for (unsigned phase=0; phase<3; ++phase) {
    writes.clear(); nowMs=0;
    MotorControl driver; driver.init(); systemState.state=SystemState::ARMED;
    Environment env; FakeScheduler timer; Profile p=profile(); uint32_t lease=0;
    {
      ScheduledDriver scheduled(driver,env,timer,"boot-A",&p,1);
      Request req;strcpy(req.bootId,"boot-A");strcpy(req.commandId,"retained");req.expectedEpoch=scheduled.epoch();req.profile=p;
      const auto prepared=scheduled.prepare(req);assert(prepared.granted);lease=scheduled.leaseId();
      if(phase) assert(scheduled.execute("boot-A","retained",req.expectedEpoch,prepared.token).scheduling==ScheduleStatus::Armed);
      if(phase==2) while(prepared.record->state==State::Running) { nowMs=timer.due;scheduled.deliver(timer.ticket); }
    }
    assert(driver.finiteProfileOwned() && driver.finiteProfileLeaseSealed(lease));
    assert(driver.finiteProfileSoftwareOff(lease) && !driver.setSpeed(2,80));
    const auto* evidence=driver.finiteProfileEvidence(lease);
    assert(evidence && evidence->opened && !evidence->closed && evidence->captureComplete);
    assertMatches(driver);
  }
}
static void delivery_owns_ticket_copy() {
  Fixture f;const Record* r=f.start();
  f.timer.clearTicketOnDisarm=true; nowMs=f.timer.due;
  assert(f.scheduled.deliver(f.timer.ticket).code==Code::Accepted);
  assert(r->nextEventIndex==1 && f.driver.getSpeed(2)==80);
  assert(f.timer.ticket.kind==WakeKind::OutputEvent && f.timer.ticket.eventIndex==1);
  f.finish(r);assert(r->state==State::Completed);assertMatches(f.driver);
}

static void early_expiry_crossed_during_registration() {
  Fixture f; const auto prepared=f.prepare(); assert(prepared.granted);
  const uint32_t expiry=f.timer.due; f.timer.registrationDelay=1;
  assert(f.fire(expiry-1).code==Code::Expired);
  assert(prepared.record->state==State::Expired && !prepared.record->executed);
  assert(!f.timer.active && !f.scheduled.hasScheduledWake() && f.scheduled.idle());
  assert(writes.size()==5);assertMatches(f.driver);
}

#endif
int main(int argc,char**argv) {
  assert(argc==2);
  if(!strcmp(argv[1],"disabled")) {
    MotorControl driver;assert(driver.init());assertMatches(driver);
  }
#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_SCHEDULER
  else if(!strcmp(argv[1],"nominal"))nominal();
  else if(!strcmp(argv[1],"early"))early_duplicate();
  else if(!strcmp(argv[1],"delay"))delay_no_catchup();
  else if(!strcmp(argv[1],"registration"))registration_failure();
  else if(!strcmp(argv[1],"later"))later_failure();
  else if(!strcmp(argv[1],"cancel"))cancellation();
  else if(!strcmp(argv[1],"wrap"))wrap();
  else if(!strcmp(argv[1],"registration_delay"))registration_delay();
  else if(!strcmp(argv[1],"recreated"))recreated_wrapper();
  else if(!strcmp(argv[1],"cancel_cleanup"))off_before_cleanup();
  else if(!strcmp(argv[1],"failure_cleanup"))failure_off_before_cleanup();
  else if(!strcmp(argv[1],"destructor_cleanup"))destruction_off_before_cleanup();
  else if(!strcmp(argv[1],"destructor_ownership"))destruction_keeps_reservation_and_quiet_lease();
  else if(!strcmp(argv[1],"ticket_alias"))delivery_owns_ticket_copy();
  else if(!strcmp(argv[1],"early_expiry"))early_expiry_crossed_during_registration();
#endif
  else return 2;
}
