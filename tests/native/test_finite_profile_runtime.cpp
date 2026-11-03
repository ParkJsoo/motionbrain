#include <new>
#include <cstdlib>
#include "control/finite_profile_runtime.h"
#include "control/finite_profile_trial_catalog.h"
#include "motor/motor_driver.h"
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
static unsigned totalWrites = 0;
static bool failRuntimeAllocation = false;
void* operator new(std::size_t size, const std::nothrow_t&) noexcept {
  return failRuntimeAllocation ? nullptr : std::malloc(size);
}
void operator delete(void* memory, const std::nothrow_t&) noexcept { std::free(memory); }
static unsigned tokenCalls = 0;

uint32_t millis() { return nowMs; }
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
void ledcSetup(uint8_t, uint32_t, uint8_t) {}
void ledcAttachPin(uint8_t, uint8_t) {}
void ledcWrite(uint8_t, uint32_t duty) { ++totalWrites; if (duty) ++nonzeroWrites; }

static void disabled() {
  assert(!finite_profile::runtime::enabled());
  finite_profile::runtime::update();
  assert(!finite_profile::runtime::motionPending());
  assert(finite_profile::runtime::profileCount()==0 && finite_profile::runtime::profileAt(0)==nullptr);
  assert(finite_profile::runtime::findProfile("anything","anything")==nullptr);
  assert(finite_profile::runtime::coreRecordsUsed()==0 && finite_profile::runtime::driverRecordsUsed()==0);
  assert(finite_profile::runtime::driver()==nullptr && finite_profile::runtime::profile()==nullptr);
  assert(!strcmp(finite_profile::runtime::bootId(),"") && tokenCalls==0 && totalWrites==0);
}
#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_RUNTIME
using namespace finite_profile;
static bool issuer(Token& token) { token.bytes[0]=static_cast<uint8_t>(++tokenCalls);return true; }
static Profile fixedProfile() {
  Profile p;memset(p.catalogHash,'a',64);p.catalogHash[64]=0;strcpy(p.profileId,"runtime-test");
  p.startLeadMs=20;p.maxLatenessMs=10;p.offConfirmWindowMs=30;p.admissionWindowMs=100;
  p.count=2;p.points[0]=Point(0,80);p.points[1]=Point(50,0);return p;
}
static void init() {
  assert(motorControl.init());systemState.state=SystemState::ARMED;BoundedActions::init();
  const Profile p=fixedProfile();assert(runtime::init(motorControl,p,BoundedActions::executor()->bootId(),issuer));
  assert(runtime::enabled() && runtime::driver() && runtime::profile());
  assert(!strcmp(runtime::bootId(),BoundedActions::executor()->bootId()));
}
static PrepareResult prepare(const char* id="runtime-command") {
  Request r;strcpy(r.bootId,runtime::bootId());strcpy(r.commandId,id);
  r.expectedEpoch=runtime::driver()->epoch();r.profile=*runtime::profile();return runtime::driver()->prepare(r);
}
static const Record* execute(const PrepareResult& p) {
  assert(p.granted);
  auto result=runtime::driver()->execute(runtime::bootId(),p.record->request.commandId,p.record->request.expectedEpoch,p.token);
  assert(result.execution.code==Code::Accepted && result.scheduling==ScheduleStatus::Armed);return p.record;
}
static void initialization() {
  assert(motorControl.init());systemState.state=SystemState::ARMED;
  Profile p=fixedProfile();Profile invalid=p;invalid.points[0].pwm=0;
  assert(!runtime::init(motorControl,invalid,"boot-runtime",issuer));
  assert(!runtime::init(motorControl,p,"",issuer));
  assert(!runtime::init(motorControl,p,"boot-runtime",nullptr));
  assert(runtime::driver()==nullptr);
  assert(runtime::init(motorControl,p,"boot-runtime",issuer));
  p.points[0].pwm=200;
  assert(runtime::profile()->points[0].pwm==80);
  assert(!runtime::init(motorControl,p,"different-boot",issuer));
  assert(!strcmp(runtime::bootId(),"boot-runtime") && totalWrites==5 && tokenCalls==0);
}
static void nominal() {
  init();auto prepared=prepare();assert(prepared.granted && !runtime::motionPending());
  const Record* r=execute(prepared);assert(runtime::motionPending() && nonzeroWrites==0);
  nowMs=r->plannedStartAt-1;runtime::update();assert(nonzeroWrites==0 && runtime::motionPending());
  nowMs=r->plannedStartAt;runtime::update();assert(nonzeroWrites==1 && motorControl.getSpeed(2)==80);
  runtime::update();assert(nonzeroWrites==1); // one-shot consumed; no duplicate output
  nowMs=r->plannedOffAt;runtime::update();
  assert(r->state==State::Completed && !runtime::motionPending() && r->quietHeld);
  const auto* evidence=runtime::driver()->outputEvidence(runtime::bootId(),"runtime-command");
  assert(evidence && evidence->count==7 && evidence->captureComplete && !evidence->closed);
  assert(evidence->entries[0].atMs==r->plannedStartAt && evidence->entries[0].pwm==80);
  assert(evidence->entries[1].atMs==r->plannedOffAt && evidence->entries[1].pwm==0);
  assert(runtime::driver()->release(runtime::bootId(),"runtime-command",r->quietEpoch).code==Code::Accepted);
  assert(evidence->closed && runtime::driver()->idle());
}
static void late_before_output() {
  init();const Record* r=execute(prepare());
  nowMs=r->plannedStartAt+runtime::profile()->maxLatenessMs+1;runtime::update();
  assert(r->state==State::Failed && r->reason==Reason::DeadlineMissed);
  assert(nonzeroWrites==0 && !runtime::motionPending() && !r->outputMayHaveOccurred);
}
static void delayed_off() {
  init();const Record* r=execute(prepare());nowMs=r->plannedStartAt;runtime::update();
  nowMs=r->plannedOffAt+11;runtime::update();
  assert(r->state==State::Failed && r->reason==Reason::DeadlineMissed && r->offWriteAt==nowMs);
  assert(nonzeroWrites==1 && !runtime::motionPending() && r->quietHeld);
  const auto writes=totalWrites;runtime::update();assert(totalWrites==writes);
}
static void cancellation() {
  init();auto prepared=prepare();const Record* r=execute(prepared);
  motorControl.invalidateFiniteProfileLease();runtime::update();
  assert(r->state==State::Cancelled && !runtime::motionPending());
  nowMs=r->plannedStartAt+1000;runtime::update();assert(nonzeroWrites==0);
  assert(runtime::driver()->execute(runtime::bootId(),"runtime-command",r->request.expectedEpoch,prepared.token).execution.code!=Code::Accepted);
}
static void expiry() {
  init();auto prepared=prepare();nowMs=prepared.record->preparedAt+runtime::profile()->admissionWindowMs;
  runtime::update();assert(prepared.record->state==State::Expired && !runtime::motionPending());
  assert(runtime::driver()->idle() && nonzeroWrites==0);
}
static void wrap() {
  init();nowMs=UINT32_MAX-10;const Record* r=execute(prepare());
  assert(r->plannedStartAt==9 && r->plannedOffAt==59);
  nowMs=9;runtime::update();assert(motorControl.getSpeed(2)==80);
  nowMs=59;runtime::update();assert(r->state==State::Completed && !runtime::motionPending());
}
static void admission() {
  init();webServer.manual=true;
  assert(!prepare().granted && tokenCalls==0 && !runtime::motionPending());
  webServer.manual=false;auto p=prepare();assert(p.granted);
  safetyGate.allowed=false;
  assert(runtime::driver()->execute(runtime::bootId(),"runtime-command",p.record->request.expectedEpoch,p.token).execution.code!=Code::Accepted);
  safetyGate.allowed=true;runtime::update();assert(nonzeroWrites==0 && !runtime::motionPending());
}
static void allocation_failure() {
  assert(motorControl.init());systemState.state=SystemState::ARMED;
  const auto p=fixedProfile();failRuntimeAllocation=true;
  assert(!runtime::init(motorControl,p,"boot-runtime",issuer));
  assert(runtime::driver()==nullptr && runtime::profile()==nullptr && !runtime::motionPending());
  assert(!strcmp(runtime::bootId(),""));runtime::update();
  assert(totalWrites==5 && nonzeroWrites==0 && tokenCalls==0);
  failRuntimeAllocation=false;
  assert(runtime::init(motorControl,p,"boot-runtime",issuer));
  assert(runtime::driver()!=nullptr && totalWrites==5);
}

static void safety_at_due(bool teleopUnsafe) {
  assert(motorControl.init());systemState.state=SystemState::ARMED;BoundedActions::init();
  Profile p=fixedProfile();p.count=3;p.points[1]=Point(50,120);p.points[2]=Point(100,0);
  assert(runtime::init(motorControl,p,BoundedActions::executor()->bootId(),issuer));
  const Record* r=execute(prepare());nowMs=r->plannedStartAt;runtime::update();
  assert(nonzeroWrites==1 && motorControl.getSpeed(2)==80);
  if(teleopUnsafe)teleopAdapter.held=true;else safetyGate.allowed=false;
  nowMs=r->plannedStartAt+50;runtime::update();
  assert(r->state==State::Failed && r->reason==Reason::SafetyChanged);
  assert(nonzeroWrites==1 && r->nextEventIndex==1 && motorControl.getSpeed(2)==0);
  assert(r->softwareOffConfirmed && r->offWriteAt==nowMs && !runtime::motionPending());
  safetyGate.allowed=true;teleopAdapter.held=false;nowMs+=100;runtime::update();
  assert(nonzeroWrites==1 && motorControl.getSpeed(2)==0);
}

static void catalog_initialization() {
  assert(motorControl.init()); systemState.state=SystemState::ARMED;
  TrialCatalog catalog;
  assert(!runtime::init(motorControl,nullptr,4,"catalog-boot",issuer));
  assert(!runtime::init(motorControl,catalog.profiles,0,"catalog-boot",issuer));
  assert(!runtime::init(motorControl,catalog.profiles,5,"catalog-boot",issuer));
  const auto original=catalog.profiles[1]; catalog.profiles[1]=catalog.profiles[0];
  assert(!runtime::init(motorControl,catalog.profiles,4,"catalog-boot",issuer));
  catalog.profiles[1]=original;
  assert(runtime::init(motorControl,catalog.profiles,4,"catalog-boot",issuer));
  assert(runtime::profileCount()==4 && runtime::profile()==runtime::profileAt(0));
  for(size_t i=0;i<4;++i) {
    assert(sameProfile(*runtime::profileAt(i),trialProfile(i)));
    assert(runtime::findProfile(catalog.profiles[i].profileId,catalog.profiles[i].catalogHash)==runtime::profileAt(i));
  }
  catalog.profiles[1].points[0].pwm=-200;
  assert(runtime::profileAt(1)->points[0].pwm==-10);
  assert(runtime::profileAt(4)==nullptr && !validProfile(trialProfile(4)));
  assert(runtime::findProfile(nullptr,"hash")==nullptr && runtime::findProfile("id",nullptr)==nullptr);
  assert(runtime::findProfile("unknown",runtime::profile()->catalogHash)==nullptr);
  assert(runtime::findProfile(runtime::profile()->profileId,runtime::profileAt(1)->catalogHash)==nullptr);
  assert(runtime::findProfile("","")==nullptr);
  assert(runtime::coreRecordsUsed()==0 && runtime::driverRecordsUsed()==0 && nonzeroWrites==0 && totalWrites==5);
}

static void four_profiles() {
  assert(motorControl.init()); systemState.state=SystemState::ARMED; BoundedActions::init();
  const TrialCatalog catalog;
  assert(runtime::init(motorControl,catalog.profiles,4,BoundedActions::executor()->bootId(),issuer));
  for(size_t n=0;n<4;++n) {
    const auto* selected=runtime::findProfile(catalog.profiles[n].profileId,catalog.profiles[n].catalogHash);
    Request request;strcpy(request.bootId,runtime::bootId());strcpy(request.commandId,selected->profileId);
    request.expectedEpoch=runtime::driver()->epoch();request.profile=*selected;
    // Matching ID/hash cannot authorize caller-supplied alternate points.
    request.profile.points[0].pwm*=2;
    assert(!runtime::driver()->prepare(request).granted && runtime::coreRecordsUsed()==n);
    request.profile=*selected;
    const auto prepared=runtime::driver()->prepare(request);const Record* record=execute(prepared);
    for(size_t i=0;i<selected->count;++i) {
      nowMs=record->plannedStartAt+selected->points[i].offsetMs;runtime::update();
      assert(motorControl.getSpeed(2)==selected->points[i].pwm);
    }
    assert(record->state==State::Completed && record->timingConformant && record->traceComplete);
    const auto* evidence=runtime::driver()->outputEvidence(runtime::bootId(),request.commandId);
    assert(evidence && evidence->count==selected->count+5 && evidence->captureComplete);
    for(size_t i=0;i<selected->count;++i) {
      assert(evidence->entries[i].pwm==selected->points[i].pwm);
      assert(evidence->entries[i].atMs==record->plannedStartAt+selected->points[i].offsetMs);
    }
    assert(runtime::driver()->release(runtime::bootId(),request.commandId,record->quietEpoch).code==Code::Accepted);
    assert(evidence->closed && runtime::driver()->idle());
    assert(runtime::coreRecordsUsed()==n+1 && runtime::driverRecordsUsed()==n+1);
  }
  assert(nonzeroWrites==32);
}

static void capacity_accounting() {
  assert(motorControl.init());systemState.state=SystemState::ARMED;BoundedActions::init();
  const TrialCatalog catalog;
  assert(runtime::init(motorControl,catalog.profiles,4,BoundedActions::executor()->bootId(),issuer));
  for(size_t i=0;i<MAX_RECORDS;++i) {
    const std::string id="reserved-"+std::to_string(i);
    Request request;strcpy(request.bootId,runtime::bootId());strcpy(request.commandId,id.c_str());
    request.expectedEpoch=runtime::driver()->epoch();request.profile=*runtime::profileAt(i%4);
    const auto prepared=runtime::driver()->prepare(request);assert(prepared.granted);
    assert(runtime::driver()->cancel(runtime::bootId(),id.c_str(),request.expectedEpoch).code==Code::Accepted);
    assert(runtime::driver()->idle() && runtime::coreRecordsUsed()==i+1 && runtime::driverRecordsUsed()==i+1);
  }
  assert(!prepare("capacity-extra").granted);
  assert(runtime::coreRecordsUsed()==32 && runtime::driverRecordsUsed()==32 && nonzeroWrites==0);
}

#endif
int main(int argc,char**argv) {
  assert(argc==2);
  if(!strcmp(argv[1],"disabled"))disabled();
#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_RUNTIME
  else if(!strcmp(argv[1],"init"))initialization();
  else if(!strcmp(argv[1],"nominal"))nominal();
  else if(!strcmp(argv[1],"late"))late_before_output();
  else if(!strcmp(argv[1],"delayed_off"))delayed_off();
  else if(!strcmp(argv[1],"cancel"))cancellation();
  else if(!strcmp(argv[1],"expiry"))expiry();
  else if(!strcmp(argv[1],"wrap"))wrap();
  else if(!strcmp(argv[1],"admission"))admission();
  else if(!strcmp(argv[1],"allocation"))allocation_failure();
  else if(!strcmp(argv[1],"due_safety"))safety_at_due(false);
  else if(!strcmp(argv[1],"due_teleop"))safety_at_due(true);
  else if(!strcmp(argv[1],"catalog"))catalog_initialization();
  else if(!strcmp(argv[1],"four_profiles"))four_profiles();
  else if(!strcmp(argv[1],"capacity"))capacity_accounting();
#endif
  else return 2;
  return 0;
}
