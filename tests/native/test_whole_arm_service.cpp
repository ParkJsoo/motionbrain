// Run production service and driver with a virtual independent timer and fake PWM.
#include "esp_timer.h"
#include "control/whole_arm_service.h"
#include "control/angle_controller.h"
#include "control/shoulder_angle_controller.h"
#include "control/command.h"
#include "input/teleop_adapter.h"
#include "motion/motion_sequence.h"
#include "motor/motor_driver.h"
#include "network/web_server.h"
#include "peripheral/shoulder_angle_sensor.h"
#include "safety/safety_monitor.h"

#include <cassert>
#include <cstring>
#include <cstdio>
#include <cstdlib>

MotorControl motorControl;
SystemStateManager systemState;
SafetyMonitor safetyMonitor;
MotionSequence motionSequence;
AngleController angleController;
ShoulderAngleSensor shoulderAngleSensor;
ShoulderAngleController shoulderAngleController;
TeleopAdapter teleopAdapter;
MotionBrainWebServer webServer;
namespace M4GoalService { bool owns() { return false; } }

static unsigned clockMs = 100;
static unsigned duty[5]{};
static unsigned halWrites=0;
static FakeTimer timer{};
static bool locked = false;
static int expireOnRead = 0;
static unsigned firstPinDelayMs = 0;
static unsigned preemptPwmAtMs = 0, callbackMs = 0;
static bool preemptAfterUnlock = false;

uint32_t millis() { return clockMs; }
int64_t esp_timer_get_time() {
  if (expireOnRead && --expireOnRead == 0) clockMs += 151;
  return int64_t(clockMs) * 1000;
}
SemaphoreHandle_t xSemaphoreCreateMutex() { return &locked; }
void xSemaphoreTake(SemaphoreHandle_t mutex, int) { assert(!*mutex); *mutex=true; }
void xSemaphoreGive(SemaphoreHandle_t mutex) {
  assert(*mutex); *mutex=false;
  if (preemptAfterUnlock) {
    preemptAfterUnlock=false;
    clockMs=callbackMs;timer.callback(nullptr);
    clockMs+=2;
  }
}
int esp_timer_create(const esp_timer_create_args_t* args, esp_timer_handle_t* handle) {
  timer.callback=args->callback; *handle=&timer; return 0;
}
int esp_timer_start_periodic(esp_timer_handle_t h, uint64_t us) { assert(h==&timer && us==5000); return 0; }
void esp_timer_delete(esp_timer_handle_t) {}
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) { clockMs+=firstPinDelayMs; firstPinDelayMs=0; }
void ledcSetup(uint8_t, uint32_t, uint8_t) {}
void ledcAttachPin(uint8_t, uint8_t) {}
void ledcWrite(uint8_t channel, uint32_t value) {
  assert(channel<5 && value<=70);++halWrites;duty[channel]=value;
  if (preemptPwmAtMs && clockMs==preemptPwmAtMs && channel==3 && value==70) {
    preemptPwmAtMs=0;preemptAfterUnlock=true;
  }
}

using whole_arm::Op;
using whole_arm::Phase;
static const char* TRIAL = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
static const char* BOOT = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
static bool apply(Op op, unsigned seq) {
  return WholeArmService::apply({op,Phase::Approach,TRIAL,seq}, WholeArmService::generation(), BOOT);
}

static void testSetState(SystemState state) {
#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
  if (systemState.getState()==SystemState::BOOT) assert(systemState.transitionTo(SystemState::IDLE));
  assert(systemState.transitionTo(state));
#else
  systemState.state=state;
#endif
}

static void begin() {
  assert(motorControl.init());
  testSetState(SystemState::ARMED);
  shoulderAngleSensor.sample=clockMs; shoulderAngleSensor.angle=220.f;
  assert(apply(Op::Start,1));
  clockMs+=50; shoulderAngleSensor.sample=clockMs;
  WholeArmService::update();
  motorControl.update();
  assert(duty[3] == 10);
}

static void stalledLoop() {
  begin();
  // Advance the timer task alone. The main loop cannot refresh angle feedback.
  clockMs+=151;
  assert(duty[3]>0);
  timer.callback(nullptr);
  assert(duty[3]==0);
  // A stale main-loop ramp write cannot undo the independent cutoff.
  assert(!WholeArmService::writePwm(4,3,-20));
  assert(duty[3]==0);
  WholeArmService::update();
  assert(!apply(Op::Keep,2));
  const auto* last=motorControl.outputLog().at(motorControl.outputLog().count()-1);
  assert(last->pwm==0 && last->reason==bounded::OutputReason::ForcedOff);
}

static void preempt() {
  begin();
  assert(!WholeArmService::beforeLegacy(CommandType::MOTOR_RUN));
  assert(WholeArmService::beforeLegacy(CommandType::DISARM));
  assert(duty[3]==0);
  assert(!apply(Op::Keep,2));
}

static void stopRace() {
  begin();
  // Normal STOP first checks the timer, then its driver write takes the mutex
  // again. Expiry in that gap must surface as a forced OFF, not completion.
  expireOnRead = 3;
  assert(!apply(Op::Stop,2));
  assert(expireOnRead == 0);
  assert(duty[3] == 0);
  const auto* last = motorControl.outputLog().at(motorControl.outputLog().count()-1);
  assert(last->pwm==0 && last->reason==bounded::OutputReason::ForcedOff);
  assert(!WholeArmService::apply({Op::Start,Phase::Close,TRIAL,3}, WholeArmService::generation(), BOOT));
  assert(!motorControl.hasPendingOutput());
}

static void staleInitialStart() {
  assert(motorControl.init());
  testSetState(SystemState::ARMED);
  shoulderAngleSensor.sample=clockMs;
  shoulderAngleSensor.angle=220.f;
  const uint32_t initialGeneration=WholeArmService::generation();
  whole_arm::Request start{Op::Start,Phase::Approach,TRIAL,1};
  assert(!WholeArmService::apply(start,initialGeneration, "cccccccccccccccccccccccccccccccc"));
  assert(!motorControl.hasPendingOutput());
  assert(WholeArmService::beforeLegacy(CommandType::JOINT_STOP));
  assert(!WholeArmService::apply(start,initialGeneration,BOOT));
  assert(!motorControl.hasPendingOutput());
  assert(!WholeArmService::owns());
}

static bool phaseRequest(Op op, Phase phase, unsigned seq) {
  return WholeArmService::apply({op,phase,TRIAL,seq}, WholeArmService::generation(), BOOT);
}

static void finishLower() {
  assert(motorControl.init());
  testSetState(SystemState::ARMED);
  shoulderAngleSensor.angle=190.f;
  Phase parsed;
  assert(WholeArmService::parsePhase("lower_correction",parsed) && parsed==Phase::LowerCorrection);
  assert(!phaseRequest(Op::Start,parsed,1));
  unsigned seq=0;
  for (Phase p : {Phase::Approach,Phase::Close,Phase::Lift,Phase::Lower}) {
    shoulderAngleSensor.sample=clockMs;
    assert(phaseRequest(Op::Start,p,++seq));
    clockMs+=50;
    shoulderAngleSensor.sample=clockMs;
    WholeArmService::update();
    motorControl.update();
    assert(phaseRequest(Op::Stop,p,++seq));
  }
}

static void correction(const char* mode, char** wire) {
  if (wire) TRIAL=wire[3];
  finishLower();
  const auto& journal=motorControl.outputLog();
  const auto before=journal.count();
  if (!strcmp(mode,"correction-slow-start")) {
    firstPinDelayMs=16;
    assert(!phaseRequest(Op::Start,Phase::LowerCorrection,9));
    assert(duty[3]==0 && journal.count()==before+1);
    assert(journal.at(before)->reason==bounded::OutputReason::ForcedOff);
    return;
  }
  if (!strcmp(mode,"correction-start-offset")) firstPinDelayMs=10;
  const auto start=clockMs+firstPinDelayMs;
  if (wire) {
    Phase phase;
    assert(WholeArmService::parsePhase(wire[0],phase));
    assert(!strcmp(wire[1],"start"));
    const whole_arm::Request packet{Op::Start,phase,wire[3],static_cast<uint32_t>(std::stoul(wire[2]))};
    assert(WholeArmService::apply(packet,static_cast<uint32_t>(std::stoul(wire[5])),wire[4]));
  } else {
    assert(phaseRequest(Op::Start,Phase::LowerCorrection,9));
  }
  assert(duty[3]==10 && journal.count()==before+1);
  assert(!phaseRequest(Op::Keep,Phase::LowerCorrection,10));
  assert(!phaseRequest(Op::Start,Phase::Release,10));
  if (!strcmp(mode,"correction-stall")) {
    clockMs+=97; timer.callback(nullptr);
    assert(duty[3]==0);
    shoulderAngleSensor.sample=clockMs;
    WholeArmService::update();
    assert(!phaseRequest(Op::Start,Phase::Release,10));
    assert(journal.count()==before+2);
    assert(journal.at(before+1)->reason==bounded::OutputReason::ForcedOff);
    return;
  }
  if (!strcmp(mode,"correction-abort")) {
    assert(WholeArmService::beforeLegacy(CommandType::STOP));
    assert(duty[3]==0);
    assert(!phaseRequest(Op::Start,Phase::LowerCorrection,10));
    assert(!phaseRequest(Op::Start,Phase::Release,10));
    return;
  }
  for (unsigned elapsed=1;elapsed<=600;++elapsed) {
    clockMs=start+elapsed;
    shoulderAngleSensor.sample=clockMs;
    timer.callback(nullptr);
    WholeArmService::update();
    motorControl.update();
  }
  assert(journal.count()==before+10);
  static const unsigned offsets[]={0,82,147,206,262,347,403,453,503,554};
  static const int expected[]={-10,-20,-30,-40,-50,-40,-30,-20,-10,0};
  for (unsigned i=0;i<10;++i) {
    auto* row=journal.at(before+i);
    assert(row->atMs==start+offsets[i] && row->motor==4 && row->pwm==expected[i]);
    assert(row->reason==bounded::OutputReason::DriverWrite);
    if (!strcmp(mode,"correction-trace"))
      std::printf("%u\t%u\t%u\t%d\n",row->sequence,row->atMs,row->motor,row->pwm);
  }
  String state;
  WholeArmService::status(state);
  if (!strcmp(mode,"correction-trace")) std::puts(state.c_str());
  assert(strstr(state.c_str(),"\"reason\":\"stopped\""));
  assert(strstr(state.c_str(),"\"active\":false"));
  assert(!motorControl.hasPendingOutput() && !motorControl.isEnabled());
  for (auto value : duty) assert(value==0);
  assert(!phaseRequest(Op::Start,Phase::LowerCorrection,9));
  assert(!phaseRequest(Op::Start,Phase::LowerCorrection,10));
  assert(!phaseRequest(Op::Stop,Phase::LowerCorrection,10));
  if (strcmp(mode,"correction-trace")) assert(phaseRequest(Op::Start,Phase::Release,10));
}

static void elbowOnce(const char* mode) {
  assert(motorControl.init());
  testSetState(SystemState::ARMED);
  shoulderAngleSensor.sample=clockMs;
  Phase parsed;
  const bool supported=WholeArmService::parsePhase("elbow_down_once",parsed);
  assert(supported==bool(MOTIONBRAIN_ENABLE_ELBOW_DOWN_ONCE));
  const whole_arm::Request start{Op::Start,Phase::ElbowDownOnce,TRIAL,1};
  const auto generation=WholeArmService::generation();
  shoulderAngleSensor.angle=211.f;
  assert(!WholeArmService::apply(start,generation,BOOT));
  shoulderAngleSensor.angle=220.f;
  if (!supported) { assert(!WholeArmService::apply(start,generation,BOOT));return; }
  assert(WholeArmService::apply(start,generation,BOOT));
  assert(!WholeArmService::apply(start,generation,BOOT));
  assert(!WholeArmService::apply({Op::Keep,Phase::ElbowDownOnce,TRIAL,2},generation,BOOT));
  assert(!WholeArmService::beforeLegacy(CommandType::MOTOR_RUN));
  for (unsigned i=0;i<6;i++) {
    clockMs+=50; shoulderAngleSensor.sample=clockMs;
    WholeArmService::update();motorControl.update();
    assert(duty[2]==(i<5 ? (i+1)*10 : 50));
    assert(duty[0]==0 && duty[1]==0 && duty[3]==0 && duty[4]==0);
  }
  if (!strcmp(mode,"elbow-stall")) {
    clockMs+=65; timer.callback(nullptr);
    assert(duty[2]==0 && !WholeArmService::writePwm(3,2,50));
    WholeArmService::update();
  } else if (!strcmp(mode,"elbow-abort")) {
    assert(WholeArmService::beforeLegacy(CommandType::DISARM));
  } else {
    clockMs+=50; shoulderAngleSensor.sample=clockMs;WholeArmService::update();
  }
  assert(duty[2]==0 && !motorControl.hasPendingOutput());
  String state;WholeArmService::status(state);
  assert(strstr(state.c_str(),"\"elbowFirstWriteOffsetUs\":50000"));
  if (!strcmp(mode,"elbow-normal")) {
    assert(strstr(state.c_str(),"\"reason\":\"completed\""));
    assert(strstr(state.c_str(),"\"elbowZeroWriteOffsetUs\":350000"));
  } else {
    assert(strstr(state.c_str(),"\"failed\":true"));
  }
  assert(!WholeArmService::apply(start,WholeArmService::generation(),BOOT));
  assert(!WholeArmService::apply({Op::Start,Phase::Approach,TRIAL,2},WholeArmService::generation(),BOOT));
}

static uint32_t outputCursor() {
  const auto& log=motorControl.outputLog();
  return log.count() ? log.at(log.count()-1)->sequence : 0;
}

static bool stepStart(Phase phase,unsigned seq,uint32_t cursor) {
  return WholeArmService::apply({Op::Start,phase,TRIAL,seq},WholeArmService::generation(),BOOT,&cursor);
}

static void runStep(Phase phase,unsigned seq) {
  const auto cursor=outputCursor();
  const auto generation=WholeArmService::generation();
  assert(!stepStart(phase,seq,cursor+1));
  assert(!phaseRequest(Op::Start,phase,seq));
  shoulderAngleSensor.sample=clockMs;
  assert(stepStart(phase,seq,cursor));
  assert(!stepStart(phase,seq,cursor));
  assert(!stepStart(phase,seq+1,cursor));
  assert(!phaseRequest(Op::Keep,phase,seq+1));
  const uint32_t ms=whole_arm::stepDurationMs(phase);
  for (unsigned elapsed=1;elapsed<=ms;elapsed++) {
    ++clockMs; shoulderAngleSensor.sample=clockMs;
    timer.callback(nullptr);WholeArmService::update();motorControl.update();
  }
  const uint8_t motor=whole_arm::phaseMotor(phase);
  const int sign=whole_arm::phasePwm(phase)>0 ? 1:-1;
  const auto& log=motorControl.outputLog();
  int powers=0;
  for (unsigned i=0;i<log.count();i++) {
    const auto* e=log.at(i);
    if (e->sequence<=cursor) continue;
    assert(e->motor==motor && e->reason==bounded::OutputReason::DriverWrite);
    if (e->sequence==outputCursor()) assert(e->pwm==0);
    else { ++powers;assert(e->pwm==powers*10*sign); }
  }
  assert(powers==((phase==Phase::ShoulderDown420 || phase==Phase::ShoulderDown650) ? 7:5));
  assert(WholeArmService::generation()==generation && WholeArmService::owns());
  assert(!motorControl.hasPendingOutput() && !motorControl.isEnabled());
  for (auto value : duty) assert(value==0);
  String state;WholeArmService::status(state);
  char field[64];
  std::snprintf(field,sizeof(field),"\"stepZeroSequence\":%u",outputCursor());
  assert(strstr(state.c_str(),field));
  std::snprintf(field,sizeof(field),"\"stepZeroWriteOffsetUs\":%u",ms*1000);
  assert(strstr(state.c_str(),field) && strstr(state.c_str(),"\"stepFirstWriteOffsetUs\":50000"));
  assert(strstr(state.c_str(),"\"reason\":\"stopped\"") && strstr(state.c_str(),"\"active\":false"));
  assert(!stepStart(phase,seq,outputCursor()));
}

static void approachSteps(const char* mode) {
  assert(motorControl.init());
  testSetState(SystemState::ARMED);
  shoulderAngleSensor.sample=clockMs;
  shoulderAngleSensor.angle=220.f;
  Phase parsed;
  assert(WholeArmService::parsePhase("shoulder_down_420",parsed)==bool(MOTIONBRAIN_ENABLE_BOUNDED_APPROACH));
  if (!MOTIONBRAIN_ENABLE_BOUNDED_APPROACH) {
    assert(!stepStart(Phase::ShoulderDown300,1,outputCursor()));
    return;
  }
  assert(parsed==Phase::ShoulderDown420);
  assert(WholeArmService::parsePhase("shoulder_down_300",parsed) && parsed==Phase::ShoulderDown300);
  assert(WholeArmService::parsePhase("elbow_down_350",parsed) && parsed==Phase::ElbowDown350);
  if (!strcmp(mode,"steps-stall") || !strcmp(mode,"steps-preempt")) {
    assert(stepStart(Phase::ShoulderDown300,1,outputCursor()));
    for(unsigned i=0;i<5;i++) {
      clockMs+=50;shoulderAngleSensor.sample=clockMs;
      WholeArmService::update();motorControl.update();
    }
    assert(duty[3]==50);
    if (!strcmp(mode,"steps-preempt")) {
      assert(WholeArmService::beforeLegacy(CommandType::STOP));
    } else {
      // Admission is off the periodic timer grid. With no loop servicing the
      // last ramp, the next callback after the early deadline must zero by cap.
      clockMs+=57;timer.callback(nullptr); // elapsed 307, first 5ms grid callback
      assert(duty[3]==50);
      clockMs+=5;timer.callback(nullptr); // elapsed 312, before the 315ms cap
      assert(duty[3]==0 && !WholeArmService::writePwm(4,3,-60));
      WholeArmService::update();
    }
    assert(duty[3]==0 && !motorControl.hasPendingOutput());
    assert(!stepStart(Phase::ShoulderDown420,2,outputCursor()));
    return;
  }
  if (!strcmp(mode,"steps-budget")) {
    for (unsigned seq=1;seq<=16;seq++) runStep(Phase::ShoulderDown420,seq);
    assert(!stepStart(Phase::ShoulderDown300,17,outputCursor()));
    return;
  }
  runStep(Phase::ShoulderDown420,1);
  if (!strcmp(mode,"steps-off-override")) {
    assert(WholeArmService::beforeLegacy(CommandType::DISARM));
    assert(!stepStart(Phase::ShoulderDown420,2,outputCursor()));
    return;
  }
  assert(!stepStart(Phase::Close,2,outputCursor())); // close still requires <=194
  assert(!phaseRequest(Op::Start,Phase::ElbowDownOnce,2));
  assert(!phaseRequest(Op::Start,Phase::Approach,2));
  runStep(Phase::ShoulderDown300,2);
  shoulderAngleSensor.angle=218.f;
  assert(!stepStart(Phase::ElbowDown350,3,outputCursor()));
  shoulderAngleSensor.angle=220.f;
  runStep(Phase::ElbowDown350,3);
  const auto cursor=outputCursor();
  assert(!WholeArmService::apply({Op::Start,Phase::ShoulderDown420,TRIAL,4},
          WholeArmService::generation()+1,BOOT,&cursor));
  assert(!WholeArmService::apply({Op::Start,Phase::ShoulderDown420,BOOT,4},
          WholeArmService::generation(),BOOT,&cursor));
  assert(!stepStart(Phase::ShoulderDown300,4,cursor-1));
  runStep(Phase::ShoulderDown420,4);
  shoulderAngleSensor.angle=193.f;
  assert(!phaseRequest(Op::Start,Phase::Close,5));
  assert(!stepStart(Phase::Close,5,outputCursor()-1));
  assert(stepStart(Phase::Close,5,outputCursor()));
  clockMs+=50;shoulderAngleSensor.sample=clockMs;WholeArmService::update();motorControl.update();
  assert(duty[0]==10 && phaseRequest(Op::Stop,Phase::Close,6));
  assert(!stepStart(Phase::ShoulderDown300,7,outputCursor()));
  assert(phaseRequest(Op::Start,Phase::Lift,7));
  clockMs+=50;shoulderAngleSensor.sample=clockMs;WholeArmService::update();motorControl.update();
  assert(duty[3]==10 && phaseRequest(Op::Stop,Phase::Lift,8));
}

static void poseWorkflow(const char* mode) {
  assert(motorControl.init());
  testSetState(SystemState::ARMED);
  shoulderAngleSensor.sample=clockMs;
  shoulderAngleSensor.angle=220.f;
  const auto generation=WholeArmService::generation();
  const char* scope="two-view-open-grasp-v1";
  const char* digest="dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd";
  Phase parsed;
  assert(WholeArmService::parsePhase("wrist_up_420",parsed)==bool(MOTIONBRAIN_ENABLE_POSE_GRASP));
  assert(WholeArmService::parsePhase("pose_close",parsed)==bool(MOTIONBRAIN_ENABLE_POSE_GRASP));
  assert(WholeArmService::parsePhase("shoulder_down_650",parsed)==bool(MOTIONBRAIN_ENABLE_POSE_GRASP));
  if (!MOTIONBRAIN_ENABLE_POSE_GRASP) {
    assert(!stepStart(Phase::WristUp420,1,outputCursor()));
    assert(!stepStart(Phase::PoseClose,1,outputCursor()));
    assert(!stepStart(Phase::ShoulderDown650,1,outputCursor()));
    return;
  }
  assert(whole_arm::phaseMotor(Phase::WristUp420)==2 && whole_arm::phasePwm(Phase::WristUp420)==50);
  assert(whole_arm::phaseMotor(Phase::PoseClose)==1 && whole_arm::phasePwm(Phase::PoseClose)==-70);
  if (!strcmp(mode,"pose-long-step")) {
    shoulderAngleSensor.angle=208.99f;
    assert(!stepStart(Phase::ShoulderDown650,1,outputCursor()));
    shoulderAngleSensor.angle=224.01f;
    assert(!stepStart(Phase::ShoulderDown650,1,outputCursor()));
    shoulderAngleSensor.angle=209.f;
    for (unsigned seq=1;seq<=10;seq++) runStep(Phase::ShoulderDown650,seq);
    assert(!stepStart(Phase::ShoulderDown420,11,outputCursor()));
    runStep(Phase::ShoulderDown300,11);
    assert(!stepStart(Phase::ShoulderDown300,12,outputCursor()));
    return;
  }
  if (!strcmp(mode,"pose-long-stall") || !strcmp(mode,"pose-long-angle")) {
    assert(stepStart(Phase::ShoulderDown650,1,outputCursor()));
    for(unsigned i=0;i<12;i++) {
      clockMs+=50;shoulderAngleSensor.sample=clockMs;WholeArmService::update();motorControl.update();
    }
    assert(duty[3]==70);
    if (!strcmp(mode,"pose-long-angle")) {
      shoulderAngleSensor.angle=208.9f;WholeArmService::update();
    } else {
      clockMs+=62;timer.callback(nullptr);
      assert(!WholeArmService::writePwm(4,3,-70));WholeArmService::update();
    }
    assert(duty[3]==0 && !motorControl.hasPendingOutput());
    assert(!stepStart(Phase::ShoulderDown300,2,outputCursor()));
    return;
  }
  if (!strcmp(mode,"pose-budgets")) {
    unsigned seq=0;
    for(unsigned i=0;i<16;i++) runStep(Phase::ElbowDown350,++seq);
    assert(!stepStart(Phase::ElbowDown350,seq+1,outputCursor()));
    for(unsigned i=0;i<4;i++) runStep(Phase::WristUp420,++seq);
    assert(!stepStart(Phase::WristUp420,seq+1,outputCursor())); // combined: 8015
    for(unsigned i=0;i<16;i++) runStep(Phase::ShoulderDown420,++seq);
    assert(!stepStart(Phase::ShoulderDown300,seq+1,outputCursor()));
    return;
  }
  if (!strcmp(mode,"pose-wrist-budget")) {
    unsigned seq=0;
    for(unsigned i=0;i<5;i++) runStep(Phase::WristUp420,++seq);
    assert(!stepStart(Phase::WristUp420,seq+1,outputCursor())); // 2610 exceeds 2600
    runStep(Phase::ElbowDown350,++seq);
    return;
  }
  if (!strcmp(mode,"pose-wrist-stall")) {
    assert(stepStart(Phase::WristUp420,1,outputCursor()));
    for(unsigned i=0;i<8;i++) {
      clockMs+=50;shoulderAngleSensor.sample=clockMs;WholeArmService::update();motorControl.update();
    }
    assert(duty[1]==50);
    clockMs+=32;timer.callback(nullptr);
    assert(duty[1]==0 && !WholeArmService::writePwm(2,1,50));WholeArmService::update();
    assert(!stepStart(Phase::ElbowDown350,2,outputCursor()));
    return;
  }
  shoulderAngleSensor.angle=218.99f;
  assert(!stepStart(Phase::WristUp420,1,outputCursor()));
  shoulderAngleSensor.angle=224.01f;
  assert(!stepStart(Phase::WristUp420,1,outputCursor()));
  shoulderAngleSensor.angle=224.f;
  runStep(Phase::WristUp420,1);
  runStep(Phase::ElbowDown350,2);
  const whole_arm::Request early{Op::Start,Phase::PoseClose,TRIAL,3};
  shoulderAngleSensor.angle=205.f;
  assert(!WholeArmService::preparePoseClose(early,generation,BOOT,outputCursor(),scope));
  runStep(Phase::ShoulderDown300,3);
  shoulderAngleSensor.angle=220.f;
  assert(!stepStart(Phase::ElbowDown350,4,outputCursor()));
  assert(!stepStart(Phase::WristUp420,4,outputCursor()));
  const uint32_t off=outputCursor();
  const whole_arm::Request start{Op::Start,Phase::PoseClose,TRIAL,4};
  shoulderAngleSensor.angle=194.f;
  assert(!WholeArmService::preparePoseClose(start,generation,BOOT,off,scope));
  shoulderAngleSensor.angle=206.01f;
  assert(!WholeArmService::preparePoseClose(start,generation,BOOT,off,scope));
  shoulderAngleSensor.angle=206.f;
  assert(!stepStart(Phase::Close,4,off));
  assert(!WholeArmService::preparePoseClose(start,generation,BOOT,off+1,scope));
  assert(!WholeArmService::preparePoseClose(start,generation+1,BOOT,off,scope));
  assert(!WholeArmService::preparePoseClose(start,generation,BOOT,off,"other"));
  assert(!stepStart(Phase::PoseClose,4,off));
  assert(WholeArmService::preparePoseClose(start,generation,BOOT,off,scope));
  assert(!WholeArmService::preparePoseClose(start,generation,BOOT,off,scope));
  assert(outputCursor()==off && !motorControl.hasPendingOutput());
  String state;WholeArmService::status(state);
  assert(!strstr(state.c_str(),"poseObservationNonce"));
  WholeArmService::status(state,true);
  const char* tag="\"poseObservationNonce\":\"";
  const char* value=strstr(state.c_str(),tag);
  assert(value);char nonce[33]{};
  std::memcpy(nonce,value+std::strlen(tag),32);
  const WholeArmService::PoseObservation good{scope,nonce,digest},wrongNonce{scope,BOOT,digest},
                                         wrongDigest{scope,nonce,"not_sha256"};
  assert(!WholeArmService::apply(start,generation,BOOT,&off,&wrongNonce));
  assert(!WholeArmService::apply(start,generation,BOOT,&off,&wrongDigest));
  if (!strcmp(mode,"pose-expired")) {
    clockMs+=4000;shoulderAngleSensor.sample=clockMs;
    assert(!WholeArmService::apply(start,generation,BOOT,&off,&good));
    assert(!WholeArmService::preparePoseClose(start,generation,BOOT,off,scope));
    assert(!motorControl.hasPendingOutput());return;
  }
  if (!strcmp(mode,"pose-angle-change")) {
    shoulderAngleSensor.angle=207.f;
    assert(!WholeArmService::apply(start,generation,BOOT,&off,&good));
    shoulderAngleSensor.angle=205.f;
    assert(!WholeArmService::apply(start,generation,BOOT,&off,&good));
    assert(!motorControl.hasPendingOutput());return;
  }
  assert(WholeArmService::apply(start,generation,BOOT,&off,&good));
  assert(!WholeArmService::apply(start,generation,BOOT,&off,&good));
  assert(!phaseRequest(Op::Start,Phase::Lift,5));
  for(unsigned i=0;i<7;i++) {
    clockMs+=50;shoulderAngleSensor.sample=clockMs;WholeArmService::update();motorControl.update();
    assert(duty[0]==10*(i+1));
  }
  if (!strcmp(mode,"pose-stall") || !strcmp(mode,"pose-abort")) {
    if (!strcmp(mode,"pose-stall")) { clockMs+=151;timer.callback(nullptr); }
    else assert(WholeArmService::beforeLegacy(CommandType::STOP));
    assert(duty[0]==0);WholeArmService::update();
    assert(!stepStart(Phase::Lift,5,outputCursor()));return;
  }
  if (!strcmp(mode,"pose-close-cap")) {
    unsigned seq=4;
    for(unsigned i=1;i<=2600;i++) {
      ++clockMs;shoulderAngleSensor.sample=clockMs;
      if(i%300==0) assert(phaseRequest(Op::Keep,Phase::PoseClose,++seq));
      timer.callback(nullptr);WholeArmService::update();motorControl.update();
    }
    assert(duty[0]==70);
    clockMs+=48;timer.callback(nullptr);WholeArmService::update(); // elapsed 2998
    assert(duty[0]==0 && !stepStart(Phase::Lift,seq+1,outputCursor()));return;
  }
  assert(phaseRequest(Op::Keep,Phase::PoseClose,5));
  assert(phaseRequest(Op::Stop,Phase::PoseClose,6));
  WholeArmService::status(state);
  assert(strstr(state.c_str(),digest) && strstr(state.c_str(),"\"stepZeroWriteOffsetUs\":350000"));
  assert(strstr(state.c_str(),"\"poseElbowBudgetUsedMs\":365"));
  assert(strstr(state.c_str(),"\"poseWristBudgetUsedMs\":435"));
  assert(strstr(state.c_str(),"\"poseShoulderBudgetUsedMs\":315"));
  assert(!phaseRequest(Op::Start,Phase::Lift,7));
  assert(!stepStart(Phase::Lift,7,off));
  assert(stepStart(Phase::Lift,7,outputCursor()));
  clockMs+=50;shoulderAngleSensor.sample=clockMs;WholeArmService::update();motorControl.update();
  assert(duty[3]==10);
  shoulderAngleSensor.angle=208.01f;WholeArmService::update();assert(duty[3]==0);
}

static void elbowWave(const char* mode) {
  assert(motorControl.init());
  String state;
  WholeArmService::status(state);
  assert(bool(strstr(state.c_str(),"\"elbowDownWaveProfile\":\"m3-50-300ms-ledger-v1\""))
         ==bool(MOTIONBRAIN_ENABLE_ELBOW_WAVE));
  if (!strcmp(mode,"wave-disabled")) return;
  testSetState(SystemState::ARMED);
  shoulderAngleSensor.angle=220.f;
  shoulderAngleSensor.sample=clockMs;
  const unsigned admitted=clockMs;
  const auto cursor=outputCursor();
  assert(stepStart(Phase::ElbowDown350,1,cursor));
  const auto& log=motorControl.outputLog();
  const auto before=log.count();
  auto tick=[&](unsigned at) {
    clockMs=at;shoulderAngleSensor.sample=clockMs;
    WholeArmService::update();motorControl.update();
  };
  tick(admitted+49);
  assert(log.count()==before && !duty[2]);
  const bool normal=!strcmp(mode,"wave-normal") || !strcmp(mode,"wave-jitter");
  const unsigned first=admitted+(!strcmp(mode,"wave-jitter") ? 54:50);
  if (!strcmp(mode,"wave-late-first")) firstPinDelayMs=10;
  tick(first);
  if (strcmp(mode,"wave-late-first")) {
    assert(duty[2]==10 && log.count()==before+1);
    if (!strcmp(mode,"wave-stall")) {
      clockMs=first+65;timer.callback(nullptr);assert(duty[2]==10);
      clockMs+=5;timer.callback(nullptr);assert(!duty[2]);
      assert(!WholeArmService::writePwm(3,2,20));
    } else if (!strcmp(mode,"wave-early")) {
      assert(!WholeArmService::writePwm(3,2,20));
    } else if (!strcmp(mode,"wave-abort")) {
      assert(WholeArmService::beforeLegacy(CommandType::STOP));
    } else if (!strcmp(mode,"wave-angle")) {
      shoulderAngleSensor.angle=224.1f;
    } else {
      const unsigned offsets[]={56,117,178,228,300};
      for (unsigned i=0;i<5;++i) {
        const bool late=(!strcmp(mode,"wave-late-edge") && i==1)
                     || (!strcmp(mode,"wave-late-zero") && i==4);
        if (late) firstPinDelayMs=10;
        const unsigned jitter=!strcmp(mode,"wave-jitter") ? (4-i):0;
        tick(first+offsets[i]+jitter);
        if (late) break;
        const auto* event=log.at(before+i+1);
        assert(event && event->atMs==first+offsets[i]+jitter && event->motor==3
               && event->pwm==(i<4 ? int((i+2)*10):0)
               && event->reason==bounded::OutputReason::DriverWrite);
      }
    }
  }
  tick(clockMs);
  for (auto value : duty) assert(value==0);
  assert(!motorControl.hasPendingOutput() && !motorControl.isEnabled());
  WholeArmService::status(state);
  if (normal) {
    assert(log.count()==before+6 && strstr(state.c_str(),"\"reason\":\"stopped\""));
    assert(strstr(state.c_str(),"\"stepZeroSequence\":"));
    assert(!stepStart(Phase::ElbowDown350,1,cursor));
    runStep(Phase::ElbowDown350,2);
  } else {
    assert(strstr(state.c_str(),"\"failed\":true") && !strstr(state.c_str(),"\"stepZeroSequence\":"));
    assert(log.at(log.count()-1)->reason==bounded::OutputReason::ForcedOff);
    assert(!stepStart(Phase::ElbowDown350,2,outputCursor()));
  }
}

#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
static void timerElbow(const char* mode) {
  assert(motorControl.init());
  const uint32_t cursor=outputCursor(),gen=WholeArmService::generation();
  shoulderAngleSensor.sample=clockMs;shoulderAngleSensor.angle=220.f;
  testSetState(SystemState::ARMED);
  assert(!stepStart(Phase::ElbowDown350,1,cursor));
  const bool low=!strcmp(mode,"timer-shadow-215");
  if (low) shoulderAngleSensor.angle=215.f;
  const char* probeId="cccccccccccccccccccccccccccccccc";
  whole_arm::Request probe{Op::Start,Phase::ElbowDown350,probeId,1};
  assert(!WholeArmService::startElbowTimingProbe(probe,gen,BOOT,cursor));
  testSetState(SystemState::IDLE);
  assert(!WholeArmService::startElbowTimingProbe(probe,gen,BOOT,cursor+1));
  auto start=clockMs;
  const auto writes=halWrites;
  assert(WholeArmService::startElbowTimingProbe(probe,gen,BOOT,cursor));
  assert(!WholeArmService::startElbowTimingProbe(probe,gen,BOOT,cursor));
  const bool shadowStall=!strcmp(mode,"timer-shadow-stall");
  for (unsigned t=1;t<=390;t++) {
    clockMs=start+t;
    if (t%5==3) timer.callback(nullptr);
    if (t%13==0 && (t<90 || t>135) && (!shadowStall || t<53)) {
      shoulderAngleSensor.sample=clockMs;WholeArmService::update();
    }
  }
  shoulderAngleSensor.sample=clockMs;WholeArmService::update();
  assert(halWrites==writes && outputCursor()==cursor);
  String state;WholeArmService::status(state);
  if (shadowStall) {
    assert(strstr(state.c_str(),"\"status\":\"failed\""));
    testSetState(SystemState::ARMED);
    assert(!stepStart(Phase::ElbowDown350,1,cursor));
    return;
  }
  assert(strstr(state.c_str(),"\"status\":\"passed\""));
  assert(strstr(state.c_str(),"\"offsetsUs\":[53000,113000,173000,233000,283000,353000]"));
  testSetState(SystemState::ARMED);
  if (low) {
    assert(!stepStart(Phase::ElbowDown350,1,cursor) && outputCursor()==cursor && halWrites==writes);
    assert(stepStart(Phase::ShoulderDown420,1,cursor));
    return;
  }
  assert(!WholeArmService::apply(probe,gen,BOOT,&cursor));
  assert(stepStart(Phase::ElbowDown350,1,cursor));
  start=clockMs;
  const bool ok=!strcmp(mode,"timer-normal");
  const bool abort=!strcmp(mode,"timer-stop");
  const bool safety=!strcmp(mode,"timer-safety");
  const bool noMain=!strcmp(mode,"timer-no-main");
  const bool timerLate=!strcmp(mode,"timer-late");
  for (unsigned t=1;t<=395;t++) {
    clockMs=start+t;
    if (t%5==3 && !(timerLate && t>103 && t<123)) timer.callback(nullptr);
    if (t==55) { assert(duty[2]==10);assert(motorControl.getSpeed(3)==10); }
    if (abort && t==86) motorControl.emergencyStop();
    if (!strcmp(mode,"timer-direct-disarm") && t==86) {
      assert(duty[2]==10 && systemState.disarm());
      assert(!duty[2] && systemState.arm());
    }
    if (safety && t==88) { shoulderAngleSensor.angle=224.1f;WholeArmService::update(); }
    if (t==118 && ok) assert(duty[2]==20); // main loop unavailable across the second edge
    if (t%13==0 && (t<90 || t>135) && t<340 && (!noMain || t<53)) {
      shoulderAngleSensor.sample=clockMs;WholeArmService::update();motorControl.update();
    }
    if (t==358) assert(!duty[2]);
  }
  shoulderAngleSensor.sample=clockMs;WholeArmService::update();
  WholeArmService::status(state);
  const auto& log=motorControl.outputLog();
  for (auto value : duty) assert(!value);
  if (ok) {
    assert(strstr(state.c_str(),"\"reason\":\"stopped\"") && outputCursor()==cursor+6);
    const unsigned offsets[]={53,113,173,233,283,353};
    for (unsigned i=0;i<6;i++) {
      const auto* e=log.at(log.count()-6+i);
      assert(e->atMs==start+offsets[i] && e->motor==3 && e->pwm==(i==5 ? 0:int(10*(i+1))));
      assert(e->reason==bounded::OutputReason::DriverWrite);
    }
    assert(!stepStart(Phase::ElbowDown350,1,outputCursor()));
    runStep(Phase::ShoulderDown420,2);
  } else {
    assert(strstr(state.c_str(),"\"failed\":true") && !strstr(state.c_str(),"\"stepZeroSequence\":"));
    assert(!stepStart(Phase::ElbowDown350,2,outputCursor()));
    const auto* firstOff=log.at(cursor+1);
    assert(firstOff && firstOff->motor==3 && firstOff->pwm==0 && firstOff->reason==bounded::OutputReason::ForcedOff);
    assert(firstOff->atMs<=start+123);
  }
}
#endif

#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
#include "shoulder_timer_cases.h"
#endif

int main(int argc, char** argv) {
  assert(argc == 2 || (argc==8 && !strcmp(argv[1],"correction-trace")));
  if (!strcmp(argv[1],"stalled")) stalledLoop();
#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
  else if (!strncmp(argv[1],"shoulder-timer-",15)) shoulderTimer(argv[1]);
  else if (!strncmp(argv[1],"timer-",6)) timerElbow(argv[1]);
#endif
  else if (!strncmp(argv[1],"wave-",5)) elbowWave(argv[1]);
  else if (!strncmp(argv[1],"pose-",5)) poseWorkflow(argv[1]);
  else if (!strncmp(argv[1],"steps-",6)) approachSteps(argv[1]);
  else if (!strncmp(argv[1],"elbow-",6)) elbowOnce(argv[1]);
  else if (!strcmp(argv[1],"preempt")) preempt();
  else if (!strcmp(argv[1],"first")) staleInitialStart();
  else if (!strcmp(argv[1],"race")) stopRace();
  else if (!strncmp(argv[1],"correction-",11)) correction(argv[1],argc==8 ? argv+2 : nullptr);
  else return 2;
}
