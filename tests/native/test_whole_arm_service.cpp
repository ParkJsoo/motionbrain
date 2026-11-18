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
static FakeTimer timer{};
static bool locked = false;
static int expireOnRead = 0;

uint32_t millis() { return clockMs; }
int64_t esp_timer_get_time() {
  if (expireOnRead && --expireOnRead == 0) clockMs += 151;
  return int64_t(clockMs) * 1000;
}
SemaphoreHandle_t xSemaphoreCreateMutex() { return &locked; }
void xSemaphoreTake(SemaphoreHandle_t mutex, int) { assert(!*mutex); *mutex=true; }
void xSemaphoreGive(SemaphoreHandle_t mutex) { assert(*mutex); *mutex=false; }
int esp_timer_create(const esp_timer_create_args_t* args, esp_timer_handle_t* handle) {
  timer.callback=args->callback; *handle=&timer; return 0;
}
int esp_timer_start_periodic(esp_timer_handle_t h, uint64_t us) { assert(h==&timer && us==5000); return 0; }
void esp_timer_delete(esp_timer_handle_t) {}
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
void ledcSetup(uint8_t, uint32_t, uint8_t) {}
void ledcAttachPin(uint8_t, uint8_t) {}
void ledcWrite(uint8_t channel, uint32_t value) { assert(channel<5 && value<=70); duty[channel]=value; }

using whole_arm::Op;
using whole_arm::Phase;
static const char* TRIAL = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
static const char* BOOT = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
static bool apply(Op op, unsigned seq) {
  return WholeArmService::apply({op,Phase::Approach,TRIAL,seq}, WholeArmService::generation(), BOOT);
}

static void begin() {
  assert(motorControl.init());
  systemState.state=SystemState::ARMED;
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
  systemState.state=SystemState::ARMED;
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

int main(int argc, char** argv) {
  assert(argc == 2);
  if (!strcmp(argv[1],"stalled")) stalledLoop();
  else if (!strcmp(argv[1],"preempt")) preempt();
  else if (!strcmp(argv[1],"first")) staleInitialStart();
  else if (!strcmp(argv[1],"race")) stopRace();
  else return 2;
}
