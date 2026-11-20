#include "whole_arm_service.h"
#if MOTIONBRAIN_ENABLE_WHOLE_ARM_POLICY
#include <Arduino.h>
#include <esp_timer.h>
#if MOTIONBRAIN_ENABLE_POSE_GRASP
#include <esp_system.h>
#endif
#include <cstring>
#include "control/angle_controller.h"
#include "control/bounded_action_service.h"
#include "control/command.h"
#include "control/m4_goal_service.h"
#include "control/shoulder_angle_controller.h"
#include "input/teleop_adapter.h"
#include "motion/motion_sequence.h"
#include "motor/motor_driver.h"
#include "network/web_server.h"
#include "peripheral/shoulder_angle_sensor.h"
#include "safety/safety_monitor.h"

extern MotorControl motorControl;
extern SystemStateManager systemState;
extern SafetyMonitor safetyMonitor;
extern MotionSequence motionSequence;
extern AngleController angleController;
extern ShoulderAngleSensor shoulderAngleSensor;
extern ShoulderAngleController shoulderAngleController;
extern TeleopAdapter teleopAdapter;
extern MotionBrainWebServer webServer;
namespace WholeArmService {
using whole_arm::Phase;
static_assert(!MOTIONBRAIN_ENABLE_BOUNDED_APPROACH || MOTIONBRAIN_ENABLE_ELBOW_DOWN_ONCE,
              "bounded approach requires the motor-3 output profile");
static_assert(!MOTIONBRAIN_ENABLE_POSE_GRASP || MOTIONBRAIN_ENABLE_BOUNDED_APPROACH,
              "pose grasp requires the bounded approach journal");
static_assert(!MOTIONBRAIN_ENABLE_ELBOW_WAVE || MOTIONBRAIN_ENABLE_POSE_GRASP,
              "the elbow waveform requires pose preparation budgets");
static_assert(!MOTIONBRAIN_ENABLE_ELBOW_TIMER || MOTIONBRAIN_ENABLE_ELBOW_WAVE,
              "the timer requires the measured elbow waveform");
namespace {
constexpr const char* NAMES[]={"approach","close","held_lift","lower","release","retreat","lower_correction"
#if MOTIONBRAIN_ENABLE_ELBOW_DOWN_ONCE
  ,"elbow_down_once"
#endif
#if MOTIONBRAIN_ENABLE_BOUNDED_APPROACH
  ,"shoulder_down_300","shoulder_down_420","elbow_down_350"
#endif
#if MOTIONBRAIN_ENABLE_POSE_GRASP
   ,"wrist_up_420","pose_close","shoulder_down_650"
#endif
};
// Arduino HTTP may block the loop while waiting for a partial request. A
// separate ESP timer task cuts PWM even then. The callback touches neither
// the HTTP state, motor bookkeeping nor its non-thread-safe output journal.
// The main loop reconciles the latched fault with hardStop/forced_off.
constexpr int64_t CUTOFF_PERIOD_US=5000;
#if MOTIONBRAIN_ENABLE_ELBOW_WAVE
// The first edge is relative to admission. The others are relative to the
// measured first LEDC write, as in the successful device output journal.
constexpr whole_arm::CorrectionStep ELBOW_WAVE[]={{50,10},{56,20},{117,30},{178,40},{228,50},{300,0}};
constexpr size_t ELBOW_EDGES=sizeof(ELBOW_WAVE)/sizeof(ELBOW_WAVE[0]);
constexpr int64_t ELBOW_LATE_US=10000;
#endif
struct Cutoff {
  SemaphoreHandle_t mutex=nullptr;
  esp_timer_handle_t timer=nullptr;
  uint8_t motor=0,channel=0;
  int16_t pwm=0;
  bool tripped=false;
  int64_t lease=0,phase=0,sensor=0;
  bool timedStep=false;
  int64_t admissionUs=0, firstWriteUs=0, zeroUs=0;
  uint32_t stepZeroSequence=0;
  int64_t correctionOriginUs=0;
  uint32_t correctionOriginMs=0;
  bool awaitingFirstCorrectionWrite=false;
#if MOTIONBRAIN_ENABLE_ELBOW_WAVE
  bool elbowWave=false;
  size_t elbowEdge=0;
#endif
#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
  bool shadow=false;
  size_t grantedEdge=0,published=0;
  int64_t permitUs=0;
  int64_t edgeAtUs[ELBOW_EDGES]{};
  uint32_t edgeAtMs[ELBOW_EDGES]{};
  bool forced=false,forcedPublished=false;
  uint32_t forcedMs=0;
  bool shoulderTimer=false,shoulderZero=false,shoulderPublished=false;
  int64_t shoulderDueUs=0;
  uint32_t shoulderZeroMs=0;
  int16_t shoulderLastPwm=0;
#endif
} cutoff;

void tripLocked() {
#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
  if (!cutoff.shadow)
#endif
  ledcWrite(cutoff.channel,0);
  if (cutoff.timedStep && !cutoff.zeroUs) cutoff.zeroUs=esp_timer_get_time();
#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
  if (cutoff.elbowWave || cutoff.shoulderTimer) { cutoff.forced=true;cutoff.forcedMs=millis(); }
#endif
  cutoff.tripped=true;
}

#if MOTIONBRAIN_ENABLE_ELBOW_WAVE
int64_t elbowDueLocked() {
  return (cutoff.elbowEdge ? cutoff.firstWriteUs : cutoff.admissionUs)
      + 1000LL*ELBOW_WAVE[cutoff.elbowEdge].atMs;
}
void armElbowDeadlineLocked() {
  const int64_t edgeLimit=elbowDueLocked()+ELBOW_LATE_US;
  const int64_t hardLimit=cutoff.admissionUs+1000LL*whole_arm::ELBOW_DOWN_CUTOFF_MS-CUTOFF_PERIOD_US;
  cutoff.phase=edgeLimit<hardLimit ? edgeLimit:hardLimit;
}
#endif
void expireLocked(int64_t t) {
#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
  if ((cutoff.elbowWave && cutoff.elbowEdge==ELBOW_EDGES) || cutoff.shoulderZero) return; // confirmed zero
#endif
  if (cutoff.motor && !cutoff.tripped &&
      (t>=cutoff.lease || t>=cutoff.phase || t>=cutoff.sensor)) {
    tripLocked();
  }
}
#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
void grantEdgeLocked(int64_t now) {
  cutoff.grantedEdge=cutoff.elbowEdge;
  cutoff.permitUs=now;
}
void resetTimedEdgesLocked(bool shadow,int64_t now) {
  cutoff.shadow=shadow;
  cutoff.published=0;
  cutoff.forced=cutoff.forcedPublished=false;
  grantEdgeLocked(now);
}
void driveEdgeLocked(int64_t now) {
  if (!cutoff.motor || !cutoff.elbowWave || cutoff.tripped || cutoff.elbowEdge>=ELBOW_EDGES ||
      now<elbowDueLocked() || cutoff.grantedEdge!=cutoff.elbowEdge || now-cutoff.permitUs>50000) return;
  const size_t i=cutoff.elbowEdge;
  const auto pwm=ELBOW_WAVE[i].pwm;
  // No direction changes, heap, logging, controller calls or bus I/O here.
  // The main control loop must have checked safety/angle since the preceding edge.
  if (!cutoff.shadow) ledcWrite(cutoff.channel,pwm);
  const auto written=esp_timer_get_time();
  cutoff.edgeAtUs[i]=written;cutoff.edgeAtMs[i]=millis();
  if (!i) cutoff.firstWriteUs=written;
  if (!pwm) cutoff.zeroUs=written;
  // Preserve even a physically written edge that finished outside its window.
  ++cutoff.elbowEdge;
  if (written>=cutoff.phase || written>=cutoff.sensor || written>=cutoff.lease) {
    tripLocked();return;
  }
  if (cutoff.elbowEdge<ELBOW_EDGES) armElbowDeadlineLocked();
}
void publishLocked() {
  if ((!cutoff.elbowWave && !cutoff.shoulderTimer) || cutoff.shadow) return;
  while (cutoff.elbowWave && cutoff.published<cutoff.elbowEdge) {
    const auto i=cutoff.published++;
    motorControl.publishTimedWrite(3,cutoff.edgeAtMs[i],ELBOW_WAVE[i].pwm,bounded::OutputReason::DriverWrite);
  }
  if (cutoff.shoulderZero && !cutoff.shoulderPublished) {
    cutoff.shoulderPublished=true;
    motorControl.publishTimedWrite(4,cutoff.shoulderZeroMs,0,bounded::OutputReason::DriverWrite);
  }
  if (cutoff.forced && !cutoff.forcedPublished) {
    cutoff.forcedPublished=true;
    motorControl.publishTimedWrite(cutoff.motor,cutoff.forcedMs,0,bounded::OutputReason::ForcedOff);
  }
}
void stopShoulderIfDueLocked(int64_t now) {
  if (!cutoff.shoulderTimer || cutoff.shoulderZero || cutoff.tripped || now<cutoff.shoulderDueUs) return;
  if (!cutoff.firstWriteUs) { tripLocked();return; }
  ledcWrite(cutoff.channel,0);
  const auto written=esp_timer_get_time();
  cutoff.zeroUs=written;
  if (written>=cutoff.phase || written>=cutoff.sensor || written>=cutoff.lease) {
    tripLocked();return;
  }
  cutoff.shoulderZero=true;
  cutoff.shoulderZeroMs=millis();
}
#endif
void timerTick(void*) {
  xSemaphoreTake(cutoff.mutex,portMAX_DELAY);
  const auto now=esp_timer_get_time();
  expireLocked(now);
#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
  driveEdgeLocked(now);
  stopShoulderIfDueLocked(now);
#endif
  xSemaphoreGive(cutoff.mutex);
}
bool setupTimer() {
  if (cutoff.timer) return true;
  if (!cutoff.mutex) cutoff.mutex=xSemaphoreCreateMutex();
  if (!cutoff.mutex) return false;
  esp_timer_create_args_t cfg{};
  cfg.callback=timerTick;cfg.name="mb_policy_stop";
  if (esp_timer_create(&cfg,&cutoff.timer)!=ESP_OK) return false;
  if (esp_timer_start_periodic(cutoff.timer,CUTOFF_PERIOD_US)==ESP_OK) return true;
  esp_timer_delete(cutoff.timer);cutoff.timer=nullptr;
  return false;
}
int64_t sensorDeadline(int64_t now) {
  uint32_t age=millis()-shoulderAngleSensor.getI2cLastUpdateMs();
  return now+1000*(int32_t(150)-int32_t(age<150 ? age:150));
}
bool serviceCutoff(bool renew) {
  if (!cutoff.timer) return true;
  xSemaphoreTake(cutoff.mutex,portMAX_DELAY);
  int64_t t=esp_timer_get_time();
  expireLocked(t);
  bool ok=!cutoff.tripped;
  if (ok && cutoff.motor) {
    cutoff.sensor=sensorDeadline(t);
    if (renew) cutoff.lease=t+750000;
  }
  xSemaphoreGive(cutoff.mutex);
  return ok;
}
void disarmCutoff() {
  if (!cutoff.mutex) return;
  xSemaphoreTake(cutoff.mutex,portMAX_DELAY);
#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
  publishLocked();
  cutoff.shadow=false;
  cutoff.elbowWave=false;
  cutoff.shoulderTimer=cutoff.shoulderZero=false;
#endif
  cutoff.motor=0;cutoff.tripped=false;cutoff.awaitingFirstCorrectionWrite=false;
  xSemaphoreGive(cutoff.mutex);
}
class Hardware : public whole_arm::Port {
#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
  mutable size_t observedEdge_=0;
#endif
  uint32_t correctionStartedAt() const override { return cutoff.correctionOriginMs; }
  uint64_t epochs_[5]{};
  bool tracked_=false;
  void remember() {
    for (uint8_t i=1;i<=5;++i) epochs_[i-1]=motorControl.controlEpoch(i);
    tracked_=true;
  }
public:
  uint32_t now() const override { return millis(); }
  whole_arm::Inputs inputsFor(SystemState expected) const {
#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
    // A callback arriving during the checks cannot receive a second permit
    // from the snapshot taken for the preceding edge.
    if (cutoff.mutex) {
      xSemaphoreTake(cutoff.mutex,portMAX_DELAY);
      observedEdge_=cutoff.elbowEdge;
      xSemaphoreGive(cutoff.mutex);
    }
#endif
    bool same=true;
    for (uint8_t i=1;tracked_ && i<=5;++i)
      same=same && epochs_[i-1]==motorControl.controlEpoch(i);
    bool ready=!normalStopFailed && same && serviceCutoff(false) && motorControl.getDefaultSpeed()==100
      && systemState.getState()==expected && !safetyMonitor.isMotionBlocked()
      && !safetyMonitor.hasLatchedFault() && !webServer.hasManualControlLeases()
      && !motionSequence.getTotalCount() && !teleopAdapter.isDeadmanHeld()
      && !teleopAdapter.isControlActive() && !M4GoalService::owns() && !m4_goal::activeWindow()
      && !angleController.isActive() && !shoulderAngleController.isActive()
      && shoulderAngleSensor.isReadyForMotion(150);
    return {ready,!motorControl.hasPendingOutput()&&!motorControl.isEnabled(),shoulderAngleSensor.getI2cDegrees()};
  }
  whole_arm::Inputs inputs() const override { return inputsFor(SystemState::ARMED); }
  bool start(Phase p) override { return startPhase(p,false); }
  bool startPhase(Phase p,bool shadow) {
    if (!setupTimer()) return false;
#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
    if (p==Phase::ElbowDown350 && !shadow && !motorControl.prepareTimedElbow()) return false;
#else
    (void)shadow;
#endif
    auto motor=phaseMotor(p);
    auto pwm=phasePwm(p);
    if (motor==4 && !shoulderAngleController.manualDirectionAllowed(pwm>0)) return false;
    xSemaphoreTake(cutoff.mutex,portMAX_DELAY);
    auto t=esp_timer_get_time();
    cutoff.timedStep=whole_arm::stepDurationMs(p)!=0 || p==Phase::PoseClose;
    if (cutoff.timedStep) {
      cutoff.admissionUs=t;
      cutoff.firstWriteUs=0;
      cutoff.zeroUs=0;
      cutoff.stepZeroSequence=0;
    }
    cutoff.motor=motor;
    cutoff.channel=motorControl.pwmChannel(motor);
    cutoff.pwm=pwm;
    cutoff.tripped=false;
    cutoff.lease=t+750000;
    cutoff.phase=t+1000LL*phaseCapMs(p);
    // Leave a complete polling period before the bounded-step receipt limit.
    // RTOS scheduling latency beyond the polling period remains measurable jitter.
    if (whole_arm::isApproachStep(p) || p==Phase::PoseClose) cutoff.phase-=CUTOFF_PERIOD_US;
    cutoff.awaitingFirstCorrectionWrite=(p==Phase::LowerCorrection);
    if (cutoff.awaitingFirstCorrectionWrite) cutoff.phase=t+1000LL*whole_arm::CORRECTION_LATE_MS;
#if MOTIONBRAIN_ENABLE_ELBOW_WAVE
    cutoff.elbowWave=p==Phase::ElbowDown350;
    cutoff.elbowEdge=0;
#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
    resetTimedEdgesLocked(shadow,t);
    cutoff.shoulderTimer=whole_arm::isApproachStep(p) && motor==4;
    cutoff.shoulderZero=cutoff.shoulderPublished=false;
    cutoff.shoulderDueUs=t+1000LL*whole_arm::stepDurationMs(p);
    cutoff.shoulderLastPwm=0;
#endif
    if (cutoff.elbowWave) armElbowDeadlineLocked();
#endif
    cutoff.sensor=sensorDeadline(t);
    xSemaphoreGive(cutoff.mutex);
#if MOTIONBRAIN_ENABLE_ELBOW_WAVE
    if (p==Phase::ElbowDown350) {
      // Leave the ordinary ramp idle. The first explicit driver write also
      // rechecks exclusivity, system state and the M3 driver's error latch.
      if (!shadow) systemState.resetTimeout();
      remember();
      return true;
    }
#endif
    bool ok=p==Phase::LowerCorrection ? motorControl.writePolicyCorrection(whole_arm::correctionWave[0].pwm)
                                      : motorControl.beginPolicyOutput(motor,pwm);
    if (ok && p==Phase::LowerCorrection) systemState.resetTimeout();
    remember();
    return ok;
  }
#if MOTIONBRAIN_ENABLE_ELBOW_WAVE
  WaveTick serviceElbowWave() override {
    xSemaphoreTake(cutoff.mutex,portMAX_DELAY);
    const int64_t t=esp_timer_get_time();
    expireLocked(t);
#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
    publishLocked();
    const bool failed=!cutoff.elbowWave || cutoff.tripped;
    const bool done=cutoff.elbowEdge==ELBOW_EDGES;
    if (!failed && !done && observedEdge_==cutoff.elbowEdge) grantEdgeLocked(t);
    xSemaphoreGive(cutoff.mutex);
    return failed ? WaveTick::Fault : done ? WaveTick::ZeroDue : WaveTick::Running;
#else
    const bool valid=cutoff.elbowWave && !cutoff.tripped && cutoff.elbowEdge<ELBOW_EDGES;
    const bool due=valid && t>=elbowDueLocked();
    const int16_t pwm=due ? ELBOW_WAVE[cutoff.elbowEdge].pwm:0;
    xSemaphoreGive(cutoff.mutex);
    if (!valid) return WaveTick::Fault;
    if (!due) return WaveTick::Running;
    if (!pwm) return WaveTick::ZeroDue;
    if (!motorControl.writePolicyElbowStep(pwm)) return WaveTick::Fault;
    remember();
    return WaveTick::Running;
#endif
  }
#endif
  bool correctionStep(uint8_t i) override {
    if (!shoulderAngleController.manualDirectionAllowed(false)
        || !motorControl.writePolicyCorrection(whole_arm::correctionWave[i].pwm)) return false;
    // Never extend an expired window. The guard also checks the previous
    // window under this mutex at the point where PWM is written.
    xSemaphoreTake(cutoff.mutex,portMAX_DELAY);
    expireLocked(esp_timer_get_time());
    const bool ok=!cutoff.tripped;
    if (ok) cutoff.phase=cutoff.correctionOriginUs+1000LL*(
        whole_arm::correctionWave[i+1].atMs+whole_arm::CORRECTION_LATE_MS);
    xSemaphoreGive(cutoff.mutex);
    remember();
    return ok;
  }
#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
  bool shoulderZeroWritten() const override {
    xSemaphoreTake(cutoff.mutex,portMAX_DELAY);
    const bool done=cutoff.shoulderZero && !cutoff.tripped;
    xSemaphoreGive(cutoff.mutex);
    return done;
  }
#endif
  void stop(Phase p) override {
    // Do not certify a normal STOP if the independent timeout won this race.
    bool stopped=serviceCutoff(false);
#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
    if (p==Phase::ElbowDown350 || (whole_arm::isApproachStep(p) && phaseMotor(p)==4)) {
      syncTimedOutputs();
      motorControl.clearTimedPins(phaseMotor(p));
    } else
#endif
    if (stopped) stopped=motorControl.endPolicyOutput(phaseMotor(p));
    if (!stopped) {
      motorControl.hardStop(phaseMotor(p));
      normalStopFailed=true;
    }
    if (cutoff.timedStep && cutoff.mutex) {
      const auto& log=motorControl.outputLog(); // publish timer edges before locking
      const auto* zero=log.count() ? log.at(log.count()-1) : nullptr;
      xSemaphoreTake(cutoff.mutex,portMAX_DELAY);
      normalStopFailed |= !cutoff.firstWriteUs || !cutoff.zeroUs
        || cutoff.zeroUs <= cutoff.firstWriteUs
        || cutoff.zeroUs-cutoff.admissionUs >= 1000LL*phaseCapMs(p);
#if MOTIONBRAIN_ENABLE_ELBOW_WAVE
      if (cutoff.elbowWave) normalStopFailed |= cutoff.tripped || cutoff.elbowEdge!=ELBOW_EDGES;
#endif
      if (whole_arm::isApproachStep(p) || p==Phase::PoseClose) {
        if (whole_arm::isApproachStep(p))
          normalStopFailed |= cutoff.zeroUs-cutoff.admissionUs < 1000LL*(whole_arm::stepDurationMs(p)-1);
        normalStopFailed |= !zero || zero->motor!=phaseMotor(p) || zero->pwm!=0
            || zero->reason!=bounded::OutputReason::DriverWrite || !zero->sequence;
        if (!normalStopFailed) cutoff.stepZeroSequence=zero->sequence;
      }
      xSemaphoreGive(cutoff.mutex);
    }
    disarmCutoff();
    remember();
  }
  void faultStop(Phase p) override {
    motorControl.hardStop(phaseMotor(p));
    disarmCutoff();
    remember();
  }
  bool normalStopFailed=false;
} hardware;
whole_arm::Controller controller(hardware);
uint32_t epoch=1;
bool observed=false;
uint64_t idleEpochs[5]{};
void advanceEpoch() { if (epoch<UINT32_MAX) ++epoch; }
uint32_t outputCursor() {
  const auto& log=motorControl.outputLog();
  return log.count() ? log.at(log.count()-1)->sequence : 0;
}
#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
struct TimingProbe {
  enum class State { Idle, Running, Passed, Failed } state=State::Idle;
  char trial[33]{};
  uint32_t cursor=0,generation=0,offsets[ELBOW_EDGES]{};
  size_t edges=0;
} probe;
void updateProbe() {
  if (probe.state!=TimingProbe::State::Running) return;
  const auto input=hardware.inputsFor(SystemState::IDLE);
  const bool valid=input.ready && input.allOff && input.shoulderDeg>=187.f && input.shoulderDeg<=224.f &&
      outputCursor()==probe.cursor && generation()==probe.generation;
  auto result=valid ? hardware.serviceElbowWave() : whole_arm::Port::WaveTick::Fault;
  xSemaphoreTake(cutoff.mutex,portMAX_DELAY);
  probe.edges=cutoff.elbowEdge;
  for (size_t i=0;i<probe.edges;i++) probe.offsets[i]=uint32_t(cutoff.edgeAtUs[i]-cutoff.admissionUs);
  if (result==whole_arm::Port::WaveTick::Fault && !cutoff.tripped) tripLocked();
  if (cutoff.tripped) result=whole_arm::Port::WaveTick::Fault;
  xSemaphoreGive(cutoff.mutex);
  if (result==whole_arm::Port::WaveTick::Running) return;
  probe.state=result==whole_arm::Port::WaveTick::ZeroDue ? TimingProbe::State::Passed : TimingProbe::State::Failed;
  disarmCutoff();
}
#endif
#if MOTIONBRAIN_ENABLE_POSE_GRASP
constexpr const char* POSE_SCOPE="two-view-open-grasp-v1";
constexpr uint32_t POSE_OBSERVATION_MS=4000;
struct PoseGate {
  uint32_t seq=0,generation=0,cursor=0,issuedAt=0;
  char nonce[33]{},acceptedDigest[65]{};
  bool consumed=false;
} pose;
bool validDigest(const char* digest) {
  if (!digest || strlen(digest)!=64) return false;
  for (unsigned i=0;i<64;i++)
    if (!((digest[i]>='0' && digest[i]<='9') || (digest[i]>='a' && digest[i]<='f'))) return false;
  return true;
}
#endif
}

#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
void syncTimedOutputs() {
  if (!cutoff.mutex) return;
  xSemaphoreTake(cutoff.mutex,portMAX_DELAY);
  publishLocked();
  xSemaphoreGive(cutoff.mutex);
}
bool cancelTimedOutput(bool allowShoulderRamp) {
  if (!cutoff.mutex) return false;
  xSemaphoreTake(cutoff.mutex,portMAX_DELAY);
  const bool owned=cutoff.motor && (cutoff.elbowWave || (cutoff.shoulderTimer && !allowShoulderRamp));
  if (owned && !cutoff.tripped) tripLocked();
  publishLocked();
  xSemaphoreGive(cutoff.mutex);
  return owned;
}
bool startElbowTimingProbe(const whole_arm::Request& req,uint32_t expected,const char* boot,uint32_t cursor) {
  if (!boot || strcmp(boot,BoundedActions::outputBootId()) || !req.trial || strlen(req.trial)!=32 ||
      strspn(req.trial,"0123456789abcdef")!=32 || req.seq!=1 || req.phase!=Phase::ElbowDown350 ||
      req.op!=whole_arm::Op::Start || expected!=generation() || expected==UINT32_MAX || cursor!=outputCursor() ||
      controller.trial()[0] || probe.state!=TimingProbe::State::Idle) return false;
  const auto inputs=hardware.inputsFor(SystemState::IDLE);
  if (!inputs.ready || !inputs.allOff || !(inputs.shoulderDeg>=187.f && inputs.shoulderDeg<=224.f)) return false;
  strcpy(probe.trial,req.trial);probe.generation=expected;probe.cursor=cursor;
  probe.state=TimingProbe::State::Failed;
  if (!hardware.startPhase(Phase::ElbowDown350,true)) return false;
  probe.state=TimingProbe::State::Running;
  return true;
}
#endif
bool writePwm(uint8_t motor,uint8_t channel,int16_t pwm) {
  if (!cutoff.mutex) { ledcWrite(channel,abs(pwm)); return true; }
  xSemaphoreTake(cutoff.mutex,portMAX_DELAY);
  const int64_t now=esp_timer_get_time();
  expireLocked(now);
#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
  if (cutoff.motor && cutoff.elbowWave) {
    if (!cutoff.tripped) tripLocked();
    xSemaphoreGive(cutoff.mutex);
    return false;
  }
  if (cutoff.shoulderTimer) {
    if (cutoff.tripped || cutoff.shoulderZero || now>=cutoff.shoulderDueUs) {
      xSemaphoreGive(cutoff.mutex);
      return false;
    }
    if (motor!=4 || channel!=cutoff.channel || pwm!=cutoff.shoulderLastPwm-10 || pwm < -70) {
      tripLocked();
      xSemaphoreGive(cutoff.mutex);
      return false;
    }
  }
#endif
  bool ok=!pwm || !cutoff.motor || (cutoff.motor==motor && !cutoff.tripped &&
      (cutoff.pwm>0 ? (pwm>0 && pwm<=cutoff.pwm) : (pwm<0 && pwm>=cutoff.pwm)));
#if MOTIONBRAIN_ENABLE_ELBOW_WAVE
  const bool wave=cutoff.motor==motor && cutoff.elbowWave;
  if (wave && (cutoff.tripped || cutoff.elbowEdge>=ELBOW_EDGES ||
      pwm!=ELBOW_WAVE[cutoff.elbowEdge].pwm || now<elbowDueLocked())) {
    if (!cutoff.tripped) tripLocked();
    ok=false;
  }
#endif
  if (ok) {
    ledcWrite(channel,abs(pwm));
    if (cutoff.timedStep && cutoff.motor && cutoff.motor==motor) {
      const int64_t written=esp_timer_get_time();
      if (pwm && !cutoff.firstWriteUs) cutoff.firstWriteUs=written;
      if (!pwm && !cutoff.zeroUs) cutoff.zeroUs=written;
#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
      if (cutoff.shoulderTimer) {
        // Called on the main task. The callback may zero immediately on mutex
        // release, so publish this edge at HAL time; the ordinary identical
        // driver record is deduplicated by ordinal and signed PWM.
        cutoff.shoulderLastPwm=pwm;
        motorControl.publishTimedWrite(4,millis(),pwm,bounded::OutputReason::DriverWrite);
        if (written>=cutoff.shoulderDueUs || written>=cutoff.sensor || written>=cutoff.lease) tripLocked();
      }
#endif
#if MOTIONBRAIN_ENABLE_ELBOW_WAVE
      if (wave) {
        expireLocked(written);
        if (!cutoff.tripped && ++cutoff.elbowEdge<ELBOW_EDGES) armElbowDeadlineLocked();
      }
#endif
    }
    if (cutoff.awaitingFirstCorrectionWrite && motor==4 && pwm==-10) {
      // Anchor the entire waveform to the confirmed first HAL write. Setup
      // and mutex acquisition before the write are outside this interval.
      cutoff.awaitingFirstCorrectionWrite=false;
      cutoff.correctionOriginUs=esp_timer_get_time();
      cutoff.correctionOriginMs=millis();
      cutoff.phase=cutoff.correctionOriginUs+1000LL*(
          whole_arm::correctionWave[1].atMs+whole_arm::CORRECTION_LATE_MS);
    }
  }
  if (cutoff.motor==motor && cutoff.tripped) ok=false;
  xSemaphoreGive(cutoff.mutex);
  return ok;
}
bool owns(){
#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
  if (probe.state==TimingProbe::State::Running) return true;
#endif
  return controller.owns();
}
void update(){
#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
  updateProbe();
#endif
  controller.tick();
}
bool parsePhase(const char* raw,Phase& phase) {
  for (unsigned i=0;i<sizeof(NAMES)/sizeof(NAMES[0]);++i) if (!strcmp(raw,NAMES[i])) { phase=static_cast<Phase>(i); return true; }
  return false;
}
uint32_t generation() {
  // Capture direct (teleop/serial/driver) control changes before the first
  // model START. ARMED alone leaves the generation intact.
  if (!controller.trial()[0]) {
    bool changed=false;
    for (uint8_t motor=1;motor<=5;motor++) {
      const auto current=motorControl.controlEpoch(motor);
      changed |= observed && current!=idleEpochs[motor-1];
      idleEpochs[motor-1]=current;
    }
    if (changed) advanceEpoch();
    observed=true;
  }
  return epoch;
}
bool preparePoseClose(const whole_arm::Request& req,uint32_t expectedGeneration,const char* boot,
                      uint32_t cursor,const char* scope) {
#if MOTIONBRAIN_ENABLE_POSE_GRASP
  controller.tick();
  const auto input=hardware.inputs();
  if (req.op!=whole_arm::Op::Start || req.phase!=Phase::PoseClose || !scope || strcmp(scope,POSE_SCOPE) ||
      !boot || strcmp(boot,BoundedActions::outputBootId()) || generation()!=expectedGeneration || epoch==UINT32_MAX ||
      !controller.owns() || controller.active() ||
      (controller.phase()!=Phase::ShoulderDown300 && controller.phase()!=Phase::ShoulderDown420 &&
       controller.phase()!=Phase::ShoulderDown650) ||
      !req.trial || strcmp(req.trial,controller.trial()) || controller.seq()==UINT32_MAX || req.seq!=controller.seq()+1 ||
      !cutoff.stepZeroSequence || cursor!=cutoff.stepZeroSequence || cursor!=outputCursor() ||
      !input.ready || !input.allOff || !(input.shoulderDeg>194.f && input.shoulderDeg<=206.f) ||
      pose.seq==req.seq) return false;
  pose.seq=req.seq;pose.cursor=cursor;pose.generation=expectedGeneration;
  pose.issuedAt=millis();pose.consumed=false;
  static constexpr char hexDigits[]="0123456789abcdef";
  for (unsigned word=0;word<4;word++) {
    uint32_t random=esp_random();
    for (unsigned digit=0;digit<8;digit++,random>>=4)
      pose.nonce[word*8+digit]=hexDigits[random&15];
  }
  return true;
#else
  (void)req;(void)expectedGeneration;(void)boot;(void)cursor;(void)scope;
  return false;
#endif
}
bool apply(const whole_arm::Request& req,uint32_t expectedGeneration,const char* expectedBoot,
           const uint32_t* afterOutputSequence,const PoseObservation* observation) {
  if (!expectedBoot || strcmp(expectedBoot,BoundedActions::outputBootId()) ||
      expectedGeneration!=generation() || epoch==UINT32_MAX) return false;
#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
  if (probe.state==TimingProbe::State::Running || !req.trial || !strcmp(probe.trial,req.trial)) return false;
  if (req.phase==Phase::ElbowDown350 && (probe.state!=TimingProbe::State::Passed || expectedGeneration!=probe.generation))
    return false;
#endif
  const bool needsCursor=req.op==whole_arm::Op::Start &&
      (whole_arm::isApproachStep(req.phase) ||
       (whole_arm::isApproachStep(controller.phase()) && (req.phase==Phase::Close || req.phase==Phase::PoseClose)) ||
       (controller.phase()==Phase::PoseClose && req.phase==Phase::Lift));
  if (needsCursor) {
    const uint32_t latest=outputCursor();
    if (!afterOutputSequence || controller.active() || *afterOutputSequence!=latest ||
        ((whole_arm::isApproachStep(controller.phase()) || controller.phase()==Phase::PoseClose) &&
         (!cutoff.stepZeroSequence || cutoff.stepZeroSequence!=latest))) return false;
  } else if (afterOutputSequence) return false;
  if (req.op==whole_arm::Op::Start && req.phase==Phase::PoseClose) {
#if MOTIONBRAIN_ENABLE_POSE_GRASP
    if (!observation || !afterOutputSequence || !req.trial || strcmp(req.trial,controller.trial()) ||
        !pose.seq || pose.consumed || pose.seq!=req.seq || pose.generation!=expectedGeneration ||
        pose.cursor!=*afterOutputSequence || millis()-pose.issuedAt>=POSE_OBSERVATION_MS ||
        !observation->scope || strcmp(observation->scope,POSE_SCOPE) ||
        !observation->nonce || strcmp(observation->nonce,pose.nonce) || !validDigest(observation->pairSha256)) return false;
    pose.consumed=true;
#else
    return false;
#endif
  } else if (observation) return false;
  bool ok=controller.apply(req);
  if (!ok) return false;
#if MOTIONBRAIN_ENABLE_POSE_GRASP
  if (observation) strcpy(pose.acceptedDigest,observation->pairSha256);
#endif
  if (req.op==whole_arm::Op::Keep && !serviceCutoff(true)) {
    controller.abort();
    return false;
  }
  return !hardware.normalStopFailed;
}
bool beforeLegacy(CommandType c) {
  switch(c) {
    case CommandType::DISARM:case CommandType::STOP:case CommandType::MOTOR_STOP:
    case CommandType::MOTOR_STOP_ALL:case CommandType::JOINT_STOP:case CommandType::JOINT_STOP_ALL:
    case CommandType::SEQUENCE_STOP:case CommandType::SEQUENCE_CLEAR:case CommandType::ROUTINE_ABORT:
#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
      cancelTimedOutput();
#endif
      advanceEpoch();controller.abort();return true;
    default:
#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
      if (probe.state==TimingProbe::State::Running) return false;
#endif
      return !owns();
  }
}
void status(String& s,bool includePreparedNonce) {
  s="{\"bootId\":\"";s+=BoundedActions::outputBootId();s+="\",\"trialId\":\"";
  s+=controller.trial();s+="\",\"phase\":\"";s+=NAMES[static_cast<unsigned>(controller.phase())];
  s+="\",\"seq\":";s+=controller.seq();s+=",\"active\":";s+=controller.active()?"true":"false";
  s+=",\"failed\":";s+=(controller.failed()||hardware.normalStopFailed)?"true":"false";
  s+=",\"controlGeneration\":";s+=generation();
#if MOTIONBRAIN_ENABLE_ELBOW_WAVE
  s+=",\"elbowDownWaveProfile\":\"m3-50-300ms-ledger-v1\"";
#endif
#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
  static const char* const statuses[]={"idle","running","passed","failed"};
  s+=",\"elbowDownWaveScheduler\":\"esp-timer-edge-v1\",\"elbowTimingProbe\":{\"trialId\":\"";
  const TimingProbe& report=probe;
  s+=report.trial;s+="\",\"status\":\"";s+=statuses[static_cast<unsigned>(probe.state)];
  s+="\",\"afterOutputSequence\":";s+=probe.cursor;s+=",\"controlGeneration\":";s+=probe.generation;
  s+=",\"offsetsUs\":[";
  for (size_t i=0;i<probe.edges;i++) { if (i) s+=",";s+=probe.offsets[i]; }
  s+="]}";
  s+=",\"shoulderStepZeroProfile\":\"timer-normal-zero-v1\"";
#endif
#if MOTIONBRAIN_ENABLE_ELBOW_DOWN_ONCE
  s+=",\"elbowDownOnceProfile\":\"m3-down-50-350ms-v1\"";
  if (controller.phase()==Phase::ElbowDownOnce && cutoff.mutex) {
    xSemaphoreTake(cutoff.mutex,portMAX_DELAY);
    const auto start=cutoff.admissionUs;
    const auto first=cutoff.firstWriteUs;
    const auto end=cutoff.zeroUs;
    xSemaphoreGive(cutoff.mutex);
    if (start && first) { s+=",\"elbowFirstWriteOffsetUs\":";s+=static_cast<uint32_t>(first-start); }
    if (start && end) { s+=",\"elbowZeroWriteOffsetUs\":";s+=static_cast<uint32_t>(end-start); }
  }
#endif
#if MOTIONBRAIN_ENABLE_BOUNDED_APPROACH
  s+=",\"boundedApproachProfile\":\"off-observe-m4-m3-v1\"";
  if ((whole_arm::isApproachStep(controller.phase()) || controller.phase()==Phase::PoseClose) && cutoff.mutex) {
    xSemaphoreTake(cutoff.mutex,portMAX_DELAY);
    const auto start=cutoff.admissionUs, first=cutoff.firstWriteUs, end=cutoff.zeroUs;
    xSemaphoreGive(cutoff.mutex);
    if (start && first) { s+=",\"stepFirstWriteOffsetUs\":";s+=static_cast<uint32_t>(first-start); }
    if (start && end) { s+=",\"stepZeroWriteOffsetUs\":";s+=static_cast<uint32_t>(end-start); }
    if (cutoff.stepZeroSequence) { s+=",\"stepZeroSequence\":";s+=cutoff.stepZeroSequence; }
  }
#endif
#if MOTIONBRAIN_ENABLE_POSE_GRASP
  s+=",\"poseGraspProfile\":\"wrist50-pose-close-v1\"";
  s+=",\"poseElbowBudgetUsedMs\":";s+=controller.elbowBudgetMs();
  s+=",\"poseWristBudgetUsedMs\":";s+=controller.wristBudgetMs();
  s+=",\"poseShoulderBudgetUsedMs\":";s+=controller.shoulderBudgetMs();
  const PoseGate& gate=pose;
  if (includePreparedNonce && gate.seq && !gate.consumed) {
    s+=",\"poseObservationNonce\":\"";s+=gate.nonce;s+="\",\"poseScope\":\"";
    s+=POSE_SCOPE;s+="\",\"poseObservationTtlMs\":";s+=POSE_OBSERVATION_MS;
  }
  if (gate.acceptedDigest[0]) {
    s+=",\"posePairSha256\":\"";s+=gate.acceptedDigest;s+="\",\"poseScope\":\"";s+=POSE_SCOPE;s+="\"";
  }
#else
  (void)includePreparedNonce;
#endif
  s+=",\"reason\":\"";s+=hardware.normalStopFailed?"stop_fault":controller.reason();s+="\",\"rawPwm\":70,\"correctionProfile\":\"demo6_m4_lower_554ms_v1\"}";
}
}
#endif
