#include "whole_arm_service.h"
#if MOTIONBRAIN_ENABLE_WHOLE_ARM_POLICY
#include <Arduino.h>
#include <esp_timer.h>
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
namespace {
constexpr const char* NAMES[]={"approach","close","held_lift","lower","release","retreat","lower_correction"
#if MOTIONBRAIN_ENABLE_ELBOW_DOWN_ONCE
  ,"elbow_down_once"
#endif
#if MOTIONBRAIN_ENABLE_BOUNDED_APPROACH
  ,"shoulder_down_300","shoulder_down_420","elbow_down_350"
#endif
};
// Arduino HTTP may block the loop while waiting for a partial request. A
// separate ESP timer task cuts PWM even then. The callback touches neither
// the HTTP state, motor bookkeeping nor its non-thread-safe output journal.
// The main loop reconciles the latched fault with hardStop/forced_off.
constexpr int64_t CUTOFF_PERIOD_US=5000;
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
} cutoff;

void expireLocked(int64_t t) {
  if (cutoff.motor && !cutoff.tripped &&
      (t>=cutoff.lease || t>=cutoff.phase || t>=cutoff.sensor)) {
    ledcWrite(cutoff.channel,0);
    if (cutoff.timedStep && !cutoff.zeroUs) cutoff.zeroUs=esp_timer_get_time();
    cutoff.tripped=true;
  }
}
void timerTick(void*) {
  xSemaphoreTake(cutoff.mutex,portMAX_DELAY);
  expireLocked(esp_timer_get_time());
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
  cutoff.motor=0;cutoff.tripped=false;cutoff.awaitingFirstCorrectionWrite=false;
  xSemaphoreGive(cutoff.mutex);
}
class Hardware : public whole_arm::Port {
  uint32_t correctionStartedAt() const override { return cutoff.correctionOriginMs; }
  uint64_t epochs_[5]{};
  bool tracked_=false;
  void remember() {
    for (uint8_t i=1;i<=5;++i) epochs_[i-1]=motorControl.controlEpoch(i);
    tracked_=true;
  }
public:
  uint32_t now() const override { return millis(); }
  whole_arm::Inputs inputs() const override {
    bool same=true;
    for (uint8_t i=1;tracked_ && i<=5;++i)
      same=same && epochs_[i-1]==motorControl.controlEpoch(i);
    bool ready=!normalStopFailed && same && serviceCutoff(false) && motorControl.getDefaultSpeed()==100
      && systemState.getState()==SystemState::ARMED && !safetyMonitor.isMotionBlocked()
      && !safetyMonitor.hasLatchedFault() && !webServer.hasManualControlLeases()
      && !motionSequence.getTotalCount() && !teleopAdapter.isDeadmanHeld()
      && !teleopAdapter.isControlActive() && !M4GoalService::owns() && !m4_goal::activeWindow()
      && !angleController.isActive() && !shoulderAngleController.isActive()
      && shoulderAngleSensor.isReadyForMotion(150);
    return {ready,!motorControl.hasPendingOutput()&&!motorControl.isEnabled(),shoulderAngleSensor.getI2cDegrees()};
  }
  bool start(Phase p) override {
    if (!setupTimer()) return false;
    auto motor=phaseMotor(p);
    auto pwm=phasePwm(p);
    if (motor==4 && !shoulderAngleController.manualDirectionAllowed(pwm>0)) return false;
    xSemaphoreTake(cutoff.mutex,portMAX_DELAY);
    auto t=esp_timer_get_time();
    cutoff.timedStep=whole_arm::stepDurationMs(p)!=0;
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
    if (whole_arm::isApproachStep(p)) cutoff.phase-=CUTOFF_PERIOD_US;
    cutoff.awaitingFirstCorrectionWrite=(p==Phase::LowerCorrection);
    if (cutoff.awaitingFirstCorrectionWrite) cutoff.phase=t+1000LL*whole_arm::CORRECTION_LATE_MS;
    cutoff.sensor=sensorDeadline(t);
    xSemaphoreGive(cutoff.mutex);
    bool ok=p==Phase::LowerCorrection ? motorControl.writePolicyCorrection(whole_arm::correctionWave[0].pwm)
                                      : motorControl.beginPolicyOutput(motor,pwm);
    if (ok && p==Phase::LowerCorrection) systemState.resetTimeout();
    remember();
    return ok;
  }
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
  void stop(Phase p) override {
    // Do not certify a normal STOP if the independent timeout won this race.
    if (!serviceCutoff(false) || !motorControl.endPolicyOutput(phaseMotor(p))) {
      motorControl.hardStop(phaseMotor(p));
      normalStopFailed=true;
    }
    if (whole_arm::stepDurationMs(p) && cutoff.mutex) {
      xSemaphoreTake(cutoff.mutex,portMAX_DELAY);
      normalStopFailed |= !cutoff.firstWriteUs || !cutoff.zeroUs
        || cutoff.zeroUs <= cutoff.firstWriteUs
        || cutoff.zeroUs-cutoff.admissionUs >= 1000LL*phaseCapMs(p);
      if (whole_arm::isApproachStep(p)) {
        normalStopFailed |= cutoff.zeroUs-cutoff.admissionUs < 1000LL*(whole_arm::stepDurationMs(p)-1);
        const auto& log=motorControl.outputLog();
        const auto* zero=log.count() ? log.at(log.count()-1) : nullptr;
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
}

bool writePwm(uint8_t motor,uint8_t channel,int16_t pwm) {
  if (!cutoff.mutex) { ledcWrite(channel,abs(pwm)); return true; }
  xSemaphoreTake(cutoff.mutex,portMAX_DELAY);
  expireLocked(esp_timer_get_time());
  bool ok=!pwm || !cutoff.motor || (cutoff.motor==motor && !cutoff.tripped &&
      (cutoff.pwm>0 ? (pwm>0 && pwm<=cutoff.pwm) : (pwm<0 && pwm>=cutoff.pwm)));
  if (ok) {
    ledcWrite(channel,abs(pwm));
    if (cutoff.timedStep && cutoff.motor && cutoff.motor==motor) {
      if (pwm && !cutoff.firstWriteUs) cutoff.firstWriteUs=esp_timer_get_time();
      if (!pwm && !cutoff.zeroUs) cutoff.zeroUs=esp_timer_get_time();
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
bool owns(){return controller.owns();}
void update(){controller.tick();}
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
bool apply(const whole_arm::Request& req,uint32_t expectedGeneration,const char* expectedBoot,
           const uint32_t* afterOutputSequence) {
  if (!expectedBoot || strcmp(expectedBoot,BoundedActions::outputBootId()) ||
      expectedGeneration!=generation() || epoch==UINT32_MAX) return false;
  const bool needsCursor=req.op==whole_arm::Op::Start &&
      (whole_arm::isApproachStep(req.phase) ||
       (whole_arm::isApproachStep(controller.phase()) && req.phase==Phase::Close));
  if (needsCursor) {
    const auto& log=motorControl.outputLog();
    const uint32_t latest=log.count() ? log.at(log.count()-1)->sequence : 0;
    if (!afterOutputSequence || controller.active() || *afterOutputSequence!=latest ||
        (whole_arm::isApproachStep(controller.phase()) &&
         (!cutoff.stepZeroSequence || cutoff.stepZeroSequence!=latest))) return false;
  } else if (afterOutputSequence) return false;
  bool ok=controller.apply(req);
  if (!ok) return false;
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
      advanceEpoch();controller.abort();return true;
    default:return !owns();
  }
}
void status(String& s) {
  s="{\"bootId\":\"";s+=BoundedActions::outputBootId();s+="\",\"trialId\":\"";
  s+=controller.trial();s+="\",\"phase\":\"";s+=NAMES[static_cast<unsigned>(controller.phase())];
  s+="\",\"seq\":";s+=controller.seq();s+=",\"active\":";s+=controller.active()?"true":"false";
  s+=",\"failed\":";s+=(controller.failed()||hardware.normalStopFailed)?"true":"false";
  s+=",\"controlGeneration\":";s+=generation();
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
  if (whole_arm::isApproachStep(controller.phase()) && cutoff.mutex) {
    xSemaphoreTake(cutoff.mutex,portMAX_DELAY);
    const auto start=cutoff.admissionUs, first=cutoff.firstWriteUs, end=cutoff.zeroUs;
    xSemaphoreGive(cutoff.mutex);
    if (start && first) { s+=",\"stepFirstWriteOffsetUs\":";s+=static_cast<uint32_t>(first-start); }
    if (start && end) { s+=",\"stepZeroWriteOffsetUs\":";s+=static_cast<uint32_t>(end-start); }
    if (cutoff.stepZeroSequence) { s+=",\"stepZeroSequence\":";s+=cutoff.stepZeroSequence; }
  }
#endif
  s+=",\"reason\":\"";s+=hardware.normalStopFailed?"stop_fault":controller.reason();s+="\",\"rawPwm\":70,\"correctionProfile\":\"demo6_m4_lower_554ms_v1\"}";
}
}
#endif
