// Production ShoulderAngleController + RobotArm + MotorControl, virtual time/IO.
// Plants are explicit assumptions, not estimates of hardware friction/inertia.
#include "control/shoulder_angle_controller.h"
#include "motion/robot_arm.h"
#include "control/event_log.h"
#include "peripheral/shoulder_angle_sensor.h"
#include "safety/safety_monitor.h"
#include "system/system_init.h"
#include <assert.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

SystemStateManager systemState;
SafetyMonitor safetyMonitor;
EventLog eventLog;
static uint32_t nowMs=100;
static uint8_t pins[40]={};
struct Write {uint32_t at; int pwm;};
static std::vector<Write> writes;
uint32_t millis(){return nowMs;}
void pinMode(uint8_t,uint8_t){}
void digitalWrite(uint8_t p,uint8_t v){assert(p<40);pins[p]=v;}
void ledcSetup(uint8_t,uint32_t,uint8_t){}
void ledcAttachPin(uint8_t,uint8_t){}
void ledcWrite(uint8_t channel,uint32_t duty){
  assert(duty<=255);
  if(channel==3) writes.push_back({nowMs,duty ? (pins[27] ? int(duty) : -int(duty)) : 0});
}
struct Rig {
  MotorControl motor;
  RobotArm arm{&motor};
  ShoulderAngleSensor sensor;
  ShoulderAngleController controller;
  float velocity=0,coastTau=.04f, rate=.4f;
  int breakaway=45;
  bool plant=true, slow=false, sample=true;
  Rig(){
    systemState.state=SystemState::IDLE;safetyMonitor.blocked=false;
    assert(motor.init());systemState.state=SystemState::ARMED;
    sensor.sampleAt=nowMs;
    controller.init(&systemState,&arm,&motor,&safetyMonitor,&sensor);
    writes.clear();eventLog.entries.clear();
  }
  void start(float target=192.42f){char message[120]; assert(controller.startAbsolute(target,75,message,sizeof(message)));}
  void tick(){
    nowMs+=10;
    const int pwm=motor.getSpeed(4);
    if(plant){
      if(slow) velocity=pwm ? -.3f : 0;
      else if(std::abs(pwm)>=breakaway)
        velocity=(pwm>0 ? 1.f:-1.f)*(std::abs(pwm)-breakaway)*rate;
      else velocity*=std::exp(-.01f/coastTau);
      sensor.angle+=velocity*.01f;
    }
    if(sample && nowMs%20==0) sensor.sampleAt=nowMs;
    controller.update();motor.update();
  }
  void finish(){uint32_t began=nowMs;while(controller.isActive() && nowMs-began<11000)tick();assert(!controller.isActive());off();}
  void off(){assert(motor.getSpeed(4)==0 && !motor.hasPendingOutput());for(int i=0;i<100;++i)tick();assert(motor.getSpeed(4)==0);}
  int corrections() const{int n=0;for(const auto& e:eventLog.entries)if(e.code=="SHOULDER_ANGLE_CORRECTION")++n;return n;}
};
static void recover(){
  Rig r;r.start();r.finish();
#if MOTIONBRAIN_ENABLE_BOUNDED_SHOULDER_DOWN_CORRECTION
  assert(r.controller.getLastStopReason()==ShoulderAngleStopReason::TARGET_REACHED);
  assert(std::fabs(r.controller.getFinalErrorDegrees())<=.4f);
  assert(r.corrections()>0 && r.corrections()<=6);
#else
  assert(r.controller.getLastStopReason()==ShoulderAngleStopReason::TARGET_MISSED);
  assert(r.corrections()==6);
  assert(std::fabs(r.controller.getFinalErrorDegrees())>.5f);
#endif
  // Initial coast stop is still target+lead+window (sample quantization aside).
  assert(ShoulderAngleController::DOWN_STOP_LEAD_DEGREES==1.5f);
  assert(ShoulderAngleController::STOP_THRESHOLD_WINDOW_DEGREES==.35f);
}
static void noMotion(){
  Rig r;r.plant=false;r.start();r.finish();
  assert(r.controller.getLastStopReason()==ShoulderAngleStopReason::NO_PROGRESS);
  assert(r.corrections()==0);
  // Once initial lead-stop is past, repeated corrections still cannot run forever.
  Rig c;c.start();while(c.corrections()==0)c.tick();c.plant=false;c.velocity=0;c.finish();
  assert(c.controller.getLastStopReason()==ShoulderAngleStopReason::TARGET_MISSED);
  assert(c.corrections()==6);
}
static void timeout(){
  Rig r;r.slow=true;r.sensor.angle=250;r.start(192.42f);uint32_t start=nowMs;r.finish();
  assert(r.controller.getLastStopReason()==ShoulderAngleStopReason::TIMEOUT);
  // finish() includes one second proving pending ramps cannot resume.
  assert(nowMs-start==11000);
}
static void faults(){
  for(int kind=0;kind<5;++kind){
    Rig r;r.start();for(int i=0;i<35;++i)r.tick();assert(r.motor.getSpeed(4)!=0);
    if(kind==0) r.sample=false;
    if(kind==1) r.sensor.angle=121;
    if(kind==2) safetyMonitor.blocked=true;
    if(kind==3) systemState.state=SystemState::IDLE;
    if(kind==4) r.controller.cancel(ShoulderAngleStopReason::MANUAL_STOP);
    r.finish();
    const ShoulderAngleStopReason reasons[]={ShoulderAngleStopReason::SENSOR_FAULT,ShoulderAngleStopReason::SOFT_LIMIT,ShoulderAngleStopReason::SAFETY_BLOCK,ShoulderAngleStopReason::STATE_CHANGED,ShoulderAngleStopReason::MANUAL_STOP};
    assert(r.controller.getLastStopReason()==reasons[kind]);
  }
}
static void reverse(){
  Rig r;r.coastTau=.3f;r.start();r.finish();
  bool down=false,up=false;for(const auto& w:writes){down|=w.pwm<0;up|=w.pwm>0;}
  assert(down && up); // High inertia may cross the goal; correction direction follows signed error.
  assert(r.corrections()<=6);
  assert(r.controller.getLastStopReason()==ShoulderAngleStopReason::TARGET_REACHED || r.controller.getLastStopReason()==ShoulderAngleStopReason::TARGET_MISSED || r.controller.getLastStopReason()==ShoulderAngleStopReason::TIMEOUT);
}
static void timerAndPwm(){
  for(int speed : {100,255}){
    Rig r;assert(r.motor.setDefaultSpeed(speed));r.start();
    while(r.corrections()==0)r.tick();
    // Freeze a nonzero error to exercise full pulse rather than target cutoff.
    r.plant=false;r.velocity=0;
    uint32_t began=nowMs;size_t beginWrite=writes.size();
    while(r.motor.getSpeed(4)==0) r.tick();
    while(r.motor.getSpeed(4)!=0) {r.tick();assert(nowMs-began<=1000);}
    const uint32_t elapsed=nowMs-began;
    assert(elapsed>=ShoulderAngleController::DOWN_CORRECTION_PULSE_MS && elapsed<=ShoulderAngleController::DOWN_CORRECTION_PULSE_MS+20);
    const int requested=speed*ShoulderAngleController::DOWN_CORRECTION_PERCENT/100;
    int peak=0;
    for(size_t i=beginWrite;i<writes.size();++i){peak=std::max(peak,std::abs(writes[i].pwm));assert(std::abs(writes[i].pwm)<=191);}
    assert(peak<=requested);
    // At default 255 the request is 191, but the ramp cannot reach it in 500ms.
    if(speed==255) assert(peak<requested);
    assert(r.controller.cancel(ShoulderAngleStopReason::MANUAL_STOP));r.off();
    String json;r.controller.appendShoulderStatusJson(json);
    assert(json.find("\"motorDefaultSpeed\":"+std::to_string(speed))!=std::string::npos);
    assert(json.find("\"downCorrectionTargetPwm\":"+std::to_string(requested))!=std::string::npos);
  }
}
static void crossingCutoff(){
  Rig r;r.start();while(r.corrections()==0)r.tick();
  r.plant=false;r.velocity=0;
  while(r.motor.getSpeed(4)==0)r.tick();
  const uint32_t began=nowMs;
  // A fresh sensor crossing triggers immediate hard OFF, before pulse expiry.
  r.sensor.angle=192.32f;
  while(nowMs%20!=0)r.tick();
  r.tick();r.tick();
  assert(r.motor.getSpeed(4)==0);
  assert(nowMs-began<ShoulderAngleController::DOWN_CORRECTION_PULSE_MS);
  r.finish();
  assert(r.controller.getLastStopReason()==ShoulderAngleStopReason::TARGET_REACHED);
}
static void reverseStuck(){
  Rig r;r.coastTau=.3f;r.start();
  bool reversed=false;
  for(int i=0;i<1000 && r.controller.isActive();++i){
    r.tick();
    if(r.motor.getSpeed(4)>0){reversed=true;r.plant=false;r.velocity=0;break;}
  }
  assert(reversed);r.finish();
  assert(r.controller.getLastStopReason()==ShoulderAngleStopReason::TARGET_MISSED);
  assert(r.corrections()==6);
}
int main(int argc,char** argv){
  assert(argc==2);
  if(!std::strcmp(argv[1],"recover"))recover();
  else if(!std::strcmp(argv[1],"no_motion"))noMotion();
  else if(!std::strcmp(argv[1],"timeout"))timeout();
  else if(!std::strcmp(argv[1],"faults"))faults();
  else if(!std::strcmp(argv[1],"reverse"))reverse();
  else if(!std::strcmp(argv[1],"timer_pwm"))timerAndPwm();
  else if(!std::strcmp(argv[1],"crossing"))crossingCutoff();
  else if(!std::strcmp(argv[1],"reverse_stuck"))reverseStuck();
  else assert(false);
  std::puts("PASS: virtual plant assumptions only; no physical motion validated");
}
