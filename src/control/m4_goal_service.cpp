#include "m4_goal_service.h"
#include "control/bounded_action_service.h"
#include "control/finite_profile_runtime.h"
#include "control/command.h"
#include "control/angle_controller.h"
#include "control/shoulder_angle_controller.h"
#include "motor/motor_driver.h"
#include "motion/motion_sequence.h"
#include "peripheral/shoulder_angle_sensor.h"
#include "safety/safety_monitor.h"
#include <WiFi.h>
#include <string.h>
extern MotorControl motorControl;
extern MotionSequence motionSequence;
extern ShoulderAngleSensor shoulderAngleSensor;
extern SystemStateManager systemState;
extern SafetyMonitor safetyMonitor;
extern AngleController angleController;
extern ShoulderAngleController shoulderAngleController;
namespace M4GoalService {
namespace {
m4_goal::Guard guard;
bool requested=false;
bool station=false;
bool quarantine=false;
bool allOff(){return !motorControl.hasPendingOutput()&&!motorControl.isEnabled();}
bool networkOK(){return !station || WiFi.status()==WL_CONNECTED;}
m4_goal::Snapshot snapshot(){
 bool other=false;
 for(uint8_t i=1;i<=5;++i) if(i!=4&&motorControl.getSpeed(i)!=0)other=true;
 return {millis(),shoulderAngleSensor.getI2cLastUpdateMs(),shoulderAngleSensor.getI2cDegrees(),
 shoulderAngleSensor.isReadyForMotion(m4_goal::SENSOR_AGE_MS),systemState.getState()==SystemState::ARMED,
 !safetyMonitor.isMotionBlocked()&&!safetyMonitor.hasLatchedFault()&&networkOK(),allOff(),
 !other&&(!guard.active()||motorControl.m4GoalOwned()),motorControl.getSpeed(4)!=0};
}
void finish(){
 requested=false;
 motorControl.endM4Goal();
 guard.confirmOff(millis(),allOff());
 quarantine=true;
 // Retain the driver barrier until a new explicit web ARM, after input drain.
 m4_goal::setActiveWindow(true);
}
}
bool owns(){return guard.owns();}
bool pending(){return requested||guard.active();}
bool queue(const m4_goal::Request& r){
#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER
 if(motorControl.finiteProfileOwned())return false;
#endif
 if(!MOTIONBRAIN_ENABLE_M4_GOAL_GUARD||!allOff()||motionSequence.getTotalCount()!=0||angleController.isActive()||shoulderAngleController.isActive()||BoundedActions::active()||finite_profile::runtime::motionPending())return false;
 if(!guard.queue(r,BoundedActions::outputBootId(),millis()))return false;
 m4_goal::setActiveWindow(true);return true;
}
bool run(const char* nonce,const char* boot,const char* scope,const char* policy){
 const auto* r=guard.current();
 if(requested||!r||strcmp(r->state,"queued")||strcmp(r->request.nonce,nonce)||strcmp(r->request.bootId,boot)||strcmp(r->request.scopeSha256,scope)||strcmp(r->request.policySha256,policy))return false;
 requested=true;station=(WiFi.getMode()&WIFI_MODE_STA)!=0;return true;
}
void cancel(const char* reason){if(guard.cancel(reason,millis()))finish();}
void update(){
 if(requested){
   requested=false;
   const auto& r=guard.current()->request;
   if(!guard.begin(r.nonce,r.bootId,r.scopeSha256,r.policySha256,snapshot())){finish();return;}
   m4_goal::setActiveWindow(true);
   if(!motorControl.beginM4Goal(r.up?127:-89)){cancel("driver_rejected");return;}
 }
 if(guard.tick(snapshot()))finish();
}
bool serialHeld(){return owns()||quarantine;}
void pollStop(){
 // During motion only this bounded byte parser owns Serial; '!' means hard OFF.
 // Other input is discarded and cannot become a delayed legacy motion command.
 for(unsigned i=0;i<64&&Serial.available()>0;++i)if(Serial.read()=='!')cancel("serial_stop");
 if(!owns()&&Serial.available()==0)quarantine=false;
}
bool beforeLegacy(CommandType type,bool explicitWeb){
 if(!owns()&&!m4_goal::activeWindow())return true;
 if(type==CommandType::MOTOR_SET_DEFAULT_SPEED && !owns() && allOff())return true;
 if(type==CommandType::ARM && !pending() && explicitWeb && !quarantine){
   if(!owns())m4_goal::setActiveWindow(false);
   return true;
 }
 cancel("legacy_command");
 switch(type){
 case CommandType::STOP:case CommandType::DISARM:case CommandType::MOTOR_STOP:
 case CommandType::MOTOR_STOP_ALL:case CommandType::JOINT_STOP:case CommandType::JOINT_STOP_ALL:
 case CommandType::SEQUENCE_STOP:case CommandType::SEQUENCE_CLEAR:case CommandType::ROUTINE_ABORT:return true;
 default:return false;
 }
}
void appendJson(String& json){
 json+="\"m4GoalGuard\":{\"schemaVersion\":\"motionbrain.m4_goal_guard.v1\",\"enabled\":";
 json+=MOTIONBRAIN_ENABLE_M4_GOAL_GUARD?"true":"false";
 json+=",\"policySha256\":\"";json+=m4_goal::POLICY_SHA;
 json+="\",\"bootId\":\"";json+=BoundedActions::outputBootId();
 json+="\",\"execution\":\"main_loop_http_deferred\",\"stopChannel\":\"serial_bang_or_teleop_safety\",\"nonceCapacity\":32,\"nonceCount\":";json+=guard.count();
 const auto* r=guard.current();
 if(r){json+=",\"record\":{";
 #define S(k,v) json+="\"" k "\":\"";json+=v;json+="\",";
 #define N(k,v) json+="\"" k "\":";json+=String(v);json+=",";
 #define GF(k,v) json+="\"" k "\":";json+=String(v,5);json+=",";
 S("nonce",r->request.nonce) S("bootId",r->request.bootId) S("scopeSha256",r->request.scopeSha256) S("policySha256",r->request.policySha256)
 S("state",r->state) S("reason",r->reason)
 GF("expectedStartDeg",r->request.expectedStartDeg) GF("goalDeg",r->request.goalDeg) GF("startAngleDeg",r->startAngleDeg) GF("lastAngleDeg",r->lastAngleDeg)
 N("percent",r->request.percent) N("durationMs",r->request.durationMs)
 N("queuedAtMs",r->queuedAtMs) N("runAtMs",r->runAtMs) N("decisionAtMs",r->decisionAtMs) N("offAtMs",r->offAtMs)
 N("sampleAtMs",r->sampleAtMs) N("sampleAgeMs",r->sampleAgeMs) N("maxTickGapMs",r->maxTickGapMs) N("firstTickDelayMs",r->firstTickDelayMs)
 json+="\"up\":";json+=r->request.up?"true":"false";
 json+=",\"offVerified\":";json+=r->offVerified?"true":"false";
 json+=",\"outputMayHaveOccurred\":";json+=r->outputMayHaveOccurred?"true":"false";json+="}";
 #undef S
 #undef N
 #undef GF
 }json+="}";
}
}
