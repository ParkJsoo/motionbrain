#include "control/m4_goal_service.h"
#include "control/command.h"
#include "control/angle_controller.h"
#include "control/shoulder_angle_controller.h"
#include "motor/motor_driver.h"
#include "motion/motion_sequence.h"
#include "peripheral/shoulder_angle_sensor.h"
#include "safety/safety_monitor.h"
#include <WiFi.h>
#include <cassert>
#include <cstring>
SystemStateManager systemState;
MotorControl motorControl;
MotionSequence motionSequence;
ShoulderAngleSensor shoulderAngleSensor;
SafetyMonitor safetyMonitor;
AngleController angleController;
ShoulderAngleController shoulderAngleController;
SerialFake Serial;WiFiFake WiFi;
uint32_t now=100;uint32_t millis(){return now;}
void pinMode(uint8_t,uint8_t){}void digitalWrite(uint8_t,uint8_t){}
void ledcSetup(uint8_t,uint32_t,uint8_t){}void ledcAttachPin(uint8_t,uint8_t){}
void ledcWrite(uint8_t,uint32_t){}
void step(uint32_t dt=20){now+=dt;shoulderAngleSensor.sample=now;M4GoalService::update();motorControl.update();}
int main(int argc,char** argv){
 assert(argc==2);int test=atoi(argv[1]);assert(motorControl.init());
 m4_goal::Request r;memset(r.nonce,'b',32);memset(r.bootId,'a',32);memset(r.scopeSha256,'c',64);strcpy(r.policySha256,m4_goal::POLICY_SHA);r.up=false;r.percent=35;r.durationMs=1200;r.expectedStartDeg=197.75f;r.goalDeg=197.22f;
 assert(M4GoalService::queue(r));assert(!motorControl.setSpeed(1,20));
 assert(M4GoalService::beforeLegacy(CommandType::ARM,true));systemState.state=SystemState::ARMED;
 assert(M4GoalService::run(r.nonce,r.bootId,r.scopeSha256,r.policySha256));
 assert(!motorControl.hasPendingOutput()); // HTTP RUN acceptance never starts output
 if(test==8){Serial.input="!";M4GoalService::pollStop();assert(!M4GoalService::pending());return 0;}
 step();assert(motorControl.m4GoalOwned());assert(!motorControl.setSpeed(1,20));
 for(int i=0;i<6;++i)step();assert(motorControl.getSpeed(4)<0);
 if(test==1)shoulderAngleSensor.angle=197.20f;
 if(test==2)shoulderAngleSensor.ready=false;
 if(test==3)safetyMonitor.blocked=true;
 if(test==4)systemState.state=SystemState::IDLE;
 if(test==5){Serial.input="!motor run 4 100\n";M4GoalService::pollStop();}
 if(test==6)WiFi.connection=0;
 if(test==7)motorControl.hardStop(1);
 if(test==9){now+=50;}
 if(test==10){while(M4GoalService::pending())step();}
 step();assert(!M4GoalService::pending());assert(!motorControl.hasPendingOutput());
 for(int i=1;i<=5;++i)assert(motorControl.getSpeed(i)==0);
 String j;M4GoalService::appendJson(j);assert(j.value.find("\"state\":\"terminal\"")!=std::string::npos);
 assert(j.value.find("\"offVerified\":true")!=std::string::npos);
 assert(!M4GoalService::queue(r));assert(!M4GoalService::run(r.nonce,r.bootId,r.scopeSha256,r.policySha256));
 step();assert(!motorControl.hasPendingOutput());
 assert(M4GoalService::beforeLegacy(CommandType::STOP,true));
 assert(M4GoalService::beforeLegacy(CommandType::SEQUENCE_CLEAR,true));
 assert(M4GoalService::beforeLegacy(CommandType::MOTOR_SET_DEFAULT_SPEED,true));
 assert(motorControl.setDefaultSpeed(100));
 assert(motorControl.setDefaultSpeed(255));
 assert(!motorControl.setSpeed(4,20));
 assert(!M4GoalService::beforeLegacy(CommandType::MOTOR_RUN));
 assert(!M4GoalService::beforeLegacy(CommandType::ARM));
 M4GoalService::pollStop();
 assert(M4GoalService::beforeLegacy(CommandType::ARM,true));
 if(test==1)printf("%s\n",j.value.c_str());
}
