#pragma once
#include <stdint.h>
#ifndef MOTIONBRAIN_ENABLE_WHOLE_ARM_POLICY
#define MOTIONBRAIN_ENABLE_WHOLE_ARM_POLICY 0
#endif
#if MOTIONBRAIN_ENABLE_WHOLE_ARM_POLICY
#include "whole_arm_action.h"
class String;
enum class CommandType : uint8_t;
namespace WholeArmService {
bool owns();
bool beforeLegacy(CommandType);
uint32_t generation();
struct PoseObservation { const char* scope; const char* nonce; const char* pairSha256; };
bool preparePoseClose(const whole_arm::Request&,uint32_t generation,const char* boot,uint32_t cursor,const char* scope);
bool apply(const whole_arm::Request&,uint32_t expectedGeneration,const char* expectedBoot,
           const uint32_t* afterOutputSequence=nullptr,const PoseObservation* observation=nullptr);
void update();
void status(String&,bool includePreparedNonce=false);
bool parsePhase(const char*,whole_arm::Phase&);
#if MOTIONBRAIN_ENABLE_ELBOW_TIMER
bool startElbowTimingProbe(const whole_arm::Request&,uint32_t generation,const char* boot,uint32_t cursor);
void syncTimedOutputs();
bool cancelTimedOutput(bool allowShoulderRamp=false);
#endif
// All ordinary PWM ramp writes pass through this guard while the experimental
// build is running. The independent timer can force the owned PWM channel low.
bool writePwm(uint8_t motor,uint8_t channel,int16_t signedPwm);
}
#endif
