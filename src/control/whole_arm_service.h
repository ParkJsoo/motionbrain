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
bool apply(const whole_arm::Request&,uint32_t expectedGeneration,const char* expectedBoot);
void update();
void status(String&);
bool parsePhase(const char*,whole_arm::Phase&);
// All ordinary PWM ramp writes pass through this guard while the experimental
// build is running. The independent timer can force the owned PWM channel low.
bool writePwm(uint8_t motor,uint8_t channel,int16_t signedPwm);
}
#endif
