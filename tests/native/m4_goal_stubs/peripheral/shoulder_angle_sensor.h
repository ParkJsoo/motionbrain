#pragma once
#include <stdint.h>
struct ShoulderAngleSensor {float angle=197.75f;uint32_t sample=100;bool ready=true;float getI2cDegrees(){return angle;}uint32_t getI2cLastUpdateMs(){return sample;}bool isReadyForMotion(uint32_t){return ready;}};
