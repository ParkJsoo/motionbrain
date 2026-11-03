#pragma once
#include "finite_profile_scheduler.h"
#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_SCHEDULER
#include <Arduino.h>
namespace finite_profile {
const char* codeName(Code code);
void appendProfileJson(String& json, const Profile& profile);
void appendRecordJson(String& json, const Record& record);
void appendEvidenceJson(String& json, const DriverOutputEvidence* evidence);
}
#endif
