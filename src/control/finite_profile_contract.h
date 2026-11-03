#ifndef MOTIONBRAIN_FINITE_PROFILE_CONTRACT_H
#define MOTIONBRAIN_FINITE_PROFILE_CONTRACT_H

// No firmware entry point or hardware port exists for this offline core.
#ifndef MOTIONBRAIN_ENABLE_FINITE_PROFILE_CORE
#define MOTIONBRAIN_ENABLE_FINITE_PROFILE_CORE 0
#endif

#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_CORE
#include <stddef.h>
#include <stdint.h>
#include <string>

namespace finite_profile {

static const size_t MAX_POINTS = 32;
// Structural resource limits, NOT physically approved PWM/time limits.
static const uint32_t MAX_PROFILE_MS = 10000;
static const uint32_t MAX_WINDOW_MS = 10000;

struct Point {
  uint32_t offsetMs;
  int16_t pwm;
  Point(uint32_t offset = 0, int16_t value = 0) : offsetMs(offset), pwm(value) {}
};

struct Profile {
  uint16_t contractVersion;
  uint16_t profileVersion;
  uint16_t timingPolicyVersion;
  char catalogHash[65];
  char profileId[33];
  uint8_t motor;
  int8_t direction;
  uint32_t startLeadMs;
  uint32_t maxLatenessMs;
  uint32_t offConfirmWindowMs;
  uint32_t admissionWindowMs;
  uint8_t count;
  Point points[MAX_POINTS];
  Profile() : contractVersion(1), profileVersion(1), timingPolicyVersion(1),
      catalogHash{}, profileId{}, motor(2), direction(1), startLeadMs(0),
      maxLatenessMs(0), offConfirmWindowMs(0), admissionWindowMs(0), count(0), points{} {}
};

bool validIdentifier(const char* value, size_t capacity);
bool validProfile(const Profile& profile);
// Fixed ASCII/UTF-8 bytes, LF line endings, decimal integers, no final spaces.
// Empty means invalid. This is the hash input, not a digest or authentication.
std::string canonicalProfile(const Profile& profile);
// Compares all execution fields, without relying on an externally supplied hash.
bool sameProfile(const Profile& left, const Profile& right);

} // namespace finite_profile
#endif
#endif
