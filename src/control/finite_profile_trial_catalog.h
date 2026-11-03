#pragma once
#include "finite_profile_runtime.h"
#include <string.h>
#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_RUNTIME
namespace finite_profile {
// A commissioning profile, not a learned action or a physical safety guarantee.
// SHA-256 of canonicalProfile with the entire catalogHash line omitted.
inline Profile trialProfile() {
  Profile p;
  strcpy(p.profileId, "m2-triangle40-350-v1");
  strcpy(p.catalogHash, "397407eec131923846d17207d9e88504e80ed118fa76716b244ceb593df05a6d");
  p.startLeadMs = 300;
  p.maxLatenessMs = 20;
  p.offConfirmWindowMs = 100;
  p.admissionWindowMs = 10000;
  p.count = 8;
  const int16_t pwm[] = {10, 20, 30, 40, 30, 20, 10, 0};
  for (uint8_t i = 0; i < p.count; ++i) p.points[i] = Point(i * 50U, pwm[i]);
  return p;
}
}
#endif
