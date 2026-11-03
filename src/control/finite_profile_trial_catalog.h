#pragma once
#include "finite_profile_runtime.h"
#include <string.h>
#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_RUNTIME
namespace finite_profile {
static const size_t TRIAL_PROFILE_COUNT = 4;
// Fixed learning candidates, not learned actions or physical safety guarantees.
// SHA-256 of canonicalProfile with the entire catalogHash line omitted.
// Index order is U4, D4, U5, D5. The default keeps the original U4 identity.
inline Profile trialProfile(size_t index = 0) {
  Profile p;
  if (index >= TRIAL_PROFILE_COUNT) return p; // Invalid, never a fallback action.
  const char* const ids[] = {
    "m2-triangle40-350-v1", "m2-triangle40-down-350-v1",
    "m2-triangle50-450-v1", "m2-triangle50-down-450-v1"
  };
  const char* const hashes[] = {
    "397407eec131923846d17207d9e88504e80ed118fa76716b244ceb593df05a6d",
    "6ab7f141a57ac34b7d324035251c8e76a04da203aa579bd1c794c2b7badc0c21",
    "7cf225f50e4d6c15049a9d05eb28026e78ce97ae4b50394167da7491b23415b5",
    "fab591ebd11131047000da4cc1e7ce332c8bb65a580c2a634f643d598b587d4a"
  };
  strcpy(p.profileId, ids[index]);
  strcpy(p.catalogHash, hashes[index]);
  p.direction = index % 2 ? -1 : 1;
  p.startLeadMs = 300;
  p.maxLatenessMs = 20;
  p.offConfirmWindowMs = 100;
  p.admissionWindowMs = 10000;
  const uint8_t peakSteps = index < 2 ? 4 : 5;
  p.count = peakSteps * 2;
  for (uint8_t i = 0; i < p.count; ++i) {
    const int16_t steps = i < peakSteps ? i + 1 : p.count - 1 - i;
    p.points[i] = Point(i * 50U, p.direction * steps * 10);
  }
  return p;
}
struct TrialCatalog {
  Profile profiles[TRIAL_PROFILE_COUNT];
  TrialCatalog() {
    for (size_t i = 0; i < TRIAL_PROFILE_COUNT; ++i) profiles[i] = trialProfile(i);
  }
};
}
#endif
