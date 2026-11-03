#ifndef CONTROL_FINITE_PROFILE_RUNTIME_H
#define CONTROL_FINITE_PROFILE_RUNTIME_H
#include <stddef.h>

#ifndef MOTIONBRAIN_ENABLE_FINITE_PROFILE_RUNTIME
#define MOTIONBRAIN_ENABLE_FINITE_PROFILE_RUNTIME 0
#endif

#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_RUNTIME
#include "finite_profile_scheduler.h"
#include "finite_profile_service_environment.h"
#if !MOTIONBRAIN_ENABLE_FINITE_PROFILE_SCHEDULER
#error "Finite profile runtime requires MOTIONBRAIN_ENABLE_FINITE_PROFILE_SCHEDULER"
#endif
#endif

namespace finite_profile {
class ScheduledDriver;
struct Profile;
namespace runtime {
static const size_t MAX_RUNTIME_PROFILES = 4;
bool enabled();
void update();
bool motionPending();
const char* bootId();
ScheduledDriver* driver();
const Profile* profile();
size_t profileCount();
const Profile* profileAt(size_t index);
const Profile* findProfile(const char* profileId, const char* catalogHash);
size_t coreRecordsUsed();
size_t driverRecordsUsed();
#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_RUNTIME
// Once per firmware boot, after controller dependencies and entropy are ready.
// The trusted catalogue is copied; no HTTP-provided PWM or points are accepted.
// One nothrow heap allocation is retained for the firmware boot; failure leaves
// the runtime unavailable, with no output. Driver and issuer must outlive it.
// All access, including calls
// through driver(), must remain serialized on the same nonreentrant loop.
bool init(MotorControl& driver, const Profile& trustedProfile, const char* bootId,
          ServiceEnvironment::TokenIssuer issuer);
bool init(MotorControl& driver, const Profile* trustedProfiles, size_t profileCount,
          const char* bootId, ServiceEnvironment::TokenIssuer issuer);
#endif
}
}
#endif
