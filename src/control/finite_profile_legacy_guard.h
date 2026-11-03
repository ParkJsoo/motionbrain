#ifndef CONTROL_FINITE_PROFILE_LEGACY_GUARD_H
#define CONTROL_FINITE_PROFILE_LEGACY_GUARD_H

#include <stdint.h>
#ifndef MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER
#define MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER 0
#endif

enum class CommandType : uint8_t;

namespace FiniteProfileLegacy {
// Competing intent invalidates the current lease before any legacy state or
// queue mutation. The caller still rejects movement while the owner remains.
#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER
bool allowed(CommandType type);
#else
inline bool allowed(CommandType) { return true; }
#endif
}
#endif
