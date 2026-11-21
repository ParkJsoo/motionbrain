#ifndef CONTROL_BOUNDED_ACTION_SERVICE_H
#define CONTROL_BOUNDED_ACTION_SERVICE_H

#include <Arduino.h>
#include "control/bounded_action_executor.h"

// Experimental physical entry point is opt-in until the documented bench tests
// are complete. Existing manual and M4 paths retain their existing behavior.
#ifndef MOTIONBRAIN_ENABLE_BOUNDED_ACTIONS
#define MOTIONBRAIN_ENABLE_BOUNDED_ACTIONS 0
#endif

enum class CommandType : uint8_t;
#ifndef MOTIONBRAIN_M2_VISUAL_PILOT
#define MOTIONBRAIN_M2_VISUAL_PILOT 0
#endif

namespace BoundedActions {
bool beforeLegacyCommand(CommandType type);
void init();
void tick();
bool enabled();
bool active();
void cancel(bounded::Reason reason);
#if MOTIONBRAIN_DUAL_VIEW_TIMER
bool cancelTarget(const bounded::Record& target);
#endif
const bounded::Executor* executor();
const char* outputBootId();
bounded::SubmitResult submit(const bounded::Request& request);
void appendStatusJson(String& json);
void appendRecordJson(String& json, const bounded::Record& record, const bool* cancelApplied = nullptr);
void appendOutputJson(String& json, bool paged = false, uint32_t after = 0, size_t limit = 8);
}
#endif
