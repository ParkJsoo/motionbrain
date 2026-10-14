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
namespace BoundedActions {
bool beforeLegacyCommand(CommandType type);
void init();
void tick();
bool enabled();
bool active();
void cancel(bounded::Reason reason);
const bounded::Executor* executor();
bounded::SubmitResult submit(const bounded::Request& request);
void appendStatusJson(String& json);
void appendRecordJson(String& json, const bounded::Record& record);
void appendOutputJson(String& json);
}
#endif
