#ifndef MOTIONBRAIN_M4_GOAL_SERVICE_H
#define MOTIONBRAIN_M4_GOAL_SERVICE_H
#include <Arduino.h>
#include "m4_goal_guard.h"
enum class CommandType : uint8_t;
namespace M4GoalService {
bool owns();
bool pending();
bool queue(const m4_goal::Request&);
bool run(const char*,const char*,const char*,const char*);
void update();
void cancel(const char* reason);
void pollStop();
bool beforeLegacy(CommandType, bool explicitWeb=false);
bool serialHeld();
void appendJson(String&);
}
#endif
