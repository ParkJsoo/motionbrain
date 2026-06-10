#ifndef CONTROL_GUARDED_ROUTINE_EXECUTOR_H
#define CONTROL_GUARDED_ROUTINE_EXECUTOR_H

#include <Arduino.h>
#include <stdint.h>

#include "control/guarded_routine.h"

#ifndef MOTIONBRAIN_ROUTINE_EXECUTOR_ENABLED
#define MOTIONBRAIN_ROUTINE_EXECUTOR_ENABLED 0
#endif

enum class GuardedRoutineExecutorResult : uint8_t {
  NOT_REQUESTED = 0,
  EXECUTOR_DISABLED,
  EXECUTOR_NOT_IMPLEMENTED
};

struct GuardedRoutineExecutorReport {
  bool attempted;
  bool enabled;
  bool executeImplemented;
  bool sequencePrepared;
  bool sequenceStarted;
  uint8_t motionStepCount;
  GuardedRoutineExecutorResult result;
  char detail[96];

  GuardedRoutineExecutorReport();
};

class GuardedRoutineExecutor {
public:
  static bool isEnabled();
  static bool executeImplemented();

  static GuardedRoutineExecutorReport describe(const GuardedRoutinePlan& plan,
                                               bool attempted);
  static bool begin(const GuardedRoutinePlan& plan,
                    GuardedRoutineExecutorReport& report);

  static void appendPolicyJson(String& json);
  static void appendReportJson(String& json,
                               const GuardedRoutineExecutorReport& report);

  static const char* resultToString(GuardedRoutineExecutorResult result);

private:
  static uint8_t countMotionSteps(const GuardedRoutinePlan& plan);
};

#endif // CONTROL_GUARDED_ROUTINE_EXECUTOR_H
