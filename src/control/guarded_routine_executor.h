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
  EXECUTOR_NOT_IMPLEMENTED,
  NO_ACTIVE_ROUTINE,
  ABORTED,
  TIMED_OUT
};

enum class GuardedRoutineExecutorState : uint8_t {
  IDLE = 0,
  PREPARED,
  RUNNING,
  ABORT_REQUESTED,
  ABORTED,
  TIMED_OUT,
  COMPLETED,
  BLOCKED
};

enum class GuardedRoutineStepResult : uint8_t {
  PENDING = 0,
  SKIPPED,
  BLOCKED
};

struct GuardedRoutineStepJournalEntry {
  uint8_t index;
  char stepId[24];
  GuardedRoutineStepKind kind;
  GuardedRoutineStepResult result;
  char detail[64];

  GuardedRoutineStepJournalEntry();
};

struct GuardedRoutineExecutorStatus {
  GuardedRoutineExecutorState state;
  char routineName[24];
  uint8_t currentStep;
  uint8_t totalSteps;
  uint8_t motionStepCount;
  uint32_t startedAtMs;
  uint32_t deadlineMs;
  uint32_t elapsedMs;
  uint32_t remainingMs;
  GuardedRoutineExecutorResult lastResult;
  char lastDetail[96];

  GuardedRoutineExecutorStatus();
};

struct GuardedRoutineExecutorReport {
  static const uint8_t MAX_STEP_JOURNAL = 8;

  bool attempted;
  bool enabled;
  bool executeImplemented;
  bool sequencePrepared;
  bool sequenceStarted;
  GuardedRoutineExecutorState state;
  uint8_t motionStepCount;
  uint8_t stepJournalCount;
  bool stepJournalTruncated;
  GuardedRoutineStepJournalEntry stepJournal[MAX_STEP_JOURNAL];
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
  static bool abort(const char* reason,
                    GuardedRoutineExecutorReport& report);
  static void update();

  static GuardedRoutineExecutorStatus status();
  static GuardedRoutineExecutorReport lastReport();

  static void appendPolicyJson(String& json);
  static void appendStatusJson(String& json);
  static void appendReportJson(String& json,
                               const GuardedRoutineExecutorReport& report);

  static const char* resultToString(GuardedRoutineExecutorResult result);
  static const char* stateToString(GuardedRoutineExecutorState state);
  static const char* stepResultToString(GuardedRoutineStepResult result);

private:
  static void buildStepJournal(const GuardedRoutinePlan& plan,
                               GuardedRoutineExecutorReport& report);
  static uint8_t countMotionSteps(const GuardedRoutinePlan& plan);
};

#endif // CONTROL_GUARDED_ROUTINE_EXECUTOR_H
