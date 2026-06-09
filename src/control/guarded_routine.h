#ifndef CONTROL_GUARDED_ROUTINE_H
#define CONTROL_GUARDED_ROUTINE_H

#include <Arduino.h>
#include <stdint.h>
#include "motion/motion_sequence.h"

enum class GuardedRoutineStepKind : uint8_t {
  CHECK = 0,
  MOTION,
  VERIFY
};

struct GuardedRoutineStep {
  const char* kindId;
  GuardedRoutineStepKind kind;
  const char* label;
  const char* detail;
  MotionJoint joint;
  MotionDirection direction;
  uint8_t percent;
  uint32_t durationMs;
  float targetDegrees;
};

struct GuardedRoutinePlan {
  const char* name;
  const char* summary;
  const char* confirmationCode;
  const char* preconditionIds;
  const GuardedRoutineStep* steps;
  uint8_t stepCount;
  uint32_t confirmationTtlMs;
  uint32_t stepTimeoutMs;
  uint32_t totalTimeoutMs;
  bool requiresOperatorConfirm;
  bool requiresArmedForExecute;
  bool requiresMotionClearForExecute;
  bool perceptionRequired;
  bool stopAfterEachMotionStep;
  bool statusCheckAfterEachStep;
};

enum class GuardedRoutinePreflightResult : uint8_t {
  DRY_RUN_ONLY = 0,
  CONFIRM_REQUIRED,
  STATE_NOT_ARMED,
  MOTION_BLOCKED,
  FAULT_LATCHED,
  SEQUENCE_ACTIVE,
  PERCEPTION_REQUIRED,
  EXECUTE_BLOCKED
};

struct GuardedRoutineExecutePreflight {
  bool operatorConfirmed;
  bool stateAllowsExecute;
  bool motionClear;
  bool faultClear;
  bool noActiveSequence;
  bool perceptionReady;
  bool executeReady;
  GuardedRoutinePreflightResult result;
};

class GuardedRoutine {
public:
  static bool getPlan(const char* name, GuardedRoutinePlan& outPlan);
  static uint8_t routineCount();
  static const char* routineNameAt(uint8_t index);

  static GuardedRoutineExecutePreflight evaluateExecutePreflight(
    const GuardedRoutinePlan& plan,
    bool operatorConfirmed,
    bool stateAllowsExecute,
    bool motionClear,
    bool faultClear,
    bool noActiveSequence,
    bool perceptionReady);

  static void appendPlanJson(String& json, const GuardedRoutinePlan& plan);
  static void appendRoutineListJson(String& json);

  static const char* preflightResultToString(GuardedRoutinePreflightResult result);
  static const char* stepKindToString(GuardedRoutineStepKind kind);
  static const char* jointToString(MotionJoint joint);
  static const char* directionToString(MotionDirection direction);
};

#endif // CONTROL_GUARDED_ROUTINE_H
