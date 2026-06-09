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

class GuardedRoutine {
public:
  static bool getPlan(const char* name, GuardedRoutinePlan& outPlan);
  static uint8_t routineCount();
  static const char* routineNameAt(uint8_t index);

  static void appendPlanJson(String& json, const GuardedRoutinePlan& plan);
  static void appendRoutineListJson(String& json);

  static const char* stepKindToString(GuardedRoutineStepKind kind);
  static const char* jointToString(MotionJoint joint);
  static const char* directionToString(MotionDirection direction);
};

#endif // CONTROL_GUARDED_ROUTINE_H
