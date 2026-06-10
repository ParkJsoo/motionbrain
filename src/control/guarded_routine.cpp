#include "control/guarded_routine.h"

#include <string.h>
#include <strings.h>

namespace {

const GuardedRoutineStep INSPECT_STEPS[] = {
  {"preflight", GuardedRoutineStepKind::CHECK, "Preflight",
   "Read state, safety block, fault latch, and sensor freshness before motion.",
   MotionJoint::GRIPPER, MotionDirection::OPEN, 0, 0, 0.0f},
  {"shoulder_preview", GuardedRoutineStepKind::MOTION, "Shoulder preview",
   "Short low-speed shoulder-up pulse for visual inspection.",
   MotionJoint::SHOULDER, MotionDirection::UP, 25, 250, 0.0f},
  {"base_preview", GuardedRoutineStepKind::MOTION, "Base preview",
   "Short low-speed base-left pulse, followed by immediate stop/status check.",
   MotionJoint::BASE, MotionDirection::LEFT, 25, 250, 0.0f},
  {"verify", GuardedRoutineStepKind::VERIFY, "Verify",
   "Stop all joints, read status, and keep the event log entry.",
   MotionJoint::GRIPPER, MotionDirection::OPEN, 0, 0, 0.0f},
};

const GuardedRoutineStep OPEN_GRIPPER_CHECK_STEPS[] = {
  {"preflight", GuardedRoutineStepKind::CHECK, "Preflight",
   "Require operator confirmation, safe state, and clear motion gate before execute.",
   MotionJoint::GRIPPER, MotionDirection::OPEN, 0, 0, 0.0f},
  {"open_gripper", GuardedRoutineStepKind::MOTION, "Open gripper",
   "Open gripper with a short bounded pulse.",
   MotionJoint::GRIPPER, MotionDirection::OPEN, 30, 300, 0.0f},
  {"verify", GuardedRoutineStepKind::VERIFY, "Verify",
   "Stop gripper and read status/events after the pulse.",
   MotionJoint::GRIPPER, MotionDirection::OPEN, 0, 0, 0.0f},
};

const GuardedRoutineStep STOW_STEPS[] = {
  {"preflight", GuardedRoutineStepKind::CHECK, "Preflight",
   "Confirm low-speed stow motion and clear any active sequence before execute.",
   MotionJoint::GRIPPER, MotionDirection::OPEN, 0, 0, 0.0f},
  {"wrist_down", GuardedRoutineStepKind::MOTION, "Wrist down",
   "Small wrist-down pulse toward a compact pose.",
   MotionJoint::WRIST, MotionDirection::DOWN, 25, 250, 0.0f},
  {"elbow_down", GuardedRoutineStepKind::MOTION, "Elbow down",
   "Small elbow-down pulse toward a compact pose.",
   MotionJoint::ELBOW, MotionDirection::DOWN, 25, 300, 0.0f},
  {"shoulder_down", GuardedRoutineStepKind::MOTION, "Shoulder down",
   "Small shoulder-down pulse toward a compact pose.",
   MotionJoint::SHOULDER, MotionDirection::DOWN, 25, 350, 0.0f},
  {"verify", GuardedRoutineStepKind::VERIFY, "Verify",
   "Stop all joints and read status/events.",
   MotionJoint::GRIPPER, MotionDirection::OPEN, 0, 0, 0.0f},
};

const GuardedRoutineStep CENTER_TARGET_DRY_RUN_STEPS[] = {
  {"preflight", GuardedRoutineStepKind::CHECK, "Preflight",
   "Require fresh host-side detection and centered/left/right alignment input.",
   MotionJoint::GRIPPER, MotionDirection::OPEN, 0, 0, 0.0f},
  {"propose_alignment", GuardedRoutineStepKind::VERIFY, "Propose alignment",
   "Create a plan only; stale or missing detection must block physical action.",
   MotionJoint::GRIPPER, MotionDirection::OPEN, 0, 0, 0.0f},
  {"base_nudge_candidate", GuardedRoutineStepKind::VERIFY, "Base nudge candidate",
   "If confirmed later, choose left/right from fresh alignment before motion.",
   MotionJoint::BASE, MotionDirection::LEFT, 25, 200, 0.0f},
  {"verify", GuardedRoutineStepKind::VERIFY, "Verify",
   "Re-read detection and controller status before any future execute path.",
   MotionJoint::GRIPPER, MotionDirection::OPEN, 0, 0, 0.0f},
};

const GuardedRoutineStep SOFT_HOME_REFERENCE_STEPS[] = {
  {"preflight", GuardedRoutineStepKind::CHECK, "Preflight",
   "Require operator-confirmed manual reference pose, clear fault state, and no active sequence.",
   MotionJoint::GRIPPER, MotionDirection::OPEN, 0, 0, 0.0f},
  {"manual_reference", GuardedRoutineStepKind::VERIFY, "Manual reference",
   "Operator places the arm at the agreed soft-home pose; firmware records no absolute joint position.",
   MotionJoint::GRIPPER, MotionDirection::OPEN, 0, 0, 0.0f},
  {"verify_limits", GuardedRoutineStepKind::VERIFY, "Verify limits",
   "Confirm this is a software reference only; no encoder-grade homing or hard-stop seeking is performed.",
   MotionJoint::GRIPPER, MotionDirection::OPEN, 0, 0, 0.0f},
};

const GuardedRoutinePlan ROUTINES[] = {
  {"inspect", "Low-speed visual inspection routine.", "confirm-inspect",
   "state_armed|motion_clear|fault_clear|operator_confirmed|no_active_sequence",
   INSPECT_STEPS,
   static_cast<uint8_t>(sizeof(INSPECT_STEPS) / sizeof(INSPECT_STEPS[0])),
   15000, 1000, 3000,
   true, true, true, false, true, true},
  {"open_gripper_check", "Open gripper with a bounded pulse and verify stop.", "confirm-open-gripper-check",
   "state_armed|motion_clear|fault_clear|operator_confirmed|no_active_sequence",
   OPEN_GRIPPER_CHECK_STEPS,
   static_cast<uint8_t>(sizeof(OPEN_GRIPPER_CHECK_STEPS) / sizeof(OPEN_GRIPPER_CHECK_STEPS[0])),
   15000, 1000, 2500,
   true, true, true, false, true, true},
  {"stow", "Move toward a compact stow pose using short relative pulses.", "confirm-stow",
   "state_armed|motion_clear|fault_clear|operator_confirmed|no_active_sequence",
   STOW_STEPS,
   static_cast<uint8_t>(sizeof(STOW_STEPS) / sizeof(STOW_STEPS[0])),
   15000, 1000, 4500,
   true, true, true, false, true, true},
  {"center_target_dry_run", "Plan a target-centering action without physical execution.", "confirm-center-target",
   "state_armed|motion_clear|fault_clear|operator_confirmed|no_active_sequence|perception_fresh|target_alignment_fresh",
   CENTER_TARGET_DRY_RUN_STEPS,
   static_cast<uint8_t>(sizeof(CENTER_TARGET_DRY_RUN_STEPS) / sizeof(CENTER_TARGET_DRY_RUN_STEPS[0])),
   15000, 1000, 2500,
   true, true, true, true, true, true},
  {"soft_home_reference", "Operator-confirmed software home/reference procedure; no automatic homing motion.", "confirm-soft-home-reference",
   "manual_reference_pose|fault_clear|operator_confirmed|no_active_sequence",
   SOFT_HOME_REFERENCE_STEPS,
   static_cast<uint8_t>(sizeof(SOFT_HOME_REFERENCE_STEPS) / sizeof(SOFT_HOME_REFERENCE_STEPS[0])),
   15000, 1000, 2000,
   true, false, false, false, false, true},
};

void appendEscaped(String& json, const char* raw) {
  const char* text = raw != nullptr ? raw : "";
  while (*text != '\0') {
    switch (*text) {
      case '\\': json += "\\\\"; break;
      case '"':  json += "\\\""; break;
      case '\n': json += "\\n"; break;
      case '\r': json += "\\r"; break;
      case '\t': json += "\\t"; break;
      default:   json += *text; break;
    }
    text++;
  }
}

bool isCenterTargetAlias(const char* name) {
  return strcasecmp(name, "center_target") == 0 ||
         strcasecmp(name, "center-target") == 0 ||
         strcasecmp(name, "center_target_dry_run") == 0;
}

bool isSoftHomeAlias(const char* name) {
  return strcasecmp(name, "home_reference") == 0 ||
         strcasecmp(name, "home-reference") == 0 ||
         strcasecmp(name, "soft_home") == 0 ||
         strcasecmp(name, "soft-home") == 0 ||
         strcasecmp(name, "soft_home_reference") == 0;
}

void appendStringArrayFromPipes(String& json, const char* values) {
  json += "[";
  if (values != nullptr && values[0] != '\0') {
    const char* segmentStart = values;
    bool first = true;
    for (const char* cursor = values; ; ++cursor) {
      if (*cursor == '|' || *cursor == '\0') {
        if (!first) {
          json += ",";
        }
        first = false;
        json += "\"";
        for (const char* c = segmentStart; c < cursor; ++c) {
          switch (*c) {
            case '\\': json += "\\\\"; break;
            case '"':  json += "\\\""; break;
            default:   json += *c; break;
          }
        }
        json += "\"";
        if (*cursor == '\0') {
          break;
        }
        segmentStart = cursor + 1;
      }
    }
  }
  json += "]";
}

} // namespace

bool GuardedRoutine::getPlan(const char* name, GuardedRoutinePlan& outPlan) {
  if (name == nullptr || name[0] == '\0') {
    return false;
  }

  for (uint8_t i = 0; i < routineCount(); ++i) {
    if (strcasecmp(name, ROUTINES[i].name) == 0 ||
        (isCenterTargetAlias(name) && strcmp(ROUTINES[i].name, "center_target_dry_run") == 0) ||
        (isSoftHomeAlias(name) && strcmp(ROUTINES[i].name, "soft_home_reference") == 0)) {
      outPlan = ROUTINES[i];
      return true;
    }
  }

  return false;
}

uint8_t GuardedRoutine::routineCount() {
  return static_cast<uint8_t>(sizeof(ROUTINES) / sizeof(ROUTINES[0]));
}

const char* GuardedRoutine::routineNameAt(uint8_t index) {
  return index < routineCount() ? ROUTINES[index].name : "";
}

GuardedRoutineExecutePreflight GuardedRoutine::evaluateExecutePreflight(
    const GuardedRoutinePlan& plan,
    bool operatorConfirmed,
    bool stateAllowsExecute,
    bool motionClear,
    bool faultClear,
    bool noActiveSequence,
    bool perceptionReady) {
  GuardedRoutineExecutePreflight preflight = {
    operatorConfirmed,
    stateAllowsExecute,
    motionClear,
    faultClear,
    noActiveSequence,
    perceptionReady,
    false,
    GuardedRoutinePreflightResult::EXECUTE_BLOCKED
  };

  if (plan.requiresOperatorConfirm && !operatorConfirmed) {
    preflight.result = GuardedRoutinePreflightResult::CONFIRM_REQUIRED;
  } else if (plan.requiresArmedForExecute && !stateAllowsExecute) {
    preflight.result = GuardedRoutinePreflightResult::STATE_NOT_ARMED;
  } else if (plan.requiresMotionClearForExecute && !motionClear) {
    preflight.result = GuardedRoutinePreflightResult::MOTION_BLOCKED;
  } else if (!faultClear) {
    preflight.result = GuardedRoutinePreflightResult::FAULT_LATCHED;
  } else if (!noActiveSequence) {
    preflight.result = GuardedRoutinePreflightResult::SEQUENCE_ACTIVE;
  } else if (plan.perceptionRequired && !perceptionReady) {
    preflight.result = GuardedRoutinePreflightResult::PERCEPTION_REQUIRED;
  } else {
    preflight.executeReady = true;
    preflight.result = GuardedRoutinePreflightResult::EXECUTE_BLOCKED;
  }

  return preflight;
}

void GuardedRoutine::appendPlanJson(String& json, const GuardedRoutinePlan& plan) {
  json += "\"routine\":{";
  json += "\"name\":\"";
  appendEscaped(json, plan.name);
  json += "\",\"summary\":\"";
  appendEscaped(json, plan.summary);
  json += "\",\"dryRunOnly\":true";
  json += ",\"preconditions\":";
  appendStringArrayFromPipes(json, plan.preconditionIds);
  json += ",\"operatorConfirmation\":{";
  json += "\"required\":";
  json += plan.requiresOperatorConfirm ? "true" : "false";
  json += ",\"code\":\"";
  appendEscaped(json, plan.confirmationCode);
  json += "\",\"ttlMs\":";
  json += String(plan.confirmationTtlMs);
  json += "}";
  json += ",\"executionPolicy\":{";
  json += "\"mode\":\"dry_run_only\"";
  json += ",\"stepTimeoutMs\":";
  json += String(plan.stepTimeoutMs);
  json += ",\"totalTimeoutMs\":";
  json += String(plan.totalTimeoutMs);
  json += ",\"stopAfterEachMotionStep\":";
  json += plan.stopAfterEachMotionStep ? "true" : "false";
  json += ",\"statusCheckAfterEachStep\":";
  json += plan.statusCheckAfterEachStep ? "true" : "false";
  json += ",\"abortCommand\":\"stop\"";
  json += "}";
  json += ",\"requiresOperatorConfirm\":";
  json += plan.requiresOperatorConfirm ? "true" : "false";
  json += ",\"requiresArmedForExecute\":";
  json += plan.requiresArmedForExecute ? "true" : "false";
  json += ",\"requiresMotionClearForExecute\":";
  json += plan.requiresMotionClearForExecute ? "true" : "false";
  json += ",\"perceptionRequired\":";
  json += plan.perceptionRequired ? "true" : "false";
  json += ",\"stepCount\":";
  json += String(plan.stepCount);
  json += ",\"steps\":[";

  for (uint8_t i = 0; i < plan.stepCount; ++i) {
    const GuardedRoutineStep& step = plan.steps[i];
    if (i > 0) {
      json += ",";
    }
    json += "{\"index\":";
    json += String(i + 1);
    json += ",\"id\":\"";
    appendEscaped(json, step.kindId);
    json += "\",\"kind\":\"";
    json += stepKindToString(step.kind);
    json += "\",\"label\":\"";
    appendEscaped(json, step.label);
    json += "\",\"detail\":\"";
    appendEscaped(json, step.detail);
    json += "\"";
    if (step.kind == GuardedRoutineStepKind::MOTION) {
      json += ",\"joint\":\"";
      json += jointToString(step.joint);
      json += "\",\"direction\":\"";
      json += directionToString(step.direction);
      json += "\",\"percent\":";
      json += String(step.percent);
      json += ",\"durationMs\":";
      json += String(step.durationMs);
      json += ",\"targetDegrees\":";
      json += String(step.targetDegrees, 1);
    }
    json += "}";
  }

  json += "]}";
}

void GuardedRoutine::appendRoutineListJson(String& json) {
  for (uint8_t i = 0; i < routineCount(); ++i) {
    if (i > 0) {
      json += ",";
    }
    json += "{\"name\":\"";
    appendEscaped(json, ROUTINES[i].name);
    json += "\",\"summary\":\"";
    appendEscaped(json, ROUTINES[i].summary);
    json += "\",\"dryRunOnly\":true";
    json += ",\"confirmRequired\":";
    json += ROUTINES[i].requiresOperatorConfirm ? "true" : "false";
    json += ",\"stepCount\":";
    json += String(ROUTINES[i].stepCount);
    json += "}";
  }
}

const char* GuardedRoutine::stepKindToString(GuardedRoutineStepKind kind) {
  switch (kind) {
    case GuardedRoutineStepKind::CHECK:  return "check";
    case GuardedRoutineStepKind::MOTION: return "motion";
    case GuardedRoutineStepKind::VERIFY: return "verify";
    default:                             return "unknown";
  }
}

const char* GuardedRoutine::preflightResultToString(GuardedRoutinePreflightResult result) {
  switch (result) {
    case GuardedRoutinePreflightResult::DRY_RUN_ONLY:          return "dry_run_only";
    case GuardedRoutinePreflightResult::CONFIRM_REQUIRED:      return "confirm_required";
    case GuardedRoutinePreflightResult::STATE_NOT_ARMED:       return "state_not_armed";
    case GuardedRoutinePreflightResult::MOTION_BLOCKED:        return "motion_blocked";
    case GuardedRoutinePreflightResult::FAULT_LATCHED:         return "fault_latched";
    case GuardedRoutinePreflightResult::SEQUENCE_ACTIVE:       return "sequence_active";
    case GuardedRoutinePreflightResult::PERCEPTION_REQUIRED:   return "perception_required";
    case GuardedRoutinePreflightResult::EXECUTE_BLOCKED:       return "execute_blocked";
    default:                                                   return "unknown";
  }
}

const char* GuardedRoutine::jointToString(MotionJoint joint) {
  switch (joint) {
    case MotionJoint::GRIPPER:  return "gripper";
    case MotionJoint::WRIST:    return "wrist";
    case MotionJoint::ELBOW:    return "elbow";
    case MotionJoint::SHOULDER: return "shoulder";
    case MotionJoint::BASE:     return "base";
    default:                    return "unknown";
  }
}

const char* GuardedRoutine::directionToString(MotionDirection direction) {
  switch (direction) {
    case MotionDirection::OPEN:  return "open";
    case MotionDirection::CLOSE: return "close";
    case MotionDirection::UP:    return "up";
    case MotionDirection::DOWN:  return "down";
    case MotionDirection::LEFT:  return "left";
    case MotionDirection::RIGHT: return "right";
    default:                     return "unknown";
  }
}
