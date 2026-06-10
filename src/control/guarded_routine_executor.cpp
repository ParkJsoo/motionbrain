#include "control/guarded_routine_executor.h"

#include <stdio.h>
#include <string.h>

#include "control/angle_controller.h"

namespace {

GuardedRoutineExecutorStatus currentStatus;
GuardedRoutineExecutorReport currentReport;

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

void refreshTiming(GuardedRoutineExecutorStatus& status) {
  if (status.startedAtMs == 0) {
    status.elapsedMs = 0;
    status.remainingMs = 0;
    return;
  }

  const uint32_t now = millis();
  status.elapsedMs = now - status.startedAtMs;
  if (status.deadlineMs == 0 || static_cast<int32_t>(status.deadlineMs - now) <= 0) {
    status.remainingMs = 0;
  } else {
    status.remainingMs = status.deadlineMs - now;
  }
}

bool isActiveState(GuardedRoutineExecutorState state) {
  return state == GuardedRoutineExecutorState::PREPARED ||
         state == GuardedRoutineExecutorState::RUNNING ||
         state == GuardedRoutineExecutorState::ABORT_REQUESTED;
}

bool directionAllowedForJoint(MotionJoint joint, MotionDirection direction) {
  switch (joint) {
    case MotionJoint::GRIPPER:
      return direction == MotionDirection::OPEN || direction == MotionDirection::CLOSE;
    case MotionJoint::WRIST:
    case MotionJoint::ELBOW:
    case MotionJoint::SHOULDER:
      return direction == MotionDirection::UP || direction == MotionDirection::DOWN;
    case MotionJoint::BASE:
      return direction == MotionDirection::LEFT || direction == MotionDirection::RIGHT;
    default:
      return false;
  }
}

bool isBaseAngleStep(const GuardedRoutineStep& step) {
  return step.joint == MotionJoint::BASE && step.targetDegrees > 0.0f;
}

bool hasValidMotionParameters(const GuardedRoutineStep& step) {
  if (!directionAllowedForJoint(step.joint, step.direction) ||
      step.percent == 0 || step.percent > 100) {
    return false;
  }

  if (isBaseAngleStep(step)) {
    return step.targetDegrees >= AngleController::MIN_TARGET_DEGREES &&
           step.targetDegrees <= AngleController::MAX_TARGET_DEGREES;
  }

  return step.durationMs > 0 && step.targetDegrees == 0.0f;
}

void publishReport(const GuardedRoutineExecutorReport& report,
                   const GuardedRoutinePlan* plan,
                   GuardedRoutineExecutorState state) {
  currentReport = report;
  currentReport.state = state;

  currentStatus.state = state;
  currentStatus.currentStep = 0;
  currentStatus.totalSteps = plan != nullptr ? plan->stepCount : 0;
  currentStatus.motionStepCount = report.motionStepCount;
  currentStatus.lastResult = report.result;
  strlcpy(currentStatus.routineName, plan != nullptr ? plan->name : "",
          sizeof(currentStatus.routineName));
  strlcpy(currentStatus.lastDetail, report.detail, sizeof(currentStatus.lastDetail));

  const uint32_t now = millis();
  currentStatus.startedAtMs = report.attempted ? now : 0;
  currentStatus.deadlineMs = 0;
  if (plan != nullptr && state == GuardedRoutineExecutorState::RUNNING) {
    currentStatus.deadlineMs = now + plan->totalTimeoutMs;
  }
  refreshTiming(currentStatus);
}

} // namespace

GuardedRoutineExecutorStatus::GuardedRoutineExecutorStatus()
  : state(GuardedRoutineExecutorState::IDLE)
  , routineName{0}
  , currentStep(0)
  , totalSteps(0)
  , motionStepCount(0)
  , startedAtMs(0)
  , deadlineMs(0)
  , elapsedMs(0)
  , remainingMs(0)
  , lastResult(GuardedRoutineExecutorResult::NOT_REQUESTED)
  , lastDetail{0} {
  strlcpy(lastDetail, "executor idle", sizeof(lastDetail));
}

GuardedRoutineStepJournalEntry::GuardedRoutineStepJournalEntry()
  : index(0)
  , stepId{0}
  , kind(GuardedRoutineStepKind::CHECK)
  , result(GuardedRoutineStepResult::PENDING)
  , detail{0} {
  strlcpy(detail, "pending", sizeof(detail));
}

GuardedRoutinePreparedStep::GuardedRoutinePreparedStep()
  : sourceIndex(0)
  , sourceStepId{0}
  , joint(MotionJoint::GRIPPER)
  , direction(MotionDirection::OPEN)
  , percent(0)
  , durationMs(0)
  , targetDegrees(0.0f)
  , stopAfterStepRequired(false)
  , statusCheckRequired(false) {
}

GuardedRoutineExecutorReport::GuardedRoutineExecutorReport()
  : attempted(false)
  , enabled(GuardedRoutineExecutor::isEnabled())
  , executeImplemented(GuardedRoutineExecutor::executeImplemented())
  , sequencePrepared(false)
  , sequenceStarted(false)
  , state(GuardedRoutineExecutorState::IDLE)
  , motionStepCount(0)
  , prepareAttempted(false)
  , prepareReady(false)
  , preparedSequenceApplied(false)
  , preparedStepCount(0)
  , preparedMotionCount(0)
  , prepareResult(GuardedRoutinePrepareResult::PREPARE_NOT_REQUESTED)
  , prepareDetail{0}
  , stepJournalCount(0)
  , stepJournalTruncated(false)
  , result(GuardedRoutineExecutorResult::NOT_REQUESTED)
  , detail{0} {
  strlcpy(prepareDetail, "prepare not requested", sizeof(prepareDetail));
  strlcpy(detail, "executor not requested", sizeof(detail));
}

bool GuardedRoutineExecutor::isEnabled() {
  return MOTIONBRAIN_ROUTINE_EXECUTOR_ENABLED != 0;
}

bool GuardedRoutineExecutor::executeImplemented() {
  return false;
}

GuardedRoutineExecutorReport GuardedRoutineExecutor::describe(
    const GuardedRoutinePlan& plan,
    bool attempted) {
  GuardedRoutineExecutorReport report;
  report.attempted = attempted;
  report.enabled = isEnabled();
  report.executeImplemented = executeImplemented();
  report.motionStepCount = countMotionSteps(plan);
  report.sequencePrepared = false;
  report.sequenceStarted = false;
  buildPreparedSequence(plan, report);

  if (!attempted) {
    report.state = GuardedRoutineExecutorState::IDLE;
    report.result = GuardedRoutineExecutorResult::NOT_REQUESTED;
    strlcpy(report.detail, "executor not requested", sizeof(report.detail));
  } else if (!report.enabled) {
    report.state = GuardedRoutineExecutorState::BLOCKED;
    report.result = GuardedRoutineExecutorResult::EXECUTOR_DISABLED;
    strlcpy(report.detail, "routine executor disabled by firmware policy",
            sizeof(report.detail));
  } else {
    report.state = GuardedRoutineExecutorState::BLOCKED;
    report.result = GuardedRoutineExecutorResult::EXECUTOR_NOT_IMPLEMENTED;
    strlcpy(report.detail, "routine physical executor is not implemented",
            sizeof(report.detail));
  }

  buildStepJournal(plan, report);
  return report;
}

bool GuardedRoutineExecutor::begin(const GuardedRoutinePlan& plan,
                                   GuardedRoutineExecutorReport& report) {
  report = describe(plan, true);
  publishReport(report, &plan, report.state);
  return report.sequenceStarted;
}

bool GuardedRoutineExecutor::abort(const char* reason,
                                   GuardedRoutineExecutorReport& report) {
  report = currentReport;
  report.attempted = true;
  report.enabled = isEnabled();
  report.executeImplemented = executeImplemented();
  report.sequencePrepared = false;
  report.sequenceStarted = false;
  report.prepareAttempted = false;
  report.prepareReady = false;
  report.preparedSequenceApplied = false;
  report.preparedStepCount = 0;
  report.preparedMotionCount = 0;
  report.prepareResult = GuardedRoutinePrepareResult::PREPARE_NOT_REQUESTED;
  strlcpy(report.prepareDetail, "prepare not requested", sizeof(report.prepareDetail));
  report.stepJournalCount = 0;
  report.stepJournalTruncated = false;

  if (!isActiveState(currentStatus.state)) {
    report.state = GuardedRoutineExecutorState::IDLE;
    report.motionStepCount = 0;
    report.result = GuardedRoutineExecutorResult::NO_ACTIVE_ROUTINE;
    strlcpy(report.detail, "no active routine executor to abort",
            sizeof(report.detail));
    publishReport(report, nullptr, GuardedRoutineExecutorState::IDLE);
    return false;
  }

  report.state = GuardedRoutineExecutorState::ABORTED;
  report.result = GuardedRoutineExecutorResult::ABORTED;
  snprintf(report.detail, sizeof(report.detail), "routine abort requested: %s",
           reason != nullptr && reason[0] != '\0' ? reason : "operator_request");
  publishReport(report, nullptr, GuardedRoutineExecutorState::ABORTED);
  return true;
}

void GuardedRoutineExecutor::update() {
  if (currentStatus.state != GuardedRoutineExecutorState::RUNNING ||
      currentStatus.deadlineMs == 0) {
    return;
  }

  if (static_cast<int32_t>(millis() - currentStatus.deadlineMs) < 0) {
    return;
  }

  currentReport.attempted = true;
  currentReport.enabled = isEnabled();
  currentReport.executeImplemented = executeImplemented();
  currentReport.sequencePrepared = false;
  currentReport.sequenceStarted = false;
  currentReport.state = GuardedRoutineExecutorState::TIMED_OUT;
  currentReport.result = GuardedRoutineExecutorResult::TIMED_OUT;
  strlcpy(currentReport.detail, "routine executor timeout",
          sizeof(currentReport.detail));

  currentStatus.state = GuardedRoutineExecutorState::TIMED_OUT;
  currentStatus.lastResult = currentReport.result;
  strlcpy(currentStatus.lastDetail, currentReport.detail,
          sizeof(currentStatus.lastDetail));
  refreshTiming(currentStatus);
}

GuardedRoutineExecutorStatus GuardedRoutineExecutor::status() {
  GuardedRoutineExecutorStatus snapshot = currentStatus;
  refreshTiming(snapshot);
  return snapshot;
}

GuardedRoutineExecutorReport GuardedRoutineExecutor::lastReport() {
  return currentReport;
}

void GuardedRoutineExecutor::appendPolicyJson(String& json) {
  GuardedRoutineExecutorReport report;
  json += "\"executor\":{";
  json += "\"enabled\":";
  json += report.enabled ? "true" : "false";
  json += ",\"executeImplemented\":";
  json += report.executeImplemented ? "true" : "false";
  json += ",\"mode\":\"skeleton_disabled_by_default\"";
  json += ",\"abortSupported\":true";
  json += ",\"timeoutSupported\":true";
  json += ",";
  appendStatusJson(json);
  json += "}";
}

void GuardedRoutineExecutor::appendStatusJson(String& json) {
  const GuardedRoutineExecutorStatus snapshot = status();
  json += "\"status\":{";
  json += "\"state\":\"";
  json += stateToString(snapshot.state);
  json += "\",\"routineName\":\"";
  appendEscaped(json, snapshot.routineName);
  json += "\",\"currentStep\":";
  json += String(snapshot.currentStep);
  json += ",\"totalSteps\":";
  json += String(snapshot.totalSteps);
  json += ",\"motionStepCount\":";
  json += String(snapshot.motionStepCount);
  json += ",\"startedAtMs\":";
  json += String(snapshot.startedAtMs);
  json += ",\"deadlineMs\":";
  json += String(snapshot.deadlineMs);
  json += ",\"elapsedMs\":";
  json += String(snapshot.elapsedMs);
  json += ",\"remainingMs\":";
  json += String(snapshot.remainingMs);
  json += ",\"lastResult\":\"";
  json += resultToString(snapshot.lastResult);
  json += "\",\"lastDetail\":\"";
  appendEscaped(json, snapshot.lastDetail);
  json += "\"}";
}

void GuardedRoutineExecutor::appendReportJson(
    String& json,
    const GuardedRoutineExecutorReport& report) {
  json += "\"executor\":{";
  json += "\"attempted\":";
  json += report.attempted ? "true" : "false";
  json += ",\"enabled\":";
  json += report.enabled ? "true" : "false";
  json += ",\"executeImplemented\":";
  json += report.executeImplemented ? "true" : "false";
  json += ",\"sequencePrepared\":";
  json += report.sequencePrepared ? "true" : "false";
  json += ",\"sequenceStarted\":";
  json += report.sequenceStarted ? "true" : "false";
  json += ",\"state\":\"";
  json += stateToString(report.state);
  json += "\"";
  json += ",\"motionStepCount\":";
  json += String(report.motionStepCount);
  json += ",\"preparedSequence\":{";
  json += "\"attempted\":";
  json += report.prepareAttempted ? "true" : "false";
  json += ",\"candidateReady\":";
  json += report.prepareReady ? "true" : "false";
  json += ",\"appliedToMotionSequence\":";
  json += report.preparedSequenceApplied ? "true" : "false";
  json += ",\"preparedStepCount\":";
  json += String(report.preparedStepCount);
  json += ",\"preparedMotionCount\":";
  json += String(report.preparedMotionCount);
  json += ",\"prepareResult\":\"";
  json += prepareResultToString(report.prepareResult);
  json += "\",\"detail\":\"";
  appendEscaped(json, report.prepareDetail);
  json += "\",\"entries\":[";
  for (uint8_t i = 0; i < report.preparedStepCount; ++i) {
    const GuardedRoutinePreparedStep& entry = report.preparedSteps[i];
    if (i > 0) {
      json += ",";
    }
    json += "{\"sourceIndex\":";
    json += String(entry.sourceIndex);
    json += ",\"sourceId\":\"";
    appendEscaped(json, entry.sourceStepId);
    json += "\",\"joint\":\"";
    json += GuardedRoutine::jointToString(entry.joint);
    json += "\",\"direction\":\"";
    json += GuardedRoutine::directionToString(entry.direction);
    json += "\",\"percent\":";
    json += String(entry.percent);
    json += ",\"durationMs\":";
    json += String(entry.durationMs);
    json += ",\"targetDegrees\":";
    json += String(entry.targetDegrees, 1);
    json += ",\"stopAfterStepRequired\":";
    json += entry.stopAfterStepRequired ? "true" : "false";
    json += ",\"statusCheckRequired\":";
    json += entry.statusCheckRequired ? "true" : "false";
    json += "}";
  }
  json += "]}";
  json += ",\"stepJournal\":{";
  json += "\"count\":";
  json += String(report.stepJournalCount);
  json += ",\"truncated\":";
  json += report.stepJournalTruncated ? "true" : "false";
  json += ",\"entries\":[";
  for (uint8_t i = 0; i < report.stepJournalCount; ++i) {
    const GuardedRoutineStepJournalEntry& entry = report.stepJournal[i];
    if (i > 0) {
      json += ",";
    }
    json += "{\"index\":";
    json += String(entry.index);
    json += ",\"id\":\"";
    appendEscaped(json, entry.stepId);
    json += "\",\"kind\":\"";
    json += GuardedRoutine::stepKindToString(entry.kind);
    json += "\",\"result\":\"";
    json += stepResultToString(entry.result);
    json += "\",\"detail\":\"";
    appendEscaped(json, entry.detail);
    json += "\"}";
  }
  json += "]}";
  json += ",\"result\":\"";
  json += resultToString(report.result);
  json += "\",\"detail\":\"";
  appendEscaped(json, report.detail);
  json += "\"}";
}

const char* GuardedRoutineExecutor::resultToString(
    GuardedRoutineExecutorResult result) {
  switch (result) {
    case GuardedRoutineExecutorResult::NOT_REQUESTED:             return "not_requested";
    case GuardedRoutineExecutorResult::EXECUTOR_DISABLED:         return "disabled";
    case GuardedRoutineExecutorResult::EXECUTOR_NOT_IMPLEMENTED:  return "not_implemented";
    case GuardedRoutineExecutorResult::NO_ACTIVE_ROUTINE:         return "no_active_routine";
    case GuardedRoutineExecutorResult::ABORTED:                   return "aborted";
    case GuardedRoutineExecutorResult::TIMED_OUT:                 return "timed_out";
    default:                                                      return "unknown";
  }
}

const char* GuardedRoutineExecutor::stateToString(
    GuardedRoutineExecutorState state) {
  switch (state) {
    case GuardedRoutineExecutorState::IDLE:             return "idle";
    case GuardedRoutineExecutorState::PREPARED:         return "prepared";
    case GuardedRoutineExecutorState::RUNNING:          return "running";
    case GuardedRoutineExecutorState::ABORT_REQUESTED:  return "abort_requested";
    case GuardedRoutineExecutorState::ABORTED:          return "aborted";
    case GuardedRoutineExecutorState::TIMED_OUT:        return "timed_out";
    case GuardedRoutineExecutorState::COMPLETED:        return "completed";
    case GuardedRoutineExecutorState::BLOCKED:          return "blocked";
    default:                                            return "unknown";
  }
}

const char* GuardedRoutineExecutor::stepResultToString(
    GuardedRoutineStepResult result) {
  switch (result) {
    case GuardedRoutineStepResult::PENDING:  return "pending";
    case GuardedRoutineStepResult::SKIPPED:  return "skipped";
    case GuardedRoutineStepResult::BLOCKED:  return "blocked";
    default:                                 return "unknown";
  }
}

const char* GuardedRoutineExecutor::prepareResultToString(
    GuardedRoutinePrepareResult result) {
  switch (result) {
    case GuardedRoutinePrepareResult::PREPARE_NOT_REQUESTED:  return "not_requested";
    case GuardedRoutinePrepareResult::PREPARE_READY:          return "ready";
    case GuardedRoutinePrepareResult::PREPARE_TOO_MANY_STEPS: return "too_many_steps";
    case GuardedRoutinePrepareResult::PREPARE_INVALID_STEP:   return "invalid_step";
    default:                                                  return "unknown";
  }
}

void GuardedRoutineExecutor::buildPreparedSequence(
    const GuardedRoutinePlan& plan,
    GuardedRoutineExecutorReport& report) {
  report.prepareAttempted = true;
  report.prepareReady = false;
  report.preparedSequenceApplied = false;
  report.preparedStepCount = 0;
  report.preparedMotionCount = 0;
  report.prepareResult = GuardedRoutinePrepareResult::PREPARE_NOT_REQUESTED;
  strlcpy(report.prepareDetail, "prepare not requested", sizeof(report.prepareDetail));

  const uint8_t motionCount = countMotionSteps(plan);
  report.preparedMotionCount = motionCount;

  if (motionCount > GuardedRoutineExecutorReport::MAX_PREPARED_STEPS) {
    report.prepareResult = GuardedRoutinePrepareResult::PREPARE_TOO_MANY_STEPS;
    strlcpy(report.prepareDetail, "too many motion steps for sequence candidate",
            sizeof(report.prepareDetail));
    return;
  }

  for (uint8_t i = 0; i < plan.stepCount; ++i) {
    const GuardedRoutineStep& step = plan.steps[i];
    if (step.kind != GuardedRoutineStepKind::MOTION) {
      continue;
    }

    if (!hasValidMotionParameters(step)) {
      report.prepareResult = GuardedRoutinePrepareResult::PREPARE_INVALID_STEP;
      snprintf(report.prepareDetail, sizeof(report.prepareDetail),
               "invalid motion step %u", i + 1);
      return;
    }

    if (report.preparedStepCount >= GuardedRoutineExecutorReport::MAX_PREPARED_STEPS) {
      report.prepareResult = GuardedRoutinePrepareResult::PREPARE_TOO_MANY_STEPS;
      strlcpy(report.prepareDetail, "prepared sequence candidate overflow",
              sizeof(report.prepareDetail));
      return;
    }

    GuardedRoutinePreparedStep& prepared = report.preparedSteps[report.preparedStepCount];
    prepared.sourceIndex = i + 1;
    strlcpy(prepared.sourceStepId, step.kindId != nullptr ? step.kindId : "",
            sizeof(prepared.sourceStepId));
    prepared.joint = step.joint;
    prepared.direction = step.direction;
    prepared.percent = step.percent;
    prepared.durationMs = step.durationMs;
    prepared.targetDegrees = step.targetDegrees;
    prepared.stopAfterStepRequired = plan.stopAfterEachMotionStep;
    prepared.statusCheckRequired = plan.statusCheckAfterEachStep;
    report.preparedStepCount++;
  }

  report.prepareReady = true;
  report.prepareResult = GuardedRoutinePrepareResult::PREPARE_READY;
  strlcpy(report.prepareDetail, "sequence candidate ready; not applied to MotionSequence",
          sizeof(report.prepareDetail));
}

void GuardedRoutineExecutor::buildStepJournal(
    const GuardedRoutinePlan& plan,
    GuardedRoutineExecutorReport& report) {
  report.stepJournalCount = 0;
  report.stepJournalTruncated = plan.stepCount > GuardedRoutineExecutorReport::MAX_STEP_JOURNAL;

  const uint8_t journalCount =
    plan.stepCount < GuardedRoutineExecutorReport::MAX_STEP_JOURNAL
      ? plan.stepCount
      : GuardedRoutineExecutorReport::MAX_STEP_JOURNAL;

  for (uint8_t i = 0; i < journalCount; ++i) {
    const GuardedRoutineStep& step = plan.steps[i];
    GuardedRoutineStepJournalEntry& entry = report.stepJournal[i];
    entry.index = i + 1;
    strlcpy(entry.stepId, step.kindId != nullptr ? step.kindId : "",
            sizeof(entry.stepId));
    entry.kind = step.kind;

    if (step.kind == GuardedRoutineStepKind::MOTION) {
      if (!report.attempted) {
        entry.result = GuardedRoutineStepResult::SKIPPED;
        strlcpy(entry.detail, "motion step skipped before executor",
                sizeof(entry.detail));
      } else {
        entry.result = GuardedRoutineStepResult::BLOCKED;
        strlcpy(entry.detail, "motion step blocked by executor policy",
                sizeof(entry.detail));
      }
    } else if (report.attempted && report.result != GuardedRoutineExecutorResult::NOT_REQUESTED) {
      entry.result = GuardedRoutineStepResult::PENDING;
      strlcpy(entry.detail, "non-motion step pending executor implementation",
              sizeof(entry.detail));
    } else {
      entry.result = GuardedRoutineStepResult::PENDING;
      strlcpy(entry.detail, "non-motion step pending",
              sizeof(entry.detail));
    }
  }

  report.stepJournalCount = journalCount;
}

uint8_t GuardedRoutineExecutor::countMotionSteps(const GuardedRoutinePlan& plan) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < plan.stepCount; ++i) {
    if (plan.steps[i].kind == GuardedRoutineStepKind::MOTION) {
      count++;
    }
  }
  return count;
}
