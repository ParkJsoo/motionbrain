#include "control/guarded_routine_executor.h"

#include <stdio.h>
#include <string.h>

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

GuardedRoutineExecutorReport::GuardedRoutineExecutorReport()
  : attempted(false)
  , enabled(GuardedRoutineExecutor::isEnabled())
  , executeImplemented(GuardedRoutineExecutor::executeImplemented())
  , sequencePrepared(false)
  , sequenceStarted(false)
  , state(GuardedRoutineExecutorState::IDLE)
  , motionStepCount(0)
  , result(GuardedRoutineExecutorResult::NOT_REQUESTED)
  , detail{0} {
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

uint8_t GuardedRoutineExecutor::countMotionSteps(const GuardedRoutinePlan& plan) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < plan.stepCount; ++i) {
    if (plan.steps[i].kind == GuardedRoutineStepKind::MOTION) {
      count++;
    }
  }
  return count;
}
