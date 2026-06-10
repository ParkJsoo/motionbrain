#include "control/guarded_routine_executor.h"

#include <stdio.h>
#include <string.h>

namespace {

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

} // namespace

GuardedRoutineExecutorReport::GuardedRoutineExecutorReport()
  : attempted(false)
  , enabled(GuardedRoutineExecutor::isEnabled())
  , executeImplemented(GuardedRoutineExecutor::executeImplemented())
  , sequencePrepared(false)
  , sequenceStarted(false)
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
    report.result = GuardedRoutineExecutorResult::NOT_REQUESTED;
    strlcpy(report.detail, "executor not requested", sizeof(report.detail));
  } else if (!report.enabled) {
    report.result = GuardedRoutineExecutorResult::EXECUTOR_DISABLED;
    strlcpy(report.detail, "routine executor disabled by firmware policy",
            sizeof(report.detail));
  } else {
    report.result = GuardedRoutineExecutorResult::EXECUTOR_NOT_IMPLEMENTED;
    strlcpy(report.detail, "routine physical executor is not implemented",
            sizeof(report.detail));
  }

  return report;
}

bool GuardedRoutineExecutor::begin(const GuardedRoutinePlan& plan,
                                   GuardedRoutineExecutorReport& report) {
  report = describe(plan, true);
  return report.sequenceStarted;
}

void GuardedRoutineExecutor::appendPolicyJson(String& json) {
  GuardedRoutineExecutorReport report;
  json += "\"executor\":{";
  json += "\"enabled\":";
  json += report.enabled ? "true" : "false";
  json += ",\"executeImplemented\":";
  json += report.executeImplemented ? "true" : "false";
  json += ",\"mode\":\"skeleton_disabled_by_default\"";
  json += "}";
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
    default:                                                      return "unknown";
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
