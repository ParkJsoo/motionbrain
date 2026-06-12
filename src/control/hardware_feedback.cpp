#include "control/hardware_feedback.h"

#include "control/angle_controller.h"

namespace {

constexpr const char* SELECTED_CLOSURE_TARGET = "base_yaw_reference";
constexpr const char* ROUTINE_BLOCK_REASON = "feedback_required";
constexpr const char* ROUTINE_BLOCK_EVENT = "ROUTINE_FEEDBACK_BLOCK";

void appendBool(String& json, const char* key, bool value) {
  json += "\"";
  json += key;
  json += "\":";
  json += value ? "true" : "false";
}

} // namespace

const char* HardwareFeedback::selectedClosureTarget() {
  return SELECTED_CLOSURE_TARGET;
}

BaseYawReferenceFeedback HardwareFeedback::baseYawReferenceStatus(
    const AngleController* angleController) {
  (void)angleController;

  BaseYawReferenceFeedback status = {
    false,
    false,
    false,
    false,
    false,
    true,
    false,
    false,
    0,
    0,
    0.0f,
    0.0f,
    HardwareFeedbackFault::NOT_INSTALLED,
    ROUTINE_BLOCK_REASON,
    "NOT_INSTALLED",
    "base_yaw_reference feedback not installed; physical routine execution disabled"
  };
  return status;
}

bool HardwareFeedback::baseYawReferenceReadyForRoutineExecution(
    const AngleController* angleController) {
  return baseYawReferenceStatus(angleController).readyForRoutineExecution;
}

const char* HardwareFeedback::faultToString(HardwareFeedbackFault fault) {
  switch (fault) {
    case HardwareFeedbackFault::READY:         return "ready";
    case HardwareFeedbackFault::NOT_INSTALLED: return "not_installed";
    case HardwareFeedbackFault::DISCONNECTED:  return "disconnected";
    case HardwareFeedbackFault::STALE:         return "stale";
    case HardwareFeedbackFault::UNREFERENCED:  return "unreferenced";
    case HardwareFeedbackFault::FAULTED:       return "faulted";
    case HardwareFeedbackFault::NO_PROGRESS:   return "no_progress";
    case HardwareFeedbackFault::TIMEOUT:       return "timeout";
    case HardwareFeedbackFault::OVERSHOOT:     return "overshoot";
    default:                                   return "unknown";
  }
}

const char* HardwareFeedback::routineBlockReason() {
  return ROUTINE_BLOCK_REASON;
}

const char* HardwareFeedback::routineBlockEventCode() {
  return ROUTINE_BLOCK_EVENT;
}

void HardwareFeedback::appendStatusJson(String& json,
                                        const AngleController* angleController) {
  const BaseYawReferenceFeedback status = baseYawReferenceStatus(angleController);

  json += "\"feedback\":{";
  json += "\"schemaVersion\":\"feedback.v0\"";
  json += ",\"selectedClosureTarget\":\"";
  json += SELECTED_CLOSURE_TARGET;
  json += "\"";
  json += ",";
  appendBool(json, "physicalRoutineExecutionAllowed",
             status.physicalRoutineExecutionAllowed);
  json += ",";
  appendBool(json, "readyForRoutineExecution", status.readyForRoutineExecution);
  json += ",\"blockReason\":\"";
  json += status.blockReason;
  json += "\",\"detail\":\"";
  json += status.detail;
  json += "\",\"baseYaw\":{";
  appendBool(json, "installed", status.installed);
  json += ",";
  appendBool(json, "available", status.available);
  json += ",";
  appendBool(json, "connected", status.connected);
  json += ",";
  appendBool(json, "fresh", status.fresh);
  json += ",";
  appendBool(json, "referenced", status.referenced);
  json += ",";
  appendBool(json, "faulted", status.faulted);
  json += ",";
  appendBool(json, "readyForRoutineExecution",
             status.readyForRoutineExecution);
  json += ",\"ageMs\":";
  json += String(status.ageMs);
  json += ",\"lastUpdateMs\":";
  json += String(status.lastUpdateMs);
  json += ",\"positionDeg\":";
  json += String(status.positionDeg, 1);
  json += ",\"velocityDps\":";
  json += String(status.velocityDps, 2);
  json += ",\"lastStopReason\":\"";
  json += status.lastStopReason;
  json += "\",\"fault\":\"";
  json += faultToString(status.fault);
  json += "\"}}";
}
