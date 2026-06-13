#include "control/hardware_feedback.h"

#include "control/angle_controller.h"

namespace {

constexpr const char* SELECTED_CLOSURE_TARGET = "base_yaw_reference";
constexpr const char* ROUTINE_BLOCK_REASON = "feedback_required";
constexpr const char* ROUTINE_BLOCK_EVENT = "ROUTINE_FEEDBACK_BLOCK";
constexpr uint8_t BASE_YAW_REFERENCE_PIN =
  static_cast<uint8_t>(MOTIONBRAIN_BASE_YAW_REFERENCE_PIN);
constexpr bool BASE_YAW_REFERENCE_ACTIVE_LOW =
  MOTIONBRAIN_BASE_YAW_REFERENCE_ACTIVE_LOW != 0;
constexpr bool BASE_YAW_REFERENCE_ENABLED =
  MOTIONBRAIN_BASE_YAW_REFERENCE_ENABLED != 0;
constexpr bool PHYSICAL_ROUTINE_ALLOWED =
  MOTIONBRAIN_BASE_YAW_REFERENCE_PHYSICAL_ROUTINE_ALLOWED != 0;
constexpr uint32_t BASE_YAW_REFERENCE_DEBOUNCE_MS =
  MOTIONBRAIN_BASE_YAW_REFERENCE_DEBOUNCE_MS;
constexpr uint32_t BASE_YAW_REFERENCE_STALE_MS =
  MOTIONBRAIN_BASE_YAW_REFERENCE_STALE_MS;

struct BaseYawReferenceAdapterState {
  bool initialized;
  bool hasSample;
  bool rawActive;
  bool stableActive;
  bool referenced;
  uint32_t rawChangedAtMs;
  uint32_t stableChangedAtMs;
  uint32_t lastSampleMs;
};

BaseYawReferenceAdapterState baseYawReferenceState = {
  false,
  false,
  false,
  false,
  false,
  0,
  0,
  0
};

void appendBool(String& json, const char* key, bool value) {
  json += "\"";
  json += key;
  json += "\":";
  json += value ? "true" : "false";
}

bool readBaseYawReferenceActive() {
  const int raw = digitalRead(BASE_YAW_REFERENCE_PIN);
  return BASE_YAW_REFERENCE_ACTIVE_LOW ? raw == LOW : raw == HIGH;
}

BaseYawReferenceFeedback notInstalledStatus() {
  BaseYawReferenceFeedback status = {
    false,
    false,
    false,
    false,
    false,
    true,
    false,
    false,
    false,
    false,
    BASE_YAW_REFERENCE_PIN,
    BASE_YAW_REFERENCE_ACTIVE_LOW,
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

} // namespace

void HardwareFeedback::initBaseYawReference() {
  if (!BASE_YAW_REFERENCE_ENABLED) {
    return;
  }

  pinMode(BASE_YAW_REFERENCE_PIN, INPUT);
  const uint32_t now = millis();
  const bool active = readBaseYawReferenceActive();

  baseYawReferenceState.initialized = true;
  baseYawReferenceState.hasSample = true;
  baseYawReferenceState.rawActive = active;
  baseYawReferenceState.stableActive = active;
  baseYawReferenceState.referenced = active;
  baseYawReferenceState.rawChangedAtMs = now;
  baseYawReferenceState.stableChangedAtMs = now;
  baseYawReferenceState.lastSampleMs = now;
}

void HardwareFeedback::updateBaseYawReference() {
  if (!BASE_YAW_REFERENCE_ENABLED) {
    return;
  }

  if (!baseYawReferenceState.initialized) {
    initBaseYawReference();
  }

  const uint32_t now = millis();
  const bool active = readBaseYawReferenceActive();

  if (!baseYawReferenceState.hasSample ||
      active != baseYawReferenceState.rawActive) {
    baseYawReferenceState.rawActive = active;
    baseYawReferenceState.rawChangedAtMs = now;
  }

  if (active != baseYawReferenceState.stableActive &&
      (now - baseYawReferenceState.rawChangedAtMs) >=
        BASE_YAW_REFERENCE_DEBOUNCE_MS) {
    baseYawReferenceState.stableActive = active;
    baseYawReferenceState.stableChangedAtMs = now;
    if (active) {
      baseYawReferenceState.referenced = true;
    }
  }

  baseYawReferenceState.hasSample = true;
  baseYawReferenceState.lastSampleMs = now;
}

const char* HardwareFeedback::selectedClosureTarget() {
  return SELECTED_CLOSURE_TARGET;
}

BaseYawReferenceFeedback HardwareFeedback::baseYawReferenceStatus(
    const AngleController* angleController) {
  if (!BASE_YAW_REFERENCE_ENABLED) {
    return notInstalledStatus();
  }

  const uint32_t now = millis();
  const bool connected = baseYawReferenceState.initialized &&
                         baseYawReferenceState.hasSample;
  const uint32_t ageMs = connected ? now - baseYawReferenceState.lastSampleMs : 0;
  const bool fresh = connected && ageMs <= BASE_YAW_REFERENCE_STALE_MS;
  const bool hardwareReady = connected && fresh && baseYawReferenceState.referenced;
  HardwareFeedbackFault fault = HardwareFeedbackFault::READY;
  const char* lastStopReason = "READY";
  const char* detail = "base_yaw_reference feedback ready; physical routine execution disabled";

  if (!connected) {
    fault = HardwareFeedbackFault::DISCONNECTED;
    lastStopReason = "DISCONNECTED";
    detail = "base_yaw_reference feedback configured but no sample is available";
  } else if (!fresh) {
    fault = HardwareFeedbackFault::STALE;
    lastStopReason = "STALE";
    detail = "base_yaw_reference feedback sample is stale";
  } else if (!baseYawReferenceState.referenced) {
    fault = HardwareFeedbackFault::UNREFERENCED;
    lastStopReason = "UNREFERENCED";
    detail = "base_yaw_reference index has not been observed since boot";
  }

  BaseYawReferenceFeedback status = {
    true,
    connected && fresh,
    connected,
    fresh,
    baseYawReferenceState.referenced,
    fault != HardwareFeedbackFault::READY,
    hardwareReady,
    hardwareReady && PHYSICAL_ROUTINE_ALLOWED,
    PHYSICAL_ROUTINE_ALLOWED,
    connected ? baseYawReferenceState.stableActive : false,
    BASE_YAW_REFERENCE_PIN,
    BASE_YAW_REFERENCE_ACTIVE_LOW,
    ageMs,
    connected ? baseYawReferenceState.lastSampleMs : 0,
    0.0f,
    angleController != nullptr ? angleController->getLastRateDegreesPerSecond() : 0.0f,
    fault,
    ROUTINE_BLOCK_REASON,
    lastStopReason,
    detail
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
  appendBool(json, "hardwareReady", status.hardwareReady);
  json += ",";
  appendBool(json, "readyForRoutineExecution",
             status.readyForRoutineExecution);
  json += ",";
  appendBool(json, "signalActive", status.signalActive);
  json += ",\"pin\":";
  json += String(status.pin);
  json += ",";
  appendBool(json, "activeLow", status.activeLow);
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
