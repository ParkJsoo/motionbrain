#include "finite_profile_json.h"
#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_SCHEDULER
namespace finite_profile {
const char* codeName(Code code) {
  switch (code) {
#define FP_CODE(name) case Code::name: return #name;
    FP_CODE(Accepted) FP_CODE(Duplicate) FP_CODE(Conflict) FP_CODE(Invalid)
    FP_CODE(Disabled) FP_CODE(BootMismatch) FP_CODE(EpochMismatch) FP_CODE(Busy)
    FP_CODE(SafetyBlocked) FP_CODE(Capacity) FP_CODE(NotFound) FP_CODE(TokenMismatch)
    FP_CODE(Expired) FP_CODE(StaleEvent) FP_CODE(NotDue) FP_CODE(Rejected) FP_CODE(EpochExhausted)
#undef FP_CODE
  }
  return "Invalid";
}
static const char* stateName(State state) {
  switch (state) {
#define FP_STATE(name) case State::name: return #name;
    FP_STATE(Reserved) FP_STATE(Running) FP_STATE(Stopping) FP_STATE(Completed)
    FP_STATE(Cancelled) FP_STATE(Expired) FP_STATE(Failed) FP_STATE(OutcomeUnknown)
#undef FP_STATE
  }
  return "OutcomeUnknown";
}
static const char* reasonName(Reason reason) {
  switch (reason) {
#define FP_REASON(name) case Reason::name: return #name;
    FP_REASON(None) FP_REASON(NominalEnd) FP_REASON(Cancelled) FP_REASON(ReservationExpired)
    FP_REASON(SafetyChanged) FP_REASON(DeadlineMissed) FP_REASON(WriteFailed) FP_REASON(EventOrder)
    FP_REASON(OffUnconfirmed) FP_REASON(TokenFailure) FP_REASON(OffStateChanged) FP_REASON(ScheduleFailed)
#undef FP_REASON
  }
  return "Unknown";
}
void appendProfileJson(String& json, const Profile& p) {
  json += "{\"profileId\":\""; json += p.profileId;
  json += "\",\"catalogHash\":\""; json += p.catalogHash;
  json += "\",\"contractVersion\":"; json += p.contractVersion;
  json += ",\"profileVersion\":"; json += p.profileVersion;
  json += ",\"timingPolicyVersion\":"; json += p.timingPolicyVersion;
  json += ",\"motor\":"; json += p.motor;
  json += ",\"direction\":"; json += p.direction;
  json += ",\"startLeadMs\":"; json += p.startLeadMs;
  json += ",\"maxLatenessMs\":"; json += p.maxLatenessMs;
  json += ",\"offConfirmWindowMs\":"; json += p.offConfirmWindowMs;
  json += ",\"admissionWindowMs\":"; json += p.admissionWindowMs;
  json += ",\"points\":[";
  for (uint8_t i = 0; i < p.count; ++i) {
    if (i) json += ',';
    json += "{\"offsetMs\":"; json += p.points[i].offsetMs;
    json += ",\"pwm\":"; json += p.points[i].pwm; json += '}';
  }
  json += "]}";
}
void appendRecordJson(String& json, const Record& r) {
  json += "{\"bootId\":\""; json += r.request.bootId;
  json += "\",\"commandId\":\""; json += r.request.commandId;
  json += "\",\"requestEpoch\":"; json += r.request.expectedEpoch;
  json += ",\"state\":\""; json += stateName(r.state);
  json += "\",\"reason\":\""; json += reasonName(r.reason); json += '"';
#define FP_NUMBER(field) json += ",\"" #field "\":"; json += r.field;
  FP_NUMBER(preparedAt) FP_NUMBER(executeAcceptedAt) FP_NUMBER(plannedStartAt)
  FP_NUMBER(plannedOffAt) FP_NUMBER(firstNonzeroWriteAt) FP_NUMBER(offWriteAt)
  FP_NUMBER(lastOffWriteAt) FP_NUMBER(offConfirmedAt) FP_NUMBER(decisionAt)
  FP_NUMBER(quietEpoch) FP_NUMBER(offChecks) FP_NUMBER(nextEventIndex)
#undef FP_NUMBER
#define FP_BOOL(field) json += ",\"" #field "\":"; json += r.field ? "true" : "false";
  FP_BOOL(executed) FP_BOOL(anyNonzeroWrite) FP_BOOL(outputMayHaveOccurred)
  FP_BOOL(offWriteRecorded) FP_BOOL(offConfirmationRecorded) FP_BOOL(softwareOffConfirmed)
  FP_BOOL(timingConformant) FP_BOOL(traceComplete) FP_BOOL(quietHeld) FP_BOOL(offFailureLatched)
#undef FP_BOOL
  json += ",\"requestedProfile\":"; appendProfileJson(json, r.request.profile);
  json += ",\"plannedEventReceipts\":[";
  for (uint8_t i = 0; i < r.request.profile.count; ++i) {
    if (i) json += ',';
    json += "{\"index\":"; json += i;
    json += ",\"present\":"; json += r.writes[i].present ? "true" : "false";
    json += ",\"atMs\":"; json += r.writes[i].atMs;
    json += ",\"pwm\":"; json += r.writes[i].pwm; json += '}';
  }
  json += "]}";
}
void appendEvidenceJson(String& json, const DriverOutputEvidence* e) {
  if (!e) { json += "null"; return; }
  json += "{\"evidence\":\"driver_write\",\"clockDomain\":\"controller_boot_millis\",\"bootId\":\"";
  json += e->bootId; json += "\",\"commandId\":\""; json += e->commandId; json += '"';
#define FP_NUMBER(field) json += ",\"" #field "\":"; json += e->field;
  FP_NUMBER(requestEpoch) FP_NUMBER(lease) FP_NUMBER(ordinal) FP_NUMBER(openedAt)
  FP_NUMBER(closedAt) FP_NUMBER(totalWrites) FP_NUMBER(dropped)
#undef FP_NUMBER
  json += ",\"closed\":"; json += e->closed ? "true" : "false";
  json += ",\"captureComplete\":"; json += e->captureComplete ? "true" : "false";
  json += ",\"entries\":[";
  for (uint8_t i = 0; i < e->count; ++i) {
    if (i) json += ',';
    const auto& w = e->entries[i];
    json += "{\"sequence\":"; json += w.sequence;
    json += ",\"atMs\":"; json += w.atMs;
    json += ",\"motor\":"; json += w.motor;
    json += ",\"pwm\":"; json += w.pwm;
    json += ",\"reason\":\""; json += bounded::toString(w.reason); json += "\"}";
  }
  json += "]}";
}
}
#endif
