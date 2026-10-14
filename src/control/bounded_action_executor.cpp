#include "bounded_action_executor.h"

#include <math.h>
#include <string.h>

namespace bounded {
namespace {
// Printable ASCII IDs avoid JSON escaping, whitespace aliases and unterminated
// fixed-buffer reads. The transport must reject rather than truncate long IDs.
bool validId(const char* value, size_t capacity) {
  if (!value || !value[0]) return false;
  for (size_t i = 0; i < capacity; ++i) {
    const unsigned char ch = static_cast<unsigned char>(value[i]);
    if (ch == 0) return true;
    if (!((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
          (ch >= '0' && ch <= '9') || ch == '-' || ch == '_' || ch == '.' || ch == ':')) return false;
  }
  return false;
}
}

Executor::Executor(Port& port, const char* bootId)
    : port_(port), bootId_{}, active_(nullptr), initialized_(validId(bootId, sizeof(bootId_))) {
  if (initialized_) strcpy(bootId_, bootId);
}

bool Executor::validRequest(const Request& r) {
  if (!validId(r.bootId, sizeof(r.bootId)) || !validId(r.commandId, sizeof(r.commandId)) ||
      r.durationMs == 0 || r.durationMs > MAX_DURATION_MS || !isfinite(r.targetDegrees)) return false;
  if (r.kind == Kind::Pulse)
    return r.motor >= 1 && r.motor <= 5 && r.motor != 4 && r.pwm != 0 &&
           r.pwm >= -255 && r.pwm <= 255 && r.targetDegrees == 0 && r.percent == 0;
  if (r.kind == Kind::Shoulder)
    return r.motor == 4 && r.pwm == 0 && r.percent >= 1 && r.percent <= 100;
  return false;
}

bool Executor::samePayload(const Request& a, const Request& b) {
  return a.kind == b.kind && a.motor == b.motor && a.pwm == b.pwm &&
         a.durationMs == b.durationMs && a.targetDegrees == b.targetDegrees && a.percent == b.percent;
}

SubmitResult Executor::submit(const Request& request, uint32_t now) {
  if (!initialized_ || !validId(request.bootId, sizeof(request.bootId)) ||
      !validId(request.commandId, sizeof(request.commandId))) return {SubmitCode::Invalid, nullptr};
  if (strcmp(request.bootId, bootId_) != 0) return {SubmitCode::BootMismatch, nullptr};
  Record* existing = registry_.find(request.commandId);
  if (existing) return {samePayload(existing->request, request) ? SubmitCode::Duplicate : SubmitCode::Conflict, existing};
  if (!validRequest(request)) return {SubmitCode::Invalid, nullptr};
  if (active_ || !port_.idle()) return {SubmitCode::Busy, nullptr};
  if (!port_.allowed()) return {SubmitCode::SafetyBlocked, nullptr};
  // Reserve the ID before invoking any output-capable operation.
  Record* record = registry_.reserve(request, now);
  if (!record) return {SubmitCode::RegistryFull, nullptr};
  active_ = record;
  if (!port_.start(request)) finish(now, State::Failed, Reason::StartFailed);
  return {SubmitCode::Accepted, record};
}

void Executor::finish(uint32_t now, State state, Reason reason, bool targetReached) {
  if (!active_) return;
  Record& record = *active_;
  port_.stop(record.request, reason);
  record.endedAtMs = port_.afterStopTime(now);
  record.elapsedMs = record.endedAtMs - record.startedAtMs;
  record.overrunMs = record.elapsedMs > record.request.durationMs ? record.elapsedMs - record.request.durationMs : 0;
  record.outputOff = port_.outputIsOff(record.request);
  record.targetReached = targetReached;
  if (state == State::Completed && record.overrunMs > 0) {
    state = State::TimedOut;
    reason = Reason::DeadlineExceeded;
  }
  record.reason = reason;
  record.state = record.outputOff ? state : State::OutcomeUnknown;
  // Unconfirmed stop latches the boot closed even if a retry later confirms off.
  // This cannot retroactively establish the original action outcome.
  if (record.outputOff && record.state != State::OutcomeUnknown) active_ = nullptr;
}

void Executor::tick(uint32_t now) {
  if (!active_) return;
  if (active_->state == State::OutcomeUnknown) {
    if (active_->outputOff) return;
    const Reason originalReason = active_->reason;
    const bool originalTargetReached = active_->targetReached;
    finish(now, State::OutcomeUnknown, originalReason, originalTargetReached);
    return;
  }
  active_->elapsedMs = now - active_->startedAtMs;
  // Deadline precedes all controller work. The integrator must call tick before
  // its shoulder-controller update and motor ramp update on every loop.
  if (active_->elapsedMs >= active_->request.durationMs) {
    const bool pulse = active_->request.kind == Kind::Pulse;
    finish(now, pulse ? State::Completed : State::TimedOut,
           pulse ? Reason::DurationElapsed : Reason::DeadlineExceeded);
    return;
  }
  if (!port_.allowed()) {
    finish(now, State::Failed, Reason::SafetyBlocked);
    return;
  }
  if (active_->request.kind == Kind::Shoulder) {
    const ShoulderResult result = port_.shoulderResult();
    if (result == ShoulderResult::Reached) finish(now, State::Completed, Reason::TargetReached, true);
    else if (result == ShoulderResult::Failed) finish(now, State::Failed, Reason::ControllerFailed);
  }
}

void Executor::cancel(uint32_t now, Reason reason) {
  if (!active_) return;
  const bool unknown = active_->state == State::OutcomeUnknown;
  finish(now, unknown ? State::OutcomeUnknown : State::Cancelled,
         unknown ? active_->reason : reason, unknown && active_->targetReached);
}

LookupResult Executor::find(const char* bootId, const char* commandId) const {
  if (!initialized_ || !validId(bootId, sizeof(bootId_)) || !validId(commandId, sizeof(Request().commandId)))
    return {LookupCode::Invalid, nullptr};
  if (strcmp(bootId, bootId_) != 0) return {LookupCode::BootMismatch, nullptr};
  for (size_t i = 0; i < registry_.count(); ++i) {
    const Record* record = registry_.at(i);
    if (strcmp(record->request.commandId, commandId) == 0) return {LookupCode::Found, record};
  }
  return {LookupCode::NotFound, nullptr};
}

const char* toString(Kind value) {
  switch (value) { case Kind::Pulse: return "pulse"; case Kind::Shoulder: return "shoulder"; }
  return "invalid";
}
const char* toString(State value) {
  switch (value) {
    case State::Running: return "running";
    case State::Completed: return "completed";
    case State::TimedOut: return "timed_out";
    case State::Cancelled: return "cancelled";
    case State::Failed: return "failed";
    case State::OutcomeUnknown: return "outcome_unknown";
  }
  return "invalid";
}
const char* toString(Reason value) {
  switch (value) {
    case Reason::None: return "none";
    case Reason::DurationElapsed: return "duration_elapsed";
    case Reason::DeadlineExceeded: return "deadline_exceeded";
    case Reason::TargetReached: return "target_reached";
    case Reason::ControllerFailed: return "controller_failed";
    case Reason::Cancelled: return "cancelled";
    case Reason::SafetyBlocked: return "safety_blocked";
    case Reason::StartFailed: return "start_failed";
    case Reason::OutputUnconfirmed: return "output_unconfirmed";
    case Reason::OperatorOverride: return "operator_override";
    case Reason::StateChanged: return "state_changed";
  }
  return "invalid";
}
const char* toString(SubmitCode value) {
  switch (value) {
    case SubmitCode::Accepted: return "accepted";
    case SubmitCode::Duplicate: return "duplicate";
    case SubmitCode::Conflict: return "conflict";
    case SubmitCode::Invalid: return "invalid";
    case SubmitCode::BootMismatch: return "boot_mismatch";
    case SubmitCode::Busy: return "busy";
    case SubmitCode::RegistryFull: return "registry_full";
    case SubmitCode::SafetyBlocked: return "safety_blocked";
  }
  return "invalid";
}
const char* toString(LookupCode value) {
  switch (value) {
    case LookupCode::Found: return "found";
    case LookupCode::NotFound: return "not_found";
    case LookupCode::BootMismatch: return "boot_mismatch";
    case LookupCode::Invalid: return "invalid";
  }
  return "invalid";
}
} // namespace bounded
