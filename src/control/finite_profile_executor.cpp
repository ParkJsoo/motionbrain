#include "finite_profile_executor.h"

#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_CORE
#include <limits.h>
#include <string.h>

namespace finite_profile {
namespace {
const uint32_t HALF_CLOCK = 0x80000000UL;

bool equalToken(const Token& a, const Token& b) {
  uint8_t difference = 0;
  for (size_t i = 0; i < sizeof(a.bytes); ++i) difference |= a.bytes[i] ^ b.bytes[i];
  return difference == 0;
}

bool nonzeroToken(const Token& token) {
  uint8_t combined = 0;
  for (size_t i = 0; i < sizeof(token.bytes); ++i) combined |= token.bytes[i];
  return combined != 0;
}

void increment(uint32_t& value) {
  if (value != UINT32_MAX) ++value;
}

bool terminal(State state) {
  return state != State::Reserved && state != State::Running && state != State::Stopping;
}

bool writeTimeValid(uint32_t before, uint32_t actual, uint32_t after) {
  const uint32_t span = after - before;
  return span < HALF_CLOCK && actual - before <= span;
}
}

Record::Record()
    : request(), state(State::Reserved), reason(Reason::None), preparedAt(0),
      executeAcceptedAt(0), plannedStartAt(0), plannedOffAt(0), firstNonzeroWriteAt(0),
      offWriteAt(0), lastOffWriteAt(0), offConfirmedAt(0), lastOffCheckAt(0),
      decisionAt(0), quietEpoch(0), offChecks(0),
      offAttempts(0), lastRetryAttemptAt(0), lastRetryWriteAt(0), lastRetryConfirmedAt(0),
      duplicateEvents(0), nextEventIndex(0), executed(false),
      anyNonzeroWrite(false), outputMayHaveOccurred(false), offWriteRecorded(false),
      offConfirmationRecorded(false), offCheckTimeRecorded(false), reservationOffInvalidated(false),
      softwareOffConfirmed(false), timingConformant(false), traceComplete(true),
      quietHeld(false), offFailureLatched(false), lastRetryWriteRecorded(false),
      lastRetryConfirmed(false), writes{} {}

Executor::Executor(Port& port, const char* bootId, const Profile* catalog, size_t catalogCount)
    : port_(port), bootId_{}, catalog_{}, catalogCount_(0), slots_{}, count_(0),
      owner_(nullptr), epoch_(1), initialized_(false), bootLatched_(false), epochExhausted_(false) {
  if (!validIdentifier(bootId, sizeof(bootId_)) || !catalog ||
      catalogCount == 0 || catalogCount > MAX_CATALOG) return;
  for (size_t i = 0; i < catalogCount; ++i) {
    if (!validProfile(catalog[i])) return;
    for (size_t j = 0; j < i; ++j) {
      if (strcmp(catalog[i].profileId, catalog[j].profileId) == 0) return;
    }
    catalog_[i] = catalog[i];
  }
  strcpy(bootId_, bootId);
  catalogCount_ = catalogCount;
  initialized_ = true;
}

bool Executor::validIdentity(const char* bootId, const char* commandId) const {
  return initialized_ && validIdentifier(bootId, sizeof(bootId_)) &&
      validIdentifier(commandId, sizeof(Request().commandId));
}

bool Executor::catalogContains(const Profile& profile) const {
  if (!validProfile(profile)) return false;
  for (size_t i = 0; i < catalogCount_; ++i) {
    if (sameProfile(profile, catalog_[i])) return true;
  }
  return false;
}

Executor::Slot* Executor::lookup(const char* commandId) {
  for (size_t i = 0; i < count_; ++i) {
    if (strcmp(slots_[i].record.request.commandId, commandId) == 0) return &slots_[i];
  }
  return nullptr;
}

const Record* Executor::find(const char* bootId, const char* commandId) const {
  if (!validIdentity(bootId, commandId) || strcmp(bootId_, bootId) != 0) return nullptr;
  for (size_t i = 0; i < count_; ++i) {
    if (strcmp(slots_[i].record.request.commandId, commandId) == 0) return &slots_[i].record;
  }
  return nullptr;
}

bool Executor::tokenAvailable(const Token& token) const {
  if (!nonzeroToken(token)) return false;
  for (size_t i = 0; i < count_; ++i) {
    if (slots_[i].tokenIssued && equalToken(token, slots_[i].token)) return false;
  }
  return true;
}

bool Executor::advanceEpoch() {
  if (epoch_ == UINT32_MAX) {
    epochExhausted_ = true;
    return false;
  }
  ++epoch_;
  return true;
}

PrepareResult Executor::prepare(const Request& request) {
  if (!validIdentity(request.bootId, request.commandId)) return PrepareResult(Code::Invalid);
  if (strcmp(bootId_, request.bootId) != 0) return PrepareResult(Code::BootMismatch);
  Slot* existing = lookup(request.commandId);
  if (existing) {
    const bool same = existing->record.request.expectedEpoch == request.expectedEpoch &&
        sameProfile(existing->record.request.profile, request.profile);
    return PrepareResult(same ? Code::Duplicate : Code::Conflict, &existing->record);
  }
  if (!catalogContains(request.profile)) return PrepareResult(Code::Invalid);
  if (epochExhausted_) return PrepareResult(Code::EpochExhausted);
  if (bootLatched_) return PrepareResult(Code::Rejected);
  if (request.expectedEpoch != epoch_) return PrepareResult(Code::EpochMismatch);
  if (owner_ || !port_.idle()) return PrepareResult(Code::Busy);
  if (!port_.allowed()) return PrepareResult(Code::SafetyBlocked);
  if (count_ == MAX_RECORDS) return PrepareResult(Code::Capacity);
  // Reserve ID and all per-command trace space before token generation.
  Slot& slot = slots_[count_++];
  slot.record.request = request;
  slot.record.preparedAt = port_.now();
  slot.record.softwareOffConfirmed = true; // Port::idle's documented zero-output invariant.
  owner_ = &slot;
  Token token;
  if (!port_.mintToken(token) || !tokenAvailable(token)) {
    finish(State::Failed, Reason::TokenFailure);
    return PrepareResult(Code::Rejected, &slot.record);
  }
  slot.token = token;
  slot.tokenIssued = true;
  PrepareResult result(Code::Accepted, &slot.record);
  result.granted = true;
  result.token = token;
  return result;
}

void Executor::expireReservation() {
  if (!owner_ || owner_->record.state != State::Reserved) return;
  const Record& record = owner_->record;
  if (port_.now() - record.preparedAt >= record.request.profile.admissionWindowMs)
    finish(State::Expired, Reason::ReservationExpired);
}

Result Executor::execute(const char* bootId, const char* commandId,
                         uint32_t expectedEpoch, const Token& token) {
  if (!validIdentity(bootId, commandId)) return Result(Code::Invalid);
  if (strcmp(bootId_, bootId) != 0) return Result(Code::BootMismatch);
  Slot* slot = lookup(commandId);
  if (!slot) return Result(Code::NotFound);
  Record& record = slot->record;
  if (expectedEpoch != record.request.expectedEpoch) return Result(Code::EpochMismatch, &record);
  if (record.state != State::Reserved) return Result(Code::Duplicate, &record);
  if (owner_ != slot || expectedEpoch != epoch_) return Result(Code::EpochMismatch, &record);
  expireReservation();
  if (record.state == State::Expired) return Result(Code::Expired, &record);
  if (blocked()) return Result(Code::Rejected, &record);
  if (!slot->tokenIssued || slot->tokenConsumed || !equalToken(token, slot->token))
    return Result(Code::TokenMismatch, &record);
  const bool allowed = port_.allowed();
  const bool idleNow = port_.idle();
  if (!idleNow) record.reservationOffInvalidated = true;
  if (!allowed || !idleNow) {
    finish(State::Failed, !idleNow ? Reason::OffStateChanged : Reason::SafetyChanged);
    return Result(Code::SafetyBlocked, &record);
  }
  const uint32_t acceptedAt = port_.now();
  // Port checks may consume time. Expiry is checked at acceptance, not only
  // before those checks, so a late token can never become a fresh schedule.
  if (acceptedAt - record.preparedAt >= record.request.profile.admissionWindowMs) {
    finish(State::Expired, Reason::ReservationExpired);
    return Result(Code::Expired, &record);
  }
  // Serialized acceptance fixes the origin; callbacks never reset it to actual output.
  slot->tokenConsumed = true;
  record.executed = true;
  record.executeAcceptedAt = acceptedAt;
  record.plannedStartAt = record.executeAcceptedAt + record.request.profile.startLeadMs;
  record.plannedOffAt = record.plannedStartAt +
      record.request.profile.points[record.request.profile.count - 1].offsetMs;
  record.state = State::Running;
  record.softwareOffConfirmed = false;
  return Result(Code::Accepted, &record);
}

Result Executor::event(const char* bootId, const char* commandId,
                       uint32_t expectedEpoch, uint8_t index) {
  if (!validIdentity(bootId, commandId)) return Result(Code::Invalid);
  if (strcmp(bootId_, bootId) != 0) return Result(Code::BootMismatch);
  Slot* slot = lookup(commandId);
  if (!slot) return Result(Code::NotFound);
  Record& record = slot->record;
  if (expectedEpoch != epoch_ || expectedEpoch != record.request.expectedEpoch)
    return Result(Code::EpochMismatch, &record);
  if (owner_ != slot || record.state != State::Running) return Result(Code::StaleEvent, &record);
  if (!port_.allowed()) {
    finish(State::Failed, Reason::SafetyChanged);
    return Result(Code::SafetyBlocked, &record);
  }
  if (index < record.nextEventIndex) {
    increment(record.duplicateEvents);
    return Result(Code::StaleEvent, &record);
  }
  if (index != record.nextEventIndex || index >= record.request.profile.count) {
    finish(State::Failed, Reason::EventOrder);
    return Result(Code::Rejected, &record);
  }
  const Profile& profile = record.request.profile;
  const Point& point = profile.points[index];
  const uint32_t due = profile.startLeadMs + point.offsetMs;
  const uint32_t before = port_.now();
  const uint32_t elapsed = before - record.executeAcceptedAt;
  if (elapsed < due) return Result(Code::NotDue, &record);
  if (elapsed > due + profile.maxLatenessMs) {
    finish(State::Failed, Reason::DeadlineMissed);
    return Result(Code::Rejected, &record);
  }
  uint32_t actual = before;
  // A failed adapter call may already have produced output. Never call it not-started.
  if (point.pwm != 0) record.outputMayHaveOccurred = true;
  const bool applied = port_.apply(profile.motor, point.pwm, actual);
  const uint32_t after = port_.now();
  if (!applied || !writeTimeValid(before, actual, after)) {
    record.traceComplete = false;
    finish(State::Failed, Reason::WriteFailed);
    return Result(Code::Rejected, &record);
  }
  Write& write = record.writes[index];
  write.present = true;
  write.atMs = actual;
  write.pwm = point.pwm;
  ++record.nextEventIndex;
  if (point.pwm != 0 && !record.anyNonzeroWrite) record.firstNonzeroWriteAt = actual;
  if (point.pwm != 0) record.anyNonzeroWrite = true;
  if (point.pwm == 0 && !record.offWriteRecorded) {
    record.offWriteRecorded = true;
    record.offWriteAt = actual;
    record.lastOffWriteAt = actual;
  }
  const uint32_t actualElapsed = actual - record.executeAcceptedAt;
  if (actualElapsed < due || actualElapsed > due + profile.maxLatenessMs) {
    finish(State::Failed, Reason::DeadlineMissed);
    return Result(Code::Rejected, &record);
  }
  if (record.nextEventIndex == profile.count) finish(State::Completed, Reason::NominalEnd);
  return Result(Code::Accepted, &record);
}

void Executor::latchUnknown() {
  if (!owner_) return;
  owner_->record.state = State::OutcomeUnknown;
  owner_->record.offFailureLatched = true;
  owner_->record.quietHeld = true;
  bootLatched_ = true;
}

bool Executor::probeOff(Proof& proof, uint32_t& checkedAt) {
  if (!owner_) return false;
  Record& record = owner_->record;
  const uint32_t before = port_.now();
  // OFF confirmation must follow every known OFF write and previous valid
  // probe. Short unsigned differences allow wrap, not backwards clock jumps.
  if (before - record.decisionAt >= HALF_CLOCK ||
      (record.offWriteRecorded && before - record.lastOffWriteAt >= HALF_CLOCK) ||
      (record.offCheckTimeRecorded && before - record.lastOffCheckAt >= HALF_CLOCK)) return false;
  proof = port_.offProof();
  checkedAt = port_.now();
  if (checkedAt - before >= HALF_CLOCK) return false;
  record.lastOffCheckAt = checkedAt;
  record.offCheckTimeRecorded = true;
  return true;
}

void Executor::confirmOff() {
  if (!owner_) return;
  Slot& slot = *owner_;
  Record& record = slot.record;
  if (record.state != State::Stopping && record.state != State::OutcomeUnknown) return;
  if (record.offChecks >= MAX_OFF_CHECKS ||
      port_.now() - record.decisionAt >= record.request.profile.offConfirmWindowMs) {
    latchUnknown();
    return;
  }
  increment(record.offChecks);
  Proof proof = Proof::Failed;
  uint32_t confirmedAt = port_.now();
  if (!probeOff(proof, confirmedAt)) {
    record.softwareOffConfirmed = false;
    record.traceComplete = false;
    latchUnknown();
    return;
  }
  if (confirmedAt - record.decisionAt >= record.request.profile.offConfirmWindowMs || proof == Proof::Failed) {
    latchUnknown();
    return;
  }
  if (proof == Proof::Pending) {
    if (record.offChecks == MAX_OFF_CHECKS) latchUnknown();
    return;
  }
  record.softwareOffConfirmed = true;
  if (!record.offConfirmationRecorded) {
    record.offConfirmedAt = confirmedAt;
    record.offConfirmationRecorded = true;
  }
  if (record.offFailureLatched) {
    record.state = State::OutcomeUnknown;
    return;
  }
  record.state = slot.intendedState;
  record.quietHeld = record.outputMayHaveOccurred;
  if (!record.quietHeld) owner_ = nullptr;
}

void Executor::finish(State state, Reason reason) {
  if (!owner_) return;
  Slot& slot = *owner_;
  Record& record = slot.record;
  if (record.state != State::Reserved && record.state != State::Running) return;
  const bool wasReserved = record.state == State::Reserved;
  if (wasReserved && (record.reservationOffInvalidated || !port_.idle())) {
    // A reservation never authorized another output. Preserve that violated
    // precondition as unknown, not a clean no-output cancellation/expiry.
    record.reservationOffInvalidated = true;
    record.outputMayHaveOccurred = true;
    record.softwareOffConfirmed = false;
    record.traceComplete = false;
    record.offFailureLatched = true;
    bootLatched_ = true;
    state = State::OutcomeUnknown;
    reason = Reason::OffStateChanged;
  }
  record.decisionAt = port_.now();
  record.reason = reason;
  record.timingConformant = state == State::Completed;
  slot.intendedState = state;
  slot.tokenConsumed = true;
  advanceEpoch(); // Invalidate every scheduled callback before touching the port.
  record.quietEpoch = epoch_;
  if (wasReserved && !record.reservationOffInvalidated) {
    record.state = state;
    record.quietHeld = false;
    owner_ = nullptr;
    return;
  }
  record.state = State::Stopping;
  record.quietHeld = true;
  record.softwareOffConfirmed = false;
  uint32_t writtenAt = port_.now();
  const uint32_t before = writtenAt;
  increment(record.offAttempts);
  const bool stopped = port_.stop(writtenAt);
  if (stopped && writeTimeValid(before, writtenAt, port_.now()) &&
      (!record.offWriteRecorded || writtenAt - record.lastOffWriteAt < HALF_CLOCK)) {
    record.lastOffWriteAt = writtenAt;
    if (!record.offWriteRecorded) {
      record.offWriteRecorded = true;
      record.offWriteAt = writtenAt;
    }
  } else {
    record.traceComplete = false;
    latchUnknown();
  }
  if (record.reservationOffInvalidated) {
    latchUnknown();
    return; // Only explicit follow-up may add a fresh OFF confirmation.
  }
  confirmOff();
}

void Executor::maintainNonRunning() {
  if (!owner_) return;
  if (owner_->record.state == State::Reserved) {
    if (!port_.idle()) {
      owner_->record.reservationOffInvalidated = true;
      finish(State::Failed, Reason::OffStateChanged);
    } else if (!port_.allowed()) finish(State::Failed, Reason::SafetyChanged);
    else expireReservation();
    return;
  }
  if (owner_->record.state == State::Stopping) confirmOff();
}

void Executor::tick() {
  maintainNonRunning();
  if (owner_ && owner_->record.state == State::Running) {
    const Record& record = owner_->record;
    event(bootId_, record.request.commandId, record.request.expectedEpoch, record.nextEventIndex);
  }
}

void Executor::poll() {
  maintainNonRunning();
  if (!owner_ || owner_->record.state != State::Running) return;
  const Record& record = owner_->record;
  if (!port_.allowed()) {
    finish(State::Failed, Reason::SafetyChanged);
    return;
  }
  const Profile& profile = record.request.profile;
  const uint32_t due = profile.startLeadMs + profile.points[record.nextEventIndex].offsetMs;
  if (port_.now() - record.executeAcceptedAt > due + profile.maxLatenessMs)
    finish(State::Failed, Reason::DeadlineMissed);
}

Result Executor::scheduleFailed(const char* bootId, const char* commandId, uint32_t expectedEpoch) {
  if (!validIdentity(bootId, commandId)) return Result(Code::Invalid);
  if (strcmp(bootId_, bootId) != 0) return Result(Code::BootMismatch);
  Slot* slot = lookup(commandId);
  if (!slot) return Result(Code::NotFound);
  Record& record = slot->record;
  if (expectedEpoch != record.request.expectedEpoch) return Result(Code::EpochMismatch, &record);
  if (terminal(record.state)) return Result(Code::Duplicate, &record);
  if (owner_ != slot) return Result(Code::Rejected, &record);
  if (record.state == State::Stopping) {
    // Confirmation cannot be scheduled; preserve the original stop reason/time.
    latchUnknown();
  } else {
    if (expectedEpoch != epoch_) return Result(Code::EpochMismatch, &record);
    finish(State::Failed, Reason::ScheduleFailed);
  }
  return Result(Code::Accepted, &record);
}

Result Executor::cancel(const char* bootId, const char* commandId, uint32_t expectedEpoch,
                        bool traceIncomplete) {
  if (!validIdentity(bootId, commandId)) return Result(Code::Invalid);
  if (strcmp(bootId_, bootId) != 0) return Result(Code::BootMismatch);
  Slot* slot = lookup(commandId);
  if (slot && (terminal(slot->record.state) || slot->record.state == State::Stopping))
    return Result(Code::Duplicate, &slot->record);
  if (owner_ && owner_ != slot) return Result(Code::Busy);
  if (expectedEpoch != epoch_) return Result(Code::EpochMismatch, slot ? &slot->record : nullptr);
  if (!slot) {
    if (blocked() || !port_.idle()) return Result(Code::Rejected);
    if (!advanceEpoch()) return Result(Code::EpochExhausted);
    return Result(Code::Accepted);
  }
  if (owner_ != slot) return Result(Code::Rejected, &slot->record);
  if (traceIncomplete) slot->record.traceComplete = false;
  finish(State::Cancelled, Reason::Cancelled);
  return Result(Code::Accepted, &slot->record);
}

Result Executor::release(const char* bootId, const char* commandId, uint32_t quietEpoch) {
  if (!validIdentity(bootId, commandId)) return Result(Code::Invalid);
  if (strcmp(bootId_, bootId) != 0) return Result(Code::BootMismatch);
  Slot* slot = lookup(commandId);
  if (!slot) return Result(Code::NotFound);
  Record& record = slot->record;
  if (!terminal(record.state)) return Result(Code::Busy, &record);
  if (!record.quietHeld) return Result(Code::Duplicate, &record);
  if (owner_ != slot || record.quietEpoch != quietEpoch || epoch_ != quietEpoch)
    return Result(Code::EpochMismatch, &record);
  if (record.state == State::OutcomeUnknown || !record.softwareOffConfirmed || bootLatched_)
    return Result(Code::Rejected, &record);
  // A historical OFF receipt cannot authorize release if another output has
  // since appeared during the quiet observation interval.
  if (!port_.idle()) return Result(Code::Busy, &record);
  record.quietHeld = false;
  owner_ = nullptr;
  advanceEpoch();
  return Result(Code::Accepted, &record);
}

Result Executor::retryOff(const char* bootId, const char* commandId) {
  if (!validIdentity(bootId, commandId)) return Result(Code::Invalid);
  if (strcmp(bootId_, bootId) != 0) return Result(Code::BootMismatch);
  Slot* slot = lookup(commandId);
  if (!slot) return Result(Code::NotFound);
  Record& record = slot->record;
  if (owner_ != slot || record.state != State::OutcomeUnknown) return Result(Code::Rejected, &record);
  uint32_t writtenAt = port_.now();
  const uint32_t before = writtenAt;
  record.lastRetryAttemptAt = before;
  record.lastRetryWriteRecorded = false;
  record.lastRetryConfirmed = false;
  record.softwareOffConfirmed = false;
  increment(record.offAttempts);
  const bool stopped = port_.stop(writtenAt);
  if (stopped && writeTimeValid(before, writtenAt, port_.now()) &&
      (!record.offWriteRecorded || writtenAt - record.lastOffWriteAt < HALF_CLOCK)) {
    record.lastRetryWriteAt = writtenAt;
    record.lastRetryWriteRecorded = true;
    record.lastOffWriteAt = writtenAt;
    if (!record.offWriteRecorded) {
      record.offWriteRecorded = true;
      record.offWriteAt = writtenAt;
    }
  } else record.traceComplete = false;
  // Explicit safety retries always attempt OFF; only the fixed-size counters
  // saturate. They do not restart automatic confirmation or unblock this boot.
  increment(record.offChecks);
  Proof proof = Proof::Failed;
  uint32_t confirmedAt = port_.now();
  if (!probeOff(proof, confirmedAt)) record.traceComplete = false;
  else if (proof == Proof::Confirmed) {
    if (!record.offConfirmationRecorded) {
      record.offConfirmedAt = confirmedAt;
      record.offConfirmationRecorded = true;
    }
    record.softwareOffConfirmed = true;
    record.lastRetryConfirmedAt = confirmedAt;
    record.lastRetryConfirmed = true;
  }
  // Separate follow-up evidence can never erase the original unknown latch.
  return Result(Code::Accepted, &record);
}

} // namespace finite_profile
#endif
