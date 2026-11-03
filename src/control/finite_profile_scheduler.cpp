#include "finite_profile_scheduler.h"

#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_SCHEDULER
#include <limits.h>
#include <string.h>
namespace finite_profile {

ScheduledDriver::ScheduledDriver(MotorControl& driver, DriverEnvironment& environment,
                                 SchedulerPort& scheduler, const char* bootId,
                                 const Profile* catalog, size_t catalogCount)
    : driver_(driver), scheduler_(scheduler), bridge_(driver, environment, bootId, catalog, catalogCount),
      owner_(nullptr), ticket_(), dueAt_(0), generation_(0), armed_(false),
      generationExhausted_(false) {}
ScheduledDriver::~ScheduledDriver() {
  ScheduledWake revoked;
  const bool hadWake = revoke(revoked);
  // Owner loss is not a scoped release. Stop only the matching capability and
  // leave its sealed lease and evidence capture open for explicit recovery.
  if (bridge_.leaseId()) {
    uint32_t at = 0;
    driver_.stopFiniteProfile(bridge_.leaseId(), at);
  }
  if (hadWake) scheduler_.disarm(revoked);
  // The bridge destructor may reassert OFF; both writes remain captured.
}

bool ScheduledDriver::sameOwner(const char* bootId, const char* commandId) const {
  return owner_ && validIdentifier(bootId, sizeof(owner_->request.bootId)) &&
      validIdentifier(commandId, sizeof(owner_->request.commandId)) &&
      strcmp(owner_->request.bootId, bootId) == 0 &&
      strcmp(owner_->request.commandId, commandId) == 0;
}
bool ScheduledDriver::matches(const ScheduledWake& ticket) const {
  return armed_ && sameOwner(ticket.bootId, ticket.commandId) &&
      ticket.requestEpoch == ticket_.requestEpoch && ticket.lease == ticket_.lease &&
      ticket.lease == bridge_.leaseId() && ticket.generation == ticket_.generation &&
      ticket.kind == ticket_.kind && ticket.eventIndex == ticket_.eventIndex;
}
bool ScheduledDriver::revoke(ScheduledWake& ticket) {
  if (!armed_) return false;
  ticket = ticket_;
  armed_ = false;
  return true;
}
void ScheduledDriver::disarm() {
  ScheduledWake revoked;
  if (revoke(revoked)) scheduler_.disarm(revoked);
}
void ScheduledDriver::failSchedule() {
  ScheduledWake revoked;
  const bool hadWake = revoke(revoked);
  if (owner_) {
    bridge_.scheduleFailed(owner_->request.bootId, owner_->request.commandId,
                           owner_->request.expectedEpoch);
    // The failed scheduling facility cannot promise another confirmation poll.
    if (owner_->state == State::Stopping)
      bridge_.scheduleFailed(owner_->request.bootId, owner_->request.commandId,
                             owner_->request.expectedEpoch);
  }
  if (hadWake) scheduler_.disarm(revoked);
}

bool ScheduledDriver::reconcile() {
  if (!owner_) { disarm(); return true; }
  WakeKind kind;
  uint8_t index = 0;
  uint32_t due = 0;
  const Profile& profile = owner_->request.profile;
  switch (owner_->state) {
    case State::Reserved:
      kind = WakeKind::ReservationExpiry;
      due = owner_->preparedAt + profile.admissionWindowMs;
      break;
    case State::Running:
      kind = WakeKind::OutputEvent;
      index = owner_->nextEventIndex;
      if (index >= profile.count) { failSchedule(); return false; }
      due = owner_->plannedStartAt + profile.points[index].offsetMs;
      break;
    case State::Stopping: {
      kind = WakeKind::OffCheck;
      const uint32_t elapsed = millis() - owner_->decisionAt;
      // One finite confirmation wake at a time. A delayed wake cannot postpone
      // the core's deadline, and its existing MAX_OFF_CHECKS remains binding.
      due = elapsed >= profile.offConfirmWindowMs ? millis() : millis() + 1;
      break;
    }
    default:
      disarm();
      if (!owner_->quietHeld && bridge_.leaseId() == 0) owner_ = nullptr;
      return true;
  }
  if (armed_ && ticket_.kind == kind && ticket_.eventIndex == index &&
      ticket_.lease == bridge_.leaseId() &&
      (kind == WakeKind::OffCheck || dueAt_ == due)) return true;
  disarm();
  if (generation_ == UINT32_MAX) {
    generationExhausted_ = true;
    failSchedule();
    return false;
  }
  ticket_ = ScheduledWake();
  strcpy(ticket_.bootId, owner_->request.bootId);
  strcpy(ticket_.commandId, owner_->request.commandId);
  ticket_.requestEpoch = owner_->request.expectedEpoch;
  ticket_.lease = bridge_.leaseId();
  ticket_.generation = ++generation_;
  ticket_.kind = kind;
  ticket_.eventIndex = index;
  dueAt_ = due;
  armed_ = true;
  if (!scheduler_.armOnce(dueAt_, ticket_)) {
    failSchedule();
    return false;
  }
  // Registration can consume time; it may not shift the accepted origin.
  if (kind != WakeKind::OffCheck) {
    bridge_.poll();
    if (owner_->state == State::Stopping) return reconcile();
    if (owner_->state != State::Reserved && owner_->state != State::Running) {
      disarm();
      return false;
    }
  }
  return true;
}

PrepareResult ScheduledDriver::prepare(const Request& request) {
  if (generationExhausted_) return PrepareResult(Code::Rejected);
  PrepareResult result = bridge_.prepare(request);
  if (result.granted) owner_ = result.record;
  const bool registered = reconcile();
  if (result.granted && (!registered || result.record->state != State::Reserved)) {
    result.code = Code::Rejected;
    result.granted = false;
    result.token = Token();
  }
  return result;
}
ScheduledExecuteResult ScheduledDriver::execute(const char* bootId, const char* commandId,
                                                uint32_t epoch, const Token& token) {
  Result result = bridge_.execute(bootId, commandId, epoch, token);
  const bool registered = reconcile();
  const ScheduleStatus status = result.code != Code::Accepted ? ScheduleStatus::NotRequested :
      (!registered ? ScheduleStatus::Failed :
       (result.record && result.record->state == State::Running && armed_ ?
        ScheduleStatus::Armed : ScheduleStatus::Failed));
  return ScheduledExecuteResult(result, status);
}
void ScheduledDriver::poll() { bridge_.poll(); reconcile(); }
Result ScheduledDriver::deliver(const ScheduledWake& incoming) {
  // A caller may pass the scheduler port's own storage, which disarm mutates.
  const ScheduledWake ticket = incoming;
  if (!matches(ticket)) return Result(Code::StaleEvent);
  disarm();
  bridge_.poll();
  Result result(Code::Accepted, owner_);
  if (owner_ && owner_->state == State::Expired) result.code = Code::Expired;
  else if (owner_ && (owner_->state == State::Failed || owner_->state == State::OutcomeUnknown))
    result.code = owner_->reason == Reason::SafetyChanged ? Code::SafetyBlocked : Code::Rejected;
  if (ticket.kind == WakeKind::OutputEvent) {
    if (owner_ && owner_->state == State::Running)
      result = bridge_.event(ticket.bootId, ticket.commandId, ticket.requestEpoch, ticket.eventIndex);
    else if (result.code == Code::Accepted) result.code = Code::Rejected;
  } else if (ticket.kind == WakeKind::ReservationExpiry && owner_ && owner_->state == State::Reserved) {
    result.code = Code::NotDue;
  }
  reconcile(); // NotDue registers a fresh generation at the original due time.
  if (result.record && result.record->state == State::Expired) result.code = Code::Expired;
  else if (result.record && (result.record->state == State::Failed || result.record->state == State::OutcomeUnknown))
    result.code = result.record->reason == Reason::SafetyChanged ? Code::SafetyBlocked : Code::Rejected;
  else if (result.record && result.record->state == State::Cancelled && ticket.kind != WakeKind::OffCheck)
    result.code = Code::Rejected;
  return result;
}
Result ScheduledDriver::cancel(const char* bootId, const char* commandId, uint32_t epoch) {
  ScheduledWake revoked;
  const bool hadWake = sameOwner(bootId, commandId) && owner_->request.expectedEpoch == epoch && revoke(revoked);
  Result result = bridge_.cancel(bootId, commandId, epoch);
  if (hadWake) scheduler_.disarm(revoked);
  reconcile(); return result;
}
Result ScheduledDriver::release(const char* bootId, const char* commandId, uint32_t quietEpoch) {
  Result result = bridge_.release(bootId, commandId, quietEpoch);
  reconcile(); return result;
}
Result ScheduledDriver::retryOff(const char* bootId, const char* commandId) {
  Result result = bridge_.retryOff(bootId, commandId);
  reconcile(); return result;
}
const Record* ScheduledDriver::find(const char* bootId, const char* commandId) const {
  return bridge_.find(bootId, commandId);
}
const DriverOutputEvidence* ScheduledDriver::outputEvidence(const char* bootId, const char* commandId) const {
  return bridge_.outputEvidence(bootId, commandId);
}
}
#endif
