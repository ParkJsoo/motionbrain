#ifndef CONTROL_FINITE_PROFILE_SCHEDULER_H
#define CONTROL_FINITE_PROFILE_SCHEDULER_H

#include "finite_profile_driver_bridge.h"
#ifndef MOTIONBRAIN_ENABLE_FINITE_PROFILE_SCHEDULER
#define MOTIONBRAIN_ENABLE_FINITE_PROFILE_SCHEDULER 0
#endif
#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_SCHEDULER
#if !MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER
#error "Finite profile scheduler requires MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER"
#endif
namespace finite_profile {

enum class WakeKind { ReservationExpiry, OutputEvent, OffCheck };
struct ScheduledWake {
  char bootId[33];
  char commandId[129];
  uint32_t requestEpoch;
  uint32_t lease;
  uint32_t generation;
  WakeKind kind;
  uint8_t eventIndex;
  ScheduledWake() : bootId{}, commandId{}, requestEpoch(0), lease(0), generation(0),
      kind(WakeKind::ReservationExpiry), eventIndex(0) {}
};

// Store a VALUE ticket, never a callback capturing the ScheduledDriver pointer.
// Controller millis() is the only clock domain. All operations and deliveries
// are externally serialized, nonblocking and nonreentrant. disarm may leave
// already-queued deliveries; identity checks must reject those stale tickets.
class SchedulerPort {
public:
  virtual ~SchedulerPort() {}
  virtual bool armOnce(uint32_t absoluteDueMs, const ScheduledWake& ticket) = 0;
  virtual void disarm(const ScheduledWake& ticket) = 0;
};
enum class ScheduleStatus { NotRequested, Armed, Failed };
struct ScheduledExecuteResult {
  Result execution;
  ScheduleStatus scheduling;
  ScheduledExecuteResult(Result result, ScheduleStatus status)
      : execution(result), scheduling(status) {}
};

// Scheduler adapter; optional loop/HTTP wiring lives in finite_profile_runtime.
// The private bridge prevents mixing event dispatch with legacy tick.
// A deterministic fake scheduler proves policy, not wall-clock responsiveness.
class ScheduledDriver final {
public:
  ScheduledDriver(MotorControl& driver, DriverEnvironment& environment,
                  SchedulerPort& scheduler, const char* bootId,
                  const Profile* catalog, size_t catalogCount);
  ScheduledDriver(const ScheduledDriver&) = delete;
  ScheduledDriver& operator=(const ScheduledDriver&) = delete;
  ~ScheduledDriver();
  PrepareResult prepare(const Request& request);
  ScheduledExecuteResult execute(const char* bootId, const char* commandId,
                                 uint32_t epoch, const Token& token);
  // Maintenance may issue OFF, but cannot apply scheduled nonzero PWM.
  void poll();
  Result deliver(const ScheduledWake& ticket);
  Result cancel(const char* bootId, const char* commandId, uint32_t epoch);
  Result release(const char* bootId, const char* commandId, uint32_t quietEpoch);
  Result retryOff(const char* bootId, const char* commandId);
  const Record* find(const char* bootId, const char* commandId) const;
  const DriverOutputEvidence* outputEvidence(const char* bootId, const char* commandId) const;
  uint32_t epoch() const { return bridge_.epoch(); }
  size_t count() const { return bridge_.count(); }
  uint32_t leaseId() const { return bridge_.leaseId(); }
  bool idle() const { return !generationExhausted_ && bridge_.idle(); }
  bool blocked() const { return generationExhausted_ || bridge_.blocked(); }
  bool initialized() const { return bridge_.initialized(); }
  bool hasScheduledWake() const { return armed_; }
  bool executionPending() const {
    return owner_ && (owner_->state == State::Running || owner_->state == State::Stopping);
  }
private:
  MotorControl& driver_;
  SchedulerPort& scheduler_;
  DriverBridge bridge_;
  const Record* owner_;
  ScheduledWake ticket_;
  uint32_t dueAt_;
  uint32_t generation_;
  bool armed_;
  bool generationExhausted_;
  bool revoke(ScheduledWake& ticket);
  void disarm();
  bool reconcile();
  void failSchedule();
  bool matches(const ScheduledWake& ticket) const;
  bool sameOwner(const char* bootId, const char* commandId) const;
};
}
#endif
#endif
