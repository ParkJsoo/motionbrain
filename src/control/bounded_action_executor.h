#ifndef MOTIONBRAIN_BOUNDED_ACTION_EXECUTOR_H
#define MOTIONBRAIN_BOUNDED_ACTION_EXECUTOR_H

#include "action_registry.h"

namespace bounded {

// The adapter must make stop unconditional (including faults/disarmed state).
// outputIsOff confirms the driver's software state, not measured electrical or
// mechanical stopping. All calls are synchronous on the firmware control loop.
class Port {
public:
  virtual ~Port() {}
  virtual bool allowed() const = 0;
  virtual bool idle() const = 0;
  virtual bool start(const Request& request) = 0;
  virtual void stop(const Request& request, Reason reason) = 0;
  virtual bool outputIsOff(const Request& request) const = 0;
  virtual ShoulderResult shoulderResult() const = 0;
  // Production adapters return millis() here to include synchronous stop latency.
  virtual uint32_t afterStopTime(uint32_t decisionNow) const { return decisionNow; }
};

class Executor {
public:
  static const uint32_t MAX_DURATION_MS = 10000;
  Executor(Port& port, const char* bootId);
  SubmitResult submit(const Request& request, uint32_t now);
  void tick(uint32_t now);
  void cancel(uint32_t now, Reason reason = Reason::Cancelled);
  LookupResult find(const char* bootId, const char* commandId) const;
  bool active() const { return active_ != nullptr; }
  uint32_t activeOrdinal() const { return active_ ? active_->ordinal : 0; }
  const char* bootId() const { return bootId_; }
  const ActionRegistry& registry() const { return registry_; }
private:
  Port& port_;
  char bootId_[33];
  ActionRegistry registry_;
  Record* active_;
  bool initialized_;
  static bool validRequest(const Request& request);
  static bool samePayload(const Request& left, const Request& right);
  void finish(uint32_t now, State state, Reason reason, bool targetReached = false);
};

} // namespace bounded
#endif
