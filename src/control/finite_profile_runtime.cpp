#include "finite_profile_runtime.h"

#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_RUNTIME
#include <string.h>
#include <new>
namespace finite_profile {
namespace runtime {
namespace {
bool sameTicket(const ScheduledWake& a, const ScheduledWake& b) {
  return a.requestEpoch == b.requestEpoch && a.lease == b.lease &&
      a.generation == b.generation && a.kind == b.kind && a.eventIndex == b.eventIndex &&
      strcmp(a.bootId, b.bootId) == 0 && strcmp(a.commandId, b.commandId) == 0;
}
// Cooperative one-shot backend: no task, interrupt, callback pointer or loop
// hidden inside the port. Its only clock is the driver's controller millis().
class LoopScheduler final : public SchedulerPort {
public:
  LoopScheduler() : ticket_(), dueAt_(0), armed_(false) {}
  bool armOnce(uint32_t dueAt, const ScheduledWake& ticket) override {
    if (armed_) return false;
    ticket_ = ticket;
    dueAt_ = dueAt;
    armed_ = true;
    return true;
  }
  void disarm(const ScheduledWake& ticket) override {
    if (armed_ && sameTicket(ticket_, ticket)) armed_ = false;
  }
  bool takeDue(uint32_t now, ScheduledWake& ticket) {
    // Valid profiles bound every horizon far below half the uint32 clock range.
    if (!armed_ || static_cast<int32_t>(now - dueAt_) < 0) return false;
    ticket = ticket_;
    armed_ = false;
    return true;
  }
private:
  ScheduledWake ticket_;
  uint32_t dueAt_;
  bool armed_;
};
struct RuntimeState {
  Profile profile;
  char boot[33];
  ServiceEnvironment environment;
  LoopScheduler timer;
  ScheduledDriver scheduled;
  RuntimeState(MotorControl& driver, const Profile& supplied, const char* bootId,
               ServiceEnvironment::TokenIssuer issuer)
      : profile(supplied), boot{}, environment(issuer), timer(),
        scheduled(driver, environment, timer, bootId, &profile, 1) {
    strcpy(boot, bootId);
  }
};
RuntimeState* state = nullptr;
}

bool enabled() { return true; }
bool init(MotorControl& driver, const Profile& trustedProfile, const char* boot,
          ServiceEnvironment::TokenIssuer issuer) {
  if (state || !issuer || !validIdentifier(boot, 33) || !validProfile(trustedProfile)) return false;
  // Allocate once at explicit boot initialization. Static DRAM is scarce on
  // ESP32; failure leaves every public pointer unavailable and produces no IO.
  RuntimeState* instance = new (std::nothrow) RuntimeState(driver, trustedProfile, boot, issuer);
  if (!instance) return false;
  if (!instance->scheduled.initialized()) { delete instance; return false; }
  // Firmware-boot lifetime: intentionally no automatic teardown/reinitialization.
  state = instance;
  return true;
}
void update() {
  if (!state) return;
  // Maintenance first fails late/missing events to OFF; it never emits planned
  // nonzero PWM. At most one due output is delivered, with no catch-up loop.
  state->scheduled.poll();
  ScheduledWake ticket;
  if (state->timer.takeDue(millis(), ticket)) state->scheduled.deliver(ticket);
}
bool motionPending() { return state && state->scheduled.executionPending(); }
const char* bootId() { return state ? state->boot : ""; }
ScheduledDriver* driver() { return state ? &state->scheduled : nullptr; }
const Profile* profile() { return state ? &state->profile : nullptr; }
}
}
#else
namespace finite_profile {
namespace runtime {
bool enabled() { return false; }
void update() {}
bool motionPending() { return false; }
const char* bootId() { return ""; }
ScheduledDriver* driver() { return nullptr; }
const Profile* profile() { return nullptr; }
}
}
#endif
