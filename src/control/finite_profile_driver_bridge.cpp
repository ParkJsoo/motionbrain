#include "finite_profile_driver_bridge.h"

#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER
namespace finite_profile {

DriverBridge::DriverBridge(MotorControl& driver, DriverEnvironment& environment,
                           const char* bootId, const Profile* catalog, size_t catalogCount)
    : driver_(driver), environment_(environment), lease_(0), bindingFailed_(false),
      ownerRecord_(nullptr), capturedRecords_{}, capturedLeases_{}, port_(*this), executor_(port_, bootId, catalog, catalogCount) {}

DriverBridge::~DriverBridge() {
  // Losing the owner must not release a reservation/quiet lock to legacy code.
  // Stop and leave the matching lease sealed. Explicit recovery is separate.
  if (lease_) { uint32_t at = 0; driver_.stopFiniteProfile(lease_, at); }
}

uint32_t DriverBridge::now() const { return millis(); }
bool DriverBridge::allowed() const {
  return !bindingFailed_ && environment_.allowed() && environment_.controllersIdle() &&
      (lease_ ? driver_.finiteProfileAllowed(lease_) : driver_.finiteProfileReady());
}
bool DriverBridge::outputIdle() const {
  if (bindingFailed_ || !environment_.controllersIdle()) return false;
  return lease_ ? driver_.finiteProfileSoftwareOff(lease_) : driver_.finiteProfileReady();
}
bool DriverBridge::idle() const {
  return initialized() && !blocked() && executor_.idle() && lease_ == 0 && outputIdle();
}
bool DriverBridge::apply(uint8_t motor, int16_t pwm, uint32_t& at) {
  return lease_ && driver_.applyFiniteProfile(lease_, motor, pwm, at);
}
bool DriverBridge::stop(uint32_t& at) {
  // Never allow a stale bridge to stop another owner's output.
  return lease_ && driver_.stopFiniteProfile(lease_, at);
}
Proof DriverBridge::offProof() {
  if (!lease_ || !driver_.finiteProfileLeaseMatches(lease_)) return Proof::Failed;
  return driver_.finiteProfileSoftwareOff(lease_) ? Proof::Confirmed : Proof::Pending;
}
bool DriverBridge::mintToken(Token& token) { return environment_.mintToken(token); }

uint32_t DriverBridge::OutputPort::now() const { return owner_.now(); }
bool DriverBridge::OutputPort::allowed() const { return owner_.allowed(); }
bool DriverBridge::OutputPort::idle() const { return owner_.outputIdle(); }
bool DriverBridge::OutputPort::apply(uint8_t motor, int16_t pwm, uint32_t& at) {
  return owner_.apply(motor, pwm, at);
}
bool DriverBridge::OutputPort::stop(uint32_t& at) { return owner_.stop(at); }
Proof DriverBridge::OutputPort::offProof() { return owner_.offProof(); }
bool DriverBridge::OutputPort::mintToken(Token& token) { return owner_.mintToken(token); }

void DriverBridge::beforeCall() {
  if (!lease_ || !ownerRecord_) return;
  if (ownerRecord_->state != State::Reserved && ownerRecord_->state != State::Running) return;
  const bool matches = driver_.finiteProfileLeaseMatches(lease_);
  if (!matches || driver_.finiteProfileLeaseSealed(lease_)) {
    // An external STOP/driver safety event occurred outside the core's trace.
    // A missing capability can only latch failure, never stop its replacement.
    if (!matches) bindingFailed_ = true;
    executor_.cancel(ownerRecord_->request.bootId, ownerRecord_->request.commandId,
                     ownerRecord_->request.expectedEpoch, true);
  }
}

void DriverBridge::afterCall() {
  if (!executor_.idle() || !lease_) return;
  // Release only after core ownership is gone and every external controller
  // is idle. A quiet owner never reaches this branch until scoped release.
  if (!environment_.controllersIdle() || !driver_.finiteProfileSoftwareOff(lease_) ||
      !driver_.releaseFiniteProfileLease(lease_)) {
    bindingFailed_ = true;
    return;
  }
  lease_ = 0;
  ownerRecord_ = nullptr;
}

PrepareResult DriverBridge::prepare(const Request& request) {
  beforeCall();
  if (bindingFailed_ && !executor_.find(request.bootId, request.commandId))
    return PrepareResult(Code::Rejected);
  PrepareResult result = executor_.prepare(request);
  if (result.granted) {
    ownerRecord_ = result.record;
    if (lease_ || !driver_.acquireFiniteProfileLease(static_cast<uint32_t>(executor_.count()), lease_)) {
      bindingFailed_ = true;
      executor_.cancel(request.bootId, request.commandId, request.expectedEpoch);
      result.granted = false;
      result.token = Token();
      result.code = Code::Rejected;
    } else if (!driver_.bindFiniteProfileEvidence(lease_, request.bootId,
                                                  request.commandId, request.expectedEpoch)) {
      // No grant escapes until a dedicated capture slot is reserved. Acquisition
      // wrote nothing; cancel while zero/idle, then release only our own lease.
      // Existing evidence is never evicted to make this admission succeed.
      executor_.cancel(request.bootId, request.commandId, request.expectedEpoch);
      result.granted = false;
      result.token = Token();
      result.code = Code::Rejected;
    } else {
      const size_t index = executor_.count() - 1;
      capturedRecords_[index] = result.record;
      capturedLeases_[index] = lease_;
    }
  }
  afterCall();
  return result;
}

Result DriverBridge::execute(const char* bootId, const char* commandId, uint32_t epoch,
                             const Token& token) {
  beforeCall();
  Result result = bindingFailed_ ? Result(Code::Rejected, find(bootId, commandId)) :
      executor_.execute(bootId, commandId, epoch, token);
  afterCall();
  return result;
}
void DriverBridge::tick() { beforeCall(); executor_.tick(); afterCall(); }
void DriverBridge::poll() { beforeCall(); executor_.poll(); afterCall(); }
Result DriverBridge::scheduleFailed(const char* bootId, const char* commandId, uint32_t epoch) {
  beforeCall(); Result result = executor_.scheduleFailed(bootId, commandId, epoch);
  afterCall(); return result;
}
Result DriverBridge::event(const char* bootId, const char* commandId, uint32_t epoch, uint8_t index) {
  beforeCall();
  Result result = bindingFailed_ ? Result(Code::Rejected, find(bootId, commandId)) :
      executor_.event(bootId, commandId, epoch, index);
  afterCall(); return result;
}
Result DriverBridge::cancel(const char* bootId, const char* commandId, uint32_t epoch) {
  beforeCall(); Result result = executor_.cancel(bootId, commandId, epoch);
  afterCall(); return result;
}
Result DriverBridge::release(const char* bootId, const char* commandId, uint32_t quietEpoch) {
  beforeCall(); Result result = executor_.release(bootId, commandId, quietEpoch);
  afterCall(); return result;
}
Result DriverBridge::retryOff(const char* bootId, const char* commandId) {
  beforeCall(); Result result = executor_.retryOff(bootId, commandId);
  afterCall(); return result;
}
const Record* DriverBridge::find(const char* bootId, const char* commandId) const {
  return executor_.find(bootId, commandId);
}

const DriverOutputEvidence* DriverBridge::outputEvidence(const char* bootId, const char* commandId) const {
  const Record* record = find(bootId, commandId);
  if (!record) return nullptr;
  // A rejected bind after wrapper recreation must not expose an older capture
  // just because boot/command/epoch were reused by the new core instance.
  for (size_t i = 0; i < MAX_RECORDS; ++i) {
    if (capturedRecords_[i] == record) return driver_.finiteProfileEvidence(capturedLeases_[i]);
  }
  return nullptr;
}

} // namespace finite_profile
#endif
