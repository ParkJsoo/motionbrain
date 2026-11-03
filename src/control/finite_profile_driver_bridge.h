#ifndef MOTIONBRAIN_FINITE_PROFILE_DRIVER_BRIDGE_H
#define MOTIONBRAIN_FINITE_PROFILE_DRIVER_BRIDGE_H

#include "finite_profile_executor.h"
#ifndef MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER
#define MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER 0
#endif
#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER
#if !MOTIONBRAIN_ENABLE_FINITE_PROFILE_CORE
#error "Finite profile driver bridge requires the finite profile core"
#endif
#include "motor/motor_driver.h"

namespace finite_profile {

// Trusted, nonblocking, externally serialized context. controllersIdle includes
// sequence, angle/shoulder controllers, teleop/deadman, manual leases and queued
// work that could resume output. ServiceEnvironment reads the firmware singleton
// graph; all calls and producer hooks still require external serialization.
class DriverEnvironment {
public:
  virtual ~DriverEnvironment() {}
  virtual bool allowed() const = 0;
  virtual bool controllersIdle() const = 0;
  virtual bool mintToken(Token& token) = 0;
};

// No global instance, HTTP entry point, scheduler, or firmware activation.
// Serialize ALL driver, bridge and environment calls, with no reentrant ports.
// Only this wrapper may mutate its Executor; callers receive const receipts.
class DriverBridge final {
public:
  DriverBridge(MotorControl& driver, DriverEnvironment& environment,
               const char* bootId, const Profile* catalog, size_t catalogCount);
  DriverBridge(const DriverBridge&) = delete;
  DriverBridge& operator=(const DriverBridge&) = delete;
  ~DriverBridge();
  PrepareResult prepare(const Request& request);
  Result execute(const char* bootId, const char* commandId, uint32_t epoch, const Token& token);
  void tick();
  Result event(const char* bootId, const char* commandId, uint32_t epoch, uint8_t index);
  Result cancel(const char* bootId, const char* commandId, uint32_t epoch);
  Result release(const char* bootId, const char* commandId, uint32_t quietEpoch);
  Result retryOff(const char* bootId, const char* commandId);
  const Record* find(const char* bootId, const char* commandId) const;
  // Actual writes are distinct from the core planned-event receipts.
  const DriverOutputEvidence* outputEvidence(const char* bootId, const char* commandId) const;
  uint32_t epoch() const { return executor_.epoch(); }
  uint32_t leaseId() const { return lease_; }
  bool blocked() const { return bindingFailed_ || executor_.blocked(); }
  bool idle() const;
  bool initialized() const { return executor_.initialized(); }
  size_t count() const { return executor_.count(); }
private:
  // Port::idle must describe driver state while a lease is held, whereas the
  // public idle status must also account for reservation/quiet ownership.
  class OutputPort final : public Port {
  public:
    explicit OutputPort(DriverBridge& owner) : owner_(owner) {}
    uint32_t now() const override;
    bool allowed() const override;
    bool idle() const override;
    bool apply(uint8_t motor, int16_t pwm, uint32_t& at) override;
    bool stop(uint32_t& at) override;
    Proof offProof() override;
    bool mintToken(Token& token) override;
  private:
    DriverBridge& owner_;
  };
  MotorControl& driver_;
  DriverEnvironment& environment_;
  uint32_t lease_;
  bool bindingFailed_;
  const Record* ownerRecord_;
  const Record* capturedRecords_[MAX_RECORDS];
  uint32_t capturedLeases_[MAX_RECORDS];
  OutputPort port_;
  Executor executor_;
  void beforeCall();
  void afterCall();
  uint32_t now() const;
  bool allowed() const;
  bool outputIdle() const;
  bool apply(uint8_t motor, int16_t pwm, uint32_t& at);
  bool stop(uint32_t& at);
  Proof offProof();
  bool mintToken(Token& token);
};

} // namespace finite_profile
#endif
#endif
