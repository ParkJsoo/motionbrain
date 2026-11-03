#ifndef MOTIONBRAIN_FINITE_PROFILE_SERVICE_ENVIRONMENT_H
#define MOTIONBRAIN_FINITE_PROFILE_SERVICE_ENVIRONMENT_H

#include "finite_profile_driver_bridge.h"

#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER
namespace finite_profile {

// Firmware service snapshot for the singleton controller graph in main.cpp.
// A trusted caller supplies a nonblocking token issuer after its entropy source
// is ready. It must not reenter any service/driver/bridge call. No default issuer,
// global bridge, catalog, scheduler, HTTP endpoint or automatic initialization.
class ServiceEnvironment final : public DriverEnvironment {
public:
  typedef bool (*TokenIssuer)(Token&);
  explicit ServiceEnvironment(TokenIssuer issuer = nullptr) : issuer_(issuer) {}
  bool allowed() const override;
  bool controllersIdle() const override;
  bool mintToken(Token& token) override;
private:
  TokenIssuer issuer_;
  bool ready() const;
};

} // namespace finite_profile
#endif
#endif
