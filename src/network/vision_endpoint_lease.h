#pragma once
#include <stdint.h>

// RAM-only discovery metadata. Never changes controller state or outputs.
class VisionEndpointLease {
public:
  static constexpr uint32_t TTL_MS = 180000;
  bool publish(const uint8_t* ip, uint16_t port, uint32_t now) {
    const bool local = ip[0] == 10 || (ip[0] == 172 && ip[1] >= 16 && ip[1] <= 31) ||
                       (ip[0] == 192 && ip[1] == 168);
    if (!local || port < 1024) return false;
    for (unsigned i = 0; i < 4; ++i) address[i] = ip[i];
    port_ = port;
    updated_ = now;
    published_ = true;
    return true;
  }
  uint32_t remaining(uint32_t now) const {
    const uint32_t age = now - updated_;
    return published_ && age < TTL_MS ? TTL_MS - age : 0;
  }
  uint8_t address[4] = {0, 0, 0, 0};
  uint16_t port() const { return port_; }
private:
  bool published_ = false;
  uint32_t updated_ = 0;
  uint16_t port_ = 8765;
};
