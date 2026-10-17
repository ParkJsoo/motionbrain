#pragma once

#include <Arduino.h>
#include <WebServer.h>
#include <stdint.h>

// Arduino WebServer keeps one accepted client while waiting up to 5 s for
// its first HTTP byte. An idle browser connection can therefore starve even
// STOP/status requests. Bound only that empty-client phase; parsing, routes,
// authentication and motor leases remain the responsibility of their owners.
class IdleConnectionWebServer : public ::WebServer {
public:
  static constexpr uint32_t FIRST_BYTE_WAIT_MS = 100;
  explicit IdleConnectionWebServer(uint16_t port) : ::WebServer(port) {}

  void handleClient() override {
    if (_currentStatus == HC_WAIT_READ && _currentClient.connected() &&
        _currentClient.available() == 0 &&
        static_cast<uint32_t>(millis() - _statusChange) >= FIRST_BYTE_WAIT_MS) {
      _currentClient.stop();
      ++idleConnectionReleases_;
    }
    // The base implementation clears the disconnected client's context.
    // The next loop accepts the next client without closing the listener.
    ::WebServer::handleClient();
  }

  uint32_t idleConnectionReleases() const { return idleConnectionReleases_; }

private:
  uint32_t idleConnectionReleases_ = 0;
};
