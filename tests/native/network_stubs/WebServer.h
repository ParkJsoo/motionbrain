#pragma once
#include <stdint.h>
#include <memory>
#include <vector>
#include <Arduino.h>
enum HTTPClientStatus { HC_NONE, HC_WAIT_READ, HC_WAIT_CLOSE };
struct ClientState { bool connected = true; int bytes = 0; int stops = 0; };
class WiFiClient {
public:
  std::shared_ptr<ClientState> value;
  bool connected() const { return value && value->connected; }
  int available() const { return value ? value->bytes : 0; }
  void stop() { if (value) { value->connected = false; ++value->stops; } }
};
// Minimal transport double; production adapter is compiled unchanged against
// this interface. Real SDK compatibility is separately checked by the build.
class WebServer {
public:
  explicit WebServer(uint16_t) {}
  virtual ~WebServer() = default;
  int handled = 0;
  int polls = 0;
  std::vector<std::shared_ptr<ClientState>> pending;
  virtual void handleClient() {
    ++polls;
    if (_currentStatus == HC_NONE && !pending.empty()) {
      _currentClient.value = pending.front(); pending.erase(pending.begin());
      _currentStatus = HC_WAIT_READ; _statusChange = millis();
    }
    if (_currentStatus == HC_WAIT_READ && _currentClient.connected() && _currentClient.available()) {
      ++handled; _currentClient.value.reset(); _currentStatus = HC_NONE;
    } else if (!_currentClient.connected()) {
      _currentClient.value.reset(); _currentStatus = HC_NONE;
    }
  }
protected:
  WiFiClient _currentClient;
  HTTPClientStatus _currentStatus = HC_NONE;
  unsigned long _statusChange = 0;
};
