#include <cassert>
#include <cstring>
#include "network/wifi_latency.h"

FakeWiFi WiFi;
wifi_ps_type_t actual = WIFI_PS_MIN_MODEM;
wifi_mode_t mode = WIFI_MODE_STA;
bool apply = true, accepted = true;
int queryError = 0, modeError = 0, writes = 0, setError = 0, driverWrites = 0;
bool FakeWiFi::setSleep(bool enabled) {
  assert(!enabled);
  ++writes;
  if (apply) actual = WIFI_PS_NONE;
  return accepted;
}
esp_err_t esp_wifi_get_ps(wifi_ps_type_t* out) { if (!queryError) *out = actual; return queryError; }
esp_err_t esp_wifi_set_ps(wifi_ps_type_t value) {
  ++driverWrites;
  if (!setError) actual = value;
  return setError;
}
esp_err_t esp_wifi_get_mode(wifi_mode_t* out) { if (!modeError) *out = mode; return modeError; }
void expect(const char* value) { assert(std::strcmp(WiFiLatency::stationPowerSave(), value) == 0); }
int main() {
  expect("min_modem");
  assert(WiFiLatency::disableStationSleep()); expect("none");
  accepted = false; // Arduino returns false for an already-selected preference.
  assert(WiFiLatency::disableStationSleep());
  actual = WIFI_PS_MIN_MODEM; apply = false; accepted = true; setError = -1;
  assert(!WiFiLatency::disableStationSleep()); expect("min_modem");
  // A cached NONE preference must not prevent recovery after a transient error.
  accepted = false; setError = 0;
  assert(WiFiLatency::disableStationSleep()); expect("none");
  actual = WIFI_PS_MAX_MODEM; expect("max_modem");
  queryError = -1; assert(!WiFiLatency::disableStationSleep()); expect("unknown");
  queryError = 0; mode = WIFI_MODE_AP; expect("not_station");
  mode = WIFI_MODE_APSTA; expect("max_modem");
  modeError = -1; expect("unknown");
  assert(writes == 5 && driverWrites == 2); // Status queries never alter power mode.
}
