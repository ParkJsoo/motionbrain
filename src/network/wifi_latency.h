#pragma once

#include <WiFi.h>
#include <esp_wifi.h>

namespace WiFiLatency {

// Avoid DTIM/modem-sleep waits for finite sensor observations. setSleep(false)
// also keeps Arduino's reconnect preference; its return value alone is not an
// acknowledgement (it can be false when the requested mode is already set).
inline bool disableStationSleep() {
  WiFi.setSleep(false);
  wifi_ps_type_t mode;
  if (esp_wifi_get_ps(&mode) != ESP_OK) return false;
  if (mode == WIFI_PS_NONE) return true;
  // Arduino can cache NONE even after its driver call failed. The same cached
  // preference would then be a no-op on reconnect; retry the driver explicitly.
  if (esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK) return false;
  return esp_wifi_get_ps(&mode) == ESP_OK && mode == WIFI_PS_NONE;
}

// Query the driver, not Arduino's cached preference. A failed query must not
// claim that low-latency mode was applied. This is diagnostic, not eligibility.
inline const char* stationPowerSave() {
  wifi_mode_t wifiMode;
  if (esp_wifi_get_mode(&wifiMode) != ESP_OK) return "unknown";
  if (wifiMode != WIFI_MODE_STA && wifiMode != WIFI_MODE_APSTA) return "not_station";
  wifi_ps_type_t mode;
  if (esp_wifi_get_ps(&mode) != ESP_OK) return "unknown";
  switch (mode) {
    case WIFI_PS_NONE: return "none";
    case WIFI_PS_MIN_MODEM: return "min_modem";
    case WIFI_PS_MAX_MODEM: return "max_modem";
    default: return "unknown";
  }
}

}  // namespace WiFiLatency
