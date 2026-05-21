#include "network/wifi_provisioning.h"

#include <Preferences.h>
#include "debug/debug_log.h"

namespace {

constexpr const char* WIFI_PREF_NAMESPACE = "mb_wifi";
constexpr const char* KEY_SSID = "ssid";
constexpr const char* KEY_PASSWORD = "password";
constexpr const char* KEY_TOKEN = "token";

void copyString(char* dest, size_t destSize, const String& value) {
  if (dest == nullptr || destSize == 0) {
    return;
  }
  strlcpy(dest, value.c_str(), destSize);
}

void drainLineEndings() {
  delay(2);
  while (Serial.available() > 0) {
    int c = Serial.peek();
    if (c != '\n' && c != '\r') {
      return;
    }
    Serial.read();
  }
}

} // namespace

bool WifiProvisioning::load(WifiProvisioningConfig& config) {
  Preferences prefs;
  if (!prefs.begin(WIFI_PREF_NAMESPACE, true)) {
    DebugLog::warn("Wi-Fi provisioning: failed to open NVS");
    return false;
  }

  String ssid = prefs.getString(KEY_SSID, "");
  String password = prefs.getString(KEY_PASSWORD, "");
  String token = prefs.getString(KEY_TOKEN, "");
  prefs.end();

  ssid.trim();
  if (ssid.length() == 0 || ssid.length() >= sizeof(config.ssid)) {
    return false;
  }

  copyString(config.ssid, sizeof(config.ssid), ssid);
  copyString(config.password, sizeof(config.password), password);
  copyString(config.commandToken, sizeof(config.commandToken), token);
  return true;
}

bool WifiProvisioning::save(const WifiProvisioningConfig& config) {
  Preferences prefs;
  if (!prefs.begin(WIFI_PREF_NAMESPACE, false)) {
    DebugLog::warn("Wi-Fi provisioning: failed to open NVS for write");
    return false;
  }

  bool ok = prefs.putString(KEY_SSID, config.ssid) > 0;
  if (config.password[0] != '\0') {
    ok = prefs.putString(KEY_PASSWORD, config.password) > 0 && ok;
  } else {
    prefs.remove(KEY_PASSWORD);
  }
  if (config.commandToken[0] != '\0') {
    ok = prefs.putString(KEY_TOKEN, config.commandToken) > 0 && ok;
  } else {
    prefs.remove(KEY_TOKEN);
  }
  prefs.end();
  return ok;
}

void WifiProvisioning::clear() {
  Preferences prefs;
  if (!prefs.begin(WIFI_PREF_NAMESPACE, false)) {
    DebugLog::warn("Wi-Fi provisioning: failed to open NVS for clear");
    return;
  }

  prefs.clear();
  prefs.end();
  DebugLog::info("Wi-Fi provisioning: stored credentials cleared");
}

bool WifiProvisioning::readLine(const char* prompt, char* buffer, size_t bufferSize, bool allowEmpty) {
  if (buffer == nullptr || bufferSize == 0) {
    return false;
  }

  while (true) {
    Serial.print(prompt);
    String line;
    while (true) {
      while (Serial.available() > 0) {
        char c = static_cast<char>(Serial.read());
        if (c == '\n' || c == '\r') {
          goto line_complete;
        }
        line += c;
      }
      delay(10);
    }

line_complete:
    drainLineEndings();
    line.trim();

    if (line.length() == 0 && !allowEmpty) {
      Serial.println("Value required.");
      continue;
    }

    if (line.length() >= bufferSize) {
      Serial.printf("Too long. Max %u characters.\n", static_cast<unsigned>(bufferSize - 1));
      continue;
    }

    strlcpy(buffer, line.c_str(), bufferSize);
    return true;
  }
}

bool WifiProvisioning::promptIfMissing(WifiProvisioningConfig& config) {
  if (load(config)) {
    DebugLog::info("Wi-Fi provisioning: loaded stored home Wi-Fi config");
    return true;
  }

  Serial.println();
  Serial.println("=== MotionBrain Wi-Fi Provisioning ===");
  Serial.println("No stored home Wi-Fi config found.");
  Serial.println("Enter values here; they will be saved to ESP32 NVS flash, not to project files.");
  Serial.println("Leave command token empty only for trusted bench testing.");

  if (!readLine("Wi-Fi SSID: ", config.ssid, sizeof(config.ssid), false)) {
    return false;
  }
  if (!readLine("Wi-Fi password: ", config.password, sizeof(config.password), true)) {
    return false;
  }
  if (!readLine("Command token (optional): ", config.commandToken, sizeof(config.commandToken), true)) {
    return false;
  }

  if (!save(config)) {
    Serial.println("Failed to save Wi-Fi config.");
    return false;
  }

  Serial.println("Wi-Fi config saved to ESP32 NVS.");
  Serial.println("Use 'CLEAR' during the boot prompt to erase it later.");
  return true;
}

bool WifiProvisioning::clearRequestedOnBoot(uint32_t timeoutMs) {
  Serial.println();
  Serial.printf("Type CLEAR within %lu ms to erase stored Wi-Fi config, or wait to continue.\n",
                static_cast<unsigned long>(timeoutMs));

  const uint32_t startedAt = millis();
  String line;
  while (millis() - startedAt < timeoutMs) {
    while (Serial.available() > 0) {
      char c = static_cast<char>(Serial.read());
      if (c == '\n' || c == '\r') {
        drainLineEndings();
        line.trim();
        if (line == "CLEAR") {
          clear();
          return true;
        }
        return false;
      }
      line += c;
    }
    delay(10);
  }

  return false;
}
