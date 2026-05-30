#ifndef WIFI_PROVISIONING_H
#define WIFI_PROVISIONING_H

#include <Arduino.h>

struct WifiProvisioningConfig {
  char ssid[33];
  char password[65];
  char commandToken[65];

  WifiProvisioningConfig()
    : ssid{0}
    , password{0}
    , commandToken{0} {
  }
};

class WifiProvisioning {
public:
  static bool load(WifiProvisioningConfig& config);
  static bool save(const WifiProvisioningConfig& config);
  static bool saveCommandToken(const char* token);
  static void clear();
  static bool promptIfMissing(WifiProvisioningConfig& config);
  static bool clearRequestedOnBoot(uint32_t timeoutMs = 3000);

private:
  static bool readLine(const char* prompt, char* buffer, size_t bufferSize, bool allowEmpty);
};

#endif // WIFI_PROVISIONING_H
