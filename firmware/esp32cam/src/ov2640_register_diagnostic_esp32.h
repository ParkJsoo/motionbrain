#pragma once
#include <Arduino.h>
#include <driver/i2c.h>
#include <driver/ledc.h>
#include <esp_ota_ops.h>
#include <esp_timer.h>
#include "ov2640_register_diagnostic.h"

#if !CONFIG_SCCB_HARDWARE_I2C_PORT1
#error Register diagnostic requires the audited camera driver on I2C port 1
#endif

namespace ov2640_diag {
// Reuse the camera-owned port, with no Wire/reinitialization. The dedicated
// build has one loop owning all SCCB calls; CAM HAL only performs DMA.
class Esp32Bus {
 public:
  uint64_t nowUs() const { return esp_timer_get_time(); }
  ReadResult read(uint8_t reg) {
    ReadResult result;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) { result.addressStatus = ESP_ERR_NO_MEM; return result; }
    esp_err_t err = i2c_master_start(cmd);
    if (err == ESP_OK) err = i2c_master_write_byte(cmd, 0x30 << 1, true);
    if (err == ESP_OK) err = i2c_master_write_byte(cmd, reg, true);
    if (err == ESP_OK) err = i2c_master_stop(cmd);
    if (err == ESP_OK) err = i2c_master_cmd_begin(I2C_NUM_1, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);
    result.addressStatus = err;
    if (err != ESP_OK) return result;
    cmd = i2c_cmd_link_create();
    if (!cmd) { result.status = ESP_ERR_NO_MEM; return result; }
    uint8_t value = 0;
    err = i2c_master_start(cmd);
    if (err == ESP_OK) err = i2c_master_write_byte(cmd, (0x30 << 1) | 1, true);
    if (err == ESP_OK) err = i2c_master_read_byte(cmd, &value, I2C_MASTER_NACK);
    if (err == ESP_OK) err = i2c_master_stop(cmd);
    if (err == ESP_OK) err = i2c_master_cmd_begin(I2C_NUM_1, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);
    result.status = err;
    if (err == ESP_OK) result.value = value;
    return result;
  }
  int writeBank(uint8_t value) {
    if (value > 1) return ESP_ERR_INVALID_ARG;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) return ESP_ERR_NO_MEM;
    esp_err_t err = i2c_master_start(cmd);
    if (err == ESP_OK) err = i2c_master_write_byte(cmd, 0x30 << 1, true);
    if (err == ESP_OK) err = i2c_master_write_byte(cmd, 0xff, true);
    if (err == ESP_OK) err = i2c_master_write_byte(cmd, value, true);
    if (err == ESP_OK) err = i2c_master_stop(cmd);
    if (err == ESP_OK) err = i2c_master_cmd_begin(I2C_NUM_1, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);
    return err;
  }
};

inline void number(String& json, uint64_t value) {
  char digits[24];
  snprintf(digits, sizeof(digits), "%llu", static_cast<unsigned long long>(value));
  json += digits;
}
inline String appElfHash() {
  const esp_app_desc_t* app = esp_ota_get_app_description();
  char hash[65];
  for (unsigned i = 0; i < 32; ++i) snprintf(hash + i*2, 3, "%02x", app->app_elf_sha256[i]);
  return String(hash);
}
inline String json(const Snapshot& s, const char* nonce, const char* boot, unsigned attempt,
                   uint32_t xclk, int sensorPid, bool closed, int deinitStatus) {
  String out;
  out.reserve(14000);
  out = "{\"schemaVersion\":\"motionbrain.ov2640_register_snapshot.v1\",\"nonce\":\"";
  out += nonce; out += "\",\"cameraBootId\":\""; out += boot;
  out += "\",\"appElfSha256\":\""; out += appElfHash();
  out += "\",\"attempt\":"; out += attempt;
  out += ",\"ok\":"; out += s.ok ? "true" : "false";
  out += ",\"failure\":\""; out += s.failure; out += "\",\"startedUs\":"; number(out, s.startedUs);
  out += ",\"endedUs\":"; number(out, s.endedUs);
  out += ",\"bankRestored\":"; out += s.bankRestored ? "true" : "false";
  out += ",\"xclkRequestedHz\":20000000,\"xclkConfiguredHz\":"; out += xclk;
  out += ",\"sensorPidReported\":"; out += sensorPid;
  out += ",\"sensorAddress\":48,\"frameSize\":\"qvga\",\"frameWidth\":320,\"frameHeight\":240,\"jpegQuality\":15";
  out += ",\"internalAcquisition\":true,\"imageExports\":0,\"imageRequests\":0,\"atomicSnapshot\":false";
  out += ",\"exposureUpperBoundUs\":null,\"exposureUpperBoundVerified\":false,\"opticalExposureVerified\":false";
  out += ",\"trainingEligible\":false,\"controlEligible\":false,\"trialClosed\":";
  out += closed ? "true" : "false";
  out += ",\"cameraDeinitStatus\":";
  if (deinitStatus == NOT_ATTEMPTED) out += "null"; else out += deinitStatus;
  out += ",\"transactions\":[";
  for (size_t i = 0; i < s.count; ++i) {
    const Transaction& t = s.transactions[i];
    if (i) out += ",";
    out += "{\"op\":\""; out += t.write ? "write_bank" : "read";
    out += "\",\"bank\":"; out += t.bank;
    out += ",\"register\":"; out += t.reg;
    out += ",\"value\":"; if (t.value < 0) out += "null"; else out += t.value;
    out += ",\"addressStatus\":"; if (t.write) out += "null"; else out += t.addressStatus;
    out += ",\"status\":"; out += t.status;
    out += ",\"startedUs\":"; number(out, t.startedUs);
    out += ",\"endedUs\":"; number(out, t.endedUs); out += "}";
  }
  out += "]}";
  return out;
}
}  // namespace ov2640_diag
