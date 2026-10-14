#ifndef MOTIONBRAIN_SOURCE_FRAME_METADATA_H
#define MOTIONBRAIN_SOURCE_FRAME_METADATA_H
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

namespace camera_source {
struct Metadata {
  bool valid = false;
  char bootId[33] = {};
  char frameId[64] = {};
  uint64_t capturedUs = 0;
  uint64_t sourceAgeUs = 0;
};

// esp_camera.h: camera_fb_t.timestamp is "since boot of the first DMA buffer
// of the frame". This is not HTTP request time or the end of exposure.
inline Metadata identify(const char* bootId, int64_t seconds, int64_t micros, int64_t nowUs) {
  Metadata result;
  if (!bootId || seconds < 0 || micros < 0 || micros >= 1000000 || nowUs <= 0 ||
      seconds > nowUs / 1000000) return result;
  for (size_t i = 0; i < 32; ++i) {
    const char c = bootId[i];
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return result;
  }
  if (bootId[32] != '\0') return result;
  const uint64_t captured = static_cast<uint64_t>(seconds) * 1000000ULL + static_cast<uint64_t>(micros);
  if (captured == 0 || captured > static_cast<uint64_t>(nowUs)) return result;
  memcpy(result.bootId, bootId, sizeof(result.bootId));
  result.capturedUs = captured;
  result.sourceAgeUs = static_cast<uint64_t>(nowUs) - captured;
  snprintf(result.frameId, sizeof(result.frameId), "%s:%llu", bootId,
           static_cast<unsigned long long>(captured));
  result.valid = true;
  return result;
}

inline size_t headers(const Metadata& metadata, char* out, size_t capacity) {
  if (!out || capacity == 0) return 0;
  out[0] = '\0';
  if (!metadata.valid) return 0; // Pixels may still be displayed; identity is unknown.
  const int written = snprintf(out, capacity,
    "X-MotionBrain-Camera-Boot-Id: %s\r\n"
    "X-MotionBrain-Frame-Id: %s\r\n"
    "X-MotionBrain-Captured-Us: %llu\r\n"
    "X-MotionBrain-Source-Age-Us: %llu\r\n"
    "X-MotionBrain-Clock-Domain: camera_boot_us\r\n"
    "X-MotionBrain-Timestamp-Semantics: first_dma_buffer\r\n",
    metadata.bootId, metadata.frameId,
    static_cast<unsigned long long>(metadata.capturedUs),
    static_cast<unsigned long long>(metadata.sourceAgeUs));
  if (written < 0 || static_cast<size_t>(written) >= capacity) {
    out[0] = '\0'; return 0;
  }
  return static_cast<size_t>(written);
}
}
#endif
