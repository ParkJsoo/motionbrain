#ifndef MOTIONBRAIN_BOUNDED_RESPONSE_WRITE_H
#define MOTIONBRAIN_BOUNDED_RESPONSE_WRITE_H

#include <stddef.h>
#include <stdint.h>

namespace camera_http {
struct SendResult {
  size_t count;
  bool retryable;
};

// The caller supplies a nonblocking send. Header and body can share startedAt.
template <typename Send, typename Clock, typename Pause>
bool writeBounded(const uint8_t* data, size_t length, size_t chunkBytes,
                  uint32_t startedAt, uint32_t budgetMs, size_t& written,
                  Send send, Clock clock, Pause pause) {
  written = 0;
  if (chunkBytes == 0 || budgetMs == 0) return false;
  while (written < length) {
    if (static_cast<uint32_t>(clock() - startedAt) >= budgetMs) return false;
    const size_t remaining = length - written;
    const size_t chunk = remaining < chunkBytes ? remaining : chunkBytes;
    const auto result = send(data + written, chunk);
    if (result.count > chunk || (result.count == 0 && !result.retryable)) return false;
    written += result.count;
    if (result.count == 0) pause();
  }
  return static_cast<uint32_t>(clock() - startedAt) < budgetMs;
}
}  // namespace camera_http
#endif
