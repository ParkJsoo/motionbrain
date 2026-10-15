#ifndef MOTIONBRAIN_FRESH_CAPTURE_H
#define MOTIONBRAIN_FRESH_CAPTURE_H
#include "source_frame_metadata.h"

namespace camera_fresh {
constexpr int64_t BUDGET_US = 200000;
constexpr unsigned MAX_FRAMES = 4;
enum class Mode { Legacy, Fresh, Invalid };
inline Mode mode(unsigned count, const char* value) {
  if (!count) return Mode::Legacy;
  return count == 1 && value && strcmp(value, "1") == 0 ? Mode::Fresh : Mode::Invalid;
}
struct Result {
  const char* error = nullptr;
  unsigned discarded = 0;
};

// This is a selection budget, NOT a cancellable SDK call or HTTP deadline.
// get() may block beyond it; check again after every get and return that buffer.
// Caller owns the successful frame. Every rejected frame is returned exactly once.
template <typename Frame, typename Get, typename Release, typename Clock, typename Alive, typename Yield>
Frame* acquire(const char* boot, int64_t requestedUs, Result& result,
               Get get, Release release, Clock clock, Alive alive, Yield yield) {
  result = Result{};
  if (requestedUs <= 0) { result.error = "fresh_invalid_clock"; return nullptr; }
  int64_t previousNow = requestedUs;
  for (unsigned attempt = 0; attempt < MAX_FRAMES; ++attempt) {
    const int64_t before = clock();
    if (before < previousNow) { result.error = "fresh_invalid_clock"; return nullptr; }
    if (before - requestedUs >= BUDGET_US) { result.error = "fresh_budget_exceeded"; return nullptr; }
    if (!alive()) { result.error = "fresh_client_disconnected"; return nullptr; }
    Frame* frame = get();
    const int64_t after = clock();
    const bool connected = alive();
    const char* failure = nullptr;
    if (after < before) failure = "fresh_invalid_clock";
    else if (after - requestedUs >= BUDGET_US) failure = "fresh_budget_exceeded";
    else if (!connected) failure = "fresh_client_disconnected";
    else if (!frame) failure = "fresh_capture_failed";
    if (failure) {
      if (frame) { release(frame); ++result.discarded; }
      result.error = failure;
      return nullptr;
    }
    const auto source = camera_source::identify(boot, frame->timestamp.tv_sec, frame->timestamp.tv_usec, after);
    if (!source.valid) {
      release(frame); ++result.discarded;
      result.error = "fresh_invalid_timestamp";
      return nullptr;
    }
    if (source.capturedUs > static_cast<uint64_t>(requestedUs)) return frame;
    release(frame); ++result.discarded;
    previousNow = after;
    if (attempt + 1 < MAX_FRAMES) yield();
  }
  result.error = "fresh_frame_limit";
  return nullptr;
}

inline size_t headers(int64_t requestedUs, unsigned discarded, char* out, size_t capacity) {
  if (!out || !capacity) return 0;
  out[0] = '\0';
  if (requestedUs <= 0 || discarded >= MAX_FRAMES) return 0;
  const int n = snprintf(out, capacity,
    "X-MotionBrain-Capture-Mode: after_request\r\n"
    "X-MotionBrain-Capture-Requested-Us: %llu\r\n"
    "X-MotionBrain-Discarded-Frames: %u\r\n",
    static_cast<unsigned long long>(requestedUs), discarded);
  if (n < 0 || static_cast<size_t>(n) >= capacity) { out[0] = '\0'; return 0; }
  return static_cast<size_t>(n);
}
}
#endif
