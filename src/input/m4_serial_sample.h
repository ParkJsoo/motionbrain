#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// RS starts a dedicated read-only frame. Everything until its newline is
// consumed here, including malformed command text; it never reaches dispatch.
class M4SampleFrame {
public:
  bool consume(char c) {
    if (c == '\x1e') { active_ = true; ready_ = false; valid_ = true; size_ = 0; return true; }
    if (!active_) return false;
    if (c == '\r' || c == '\n') { active_ = false; ready_ = true; return true; }
    if (size_ >= 9 || !isHex(c)) valid_ = false;
    else nonce_[size_++] = c;
    return true;
  }
  bool ready() const { return ready_; }
  bool fresh() const { return valid_ && size_ == 9 && nonce_[0] == '2'; }
  const char* nonce() { nonce_[size_] = '\0'; return valid_ && (size_ == 8 || fresh()) ? nonce_ + (fresh() ? 1 : 0) : nullptr; }
  void reset() { active_ = false; ready_ = false; size_ = 0; }
  static bool isHex(char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'); }
private:
  bool active_ = false, ready_ = false, valid_ = true;
  uint8_t size_ = 0;
  char nonce_[10] = {};
};

inline int formatM4Sample(char* out, size_t capacity, const char* nonce, const char* boot,
                          uint16_t raw, uint32_t polledMs, uint32_t reportedMs, uint8_t flags) {
  if (!out || !nonce || !boot || strlen(nonce) != 8 || strlen(boot) != 32 || raw > 4095) return 0;
  for (size_t i = 0; i < 8; ++i) if (!M4SampleFrame::isHex(nonce[i])) return 0;
  for (size_t i = 0; i < 32; ++i) if (!M4SampleFrame::isHex(boot[i])) return 0;
  const int n = snprintf(out, capacity, "M4S1 %s %s %04x %08lx %08lx %02x\n", nonce, boot,
                         static_cast<unsigned>(raw), static_cast<unsigned long>(polledMs),
                         static_cast<unsigned long>(reportedMs), static_cast<unsigned>(flags));
  return n > 0 && static_cast<size_t>(n) < capacity ? n : 0;
}

// One pending read; no forced sensor polling, stream, or retry. The normal
// control loop must advance its poll marker strictly beyond request receipt.
class PendingM4Sample {
public:
  bool begin(const char* nonce, uint32_t now) {
    if (active_ || !nonce || strlen(nonce) != 8) return false;
    for (size_t i=0; i<8; ++i) if (!M4SampleFrame::isHex(nonce[i])) return false;
    memcpy(nonce_, nonce, 9); received_ = now; active_ = true; return true;
  }
  bool ready(uint32_t marker, uint32_t now, bool safe) {
    if (!active_) return false;
    if (!safe || uint32_t(now - received_) > 150) { cancel(); return false; }
    const uint32_t advance = marker - received_;
    return advance > 0 && advance <= uint32_t(now - received_);
  }
  bool active() const { return active_; }
  void cancel() { active_ = false; }
  const char* nonce() const { return nonce_; }
  uint32_t receivedMs() const { return received_; }
private:
  bool active_ = false;
  uint32_t received_ = 0;
  char nonce_[9] = {};
};

inline int formatFreshM4Sample(char* out, size_t capacity, const char* nonce, const char* boot,
                               uint16_t raw, uint32_t polledMs, uint32_t reportedMs,
                               uint32_t receivedMs, uint8_t flags) {
  char checked[80];
  if (!formatM4Sample(checked, sizeof(checked), nonce, boot, raw, polledMs, reportedMs, flags)) return 0;
  const uint32_t wait = reportedMs - receivedMs, advance = polledMs - receivedMs;
  if (!out || wait > 150 || advance == 0 || advance > wait) return 0;
  const int n = snprintf(out, capacity, "M4S2 %s %s %04x %08lx %08lx %08lx %02x\n", nonce, boot,
                         static_cast<unsigned>(raw), static_cast<unsigned long>(polledMs),
                         static_cast<unsigned long>(reportedMs), static_cast<unsigned long>(receivedMs),
                         static_cast<unsigned>(flags));
  return n > 0 && static_cast<size_t>(n) < capacity ? n : 0;
}
