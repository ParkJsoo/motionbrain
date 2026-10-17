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
  bool fresh() const { return valid_ && size_ == 9 && (nonce_[0] == '2' || nonce_[0] == '3'); }
  bool compact() const { return fresh() && nonce_[0] == '3'; }
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
  bool begin(const char* nonce, uint32_t now, bool compact = false) {
    if (active_ || !nonce || strlen(nonce) != 8) return false;
    for (size_t i=0; i<8; ++i) if (!M4SampleFrame::isHex(nonce[i])) return false;
    memcpy(nonce_, nonce, 9); received_ = now; compact_ = compact; active_ = true; return true;
  }
  bool ready(uint32_t marker, uint32_t now, bool safe) {
    if (!active_) return false;
    if (!safe || uint32_t(now - received_) > 150) { cancel(); return false; }
    const uint32_t advance = marker - received_;
    return advance > 0 && advance <= uint32_t(now - received_);
  }
  bool active() const { return active_; }
  void cancel() { active_ = false; compact_ = false; }
  bool compact() const { return compact_; }
  const char* nonce() const { return nonce_; }
  uint32_t receivedMs() const { return received_; }
private:
  bool active_ = false, compact_ = false;
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

// CRC-32/IEEE, reflected polynomial 0xedb88320, initial/final xor 0xffffffff.
// Matches zlib.crc32(data); detects corruption, not authenticity or physical time.
inline uint32_t m4SampleCrc32(const uint8_t* data, size_t length) {
  uint32_t crc = 0xffffffffu;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
  }
  return crc ^ 0xffffffffu;
}

// S3 preserves the complete S2 information with no reconstructed timestamps.
// Payload: nonce[4], boot[16], raw[2], polled[4], reported[4], received[4],
// flags[1], CRC[4], all multibyte numbers big-endian. CRC covers the first 35
// bytes. The 39-byte payload encodes as exactly 52 Base64 characters without
// padding. "M4S3 " + payload + LF is 58 bytes; the caller also needs a NUL byte.
// Marker semantics, normal sensor polling and the 150ms fresh window are unchanged.
inline int formatCompactFreshM4Sample(char* out, size_t capacity, const char* nonce, const char* boot,
                                      uint16_t raw, uint32_t polledMs, uint32_t reportedMs,
                                      uint32_t receivedMs, uint8_t flags) {
  char checked[90];
  if (!out || capacity < 59 ||
      !formatFreshM4Sample(checked, sizeof(checked), nonce, boot, raw,
                           polledMs, reportedMs, receivedMs, flags)) return 0;
  uint8_t data[39] = {};
  size_t offset = 0;
  const auto nibble = [](char c) -> uint8_t {
    return static_cast<uint8_t>(c <= '9' ? c - '0' : c - 'a' + 10);
  };
  const char* hexFields[] = {nonce, boot};
  for (const char* hex : hexFields) {
    for (size_t i = 0; hex[i]; i += 2)
      data[offset++] = static_cast<uint8_t>((nibble(hex[i]) << 4) | nibble(hex[i + 1]));
  }
  data[offset++] = static_cast<uint8_t>(raw >> 8);
  data[offset++] = static_cast<uint8_t>(raw);
  const auto put32 = [&data, &offset](uint32_t value) {
    data[offset++] = static_cast<uint8_t>(value >> 24);
    data[offset++] = static_cast<uint8_t>(value >> 16);
    data[offset++] = static_cast<uint8_t>(value >> 8);
    data[offset++] = static_cast<uint8_t>(value);
  };
  put32(polledMs); put32(reportedMs); put32(receivedMs);
  data[offset++] = flags;
  const uint32_t crc = m4SampleCrc32(data, offset);
  put32(crc);
  static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  memcpy(out, "M4S3 ", 5);
  size_t written = 5;
  for (size_t i = 0; i < sizeof(data); i += 3) {
    const uint32_t block = (uint32_t(data[i]) << 16) | (uint32_t(data[i + 1]) << 8) | data[i + 2];
    out[written++] = alphabet[(block >> 18) & 63];
    out[written++] = alphabet[(block >> 12) & 63];
    out[written++] = alphabet[(block >> 6) & 63];
    out[written++] = alphabet[block & 63];
  }
  out[written++] = '\n'; out[written] = '\0';
  return static_cast<int>(written);
}
