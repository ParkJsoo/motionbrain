#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// No sensor controls: the only write the bus interface exposes is BANK_SEL.
// This is a sequential register observation, never an atomic exposure sample.
namespace ov2640_diag {
constexpr int NOT_ATTEMPTED = -32768;
constexpr uint64_t SNAPSHOT_US = 750000;
constexpr uint64_t SESSION_US = 30000000;
constexpr uint64_t READY_US = 60000000;
constexpr unsigned MAX_ATTEMPTS = 3;
constexpr uint64_t REQUEST_US = 250000;
constexpr uint64_t FINISH_RESERVE_US = 1000000;
constexpr uint8_t SENSOR_REGS[] = {
    0x0a, 0x0b, 0x1c, 0x1d, 0x03, 0x0c, 0x11, 0x12, 0x13,
    0x04, 0x10, 0x45, 0x2a, 0x2b, 0x2d, 0x2e, 0x46, 0x47,
    0x17, 0x18, 0x19, 0x1a, 0x32, 0x48, 0x49, 0x4e, 0x4f, 0x50};
constexpr uint8_t DSP_REGS[] = {0x05, 0xc2, 0xd3, 0xda, 0xe0, 0x86};
constexpr size_t MAX_TRANSACTIONS = 48;

struct ReadResult {
  int addressStatus = NOT_ATTEMPTED;
  int status = NOT_ATTEMPTED;
  int value = -1;
};
struct Transaction {
  bool write = false;
  int bank = -1;
  uint8_t reg = 0xff;
  int value = -1;
  int addressStatus = NOT_ATTEMPTED;
  int status = NOT_ATTEMPTED;
  uint64_t startedUs = 0;
  uint64_t endedUs = 0;
};
struct Snapshot {
  Transaction transactions[MAX_TRANSACTIONS];
  size_t count = 0;
  uint64_t startedUs = 0;
  uint64_t endedUs = 0;
  bool ok = false;
  bool bankRestored = false;
  const char* failure = "";
};

template<class Bus> int read(Bus& bus, Snapshot& out, int bank, uint8_t reg) {
  Transaction& t = out.transactions[out.count++];
  t.bank = bank; t.reg = reg; t.startedUs = bus.nowUs();
  const ReadResult value = bus.read(reg);
  t.addressStatus = value.addressStatus; t.status = value.status;
  t.value = value.addressStatus == 0 && value.status == 0 ? value.value : -1;
  t.endedUs = bus.nowUs();
  return t.value;
}
template<class Bus> bool bank(Bus& bus, Snapshot& out, uint8_t value) {
  Transaction& t = out.transactions[out.count++];
  t.write = true; t.bank = value; t.value = value; t.startedUs = bus.nowUs();
  t.status = bus.writeBank(value); t.endedUs = bus.nowUs();
  return t.status == 0;
}
template<class Bus> bool expired(Bus& bus, const Snapshot& out) {
  const uint64_t now = bus.nowUs();
  return now < out.startedUs || now - out.startedUs >= SNAPSHOT_US;
}

template<class Bus> Snapshot observe(Bus& bus) {
  Snapshot out;
  out.startedUs = bus.nowUs();
  const int original = read(bus, out, -1, 0xff);
  if (original != 0 && original != 1) {
    out.failure = "initial_bank_unverified";
    out.endedUs = bus.nowUs();
    return out;  // Never write if there is no known bank to restore.
  }
  for (int selected = 1; selected >= 0 && !*out.failure; --selected) {
    if (expired(bus, out)) { out.failure = "snapshot_deadline"; break; }
    if (!bank(bus, out, selected)) { out.failure = "bank_select_failed"; break; }
    if (read(bus, out, selected, 0xff) != selected) {
      out.failure = "bank_select_unverified"; break;
    }
    const uint8_t* regs = selected ? SENSOR_REGS : DSP_REGS;
    const size_t count = selected ? sizeof(SENSOR_REGS) : sizeof(DSP_REGS);
    for (size_t i = 0; i < count; ++i) {
      if (expired(bus, out)) { out.failure = "snapshot_deadline"; break; }
      const int value = read(bus, out, selected, regs[i]);
      if (value < 0 || value > 255) { out.failure = "register_read_failed"; break; }
      if (selected == 1 && regs[i] == 0x0a && value != 0x26) {
        out.failure = "sensor_pid_mismatch"; break;
      }
    }
  }
  // Cleanup is attempted once even after a failed select (which may have taken
  // effect). Driver bank cache was not touched; only restored hardware is safe.
  const bool restored = bank(bus, out, original);
  const int verified = read(bus, out, original, 0xff);
  out.bankRestored = restored && verified == original;
  if (!out.bankRestored) out.failure = "bank_restore_failed";
  if (!*out.failure && expired(bus, out)) out.failure = "snapshot_deadline";
  out.endedUs = bus.nowUs();
  out.ok = !*out.failure;
  return out;
}

inline bool validNonce(const char* nonce) {
  if (!nonce || strlen(nonce) != 16) return false;
  for (unsigned i = 0; i < 16; ++i)
    if (!((nonce[i] >= '0' && nonce[i] <= '9') || (nonce[i] >= 'a' && nonce[i] <= 'f'))) return false;
  return true;
}

struct TrialGate {
  unsigned attempts = 0;
  uint64_t readyUs = 0;
  uint64_t firstUs = 0;
  bool fault = false;
  char nonces[MAX_ATTEMPTS][17] = {};
  bool elapsed(uint64_t now) const {
    return now < readyUs || now-readyUs >= READY_US ||
        (attempts && (now < firstUs || now-firstUs >= SESSION_US));
  }
  const char* begin(const char* nonce, uint64_t now) {
    if (!validNonce(nonce)) return "invalid_nonce";
    if (fault || attempts >= MAX_ATTEMPTS || elapsed(now)) return "trial_closed";
    if (READY_US - (now-readyUs) < FINISH_RESERVE_US ||
        (attempts && SESSION_US-(now-firstUs) < FINISH_RESERVE_US)) return "trial_closed";
    for (unsigned i = 0; i < attempts; ++i) {
      if (strcmp(nonce, nonces[i]) == 0) { fault = true; return "nonce_reused"; }
    }
    if (!attempts) firstUs = now;
    strcpy(nonces[attempts++], nonce);
    return "";
  }
};

// Fixed route parser, bounded in bytes and elapsed time. No generic HTTP parser,
// body, redirects, arbitrary registers or write controls are accepted.
struct RequestParser {
  char data[1537] = {};
  size_t size = 0;
  bool complete = false;
  bool failed = false;
  uint64_t startedUs;
  explicit RequestParser(uint64_t now) : startedUs(now) {}
  bool expired(uint64_t now) const { return now < startedUs || now-startedUs >= REQUEST_US; }
  bool feed(char ch, uint64_t now) {
    if (failed || complete || expired(now) || size >= sizeof(data)-1 || ch == '\0') {
      failed = true; return false;
    }
    data[size++] = ch; data[size] = '\0';
    complete = size >= 4 && memcmp(data+size-4, "\r\n\r\n", 4) == 0;
    return true;
  }
  bool nonce(char* output) const {
    const char* prefix = "GET /diagnostics/ov2640?nonce=";
    const size_t prefixSize = strlen(prefix);
    if (!complete || failed || size < prefixSize+16+13 ||
        strncmp(data, prefix, prefixSize) ||
        memcmp(data+prefixSize+16, " HTTP/1.1\r\n", 11)) return false;
    memcpy(output, data+prefixSize, 16); output[16] = '\0';
    return validNonce(output);
  }
};
}  // namespace ov2640_diag
