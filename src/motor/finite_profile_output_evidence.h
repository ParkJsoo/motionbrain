#ifndef MOTIONBRAIN_FINITE_PROFILE_OUTPUT_EVIDENCE_H
#define MOTIONBRAIN_FINITE_PROFILE_OUTPUT_EVIDENCE_H

#ifndef MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER
#define MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER 0
#endif
#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER
#include "applied_output_log.h"
#include <limits.h>
#include <string.h>

namespace finite_profile {

struct DriverOutputEntry {
  uint32_t sequence;
  uint32_t atMs;
  int16_t pwm;
  uint8_t motor;
  bounded::OutputReason reason;
};

struct DriverOutputEvidence {
  static const size_t CAPACITY = 64;
  char bootId[33];
  char commandId[129];
  uint32_t requestEpoch;
  uint32_t lease;
  uint32_t ordinal;
  uint32_t openedAt;
  uint32_t closedAt;
  uint32_t totalWrites;
  uint32_t dropped;
  uint8_t count;
  bool opened;
  bool closed;
  bool captureComplete;
  DriverOutputEntry entries[CAPACITY];
  DriverOutputEvidence() : bootId{}, commandId{}, requestEpoch(0), lease(0), ordinal(0),
      openedAt(0), closedAt(0), totalWrites(0), dropped(0), count(0), opened(false),
      closed(false), captureComplete(true), entries{} {}
};

// Driver-owned lifetime, no allocation, eviction, deduplication or pointer sink.
// All calls share the driver's external serialization requirement.
class DriverEvidenceStore {
public:
  static const size_t CAPACITY = 32;
  DriverEvidenceStore() : records_{}, count_(0), active_(CAPACITY) {}
  bool bind(const char* boot, const char* command, uint32_t epoch,
            uint32_t lease, uint32_t ordinal, uint32_t now) {
    if (!valid(boot, 33) || !valid(command, 129) || !epoch || !lease || !ordinal ||
        active_ != CAPACITY || count_ == CAPACITY || find(boot, command) || find(lease)) return false;
    DriverOutputEvidence& record = records_[count_];
    strcpy(record.bootId, boot); strcpy(record.commandId, command);
    record.requestEpoch = epoch; record.lease = lease; record.ordinal = ordinal;
    record.openedAt = now; record.opened = true;
    active_ = count_++;
    return true;
  }
  bool canWriteNonzero(uint32_t lease) const {
    if (active_ == CAPACITY) return true; // Low-level unbound leases remain supported.
    const DriverOutputEvidence& record = records_[active_];
    return record.lease == lease && record.captureComplete &&
        record.count < DriverOutputEvidence::CAPACITY;
  }
  void append(uint32_t lease, uint32_t ordinal, uint32_t now, uint8_t motor,
              int16_t pwm, bounded::OutputReason reason) {
    if (active_ == CAPACITY) return;
    DriverOutputEvidence& record = records_[active_];
    const bool exhausted = record.totalWrites == UINT32_MAX;
    increment(record.totalWrites);
    if (exhausted || record.lease != lease || record.ordinal != ordinal ||
        record.count == DriverOutputEvidence::CAPACITY) {
      record.captureComplete = false; increment(record.dropped); return;
    }
    record.entries[record.count++] = {record.totalWrites, now, pwm, motor, reason};
  }
  void close(uint32_t lease, uint32_t now) {
    if (active_ == CAPACITY || records_[active_].lease != lease) return;
    records_[active_].closed = true; records_[active_].closedAt = now;
    active_ = CAPACITY;
  }
  const DriverOutputEvidence* find(const char* boot, const char* command) const {
    if (!valid(boot, 33) || !valid(command, 129)) return nullptr;
    for (size_t i = 0; i < count_; ++i)
      if (!strcmp(records_[i].bootId, boot) && !strcmp(records_[i].commandId, command)) return &records_[i];
    return nullptr;
  }
  const DriverOutputEvidence* find(uint32_t lease) const {
    if (!lease) return nullptr;
    for (size_t i = 0; i < count_; ++i)
      if (records_[i].lease == lease) return &records_[i];
    return nullptr;
  }
  size_t count() const { return count_; }
private:
  DriverOutputEvidence records_[CAPACITY];
  size_t count_;
  size_t active_;
  static void increment(uint32_t& value) { if (value != UINT32_MAX) ++value; }
  static bool valid(const char* value, size_t capacity) {
    if (!value || !value[0]) return false;
    for (size_t i = 0; i < capacity; ++i) {
      const unsigned char c = static_cast<unsigned char>(value[i]);
      if (!c) return true;
      if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
            (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == ':')) return false;
    }
    return false;
  }
};

} // namespace finite_profile
#endif
#endif
