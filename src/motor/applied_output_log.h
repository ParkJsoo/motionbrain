#ifndef MOTIONBRAIN_APPLIED_OUTPUT_LOG_H
#define MOTIONBRAIN_APPLIED_OUTPUT_LOG_H

#include <stddef.h>
#include <stdint.h>

namespace bounded {

enum class OutputReason : uint8_t { DriverWrite, ForcedOff };
struct OutputEntry {
  uint32_t sequence;
  uint32_t atMs;
  uint32_t actionOrdinal;
  uint8_t motor;
  int16_t pwm;
  OutputReason reason;
};

// Records values actually written by the driver, not requested target speed.
// This is software output evidence, not electrical measurement or joint travel.
class AppliedOutputLog {
public:
  static const size_t CAPACITY = 128;
  AppliedOutputLog() : entries_{}, head_(0), count_(0), nextSequence_(1), dropped_(0),
                       known_{}, lastPwm_{}, lastOrdinal_{} {}
  bool append(uint32_t now, uint32_t ordinal, uint8_t motor, int16_t pwm, OutputReason reason) {
    if (motor < 1 || motor > 5 || pwm < -255 || pwm > 255) return false;
    if (reason == OutputReason::ForcedOff && pwm != 0) return false;
    const size_t index = motor - 1;
    if (reason == OutputReason::DriverWrite && known_[index] &&
        lastPwm_[index] == pwm && lastOrdinal_[index] == ordinal) return false;
    known_[index] = true;
    lastPwm_[index] = pwm;
    lastOrdinal_[index] = ordinal;
    if (count_ == CAPACITY) { head_ = (head_ + 1) % CAPACITY; ++dropped_; }
    else ++count_;
    entries_[(head_ + count_ - 1) % CAPACITY] = {nextSequence_++, now, ordinal, motor, pwm, reason};
    return true;
  }
  size_t count() const { return count_; }
  uint32_t dropped() const { return dropped_; }
  const OutputEntry* at(size_t chronologicalIndex) const {
    return chronologicalIndex < count_ ? &entries_[(head_ + chronologicalIndex) % CAPACITY] : nullptr;
  }
private:
  OutputEntry entries_[CAPACITY];
  size_t head_;
  size_t count_;
  uint32_t nextSequence_;
  uint32_t dropped_;
  bool known_[5];
  int16_t lastPwm_[5];
  uint32_t lastOrdinal_[5];
};

inline const char* toString(OutputReason reason) {
  return reason == OutputReason::ForcedOff ? "forced_off" : "driver_write";
}
} // namespace bounded
#endif
