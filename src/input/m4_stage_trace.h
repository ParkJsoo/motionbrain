#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <inttypes.h>
#include "input/m4_serial_sample.h"

// Passive firmware observations. Neither millis nor these microsecond markers
// identifies the AS5600 physical conversion or the UART/USB transmission end.
class M4StageTrace {
public:
  static const size_t CAPACITY = 8;
  struct Record {
    uint64_t sequence;
    char nonce[9];
    uint8_t protocol;
    const char* status;
    uint32_t requestReceivedMs, pollMarkerMs, replyReportedMs, replyPollMarkerMs;
    uint32_t pollAttempts;
    bool adcSampled, pollObserved, angleObserved, pollSucceeded, angleSucceeded, replyObserved, writeObserved;
    bool requestLoopObserved, requestLoopGapObserved, replyLoopObserved, angleCompletedObserved, pollCompletedObserved;
    uint16_t replyRaw;
    size_t replyBytes, writtenBytes;
    uint64_t requestParsedUs, requestLoopStartedUs, requestLoopGapUs;
    uint64_t sensorUpdateEnteredUs, afterAdcUs, pollStartedUs, angleReadStartedUs;
    uint64_t angleReadCompletedUs, pollCompletedUs, replyEnqueueUs, replyWriteReturnedUs;
    uint64_t replyLoopStartedUs, maxPendingLoopGapUs;
  };

  void loopStarted(uint64_t now) {
    lastLoopGapObserved_ = loopSeen_;
    lastLoopGap_ = loopSeen_ ? now - lastLoopStarted_ : 0;
    lastLoopStarted_ = now; loopSeen_ = true;
    if (active_ && lastLoopGap_ > current().maxPendingLoopGapUs)
      current().maxPendingLoopGapUs = lastLoopGap_;
  }
  bool begin(const char* boot, const char* nonce, uint32_t receivedMs, uint64_t parsedUs, bool compact) {
    char checked[80];
    if (active_ || !formatM4Sample(checked, sizeof(checked), nonce, boot, 0, 1, 1, 31)
        || (boot_[0] && strcmp(boot_, boot)) || nextSequence_ == UINT64_MAX) return false;
    memcpy(boot_, boot, 33);
    Record& r = records_[next_]; r = Record{};
    r.sequence = nextSequence_++; memcpy(r.nonce, nonce, 9);
    r.protocol = compact ? 3 : 2; r.status = "pending";
    r.requestReceivedMs = receivedMs; r.requestParsedUs = parsedUs;
    r.requestLoopStartedUs = lastLoopStarted_; r.requestLoopGapUs = lastLoopGap_;
    r.requestLoopObserved = loopSeen_; r.requestLoopGapObserved = lastLoopGapObserved_;
    activeIndex_ = next_; next_ = (next_ + 1) % CAPACITY;
    if (count_ == CAPACITY) { if (dropped_ != UINT64_MAX) ++dropped_; }
    else ++count_;
    active_ = true; return true;
  }
  bool active() const { return active_; }
  // Only the latest poll attempt is retained; pollAttempts preserves its denominator.
  void pollStarted(uint32_t marker, uint64_t entry, uint64_t afterAdc, uint64_t start, bool adcSampled) {
    if (!active_) return;
    Record& r = current();
    r.adcSampled = adcSampled;
    ++r.pollAttempts; r.pollObserved = true; r.pollSucceeded = false;
    r.angleObserved = false; r.angleSucceeded = false;
    r.angleCompletedObserved = false; r.pollCompletedObserved = false;
    r.pollMarkerMs = marker; r.sensorUpdateEnteredUs = entry; r.afterAdcUs = afterAdc;
    r.pollStartedUs = start; r.angleReadStartedUs = r.angleReadCompletedUs = r.pollCompletedUs = 0;
  }
  void angleStarted(uint64_t now) {
    if (active_ && current().pollObserved) {
      current().angleObserved = true; current().angleReadStartedUs = now;
    }
  }
  void angleCompleted(uint64_t now, bool success) {
    if (active_ && current().angleObserved) {
      current().angleReadCompletedUs = now; current().angleSucceeded = success; current().angleCompletedObserved = true;
    }
  }
  void pollCompleted(uint64_t now, bool success) {
    if (active_ && current().pollObserved) {
      current().pollCompletedUs = now; current().pollSucceeded = success; current().pollCompletedObserved = true;
    }
  }
  void replyEnqueued(uint32_t reported, uint32_t marker, uint16_t raw, size_t bytes, uint64_t now) {
    if (!active_) return;
    Record& r = current(); r.replyObserved = true; r.replyReportedMs = reported;
    r.replyPollMarkerMs = marker; r.replyRaw = raw; r.replyBytes = bytes;
    r.replyEnqueueUs = now; r.replyLoopStartedUs = lastLoopStarted_; r.replyLoopObserved = loopSeen_;
  }
  void writeReturned(size_t bytes, uint64_t now) {
    if (!active_ || !current().replyObserved) return;
    Record& r = current(); r.writeObserved = true; r.writtenBytes = bytes;
    r.replyWriteReturnedUs = now; r.status = bytes == r.replyBytes ? "replied" : "write_failed";
    active_ = false;
  }
  void cancel(bool expired = false) {
    if (active_) { current().status = expired ? "expired" : "cancelled"; active_ = false; }
  }
  void formatFailed() {
    if (active_) { current().status = "format_failed"; active_ = false; }
  }
  const Record* at(size_t index) const {
    return index < count_ ? &records_[(next_ + CAPACITY - count_ + index) % CAPACITY] : nullptr;
  }
  size_t count() const { return count_; }
  uint64_t dropped() const { return dropped_; }
  uint64_t nextSequence() const { return nextSequence_; }

  // Read-only snapshot formatting. No clear, activation, forced polling or I/O.
  // Incomplete stage markers are null, including unavailable previous-loop gap.
  bool formatJson(char* out, size_t capacity, const char* currentBoot) const {
    char checked[80];
    if (!out || !formatM4Sample(checked, sizeof(checked), "00000000", currentBoot, 0, 1, 1, 31)
        || (boot_[0] && strcmp(boot_, currentBoot))) return false;
    Writer w(out, capacity);
    w.add("{\"schemaVersion\":\"motionbrain.m4_stage_trace.v1\",\"bootId\":\"%s\","
          "\"clockDomain\":\"esp_timer_since_boot_us\",\"clockRateCalibrated\":false,"
          "\"physicalSampleTimeVerified\":false,\"txCompleteVerified\":false,"
          "\"capacity\":8,\"dropped\":%" PRIu64 ",\"nextSequence\":%" PRIu64 ",\"active\":%s,\"records\":[",
          currentBoot, dropped_, nextSequence_, active_ ? "true" : "false");
    for (size_t i = 0; i < count_; ++i) {
      const Record& r = *at(i);
      w.add("%s{\"sequence\":%" PRIu64 ",\"nonce\":\"%s\",\"protocol\":%u,\"status\":\"%s\","
            "\"requestReceivedMs\":%" PRIu32 ",\"requestParsedUs\":%" PRIu64 ","
            "\"pollAttempts\":%" PRIu32 ",\"pollSucceeded\":%s,\"angleSucceeded\":%s,\"adcSampled\":%s",
            i ? "," : "", r.sequence, r.nonce, unsigned(r.protocol), r.status, r.requestReceivedMs,
            r.requestParsedUs, r.pollAttempts, r.pollSucceeded ? "true" : "false", r.angleSucceeded ? "true" : "false", r.adcSampled ? "true" : "false");
      w.value("requestLoopStartedUs", r.requestLoopStartedUs, r.requestLoopObserved);
      w.value("requestLoopGapUs", r.requestLoopGapUs, r.requestLoopGapObserved);
      w.value("maxPendingLoopGapUs", r.maxPendingLoopGapUs, true);
      w.value("pollMarkerMs", r.pollMarkerMs, r.pollObserved);
      w.value("sensorUpdateEnteredUs", r.sensorUpdateEnteredUs, r.pollObserved);
      w.value("afterAdcUs", r.afterAdcUs, r.pollObserved);
      w.value("pollStartedUs", r.pollStartedUs, r.pollObserved);
      w.value("angleReadStartedUs", r.angleReadStartedUs, r.angleObserved);
      w.value("angleReadCompletedUs", r.angleReadCompletedUs, r.angleCompletedObserved);
      w.value("pollCompletedUs", r.pollCompletedUs, r.pollCompletedObserved);
      w.value("replyReportedMs", r.replyReportedMs, r.replyObserved);
      w.value("replyPollMarkerMs", r.replyPollMarkerMs, r.replyObserved);
      w.value("replyRaw", r.replyRaw, r.replyObserved);
      w.value("replyEnqueueUs", r.replyEnqueueUs, r.replyObserved);
      w.value("replyLoopStartedUs", r.replyLoopStartedUs, r.replyLoopObserved);
      w.value("replyBytes", r.replyBytes, r.replyObserved);
      w.value("replyWriteReturnedUs", r.replyWriteReturnedUs, r.writeObserved);
      w.value("writtenBytes", r.writtenBytes, r.writeObserved);
      w.add("}");
    }
    w.add("]}"); return w.ok;
  }
private:
  struct Writer {
    char* out; size_t cap, used = 0; bool ok = true;
    Writer(char* value, size_t capacity) : out(value), cap(capacity) { if (cap) out[0] = 0; }
    void add(const char* format, ...) {
      if (!ok) return;
      va_list args; va_start(args, format);
      const int n = vsnprintf(out + used, cap > used ? cap - used : 0, format, args); va_end(args);
      if (n < 0 || size_t(n) >= cap - used) { ok = false; return; }
      used += size_t(n);
    }
    void value(const char* key, uint64_t n, bool observed) {
      if (observed) add(",\"%s\":%" PRIu64, key, n);
      else add(",\"%s\":null", key);
    }
  };
  Record& current() { return records_[activeIndex_]; }
  Record records_[CAPACITY] = {};
  char boot_[33] = {};
  size_t next_ = 0, count_ = 0, activeIndex_ = 0;
  bool active_ = false, loopSeen_ = false, lastLoopGapObserved_ = false;
  uint64_t dropped_ = 0, nextSequence_ = 1, lastLoopStarted_ = 0, lastLoopGap_ = 0;
};
