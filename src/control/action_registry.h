#ifndef MOTIONBRAIN_ACTION_REGISTRY_H
#define MOTIONBRAIN_ACTION_REGISTRY_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace bounded {

enum class Kind : uint8_t { Pulse, Shoulder };
enum class State : uint8_t { Running, Completed, TimedOut, Cancelled, Failed, OutcomeUnknown };
enum class Reason : uint8_t {
  None, DurationElapsed, DeadlineExceeded, TargetReached, ControllerFailed,
  Cancelled, SafetyBlocked, StartFailed, OutputUnconfirmed, OperatorOverride, StateChanged
};
enum class SubmitCode : uint8_t {
  Accepted, Duplicate, Conflict, Invalid, BootMismatch, Busy, RegistryFull, SafetyBlocked
};
enum class LookupCode : uint8_t { Found, NotFound, BootMismatch, Invalid };
enum class ShoulderResult : uint8_t { Running, Reached, Failed };

struct Request {
  char bootId[33];
  char commandId[129];
  Kind kind;
  uint8_t motor;
  int16_t pwm;
  uint32_t durationMs;
  float targetDegrees;
  uint8_t percent;
  Request() : bootId{}, commandId{}, kind(Kind::Pulse), motor(0), pwm(0),
              durationMs(0), targetDegrees(0), percent(0) {}
};

struct Record {
  uint32_t ordinal;
  Request request;
  State state;
  Reason reason;
  uint32_t startedAtMs;
  uint32_t endedAtMs;
  uint32_t elapsedMs;
  uint32_t overrunMs;
  bool outputOff;
  bool targetReached;
  Record() : ordinal(0), state(State::Running), reason(Reason::None),
             startedAtMs(0), endedAtMs(0), elapsedMs(0), overrunMs(0),
             outputOff(false), targetReached(false) {}
};

struct SubmitResult { SubmitCode code; const Record* record; };
struct LookupResult { LookupCode code; const Record* record; };

// Records never expire or evict within a boot. A full registry rejects new IDs.
// This intentionally trades availability for immunity to delayed duplicate requests.
class ActionRegistry {
public:
  static const size_t CAPACITY = 32;
  ActionRegistry() : count_(0) {}
  size_t count() const { return count_; }
  const Record* at(size_t index) const { return index < count_ ? &records_[index] : nullptr; }
private:
  Record records_[CAPACITY];
  size_t count_;
  friend class Executor;
  Record* find(const char* commandId) {
    for (size_t i = 0; i < count_; ++i)
      if (strcmp(records_[i].request.commandId, commandId) == 0) return &records_[i];
    return nullptr;
  }
  Record* reserve(const Request& request, uint32_t now) {
    if (count_ == CAPACITY) return nullptr;
    Record& record = records_[count_++];
    record.ordinal = static_cast<uint32_t>(count_);
    record.request = request;
    record.startedAtMs = now;
    return &record;
  }
};

const char* toString(Kind value);
const char* toString(State value);
const char* toString(Reason value);
const char* toString(SubmitCode value);
const char* toString(LookupCode value);

} // namespace bounded
#endif
