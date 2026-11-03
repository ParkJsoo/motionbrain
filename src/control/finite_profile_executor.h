#ifndef MOTIONBRAIN_FINITE_PROFILE_EXECUTOR_H
#define MOTIONBRAIN_FINITE_PROFILE_EXECUTOR_H

#include "finite_profile_contract.h"

#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_CORE
namespace finite_profile {

static const size_t MAX_RECORDS = 32;
static const size_t MAX_CATALOG = 32;
static const uint32_t MAX_OFF_CHECKS = 64;

struct Token {
  uint8_t bytes[16];
  Token() : bytes{} {}
};

enum class Proof { Pending, Confirmed, Failed };
enum class State { Reserved, Running, Stopping, Completed, Cancelled, Expired, Failed, OutcomeUnknown };
enum class Reason {
  None, NominalEnd, Cancelled, ReservationExpired, SafetyChanged,
  DeadlineMissed, WriteFailed, EventOrder, OffUnconfirmed, TokenFailure, OffStateChanged
};
enum class Code {
  Accepted, Duplicate, Conflict, Invalid, Disabled, BootMismatch, EpochMismatch,
  Busy, SafetyBlocked, Capacity, NotFound, TokenMismatch, Expired,
  StaleEvent, NotDue, Rejected, EpochExhausted
};

struct Request {
  char bootId[33];
  char commandId[129];
  uint32_t expectedEpoch;
  Profile profile;
  Request() : bootId{}, commandId{}, expectedEpoch(0), profile() {}
};

struct Write {
  bool present;
  uint32_t atMs;
  int16_t pwm;
  Write() : present(false), atMs(0), pwm(0) {}
};

// Read-only receipt. No execution token is exposed by records or lookups.
struct Record {
  Request request;
  State state;
  Reason reason;
  uint32_t preparedAt;
  uint32_t executeAcceptedAt;
  uint32_t plannedStartAt;
  uint32_t plannedOffAt;
  uint32_t firstNonzeroWriteAt;
  uint32_t offWriteAt;
  uint32_t lastOffWriteAt;
  uint32_t offConfirmedAt;
  uint32_t lastOffCheckAt;
  uint32_t decisionAt;
  uint32_t quietEpoch;
  uint32_t offChecks;
  uint32_t offAttempts;
  uint32_t lastRetryAttemptAt;
  uint32_t lastRetryWriteAt;
  uint32_t lastRetryConfirmedAt;
  uint32_t duplicateEvents;
  uint8_t nextEventIndex;
  bool executed;
  bool anyNonzeroWrite;
  bool outputMayHaveOccurred;
  bool offWriteRecorded;
  bool offConfirmationRecorded;
  bool offCheckTimeRecorded;
  bool reservationOffInvalidated;
  bool softwareOffConfirmed;
  bool timingConformant;
  bool traceComplete;
  bool quietHeld;
  bool offFailureLatched;
  bool lastRetryWriteRecorded;
  bool lastRetryConfirmed;
  Write writes[MAX_POINTS];
  Record();
};

struct Result {
  Code code;
  const Record* record;
  Result(Code value = Code::Invalid, const Record* receipt = nullptr) : code(value), record(receipt) {}
};
struct PrepareResult {
  Code code;
  const Record* record;
  bool granted;
  Token token;
  PrepareResult(Code value = Code::Invalid, const Record* receipt = nullptr)
      : code(value), record(receipt), granted(false), token() {}
};

// Pure synchronous adapter; no production implementation exists. All public
// Executor calls must be externally serialized. Ports must never re-enter the
// Executor. No lock, RTOS scheduling or electrical OFF guarantee is implemented.
// apply/stop report the actual driver-write timestamp, not the later receipt time.
// offProof is nonblocking: Pending waits within the finite confirmation window;
// Failed latches the boot, even if a later explicit retry confirms software OFF.
class Port {
public:
  virtual ~Port() {}
  virtual uint32_t now() const = 0;
  virtual bool allowed() const = 0;
  // Includes all driver targets/current outputs being zero and no other owner.
  virtual bool idle() const = 0;
  virtual bool apply(uint8_t motor, int16_t pwm, uint32_t& writtenAtMs) = 0;
  virtual bool stop(uint32_t& writtenAtMs) = 0;
  virtual Proof offProof() = 0;
  virtual bool mintToken(Token& token) = 0;
};

class Executor {
public:
  Executor(Port& port, const char* bootId, const Profile* catalog, size_t catalogCount);
  PrepareResult prepare(const Request& request);
  Result execute(const char* bootId, const char* commandId, uint32_t expectedEpoch, const Token& token);
  void tick();
  // Adapters may deliver scheduled callbacks explicitly; tick dispatches only
  // the single next due event. No catch-up or arbitrary output method exists.
  Result event(const char* bootId, const char* commandId, uint32_t expectedEpoch, uint8_t index);
  Result cancel(const char* bootId, const char* commandId, uint32_t expectedEpoch);
  Result release(const char* bootId, const char* commandId, uint32_t quietEpoch);
  Result retryOff(const char* bootId, const char* commandId);
  const Record* find(const char* bootId, const char* commandId) const;
  uint32_t epoch() const { return epoch_; }
  bool blocked() const { return bootLatched_ || epochExhausted_; }
  bool idle() const { return owner_ == nullptr && !blocked(); }
  size_t count() const { return count_; }
  bool initialized() const { return initialized_; }
private:
  struct Slot {
    Record record;
    Token token;
    bool tokenIssued;
    bool tokenConsumed;
    State intendedState;
    Slot() : record(), token(), tokenIssued(false), tokenConsumed(false), intendedState(State::Failed) {}
  };
  Port& port_;
  char bootId_[33];
  Profile catalog_[MAX_CATALOG];
  size_t catalogCount_;
  Slot slots_[MAX_RECORDS];
  size_t count_;
  Slot* owner_;
  uint32_t epoch_;
  bool initialized_;
  bool bootLatched_;
  bool epochExhausted_;
  bool validIdentity(const char* bootId, const char* commandId) const;
  bool catalogContains(const Profile& profile) const;
  Slot* lookup(const char* commandId);
  bool tokenAvailable(const Token& token) const;
  bool advanceEpoch();
  void finish(State state, Reason reason);
  void confirmOff();
  bool probeOff(Proof& proof, uint32_t& checkedAt);
  void latchUnknown();
  void expireReservation();
};

} // namespace finite_profile
#endif
#endif
