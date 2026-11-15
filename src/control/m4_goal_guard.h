#ifndef MOTIONBRAIN_M4_GOAL_GUARD_H
#define MOTIONBRAIN_M4_GOAL_GUARD_H
#include <stdint.h>
#include <stddef.h>
#ifndef MOTIONBRAIN_ENABLE_M4_GOAL_GUARD
#define MOTIONBRAIN_ENABLE_M4_GOAL_GUARD 0
#endif
// Pure state machine; m4_goal_service supplies HTTP/sensor/driver integration.
// A true tick/cancel result requests adapter hardOFF; it does not perform OFF.
// Default build disabled; explicit commissioning environment only.
namespace m4_goal {
static constexpr const char* POLICY_SHA = "ee16b5b5f8cbc1df7d4927dd4139b7f8b767927eb39c72de5fb843d6494d09f9";
static constexpr const char* CURRENT_POLICY_SHA = "77d5ce74eacb65cb37ee2875a611e32489894b981c0aaf0b41442cf617b8f87a";
static constexpr uint32_t DURATION_MS=1200, SENSOR_AGE_MS=60, TICK_GAP_MS=40;
static constexpr size_t CAPACITY=32;
struct Request {
  char nonce[33], bootId[33], scopeSha256[65], policySha256[65];
  bool up; uint8_t percent; uint32_t durationMs; float expectedStartDeg,goalDeg,tableReferenceDeg;
  Request();
};
struct Snapshot {
  uint32_t now,sampleAt; float angle; bool ready,armed,safetyClear,allOff,outputAllowed,moving;
};
struct Record {
  Request request; const char* state; const char* reason;
  uint32_t queuedAtMs,runAtMs,decisionAtMs,offAtMs,sampleAtMs,sampleAgeMs,maxTickGapMs,firstTickDelayMs;
  float startAngleDeg,lastAngleDeg; bool offVerified,outputMayHaveOccurred,offConfirmationRecorded;
  Record();
};
class Guard {
 public:
  Guard();
  bool queue(const Request&,const char* boot,uint32_t now);
  bool begin(const char* nonce,const char* boot,const char* scope,const char* policy,const Snapshot&);
  bool tick(const Snapshot&); // true => adapter must hardOFF before any next driver update
  bool cancel(const char* reason,uint32_t now);
  void confirmOff(uint32_t at,bool verified);
  bool active() const;
  bool owns() const;
  const Record* current() const;
  const Record* find(const char* nonce) const;
  size_t count() const { return count_; }
  static bool valid(const Request&);
 private:
  Record records_[CAPACITY]; size_t count_; int current_; uint32_t previousTick_; bool ticked_;
  bool terminal(const char* reason,uint32_t now);
};
#if MOTIONBRAIN_ENABLE_M4_GOAL_GUARD
bool activeWindow();
void setActiveWindow(bool);
#else
inline bool activeWindow(){return false;}
inline void setActiveWindow(bool){}
#endif
}
#endif
