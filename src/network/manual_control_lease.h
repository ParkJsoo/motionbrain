#ifndef MOTIONBRAIN_MANUAL_CONTROL_LEASE_H
#define MOTIONBRAIN_MANUAL_CONTROL_LEASE_H
#include <stdint.h>
#include <stddef.h>

namespace manual_control {
constexpr uint32_t TTL_MS=750;
constexpr size_t ID_CAPACITY=44; // boot32-motor1-generation8, plus NUL
struct Request {
  bool start;
  uint8_t motor;
  bool forward;
  uint8_t percent;
  char boot[33];
  uint32_t generation;
  char leaseId[ID_CAPACITY];
  uint32_t sequence;
  Request();
};
// No Arduino/String conversion: malformed/overflow/duplicate wire fields must
// never reach dispatch. HTTP duplicate-field checking remains at the adapter.
bool parseRequest(const char* mode,const char* boot,const char* generation,
                  const char* leaseId,const char* sequence,uint8_t motor,
                  bool forward,uint8_t percent,Request& out);
class Port {
public:
  virtual ~Port() {}
  virtual uint32_t now() const=0;
  virtual uint64_t controlEpoch(uint8_t motor) const=0;
  virtual bool dispatch()=0;
  virtual void expired(uint8_t motor)=0;
};
struct State {
  uint32_t generation,acceptedAt,lastSequence;
  uint64_t observedEpoch;
  bool active,forward,exhausted;
  char leaseId[ID_CAPACITY];
};
class Leases {
public:
  Leases();
  bool init(const char* boot,Port& port);
  const char* bootId() const{return boot_;}
  const State* state(uint8_t motor) const;
  bool anyActive() const;
  // External serial/OOB STOP or another target owner invalidates the token;
  // observation alone never stops the new owner.
  void synchronize(Port& port);
  uint32_t tick(Port& port);
  void invalidate(uint8_t motor,Port& port);
  void clear(Port& port);
  // False means no lease receipt. dispatched distinguishes existing dispatcher
  // rejection from pre-dispatch admission rejection. Rejection never stops a
  // fresh gesture; only the independent expiry watchdog writes OFF.
  bool execute(const Request& request,Port& port,bool& dispatched,const char*& reason,
               uint32_t* checkedAt = nullptr);
private:
  char boot_[33];
  State states_[5];
  void revoke(uint8_t index);
};
}
#endif
