#pragma once
#include <stdint.h>

namespace whole_arm {
enum class Phase : uint8_t { Approach, Close, Lift, Lower, Release, Retreat };
enum class Op : uint8_t { Start, Keep, Stop };
uint32_t phaseCapMs(Phase);
uint8_t phaseMotor(Phase);
int16_t phasePwm(Phase);
struct Request { Op op; Phase phase; const char* trial; uint32_t seq; };
struct Inputs { bool ready; bool allOff; float shoulderDeg; };
class Port {
public:
  virtual uint32_t now() const = 0;
  virtual Inputs inputs() const = 0;
  virtual bool start(Phase phase) = 0;
  virtual void stop(Phase phase) = 0;
  virtual void faultStop(Phase phase) { stop(phase); }
};
class Controller {
public:
  explicit Controller(Port& port) : port_(port) {}
  bool apply(const Request&);
  void tick();
  void abort();
  bool active() const { return active_; }
  bool failed() const { return failed_; }
  bool owns() const { return trial_[0] && !failed_ && !done_; }
  uint32_t seq() const { return seq_; }
  Phase phase() const { return phase_; }
  const char* trial() const { return trial_; }
  const char* reason() const { return reason_; }
private:
  void fail(const char*);
  Port& port_;
  char trial_[33]{};
  uint32_t seq_=0, started_=0, renewed_=0;
  Phase phase_=Phase::Approach;
  bool active_=false, failed_=false, done_=false;
  const char* reason_="idle";
};
}
