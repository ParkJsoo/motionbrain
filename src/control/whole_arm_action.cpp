#include "whole_arm_action.h"
#include <cmath>
#include <cstring>
#include <limits>

namespace whole_arm {
namespace {
constexpr uint32_t CAPS[] = {7000,4000,10000,4000,3000,14000};
constexpr float MAX_ANGLE[] = {224,194,208,208,194,224};
bool validTrial(const char* id) {
  if (!id || std::strlen(id)!=32) return false;
  for (unsigned i=0;i<32;i++)
    if (!((id[i]>='0'&&id[i]<='9')||(id[i]>='a'&&id[i]<='f'))) return false;
  return true;
}
bool validInputs(Inputs s, Phase phase) {
  return s.ready && std::isfinite(s.shoulderDeg) && s.shoulderDeg>=187
      && s.shoulderDeg<=MAX_ANGLE[static_cast<unsigned>(phase)];
}
}
uint32_t phaseCapMs(Phase p) { return CAPS[static_cast<unsigned>(p)]; }
uint8_t phaseMotor(Phase p) { return p==Phase::Close || p==Phase::Release ? 1 : 4; }
int16_t phasePwm(Phase p) { return p==Phase::Lift || p==Phase::Release || p==Phase::Retreat ? 70 : -70; }
void Controller::fail(const char* reason) {
  if (active_) port_.faultStop(phase_);
  active_=false;
  failed_=true;
  reason_=reason;
}
void Controller::abort() { if (owns()) fail("interrupted"); }
void Controller::tick() {
  if (!owns()) return;
  Inputs s=port_.inputs();
  if (!s.ready) { fail("interlock"); return; }
  if (!active_) {
    if (!s.allOff) fail("off_unverified");
    return;
  }
  const auto t=port_.now();
  if (!validInputs(s,phase_)) fail("angle_limit");
  else if (t-started_>=phaseCapMs(phase_)) fail("phase_timeout");
  else if (t-renewed_>=750) fail("lease_expired");
}

bool Controller::apply(const Request& r) {
  tick();
  if (done_ || failed_ || !validTrial(r.trial) ||
      static_cast<unsigned>(r.phase)>static_cast<unsigned>(Phase::Retreat) ||
      (trial_[0] && std::strcmp(trial_,r.trial)) ||
      seq_==std::numeric_limits<uint32_t>::max() || r.seq!=seq_+1) return false;
  if (r.op==Op::Start) {
    const unsigned expected=seq_ ? static_cast<unsigned>(phase_)+1 : 0;
    if (active_ || static_cast<unsigned>(r.phase)!=expected) return false;
    const Inputs s=port_.inputs();
    if (!s.allOff || !validInputs(s,r.phase)) return false;
    std::strcpy(trial_,r.trial);
    phase_=r.phase;
    seq_=r.seq;
    started_=renewed_=port_.now();
    active_=true;
    if (!port_.start(phase_)) { fail("driver_rejected"); return false; }
    reason_="active";
    return true;
  }
  if (!active_ || phase_!=r.phase) return false;
  if (r.op==Op::Keep) {
    seq_=r.seq;
    renewed_=port_.now();
    return true;
  }
  if (r.op!=Op::Stop) return false;
  seq_=r.seq;
  port_.stop(phase_);
  active_=false;
  const auto stopped=port_.inputs();
  if (!stopped.ready || !stopped.allOff) { fail("off_unverified"); return false; }
  done_=phase_==Phase::Retreat;
  reason_=done_ ? "completed" : "stopped";
  return true;
}
}
