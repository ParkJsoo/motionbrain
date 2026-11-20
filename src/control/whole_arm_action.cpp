#include "whole_arm_action.h"
#include <cmath>
#include <cstring>
#include <limits>

namespace whole_arm {
const CorrectionStep correctionWave[CORRECTION_STEPS] = {
  {0,-10},{82,-20},{147,-30},{206,-40},{262,-50},
  {347,-40},{403,-30},{453,-20},{503,-10},{554,0}
};
namespace {
constexpr uint32_t CAPS[] = {7000,4000,10000,4000,3000,14000,554+CORRECTION_LATE_MS,ELBOW_DOWN_CUTOFF_MS,315,435,365};
constexpr float MAX_ANGLE[] = {224,194,208,208,194,224,194,224,224,224,224};
bool validTrial(const char* id) {
  if (!id || std::strlen(id)!=32) return false;
  for (unsigned i=0;i<32;i++)
    if (!((id[i]>='0'&&id[i]<='9')||(id[i]>='a'&&id[i]<='f'))) return false;
  return true;
}
bool validInputs(Inputs s, Phase phase) {
  return s.ready && std::isfinite(s.shoulderDeg) && s.shoulderDeg>=((phase==Phase::ElbowDownOnce || phase==Phase::ElbowDown350) ? 219.f : 187.f)
      && s.shoulderDeg<=MAX_ANGLE[static_cast<unsigned>(phase)];
}
}
bool isApproachStep(Phase p) {
  return MOTIONBRAIN_ENABLE_BOUNDED_APPROACH &&
      (p==Phase::ShoulderDown300 || p==Phase::ShoulderDown420 || p==Phase::ElbowDown350);
}
uint32_t stepDurationMs(Phase p) {
  switch (p) {
    case Phase::ShoulderDown300: return 300;
    case Phase::ShoulderDown420: return 420;
    case Phase::ElbowDown350: case Phase::ElbowDownOnce: return ELBOW_DOWN_MS;
    default: return 0;
  }
}
uint32_t phaseCapMs(Phase p) { return CAPS[static_cast<unsigned>(p)]; }
uint8_t phaseMotor(Phase p) { if (p==Phase::ElbowDownOnce || p==Phase::ElbowDown350) return 3;
  return p==Phase::Close || p==Phase::Release ? 1 : 4; }
int16_t phasePwm(Phase p) {
  if (p==Phase::ElbowDownOnce || p==Phase::ElbowDown350) return 50;
  if (p==Phase::LowerCorrection) return -50;
  return p==Phase::Lift || p==Phase::Release || p==Phase::Retreat ? 70 : -70;
}
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
  else if (stepDurationMs(phase_) && t-started_>=stepDurationMs(phase_)) finish();
  else if (phase_==Phase::LowerCorrection) {
    const auto& step=correctionWave[nextCorrectionStep_];
    const uint32_t elapsed=t-started_;
    if (elapsed>=step.atMs+CORRECTION_LATE_MS) fail("correction_step_late");
    else if (elapsed>=step.atMs) {
      if (!step.pwm) finish();
      else if (!port_.correctionStep(nextCorrectionStep_)) fail("correction_write_failed");
      else ++nextCorrectionStep_;
    }
  }
}

bool Controller::apply(const Request& r) {
  tick();
  if (done_ || failed_ || !validTrial(r.trial) ||
      static_cast<unsigned>(r.phase)>static_cast<unsigned>(MOTIONBRAIN_ENABLE_BOUNDED_APPROACH ? Phase::ElbowDown350 :
                                   MOTIONBRAIN_ENABLE_ELBOW_DOWN_ONCE ? Phase::ElbowDownOnce : Phase::LowerCorrection) ||
      (trial_[0] && std::strcmp(trial_,r.trial)) ||
      seq_==std::numeric_limits<uint32_t>::max() || r.seq!=seq_+1) return false;
  if (r.op==Op::Start) {
    const Phase expected=!seq_ ? Phase::Approach : phase_==Phase::LowerCorrection
      ? Phase::Release : static_cast<Phase>(static_cast<unsigned>(phase_)+1);
    const bool optionalCorrection=phase_==Phase::Lower && r.phase==Phase::LowerCorrection;
    const bool elbowOnce=MOTIONBRAIN_ENABLE_ELBOW_DOWN_ONCE && !seq_ && r.phase==Phase::ElbowDownOnce;
    bool allowed=r.phase==expected || optionalCorrection || elbowOnce;
    if (isApproachStep(r.phase))
      allowed=(!seq_ || isApproachStep(phase_)) &&
          approachBudgetMs_+phaseCapMs(r.phase)<=phaseCapMs(Phase::Approach);
    else if (isApproachStep(phase_))
      allowed=r.phase==Phase::Close;
    if (active_ || !allowed) return false;
    const Inputs s=port_.inputs();
    if (!s.allOff || !validInputs(s,r.phase)) return false;
    std::strcpy(trial_,r.trial);
    phase_=r.phase;
    if (isApproachStep(phase_)) approachBudgetMs_+=phaseCapMs(phase_);
    seq_=r.seq;
    started_=renewed_=port_.now();
    nextCorrectionStep_=1;
    active_=true;
    if (!port_.start(phase_)) { fail("driver_rejected"); return false; }
    if (isApproachStep(phase_)) started_=port_.now();
    if (phase_==Phase::LowerCorrection) started_=port_.correctionStartedAt();
    reason_="active";
    return true;
  }
  if (!active_ || phase_!=r.phase) return false;
  if (phase_==Phase::LowerCorrection || stepDurationMs(phase_)) {
    if (r.op==Op::Stop) fail("interrupted");
    return false;
  }
  if (r.op==Op::Keep) {
    seq_=r.seq;
    renewed_=port_.now();
    return true;
  }
  if (r.op!=Op::Stop) return false;
  seq_=r.seq;
  return finish();
}

bool Controller::finish() {
  port_.stop(phase_);
  active_=false;
  const auto stopped=port_.inputs();
  if (!stopped.ready || !stopped.allOff) { fail("off_unverified"); return false; }
  done_=phase_==Phase::Retreat || phase_==Phase::ElbowDownOnce;
  reason_=done_ ? "completed" : "stopped";
  return true;
}
}
