#pragma once
#include <stdint.h>

#ifndef MOTIONBRAIN_ENABLE_ELBOW_DOWN_ONCE
#define MOTIONBRAIN_ENABLE_ELBOW_DOWN_ONCE 0
#endif

#ifndef MOTIONBRAIN_ENABLE_BOUNDED_APPROACH
#define MOTIONBRAIN_ENABLE_BOUNDED_APPROACH 0
#endif

#ifndef MOTIONBRAIN_ENABLE_POSE_GRASP
#define MOTIONBRAIN_ENABLE_POSE_GRASP 0
#endif

namespace whole_arm {
constexpr uint32_t ELBOW_DOWN_MS=350;
constexpr uint32_t ELBOW_DOWN_CUTOFF_MS=365;
enum class Phase : uint8_t { Approach, Close, Lift, Lower, Release, Retreat, LowerCorrection, ElbowDownOnce,
                             ShoulderDown300, ShoulderDown420, ElbowDown350, WristUp420, PoseClose, ShoulderDown650 };
bool isApproachStep(Phase);
uint32_t stepDurationMs(Phase);
enum class Op : uint8_t { Start, Keep, Stop };
uint32_t phaseCapMs(Phase);
uint8_t phaseMotor(Phase);
int16_t phasePwm(Phase);
struct CorrectionStep { uint16_t atMs; int16_t pwm; };
constexpr uint8_t CORRECTION_STEPS=10;
constexpr uint32_t CORRECTION_LATE_MS=15;
extern const CorrectionStep correctionWave[CORRECTION_STEPS];
struct Request { Op op; Phase phase; const char* trial; uint32_t seq; };
struct Inputs { bool ready; bool allOff; float shoulderDeg; };
class Port {
public:
  virtual uint32_t now() const = 0;
  virtual Inputs inputs() const = 0;
  virtual bool start(Phase phase) = 0;
  virtual uint32_t correctionStartedAt() const = 0;
  virtual bool correctionStep(uint8_t index) = 0;
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
#if MOTIONBRAIN_ENABLE_POSE_GRASP
  uint32_t elbowBudgetMs() const { return elbowBudgetMs_; }
  uint32_t wristBudgetMs() const { return wristBudgetMs_; }
  uint32_t shoulderBudgetMs() const { return approachBudgetMs_; }
#endif
  Phase phase() const { return phase_; }
  const char* trial() const { return trial_; }
  const char* reason() const { return reason_; }
private:
  bool canStartStep(Phase) const;
  void chargeStep(Phase);
  void fail(const char*);
  bool finish();
  Port& port_;
  char trial_[33]{};
  uint32_t seq_=0, started_=0, renewed_=0;
  uint8_t nextCorrectionStep_=0;
  uint32_t approachBudgetMs_=0;
#if MOTIONBRAIN_ENABLE_POSE_GRASP
  uint32_t elbowBudgetMs_=0,wristBudgetMs_=0;
#endif
  Phase phase_=Phase::Approach;
  bool active_=false, failed_=false, done_=false;
  const char* reason_="idle";
};
}
