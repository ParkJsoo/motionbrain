// The fake timer runs even when the main loop misses the last PWM update.
// All nonzero edges below come from the production MotorControl ramp.
static void shoulderTimer(const char* mode) {
  assert(motorControl.init());
  testSetState(SystemState::ARMED);
  shoulderAngleSensor.angle=215.f;
  shoulderAngleSensor.sample=clockMs;
  Phase p=Phase::ShoulderDown420;
  if (!strcmp(mode,"shoulder-timer-300")) p=Phase::ShoulderDown300;
  if (!strcmp(mode,"shoulder-timer-650") || !strcmp(mode,"shoulder-timer-angle")) p=Phase::ShoulderDown650;
  const unsigned nominal=whole_arm::stepDurationMs(p);
  const unsigned start=clockMs;
  const auto initial=outputCursor();
  String state;
  WholeArmService::status(state);
  assert(strstr(state.c_str(),"\"shoulderStepZeroProfile\":\"timer-normal-zero-v1\""));
  assert(stepStart(p,1,initial));
  if (!strcmp(mode,"shoulder-timer-handoff-race")) {
    for (unsigned t=50;t<=300;t+=50) {
      clockMs=start+t;shoulderAngleSensor.sample=clockMs;
      WholeArmService::update();motorControl.update();
    }
    clockMs=start+403;shoulderAngleSensor.sample=clockMs;WholeArmService::update();
    preemptPwmAtMs=clockMs;callbackMs=start+423;
    motorControl.update();
    assert(!preemptPwmAtMs && clockMs==start+425 && !duty[3]);
    WholeArmService::update();
    const auto& log=motorControl.outputLog();
    const auto* power=log.at(log.count()-2);
    const auto* zero=log.at(log.count()-1);
    assert(power->motor==4 && power->pwm==-70 && power->atMs==start+403);
    assert(zero->motor==4 && zero->pwm==0 && zero->atMs==start+423 && zero->sequence==power->sequence+1);
    assert(power->reason==bounded::OutputReason::DriverWrite && zero->reason==bounded::OutputReason::DriverWrite);
    WholeArmService::status(state);
    assert(strstr(state.c_str(),"\"reason\":\"stopped\""));
    return;
  }
  const bool stale=!strcmp(mode,"shoulder-timer-stale");
  const bool late=!strcmp(mode,"shoulder-timer-missed");
  const bool angle=!strcmp(mode,"shoulder-timer-angle");
  const bool abort=!strcmp(mode,"shoulder-timer-stop");
  const bool postAbort=!strcmp(mode,"shoulder-timer-stop-after-zero");
  const bool replay=!strcmp(mode,"shoulder-timer-recorded");
  for (unsigned elapsed=1; elapsed<=nominal+13; ++elapsed) {
    clockMs=start+elapsed;
    if (elapsed%5==3 && (!late || elapsed<nominal || elapsed>=nominal+10)) timer.callback(nullptr);
    if (abort && elapsed==nominal-3) assert(WholeArmService::beforeLegacy(CommandType::STOP));
    if (angle && elapsed==nominal-4) {
      shoulderAngleSensor.angle=208.75f;
      shoulderAngleSensor.sample=clockMs;
      WholeArmService::update();
    }
    // Millisecond offsets of six M4 HAL ramp writes from an interrupted 420 ms run.
    const bool observedMain=elapsed==51 || elapsed==119 || elapsed==202 || elapsed==258 ||
                            elapsed==312 || elapsed==362;
    const bool main=replay ? observedMain : (elapsed%10==0 && elapsed+29<nominal);
    if (main && (!stale || elapsed+155<nominal)) {
      shoulderAngleSensor.sample=clockMs;
      WholeArmService::update();motorControl.update();
    }
    if (postAbort && elapsed==nominal+4) assert(WholeArmService::beforeLegacy(CommandType::DISARM));
    if (elapsed==nominal-15 && !stale) assert(duty[3]>0);
    if (!late && elapsed>=nominal+3) assert(duty[3]==0);
  }
  assert(duty[3]==0);
  // Simulate a late main iteration in which the ramp runs before policy ownership
  // is reconciled. A stale ramp must never re-energize M4.
  clockMs=start+nominal+47;
  shoulderAngleSensor.sample=clockMs;
  motorControl.update();
  assert(!duty[3]);
  WholeArmService::update();
  WholeArmService::status(state);
  const auto& log=motorControl.outputLog();
  const bool normal=!(late || stale || abort || angle || postAbort);
  if (normal) {
    assert(strstr(state.c_str(),"\"reason\":\"stopped\"") && strstr(state.c_str(),"\"failed\":false"));
    assert(!motorControl.hasPendingOutput() && !motorControl.isEnabled());
    unsigned previous=0,zeros=0;
    for (unsigned i=0;i<log.count();++i) {
      const auto* event=log.at(i);
      if (event->sequence<=initial) continue;
      assert(event->motor==4 && event->reason==bounded::OutputReason::DriverWrite);
      if (!event->pwm) {
        ++zeros;
        assert(event->atMs==start+nominal+3 && event->sequence==outputCursor());
      } else {
        assert(!zeros && event->pwm==-int(previous+10));
        previous+=10;
      }
    }
    assert(zeros==1 && (!replay || previous==60));
    char field[64];
    std::snprintf(field,sizeof(field),"\"stepZeroWriteOffsetUs\":%u",(nominal+3)*1000);
    assert(strstr(state.c_str(),field));
    std::snprintf(field,sizeof(field),"\"stepZeroSequence\":%u",outputCursor());
    assert(strstr(state.c_str(),field));
    assert(!stepStart(p,1,outputCursor()) && !stepStart(p,2,initial));
    assert(stepStart(Phase::ShoulderDown300,2,outputCursor()));
  } else {
    assert(strstr(state.c_str(),"\"failed\":true"));
    assert(!stepStart(p,2,outputCursor()));
    if (!postAbort && !abort) {
      const auto* last=log.at(log.count()-1);
      assert(last->pwm==0 && last->reason==bounded::OutputReason::ForcedOff);
    }
  }
}
