// Actual production core and log; fake time and port do not prove motor motion.
#include "control/bounded_action_executor.h"
#include "control/numeric_validation.h"
#include "motor/applied_output_log.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

using namespace bounded;

static Request pulse(const char* id = "action-1", uint32_t duration = 100) {
  Request request;
  std::strcpy(request.bootId, "boot-A");
  std::strcpy(request.commandId, id);
  request.motor = 1;
  request.pwm = 70;
  request.durationMs = duration;
  return request;
}

static Request shoulder(const char* id = "shoulder-1", uint32_t duration = 1000) {
  Request request = pulse(id, duration);
  request.kind = Kind::Shoulder;
  request.motor = 4;
  request.pwm = 0;
  request.targetDegrees = 250.0f;
  request.percent = 75;
  return request;
}

struct FakePort : Port {
  bool safe = true;
  bool externalIdle = true;
  bool startWorks = true;
  bool stopWorks = true;
  bool off = true;
  unsigned starts = 0;
  unsigned stops = 0;
  uint32_t now = 0;
  uint32_t stopLatency = 0;
  Executor* executor = nullptr;
  ShoulderResult result = ShoulderResult::Running;
  AppliedOutputLog log;
  std::vector<int16_t> writes;
  bool allowed() const override { return safe; }
  bool idle() const override { return externalIdle && off; }
  bool start(const Request& request) override {
    ++starts;
    // Model partial start even when the start routine reports failure.
    off = false;
    const int16_t applied = request.kind == Kind::Pulse ? 10 : 20;
    writes.push_back(applied);
    log.append(now, executor->activeOrdinal(), request.motor, applied, OutputReason::DriverWrite);
    return startWorks;
  }
  void stop(const Request& request, Reason) override {
    ++stops;
    now += stopLatency;
    if (stopWorks) {
      off = true;
      writes.push_back(0);
      log.append(now, executor->activeOrdinal(), request.motor, 0, OutputReason::ForcedOff);
    }
  }
  bool outputIsOff(const Request&) const override { return off; }
  ShoulderResult shoulderResult() const override { return result; }
  uint32_t afterStopTime(uint32_t) const override { return now; }
};

struct Fixture {
  FakePort port;
  Executor executor;
  Fixture() : executor(port, "boot-A") { port.executor = &executor; }
  SubmitResult submit(const Request& request, uint32_t now = 0) {
    port.now = now;
    return executor.submit(request, now);
  }
  void tick(uint32_t now) { port.now = now; executor.tick(now); }
  void cancel(uint32_t now) { port.now = now; executor.cancel(now); }
};

static void identity() {
  Fixture f;
  Request wrong = pulse();
  std::strcpy(wrong.bootId, "boot-previous");
  assert(f.submit(wrong).code == SubmitCode::BootMismatch);
  assert(f.port.starts == 0 && f.executor.registry().count() == 0);
  assert(f.executor.find("boot-previous", "action-1").code == LookupCode::BootMismatch);
  assert(f.executor.find("boot-A", "unknown").code == LookupCode::NotFound);
  const SubmitResult accepted = f.submit(pulse(), 10);
  assert(accepted.code == SubmitCode::Accepted && accepted.record);
  assert(accepted.record->state == State::Running && accepted.record->ordinal == 1);
  assert(accepted.record->startedAtMs == 10 && f.executor.active());
  assert(f.submit(pulse(), 99).code == SubmitCode::Duplicate);
  assert(accepted.record->startedAtMs == 10); // Retry never refreshes time.
  Request changed = pulse();
  changed.pwm = -70;
  assert(f.submit(changed).code == SubmitCode::Conflict);
  changed = pulse(); changed.durationMs = 101;
  assert(f.submit(changed).code == SubmitCode::Conflict);
  Request invalidConflict = pulse(); invalidConflict.durationMs = 0;
  assert(f.submit(invalidConflict).code == SubmitCode::Conflict);
  assert(f.submit(pulse("other")).code == SubmitCode::Busy);
  assert(f.port.starts == 1 && f.executor.registry().count() == 1);
  f.tick(110);
  assert(f.submit(pulse(), 10000).code == SubmitCode::Duplicate);
  assert(f.submit(changed).code == SubmitCode::Conflict);
  assert(f.port.starts == 1 && f.port.stops == 1);
  assert(f.executor.find("boot-A", "action-1").record == accepted.record);
  assert(accepted.record->state == State::Completed);
  // A new boot cannot identify old records or accept a delayed old-boot action.
  FakePort rebootPort;
  Executor reboot(rebootPort, "boot-B"); rebootPort.executor = &reboot;
  assert(reboot.find("boot-A", "action-1").code == LookupCode::BootMismatch);
  assert(reboot.submit(pulse(), 0).code == SubmitCode::BootMismatch);
  assert(rebootPort.starts == 0);
  f.port.externalIdle = false;
  assert(f.submit(pulse("external-motion")).code == SubmitCode::Busy);
}

static void validation() {
  Fixture f;
  std::vector<Request> bad;
  Request r = pulse(); r.commandId[0] = 0; bad.push_back(r);
  r = pulse(); std::strcpy(r.commandId, "bad id"); bad.push_back(r);
  r = pulse(); std::strcpy(r.commandId, "bad\"id"); bad.push_back(r);
  r = pulse(); std::memset(r.commandId, 'a', sizeof(r.commandId)); bad.push_back(r);
  r = pulse(); std::memset(r.bootId, 'a', sizeof(r.bootId)); bad.push_back(r);
  r = pulse(); r.durationMs = 0; bad.push_back(r);
  r = pulse(); r.durationMs = 10001; bad.push_back(r);
  r = pulse(); r.durationMs = UINT32_MAX; bad.push_back(r);
  r = pulse(); r.motor = 0; bad.push_back(r);
  r = pulse(); r.motor = 6; bad.push_back(r);
  r = pulse(); r.motor = 4; bad.push_back(r); // M4 requires the existing closed loop.
  r = pulse(); r.pwm = 0; bad.push_back(r);
  r = pulse(); r.pwm = 256; bad.push_back(r);
  r = pulse(); r.pwm = -256; bad.push_back(r);
  r = pulse(); r.percent = 1; bad.push_back(r);
  r = pulse(); r.targetDegrees = 1; bad.push_back(r);
  r = pulse(); r.kind = static_cast<Kind>(99); bad.push_back(r);
  r = shoulder(); r.motor = 3; bad.push_back(r);
  r = shoulder(); r.pwm = 1; bad.push_back(r);
  r = shoulder(); r.percent = 0; bad.push_back(r);
  r = shoulder(); r.percent = 101; bad.push_back(r);
  r = shoulder(); r.targetDegrees = std::numeric_limits<float>::quiet_NaN(); bad.push_back(r);
  r = shoulder(); r.targetDegrees = std::numeric_limits<float>::infinity(); bad.push_back(r);
  r = shoulder(); r.targetDegrees = -std::numeric_limits<float>::infinity(); bad.push_back(r);
  for (const auto& request : bad) assert(f.submit(request).code == SubmitCode::Invalid);
  assert(f.port.starts == 0 && f.port.stops == 0 && f.executor.registry().count() == 0);
  assert(f.executor.find(nullptr, "a").code == LookupCode::Invalid);
  assert(f.executor.find("boot-A", nullptr).code == LookupCode::Invalid);
  assert(f.executor.find("boot-A", "bad id").code == LookupCode::Invalid);
  FakePort invalidPort;
  Executor invalid(invalidPort, ""); invalidPort.executor = &invalid;
  assert(invalid.submit(pulse(), 0).code == SubmitCode::Invalid);
  assert(invalidPort.starts == 0);
  // Exact valid limits; no truncation of the full 128-character command ID.
  r = pulse(); std::memset(r.commandId, 'x', 128); r.commandId[128] = 0;
  r.durationMs = 1; r.pwm = -255;
  assert(f.submit(r).code == SubmitCode::Accepted); f.tick(1);
  r = pulse("max"); r.durationMs = 10000; r.pwm = 255;
  assert(f.submit(r, 10).code == SubmitCode::Accepted); f.tick(10010);
  assert(f.port.starts == 2);
}

static void capacity() {
  Fixture f;
  for (unsigned i = 0; i < 32; ++i) {
    const std::string id = "command-" + std::to_string(i);
    const Request request = pulse(id.c_str(), 1);
    const auto result = f.submit(request, i * 2);
    assert(result.code == SubmitCode::Accepted && result.record->ordinal == i + 1);
    f.tick(i * 2 + 1);
  }
  assert(f.executor.registry().count() == 32 && f.port.starts == 32);
  assert(f.submit(pulse("overflow")).code == SubmitCode::RegistryFull);
  assert(f.submit(pulse("command-0", 1)).code == SubmitCode::Duplicate);
  assert(f.submit(pulse("command-31", 1)).code == SubmitCode::Duplicate);
  assert(f.executor.find("boot-A", "command-0").record->state == State::Completed);
  assert(f.executor.registry().at(32) == nullptr && f.port.starts == 32);
}

static void deadline() {
  Fixture f;
  const auto r = f.submit(pulse(), 1000);
  f.tick(1099);
  assert(f.executor.active() && f.port.stops == 0);
  assert(r.record->elapsedMs == 99);
  f.tick(1100);
  assert(!f.executor.active() && r.record->outputOff);
  assert(r.record->state == State::Completed && r.record->reason == Reason::DurationElapsed);
  assert(r.record->elapsedMs == 100 && r.record->overrunMs == 0 && !r.record->targetReached);
  assert(f.port.log.at(0)->pwm == 10 && f.port.log.at(0)->actionOrdinal == 1);
  assert(f.port.log.at(1)->pwm == 0 && f.port.log.at(1)->atMs == 1100);
  f.tick(5000); assert(f.port.stops == 1);
  const auto delayed = f.submit(pulse("late"), 5000);
  f.tick(5137);
  assert(delayed.record->elapsedMs == 137 && delayed.record->overrunMs == 37);
  assert(delayed.record->state == State::TimedOut && delayed.record->reason == Reason::DeadlineExceeded);
  Fixture slowStop;
  slowStop.port.stopLatency = 7;
  const auto slow = slowStop.submit(pulse(), 1000);
  slowStop.tick(1100);
  assert(slow.record->endedAtMs == 1107 && slow.record->elapsedMs == 107);
  assert(slow.record->state == State::TimedOut && slow.record->overrunMs == 7);
  assert(slowStop.port.log.at(1)->atMs == 1107);
  // uint32_t millis wrap: elapsed subtraction remains correct across rollover.
  Fixture wrap;
  const auto wrapped = wrap.submit(pulse("wrap", 50), UINT32_MAX - 20);
  wrap.tick(28); assert(wrap.executor.active() && wrap.port.stops == 0);
  wrap.tick(29); assert(!wrap.executor.active());
  assert(wrapped.record->elapsedMs == 50 && wrapped.record->overrunMs == 0);
}

static void unknown() {
  Fixture f;
  f.port.stopWorks = false;
  const auto r = f.submit(pulse(), 0);
  f.tick(100);
  assert(r.record->state == State::OutcomeUnknown && !r.record->outputOff);
  assert(f.executor.active() && f.port.stops == 1);
  assert(f.submit(pulse("blocked"), 101).code == SubmitCode::Busy);
  assert(f.submit(pulse(), 101).code == SubmitCode::Duplicate);
  f.tick(120);
  assert(f.port.starts == 1 && f.port.stops == 2 && f.executor.active());
  f.port.stopWorks = true;
  f.tick(130);
  assert(f.executor.active() && r.record->outputOff); // Unknown latches this boot closed.
  assert(r.record->state == State::OutcomeUnknown && r.record->reason == Reason::DurationElapsed);
  assert(f.port.starts == 1 && f.port.stops == 3);
  f.tick(200); f.tick(300); assert(f.port.stops == 3);
  assert(f.submit(pulse("still-blocked"), 300).code == SubmitCode::Busy);
  assert(f.submit(pulse(), 300).code == SubmitCode::Duplicate);
  assert(f.port.starts == 1); // Resolving software-off never retries the action.
  assert(f.executor.find("boot-A", "action-1").record->state == State::OutcomeUnknown);
}

static void m4() {
  Fixture reached;
  const auto r = reached.submit(shoulder(), 100);
  reached.tick(200); assert(reached.executor.active());
  reached.port.result = ShoulderResult::Reached;
  reached.tick(300);
  assert(r.record->state == State::Completed && r.record->targetReached && r.record->outputOff);
  assert(r.record->reason == Reason::TargetReached && r.record->elapsedMs == 200);
  Fixture failed;
  const auto fail = failed.submit(shoulder());
  failed.port.result = ShoulderResult::Failed; failed.tick(50);
  assert(fail.record->state == State::Failed && fail.record->reason == Reason::ControllerFailed);
  assert(!fail.record->targetReached && fail.record->outputOff);
  Fixture timeout;
  const auto timed = timeout.submit(shoulder());
  timeout.port.result = ShoulderResult::Reached; // Deadline wins over late feedback.
  timeout.tick(1000);
  assert(timed.record->state == State::TimedOut && timed.record->reason == Reason::DeadlineExceeded);
  assert(!timed.record->targetReached && timed.record->outputOff);
  Fixture unknown;
  const auto unconfirmed = unknown.submit(shoulder());
  unknown.port.result = ShoulderResult::Reached; unknown.port.stopWorks = false;
  unknown.tick(200);
  assert(unconfirmed.record->state == State::OutcomeUnknown && unconfirmed.record->targetReached);
  unknown.port.stopWorks = true; unknown.cancel(210);
  assert(unconfirmed.record->state == State::OutcomeUnknown && unconfirmed.record->targetReached);
  assert(unconfirmed.record->reason == Reason::TargetReached && unknown.executor.active());
  assert(unknown.submit(shoulder("blocked")).code == SubmitCode::Busy);
}

static void cancel() {
  Fixture start;
  start.port.startWorks = false;
  const auto r = start.submit(pulse(), 1);
  assert(r.code == SubmitCode::Accepted); // ID consumed, despite failed start.
  assert(r.record->state == State::Failed && r.record->reason == Reason::StartFailed);
  assert(r.record->outputOff && start.port.starts == 1 && start.port.stops == 1);
  assert(start.submit(pulse()).code == SubmitCode::Duplicate && start.port.starts == 1);
  Fixture partial;
  partial.port.startWorks = false; partial.port.stopWorks = false;
  const auto p = partial.submit(pulse());
  assert(p.record->state == State::OutcomeUnknown && partial.executor.active());
  partial.tick(1); assert(partial.port.starts == 1 && partial.port.stops == 2);
  Fixture unsafe;
  unsafe.port.safe = false;
  assert(unsafe.submit(pulse()).code == SubmitCode::SafetyBlocked);
  assert(unsafe.port.starts == 0 && unsafe.executor.registry().count() == 0);
  unsafe.port.safe = true;
  const auto s = unsafe.submit(pulse());
  unsafe.port.safe = false; unsafe.tick(25);
  assert(s.record->state == State::Failed && s.record->reason == Reason::SafetyBlocked);
  assert(s.record->outputOff && !unsafe.executor.active());
  Fixture manual;
  const auto m = manual.submit(pulse(), 10);
  manual.cancel(20);
  assert(m.record->state == State::Cancelled && m.record->reason == Reason::Cancelled);
  assert(m.record->elapsedMs == 10 && m.record->outputOff && !manual.executor.active());
  manual.cancel(30); assert(manual.port.stops == 1);
}

static void output() {
  AppliedOutputLog log;
  assert(log.count() == 0 && log.dropped() == 0 && log.at(0) == nullptr);
  assert(!log.append(0, 1, 0, 10, OutputReason::DriverWrite));
  assert(!log.append(0, 1, 6, 10, OutputReason::DriverWrite));
  assert(!log.append(0, 1, 1, 256, OutputReason::DriverWrite));
  assert(!log.append(0, 1, 1, -256, OutputReason::DriverWrite));
  assert(!log.append(0, 1, 1, 10, OutputReason::ForcedOff));
  assert(log.count() == 0);
  for (int16_t pwm = 10; pwm <= 70; pwm += 10) {
    assert(log.append(static_cast<uint32_t>(pwm * 5), 1, 1, pwm, OutputReason::DriverWrite));
    assert(!log.append(static_cast<uint32_t>(pwm * 5 + 1), 1, 1, pwm, OutputReason::DriverWrite));
  }
  assert(log.count() == 7 && log.at(0)->pwm == 10 && log.at(6)->pwm == 70);
  assert(log.append(400, 2, 1, 70, OutputReason::DriverWrite)); // Same PWM, new action context.
  assert(log.append(401, 2, 2, 70, OutputReason::DriverWrite)); // Separate motor history.
  assert(log.append(402, 2, 1, -70, OutputReason::DriverWrite));
  assert(log.append(410, 2, 1, 0, OutputReason::ForcedOff));
  assert(log.append(411, 2, 1, 0, OutputReason::ForcedOff)); // Every explicit forced stop survives.
  assert(!log.append(412, 2, 1, 0, OutputReason::DriverWrite));
  assert(log.count() == 12 && log.at(11)->reason == OutputReason::ForcedOff);
  for (size_t i = 0; i < log.count(); ++i) assert(log.at(i)->sequence == i + 1);
}

static void overflow() {
  AppliedOutputLog log;
  for (uint32_t i = 0; i < 150; ++i)
    assert(log.append(i * 2, 5, 3, static_cast<int16_t>(i % 2 ? 10 : 20), OutputReason::DriverWrite));
  assert(log.count() == 128 && log.dropped() == 22);
  assert(log.at(0)->sequence == 23 && log.at(0)->atMs == 44);
  assert(log.at(127)->sequence == 150 && log.at(127)->atMs == 298);
  assert(log.at(128) == nullptr);
  assert(!log.append(999, 5, 3, 10, OutputReason::DriverWrite));
  assert(log.dropped() == 22); // Suppressed repeats do not count as lost entries.
  assert(log.append(1000, 5, 3, 0, OutputReason::ForcedOff));
  assert(log.dropped() == 23 && log.at(127)->sequence == 151);
}

static void numeric() {
  long integer = 42;
  const char* badIntegers[] = {
    nullptr, "", "-", "+1", "true", "false", "1.0", "12ms", "1e2", "0x10",
    " 1", "1 ", "1\n", "\t1", "--1", "1-2", "-256", "256",
    "99999999999999999999999999999999999999999999999",
    "-99999999999999999999999999999999999999999999999"
  };
  for (const char* text : badIntegers) {
    assert(!NumericValidation::integer(text, -255, 255, integer));
    assert(integer == 42); // Rejection does not return a partial or clamped value.
  }
  assert(NumericValidation::integer("-255", -255, 255, integer) && integer == -255);
  assert(NumericValidation::integer("255", -255, 255, integer) && integer == 255);
  assert(NumericValidation::integer("0007", 1, 10000, integer) && integer == 7);
  assert(!NumericValidation::integer("-1", 1, 10000, integer) && integer == 7);
  assert(!NumericValidation::integer("0", 1, 10000, integer) && integer == 7);
  assert(!NumericValidation::integer("10001", 1, 10000, integer) && integer == 7);
  assert(NumericValidation::integer("1", 1, 10000, integer) && integer == 1);
  assert(NumericValidation::integer("10000", 1, 10000, integer) && integer == 10000);
  assert(NumericValidation::integer("-0", 0, 0, integer) && integer == 0);

  float real = 42.0f;
  const char* badReals[] = {
    nullptr, "", "NaN", "nan", "NAN", "Inf", "+inf", "-infinity", "true", "false",
    "1.0deg", "1.0 2", " 1", "1 ", "1\n", ".", "+", "-", "--1", "1.2.3",
    "1e", "1e+", "1e2x", "0x1p2", "1e9999", "-1e9999", "1e-9999", "-256", "256"
  };
  for (const char* text : badReals) {
    assert(!NumericValidation::real(text, -255.0f, 255.0f, real));
    assert(real == 42.0f);
  }
  assert(NumericValidation::real("-255", -255, 255, real) && real == -255.0f);
  assert(NumericValidation::real("255", -255, 255, real) && real == 255.0f);
  assert(NumericValidation::real("+2.5e1", -255, 255, real) && real == 25.0f);
  assert(NumericValidation::real("-.5", -255, 255, real) && real == -0.5f);
  assert(NumericValidation::real("1.", -255, 255, real) && real == 1.0f);
  assert(NumericValidation::real("-0", 0, 0, real) && real == 0.0f);
  assert(NumericValidation::real("122.08", 122.08f, 301.02f, real) && real == 122.08f);
  assert(NumericValidation::real("301.02", 122.08f, 301.02f, real) && real == 301.02f);
  assert(!NumericValidation::real("301.03", 122.08f, 301.02f, real) && real == 301.02f);
}

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  const std::string name(argv[1]);
  if (name == "identity") identity();
  else if (name == "validation") validation();
  else if (name == "capacity") capacity();
  else if (name == "deadline") deadline();
  else if (name == "unknown") unknown();
  else if (name == "m4") m4();
  else if (name == "cancel") cancel();
  else if (name == "output") output();
  else if (name == "overflow") overflow();
  else if (name == "numeric") numeric();
  else return 2;
  std::printf("PASS %s (virtual time/fake output; no device)\n", argv[1]);
  return 0;
}
