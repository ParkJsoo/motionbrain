#include "control/bounded_action_service.h"
#include <esp_system.h>
#include "control/command.h"
#include "control/safety_gate.h"
#include "control/angle_controller.h"
#include "control/shoulder_angle_controller.h"
#include "input/teleop_adapter.h"
#include "motion/motion_sequence.h"
#include "motor/motor_driver.h"
#include "network/web_server.h"
#include "system/system_init.h"

extern MotorControl motorControl;
extern SystemStateManager systemState;
extern SafetyGate safetyGate;
extern AngleController angleController;
extern ShoulderAngleController shoulderAngleController;
extern MotionSequence motionSequence;
extern TeleopAdapter teleopAdapter;
extern MotionBrainWebServer webServer;

namespace {
class HardwarePort : public bounded::Port {
public:
  bounded::Executor* owner = nullptr;
  bool allowed() const override {
    Command command;
    command.type = CommandType::MOTOR_RUN;
    CommandResult result;
    return safetyGate.allows(command, result);
  }
  bool idle() const override {
    return !motorControl.hasPendingOutput() && motionSequence.getState() != SequenceState::RUNNING &&
      !angleController.isActive() && !shoulderAngleController.isActive() &&
      !teleopAdapter.isControlActive() && !teleopAdapter.isDeadmanHeld();
  }
  bool start(const bounded::Request& request) override {
    motorControl.setActionOrdinal(owner->activeOrdinal());
    webServer.clearManualControlLeases();
    teleopAdapter.resetOutputCache();
    systemState.resetTimeout();
    if (request.kind == bounded::Kind::Shoulder) {
      char message[96];
      return shoulderAngleController.startAbsolute(request.targetDegrees, request.percent,
                                                    message, sizeof(message));
    }
    return motorControl.setSpeed(request.motor, request.pwm);
  }
  void stop(const bounded::Request& request, bounded::Reason) override {
    if (request.kind == bounded::Kind::Shoulder && shoulderAngleController.isActive()) {
      shoulderAngleController.cancel(ShoulderAngleStopReason::MANUAL_STOP, "bounded action ended");
    }
    motorControl.hardStopAll();
  }
  uint32_t afterStopTime(uint32_t) const override { return millis(); }
  bool outputIsOff(const bounded::Request&) const override {
    return !motorControl.hasPendingOutput() && !shoulderAngleController.isActive();
  }
  bounded::ShoulderResult shoulderResult() const override {
    if (shoulderAngleController.isActive()) return bounded::ShoulderResult::Running;
    return shoulderAngleController.getLastStopReason() == ShoulderAngleStopReason::TARGET_REACHED
      ? bounded::ShoulderResult::Reached : bounded::ShoulderResult::Failed;
  }
};
HardwarePort port;
bounded::Executor* core = nullptr;
void clearFinishedContext() {
  if (core && !core->active()) motorControl.setActionOrdinal(0);
}
}

namespace BoundedActions {
bool beforeLegacyCommand(CommandType type) {
  if (!active()) return true;
  switch (type) {
    case CommandType::LIGHT_ON:
    case CommandType::LIGHT_OFF:
    case CommandType::LIGHT_TOGGLE:
    case CommandType::ROUTINE_DRY_RUN:
    case CommandType::SEQUENCE_ADD:
      return true;
    case CommandType::STOP:
    case CommandType::DISARM:
    case CommandType::MOTOR_STOP:
    case CommandType::MOTOR_STOP_ALL:
    case CommandType::JOINT_STOP:
    case CommandType::JOINT_STOP_ALL:
    case CommandType::SEQUENCE_STOP:
    case CommandType::SEQUENCE_CLEAR:
    case CommandType::ROUTINE_ABORT:
      cancel(bounded::Reason::Cancelled);
      return true; // Unknown must never block the existing emergency stop path.
    default:
      cancel(bounded::Reason::OperatorOverride);
      return !active();
  }
}
void init() {
  if (core) return;
  // Called after Wi-Fi initialization, when ESP32's hardware entropy source is active.
  char bootId[33];
  snprintf(bootId, sizeof(bootId), "%08lx%08lx%08lx%08lx",
    static_cast<unsigned long>(esp_random()), static_cast<unsigned long>(esp_random()),
    static_cast<unsigned long>(esp_random()), static_cast<unsigned long>(esp_random()));
  static bounded::Executor instance(port, bootId);
  core = &instance;
  port.owner = core;
}
bool enabled() { return MOTIONBRAIN_ENABLE_BOUNDED_ACTIONS != 0; }
bool active() { return core && core->active(); }
const bounded::Executor* executor() { return core; }
void cancel(bounded::Reason reason) {
  if (core) core->cancel(millis(), reason);
  clearFinishedContext();
}
void tick() {
  if (!core) return;
  if (active() && teleopAdapter.isDeadmanHeld()) cancel(bounded::Reason::OperatorOverride);
  core->tick(millis());
  clearFinishedContext();
}
bounded::SubmitResult submit(const bounded::Request& request) {
  if (!core || !enabled()) return {bounded::SubmitCode::SafetyBlocked, nullptr};
#if MOTIONBRAIN_M2_VISUAL_PILOT
  // Matches RobotArm::WRIST_UP_IS_FORWARD=true. Verify physical direction before use.
  // This profile narrows /actions only; it does not replace legacy safety/ownership.
  if (request.kind != bounded::Kind::Pulse || request.motor != 2 ||
      request.pwm <= 0 || request.pwm > 180 || request.durationMs > 1500)
    return {bounded::SubmitCode::SafetyBlocked, nullptr};
#endif
  auto result = core->submit(request, millis());
  clearFinishedContext();
  return result;
}
void appendStatusJson(String& json) {
  json += "{\"schemaVersion\":\"motionbrain.bounded_action.v1\",\"enabled\":";
  json += enabled() ? "true" : "false";
  json += ",\"profile\":\"";
  json += MOTIONBRAIN_M2_VISUAL_PILOT ? "m2_up_pwm180_ms1500" : "general";
  json += "\"";
  json += ",\"bootId\":\"";
  if (core) json += core->bootId();
  json += "\",\"active\":";
  json += active() ? "true" : "false";
  json += ",\"recordCount\":";
  json += core ? core->registry().count() : 0;
  json += ",\"recordCapacity\":32,\"clockDomain\":\"controller_boot_millis\"}";
}
void appendRecordJson(String& json, const bounded::Record& r) {
  json += "{\"bootId\":\""; json += r.request.bootId;
  json += "\",\"commandId\":\""; json += r.request.commandId;
  json += "\",\"ordinal\":"; json += r.ordinal;
  json += ",\"state\":\""; json += bounded::toString(r.state);
  json += "\",\"reason\":\""; json += bounded::toString(r.reason);
  json += "\",\"motor\":"; json += r.request.motor;
  json += ",\"kind\":\""; json += bounded::toString(r.request.kind); json += "\"";
  json += ",\"requestedPwm\":"; json += r.request.pwm;
  json += ",\"durationMs\":"; json += r.request.durationMs;
  json += ",\"targetDegrees\":"; json += String(r.request.targetDegrees, 6);
  json += ",\"percent\":"; json += r.request.percent;
  json += ",\"startedAtMs\":"; json += r.startedAtMs;
  json += ",\"endedAtMs\":"; json += r.endedAtMs;
  json += ",\"elapsedMs\":"; json += r.elapsedMs;
  json += ",\"overrunMs\":"; json += r.overrunMs;
  json += ",\"outputOff\":"; json += r.outputOff ? "true" : "false";
  json += ",\"targetReached\":"; json += r.targetReached ? "true" : "false";
  json += ",\"taskSuccess\":null,\"visuallySettled\":null}";
}
void appendOutputJson(String& json) {
  const auto& log = motorControl.outputLog();
  json += "{\"bootId\":\""; if (core) json += core->bootId();
  json += "\",\"evidence\":\"driver_write\",\"clockDomain\":\"controller_boot_millis\",\"dropped\":";
  json += log.dropped();
  json += ",\"entries\":[";
  for (size_t i = 0; i < log.count(); ++i) {
    const auto* entry = log.at(i);
    if (i) json += ',';
    json += "{\"sequence\":"; json += entry->sequence;
    json += ",\"atMs\":"; json += entry->atMs;
    json += ",\"actionOrdinal\":"; json += entry->actionOrdinal;
    json += ",\"motor\":"; json += entry->motor;
    json += ",\"pwm\":"; json += entry->pwm;
    json += ",\"reason\":\"";
    json += entry->reason == bounded::OutputReason::ForcedOff ? "forced_off" : "driver_write";
    json += "\"}";
  }
  json += "]}";
}
}
