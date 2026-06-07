#include "input/teleop_adapter.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "control/angle_controller.h"
#include "control/command.h"
#include "control/command_bus.h"
#include "control/dispatcher.h"
#include "control/event_log.h"
#include "debug/debug_log.h"
#include "motion/motion_sequence.h"
#include "motion/robot_arm.h"
#include "motor/motor_driver.h"
#include "safety/safety_monitor.h"
#include "system/system_init.h"

extern EventLog eventLog;

namespace {

constexpr float REACH_TO_ELBOW_WEIGHT       = 0.65f;
constexpr float REACH_TO_SHOULDER_WEIGHT    = 0.35f;
constexpr float LIFT_TO_SHOULDER_WEIGHT     = 1.00f;
constexpr float LIFT_TO_ELBOW_WEIGHT        = 0.00f;
constexpr float LIFT_RESPONSE_GAIN          = 2.00f;
constexpr float LIFT_ROLL_DEADZONE_DEG      = 2.0f;
constexpr float LIFT_ROLL_FULL_SCALE_DEG    = 14.0f;
constexpr float LIFT_REACH_SUPPRESSION_START = 0.05f;
constexpr float LIFT_REACH_SUPPRESSION_MAX   = 1.00f;
constexpr float WRIST_BLEND_START           = 0.30f;
constexpr float WRIST_BLEND_MAX             = 0.25f;
constexpr float TWIST_TO_BASE_WEIGHT        = 1.25f;
constexpr uint8_t LIFT_OUTPUT_CAP_PERCENT   = 100;
constexpr uint8_t BASE_OUTPUT_CAP_PERCENT   = 50;

// 실제 기구 방향은 실기에서 맞춘다. 지금은 teleop 골격용 기본 부호다.
constexpr float REACH_TO_ELBOW_SIGN         = 1.0f;
constexpr float REACH_TO_SHOULDER_SIGN      = 1.0f;
constexpr float REACH_TO_WRIST_SIGN         = 1.0f;
constexpr float LIFT_TO_SHOULDER_SIGN       = 1.0f;
constexpr float LIFT_TO_ELBOW_SIGN          = 1.0f;
constexpr float TWIST_TO_BASE_SIGN          = 1.0f;

bool extractRawValue(const String& json, const char* key, String& outValue) {
  String pattern = "\"";
  pattern += key;
  pattern += "\"";

  int keyPos = json.indexOf(pattern);
  if (keyPos < 0) {
    return false;
  }

  int colonPos = json.indexOf(':', keyPos + pattern.length());
  if (colonPos < 0) {
    return false;
  }

  int start = colonPos + 1;
  while (start < json.length() && isspace(static_cast<unsigned char>(json[start]))) {
    start++;
  }

  int end = start;
  bool inString = false;
  while (end < json.length()) {
    char c = json[end];
    if (c == '"' && (end == start || json[end - 1] != '\\')) {
      inString = !inString;
    }
    if (!inString && (c == ',' || c == '}')) {
      break;
    }
    end++;
  }

  outValue = json.substring(start, end);
  outValue.trim();

  if (outValue.length() >= 2 && outValue[0] == '"' && outValue[outValue.length() - 1] == '"') {
    outValue = outValue.substring(1, outValue.length() - 1);
  }

  return outValue.length() > 0;
}

bool extractBool(const String& json, const char* key, bool& outValue) {
  String rawValue;
  if (!extractRawValue(json, key, rawValue)) {
    return false;
  }

  rawValue.toLowerCase();
  if (rawValue == "true") {
    outValue = true;
    return true;
  }
  if (rawValue == "false") {
    outValue = false;
    return true;
  }
  return false;
}

bool extractUInt32(const String& json, const char* key, uint32_t& outValue) {
  String rawValue;
  if (!extractRawValue(json, key, rawValue)) {
    return false;
  }

  char* endPtr = nullptr;
  unsigned long parsed = strtoul(rawValue.c_str(), &endPtr, 10);
  if (endPtr == rawValue.c_str()) {
    return false;
  }

  outValue = static_cast<uint32_t>(parsed);
  return true;
}

bool extractFloat(const String& json, const char* key, float& outValue) {
  String rawValue;
  if (!extractRawValue(json, key, rawValue)) {
    return false;
  }

  char* endPtr = nullptr;
  float parsed = strtof(rawValue.c_str(), &endPtr);
  if (endPtr == rawValue.c_str()) {
    return false;
  }

  outValue = parsed;
  return true;
}

} // namespace

TeleopAdapter::TeleopAdapter()
  : serial_(&Serial1)
  , systemState_(nullptr)
  , motorControl_(nullptr)
  , motionSequence_(nullptr)
  , safetyMonitor_(nullptr)
  , angleController_(nullptr)
  , commandBus_(nullptr)
  , dispatcher_(nullptr)
  , lastFrame_()
  , embeddedSafetySnapshot_()
  , lineBuffer_{0}
  , lineIndex_(0)
  , overflowDropping_(false)
  , controlActive_(false)
  , lastFrameReceivedMs_(0)
  , packetsReceived_(0)
  , parseErrors_(0)
  , embeddedSafetyPacketsReceived_(0)
  , lastHandledLedToggleSeq_(0)
  , lastParserWarningMs_(0)
  , suppressedParserWarnings_(0)
  , lastStopReason_(TeleopStopReason::NONE)
  , rollLiftSession_(0)
  , rollLiftNeutralDeg_(0.0f)
  , rollLiftNeutralValid_(false)
  , appliedGripPercent_(0)
  , appliedWristPercent_(0)
  , appliedElbowPercent_(0)
  , appliedShoulderPercent_(0)
  , appliedBasePercent_(0) {
}

bool TeleopAdapter::init(SystemStateManager* systemState,
                         MotorControl* motorControl,
                         MotionSequence* motionSequence,
                         SafetyMonitor* safetyMonitor,
                         AngleController* angleController,
                         CommandBus* commandBus,
                         Dispatcher* dispatcher) {
  systemState_ = systemState;
  motorControl_ = motorControl;
  motionSequence_ = motionSequence;
  safetyMonitor_ = safetyMonitor;
  angleController_ = angleController;
  commandBus_ = commandBus;
  dispatcher_ = dispatcher;

  serial_->setRxBufferSize(RX_BUFFER_SIZE);
  serial_->begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  DebugLog::info("Teleop adapter initialized (Serial1 RX=%d @ %lu, timeout=%lums, rxbuf=%u)",
                 RX_PIN, BAUD_RATE, LINK_TIMEOUT_MS, static_cast<unsigned>(RX_BUFFER_SIZE));
  return isReady();
}

void TeleopAdapter::update() {
  processInput();
  updateControl();
}

void TeleopAdapter::processInput() {
  while (serial_ != nullptr && serial_->available() > 0) {
    processIncomingByte(static_cast<char>(serial_->read()));
  }
}

void TeleopAdapter::updateControl() {
  uint32_t now = millis();
  if (isConnected()) {
    handleFreshFrame(now);
  } else if (lastFrameReceivedMs_ > 0) {
    if (controlActive_) {
      stopControlledOutputs(TeleopStopReason::FRAME_TIMEOUT, "frame timeout");
    } else {
      lastStopReason_ = TeleopStopReason::FRAME_TIMEOUT;
    }
  }
}

bool TeleopAdapter::isReady() const {
  return systemState_ != nullptr
      && motorControl_ != nullptr
      && safetyMonitor_ != nullptr
      && angleController_ != nullptr
      && commandBus_ != nullptr
      && dispatcher_ != nullptr;
}

bool TeleopAdapter::isConnected() const {
  return lastFrameReceivedMs_ > 0 && getLastFrameAgeMs() <= LINK_TIMEOUT_MS;
}

bool TeleopAdapter::isDeadmanHeld() const {
  return isConnected() && lastFrame_.deadman;
}

bool TeleopAdapter::isControlActive() const {
  return controlActive_;
}

uint32_t TeleopAdapter::getLastFrameAgeMs() const {
  if (lastFrameReceivedMs_ == 0) {
    return 0;
  }
  return millis() - lastFrameReceivedMs_;
}

uint32_t TeleopAdapter::getPacketsReceived() const {
  return packetsReceived_;
}

uint32_t TeleopAdapter::getParseErrors() const {
  return parseErrors_;
}

uint32_t TeleopAdapter::getLastSequence() const {
  return lastFrame_.sequence;
}

uint32_t TeleopAdapter::getLastSession() const {
  return lastFrame_.session;
}

uint32_t TeleopAdapter::getLastLedToggleSeq() const {
  return lastFrame_.ledToggleSeq;
}

bool TeleopAdapter::getLastGripOpen() const {
  return lastFrame_.gripOpen;
}

bool TeleopAdapter::getLastGripClose() const {
  return lastFrame_.gripClose;
}

float TeleopAdapter::getLastReach() const {
  return lastFrame_.reach;
}

float TeleopAdapter::getLastLift() const {
  return lastFrame_.lift;
}

float TeleopAdapter::getLastTwist() const {
  return lastFrame_.twist;
}

const char* TeleopAdapter::getLastStopReasonString() const {
  return stopReasonToString(lastStopReason_);
}

bool TeleopAdapter::hasEmbeddedSafetySnapshot() const {
  return embeddedSafetySnapshot_.connected;
}

const SensorSnapshot& TeleopAdapter::getEmbeddedSafetySnapshot() const {
  return embeddedSafetySnapshot_;
}

uint32_t TeleopAdapter::getEmbeddedSafetyAgeMs() const {
  if (!embeddedSafetySnapshot_.connected || embeddedSafetySnapshot_.lastUpdateMs == 0) {
    return 0;
  }
  return millis() - embeddedSafetySnapshot_.lastUpdateMs;
}

uint32_t TeleopAdapter::getEmbeddedSafetyPacketsReceived() const {
  return embeddedSafetyPacketsReceived_;
}

const char* TeleopAdapter::stopReasonToString(TeleopStopReason reason) {
  switch (reason) {
    case TeleopStopReason::NONE:            return "NONE";
    case TeleopStopReason::DEADMAN_RELEASE: return "DEADMAN_RELEASE";
    case TeleopStopReason::FRAME_TIMEOUT:   return "FRAME_TIMEOUT";
    case TeleopStopReason::NOT_ARMED:       return "NOT_ARMED";
    case TeleopStopReason::SAFETY_BLOCK:    return "SAFETY_BLOCK";
    default:                                return "UNKNOWN";
  }
}

void TeleopAdapter::processIncomingByte(char c) {
  if (c == '\r') {
    return;
  }

  // 프레임 중간에 새 JSON 시작 문자가 보이면, 이전 프레임은 깨진 것으로 보고 최신 시작점으로 재동기화한다.
  if (c == '{' && lineIndex_ > 0) {
    parseErrors_++;
    warnParserDrop("framing resync");
    overflowDropping_ = false;
    lineIndex_ = 0;
    lineBuffer_[0] = '\0';
  }

  // UART attach/reset 시 중간 프레임 조각이나 잡음을 만나도 JSON 시작 전까지는 버린다.
  if (lineIndex_ == 0 && c != '{' && c != '\n') {
    return;
  }

  if (overflowDropping_) {
    if (c == '\n') {
      overflowDropping_ = false;
      lineIndex_ = 0;
      lineBuffer_[0] = '\0';
    }
    return;
  }

  if (c == '\n') {
    if (lineIndex_ == 0) {
      return;
    }

    lineBuffer_[lineIndex_] = '\0';
    TeleopFrame parsedFrame;
    if (parseTeleopLine(lineBuffer_, parsedFrame)) {
      handleFrame(parsedFrame, millis());
    } else {
      parseErrors_++;
      warnParserDrop("parse failed", lineBuffer_);
    }

    lineIndex_ = 0;
    lineBuffer_[0] = '\0';
    return;
  }

  if (lineIndex_ >= (LINE_BUFFER_SIZE - 1)) {
    overflowDropping_ = true;
    parseErrors_++;
    warnParserDrop("line overflow");
    lineIndex_ = 0;
    lineBuffer_[0] = '\0';
    return;
  }

  lineBuffer_[lineIndex_++] = c;
  lineBuffer_[lineIndex_] = '\0';
}

bool TeleopAdapter::parseTeleopLine(const char* line, TeleopFrame& outFrame) const {
  if (line == nullptr) {
    return false;
  }

  String json(line);
  json.trim();
  if (json.length() == 0 || json[0] != '{') {
    return false;
  }

  String typeValue;
  if (!extractRawValue(json, "type", typeValue)) {
    return false;
  }
  typeValue.toLowerCase();
  if (typeValue != "teleop") {
    return false;
  }

  if (!extractUInt32(json, "ts_ms", outFrame.sourceTimestampMs)) return false;
  if (!extractUInt32(json, "seq", outFrame.sequence)) return false;
  if (!extractUInt32(json, "session", outFrame.session)) return false;
  if (!extractBool(json, "deadman", outFrame.deadman)) return false;
  if (!extractFloat(json, "reach", outFrame.reach)) return false;
  if (!extractFloat(json, "lift", outFrame.lift)) return false;
  if (!extractFloat(json, "twist", outFrame.twist)) return false;
  if (!extractBool(json, "grip_open", outFrame.gripOpen)) return false;
  if (!extractBool(json, "grip_close", outFrame.gripClose)) return false;
  if (!extractUInt32(json, "led_toggle_seq", outFrame.ledToggleSeq)) return false;

  SensorSnapshot safetySnapshot;
  bool hasImuOk = extractBool(json, "imu_ok", safetySnapshot.imuOk);
  bool hasRangeOk = extractBool(json, "range_ok", safetySnapshot.rangeOk);
  bool hasVibe = extractFloat(json, "vibe", safetySnapshot.vibe);
  bool hasDistance = extractFloat(json, "dist_cm", safetySnapshot.distanceCm);
  bool hasRoll = extractFloat(json, "roll", safetySnapshot.roll);
  bool hasPitch = extractFloat(json, "pitch", safetySnapshot.pitch);
  bool hasGyroX = extractFloat(json, "gyro_x", safetySnapshot.gyroX);
  bool hasGyroY = extractFloat(json, "gyro_y", safetySnapshot.gyroY);
  bool hasGyroZ = extractFloat(json, "gyro_z", safetySnapshot.gyroZ);
  extractUInt32(json, "imu_status", safetySnapshot.imuStatus);
  extractUInt32(json, "imu_addr", safetySnapshot.imuAddress);
  extractUInt32(json, "imu_error", safetySnapshot.imuError);
  extractBool(json, "i2c_scl", safetySnapshot.i2cSclHigh);
  extractBool(json, "i2c_sda", safetySnapshot.i2cSdaHigh);
  bool hasAnySafetyField = hasImuOk || hasRangeOk || hasVibe || hasDistance ||
                           hasRoll || hasPitch || hasGyroX || hasGyroY || hasGyroZ;
  if (hasAnySafetyField) {
    if (!hasImuOk || !hasRangeOk || !hasVibe || !hasDistance) {
      return false;
    }
    safetySnapshot.connected = true;
    safetySnapshot.sourceTimestampMs = outFrame.sourceTimestampMs;
    safetySnapshot.obstacleSafetyEnabled = false;
    safetySnapshot.vibrationSafetyEnabled = false;
    outFrame.hasSafetySnapshot = true;
    outFrame.safetySnapshot = safetySnapshot;
  }

  outFrame.reach = clampUnit(outFrame.reach);
  outFrame.lift = clampUnit(outFrame.lift);
  outFrame.twist = clampUnit(outFrame.twist);
  return true;
}

void TeleopAdapter::handleFrame(const TeleopFrame& frame, uint32_t now) {
  bool firstPacket = (packetsReceived_ == 0);

  lastFrame_ = frame;
  lastFrameReceivedMs_ = now;
  packetsReceived_++;

  if (frame.hasSafetySnapshot) {
    embeddedSafetySnapshot_ = frame.safetySnapshot;
    embeddedSafetySnapshot_.connected = true;
    embeddedSafetySnapshot_.lastUpdateMs = now;
    embeddedSafetyPacketsReceived_++;
  }

  if (firstPacket) {
    DebugLog::info("Teleop adapter connected - first frame received");
    eventLog.push("teleop", "CONNECTED", EventSeverity::INFO, "first teleop frame");
  }

  if (embeddedSafetyPacketsReceived_ == 1 && frame.hasSafetySnapshot) {
    DebugLog::info("Teleop embedded safety telemetry connected");
    eventLog.push("safety", "TELEOP_EMBEDDED_CONNECTED", EventSeverity::INFO, "teleop frame safety fields");
  }

  updateLedToggleIfNeeded();
}

void TeleopAdapter::handleFreshFrame(uint32_t now) {
  (void)now;

  if (!isReady()) {
    stopControlledOutputs(TeleopStopReason::FRAME_TIMEOUT, "adapter dependencies missing", false);
    return;
  }

  if (!lastFrame_.deadman) {
    if (controlActive_) {
      stopControlledOutputs(TeleopStopReason::DEADMAN_RELEASE, "deadman released");
    } else {
      lastStopReason_ = TeleopStopReason::DEADMAN_RELEASE;
    }
    return;
  }

  if (systemState_->getState() != SystemState::ARMED) {
    if (controlActive_) {
      stopControlledOutputs(TeleopStopReason::NOT_ARMED, systemState_->getStateString());
    } else {
      lastStopReason_ = TeleopStopReason::NOT_ARMED;
    }
    return;
  }

  if (safetyMonitor_->isMotionBlocked()) {
    if (controlActive_) {
      stopControlledOutputs(TeleopStopReason::SAFETY_BLOCK, safetyMonitor_->getBlockReasonString());
    } else {
      lastStopReason_ = TeleopStopReason::SAFETY_BLOCK;
    }
    return;
  }

  systemState_->resetTimeout();
  applyContinuousOutputs();
}

void TeleopAdapter::stopControlledOutputs(TeleopStopReason reason, const char* detail, bool updateReason) {
  if (motorControl_ != nullptr) {
    motorControl_->hardStopAll();
  }
  appliedGripPercent_ = 0;
  appliedWristPercent_ = 0;
  appliedElbowPercent_ = 0;
  appliedShoulderPercent_ = 0;
  appliedBasePercent_ = 0;

  if (controlActive_) {
    DebugLog::info("Teleop stop: %s%s%s",
                   stopReasonToString(reason),
                   detail != nullptr ? " - " : "",
                   detail != nullptr ? detail : "");
    eventLog.push("teleop", "STOP", EventSeverity::INFO, detail != nullptr ? detail : stopReasonToString(reason));
  }

  controlActive_ = false;
  if (updateReason) {
    lastStopReason_ = reason;
  }
}

void TeleopAdapter::updateLedToggleIfNeeded() {
  if (lastFrame_.ledToggleSeq <= lastHandledLedToggleSeq_) {
    return;
  }

  lastHandledLedToggleSeq_ = lastFrame_.ledToggleSeq;
  if (submitLightToggle()) {
    eventLog.push("teleop", "LED_TOGGLE", EventSeverity::INFO, "teleop edge");
  }
}

void TeleopAdapter::applyContinuousOutputs() {
  float reach = clampUnit(lastFrame_.reach);
  float lift = clampUnit(lastFrame_.lift);
  float twist = clampUnit(lastFrame_.twist);

  if (lastFrame_.hasSafetySnapshot) {
    if (!rollLiftNeutralValid_ || rollLiftSession_ != lastFrame_.session) {
      rollLiftSession_ = lastFrame_.session;
      rollLiftNeutralDeg_ = lastFrame_.safetySnapshot.roll;
      rollLiftNeutralValid_ = true;
    }

    float rawRollLift = normalizeAxis((lastFrame_.safetySnapshot.roll - rollLiftNeutralDeg_) * LIFT_TO_SHOULDER_SIGN,
                                      LIFT_ROLL_DEADZONE_DEG,
                                      LIFT_ROLL_FULL_SCALE_DEG);
    if (absf(rawRollLift) > absf(lift)) {
      lift = rawRollLift;
    }
  }

  float liftCommand = clampUnit(lift * LIFT_RESPONSE_GAIN);
  float reachForArm = reach;
  float absLiftCommand = absf(liftCommand);
  if (absLiftCommand > LIFT_REACH_SUPPRESSION_START) {
    float suppression = (absLiftCommand - LIFT_REACH_SUPPRESSION_START) / (1.0f - LIFT_REACH_SUPPRESSION_START);
    if (suppression > 1.0f) suppression = 1.0f;
    reachForArm *= (1.0f - (suppression * LIFT_REACH_SUPPRESSION_MAX));
    twist = 0.0f;
  }

  float shoulder = clampUnit((liftCommand * LIFT_TO_SHOULDER_SIGN * LIFT_TO_SHOULDER_WEIGHT) +
                             (reachForArm * REACH_TO_SHOULDER_SIGN * REACH_TO_SHOULDER_WEIGHT));
  float elbow = clampUnit((liftCommand * LIFT_TO_ELBOW_SIGN * LIFT_TO_ELBOW_WEIGHT) +
                          (reachForArm * REACH_TO_ELBOW_SIGN * REACH_TO_ELBOW_WEIGHT));

  float wrist = 0.0f;
  float absReach = absf(reachForArm);
  if (absReach > WRIST_BLEND_START) {
    float wristBlend = (absReach - WRIST_BLEND_START) / (1.0f - WRIST_BLEND_START);
    if (wristBlend > 1.0f) wristBlend = 1.0f;
    wrist = (reachForArm >= 0.0f ? 1.0f : -1.0f) * wristBlend * WRIST_BLEND_MAX * REACH_TO_WRIST_SIGN;
  }

  float base = clampUnit(twist * TWIST_TO_BASE_SIGN * TWIST_TO_BASE_WEIGHT);

  int8_t gripPercent = 0;
  if (lastFrame_.gripOpen && !lastFrame_.gripClose) {
    gripPercent = static_cast<int8_t>(GRIPPER_BUTTON_PERCENT);
  } else if (lastFrame_.gripClose && !lastFrame_.gripOpen) {
    gripPercent = -static_cast<int8_t>(GRIPPER_BUTTON_PERCENT);
  }

  int8_t wristPercent = quantizeNormalized(wrist);
  uint8_t armOutputCapPercent = absLiftCommand >= 0.05f ? LIFT_OUTPUT_CAP_PERCENT : CONTINUOUS_OUTPUT_CAP_PERCENT;
  int8_t elbowPercent = quantizeNormalized(elbow, armOutputCapPercent);
  int8_t shoulderPercent = quantizeNormalized(shoulder, armOutputCapPercent);
  int8_t basePercent = quantizeNormalized(base, BASE_OUTPUT_CAP_PERCENT);

  if (basePercent != 0 && angleController_ != nullptr && angleController_->isActive()) {
    angleController_->cancel(AngleControllerStopReason::OVERRIDDEN, "teleop twist");
  }

  if ((gripPercent != 0 || wristPercent != 0 || elbowPercent != 0 || shoulderPercent != 0 || basePercent != 0)
      && motionSequence_ != nullptr && motionSequence_->getState() == SequenceState::RUNNING) {
    motionSequence_->stop();
    eventLog.push("teleop", "SEQUENCE_OVERRIDE", EventSeverity::WARN, "teleop motion override");
  }

  applyJointOutputs(gripPercent, wristPercent, elbowPercent, shoulderPercent, basePercent);
  controlActive_ = (gripPercent != 0 || wristPercent != 0 || elbowPercent != 0 || shoulderPercent != 0 || basePercent != 0);
}

void TeleopAdapter::applyJointOutputs(int8_t gripPercent,
                                      int8_t wristPercent,
                                      int8_t elbowPercent,
                                      int8_t shoulderPercent,
                                      int8_t basePercent) {
  applyJointSemanticPercent(MotorControl::MOTOR_1, gripPercent, appliedGripPercent_, RobotArm::GRIPPER_OPEN_IS_FORWARD);
  applyJointSemanticPercent(MotorControl::MOTOR_2, wristPercent, appliedWristPercent_, RobotArm::WRIST_UP_IS_FORWARD);
  applyJointSemanticPercent(MotorControl::MOTOR_3, elbowPercent, appliedElbowPercent_, RobotArm::ELBOW_UP_IS_FORWARD);
  applyJointSemanticPercent(MotorControl::MOTOR_4, shoulderPercent, appliedShoulderPercent_, RobotArm::SHOULDER_UP_IS_FORWARD);
  applyJointSemanticPercent(MotorControl::MOTOR_5, basePercent, appliedBasePercent_, RobotArm::BASE_LEFT_IS_FORWARD);
}

void TeleopAdapter::applyJointSemanticPercent(uint8_t motorId, int8_t requestedPercent, int8_t& appliedPercent, bool positiveMeansForward) {
  if (motorControl_ == nullptr) {
    return;
  }

  int8_t quantizedPercent = 0;
  if (requestedPercent > 0) {
    quantizedPercent = quantizePercentMagnitude(static_cast<uint8_t>(requestedPercent));
  } else if (requestedPercent < 0) {
    quantizedPercent = -static_cast<int8_t>(quantizePercentMagnitude(static_cast<uint8_t>(-requestedPercent)));
  }

  if (quantizedPercent == appliedPercent) {
    return;
  }

  if (quantizedPercent == 0) {
    motorControl_->stop(motorId);
    appliedPercent = 0;
    return;
  }

  bool semanticPositive = quantizedPercent > 0;
  bool rawForward = semanticPositive ? positiveMeansForward : !positiveMeansForward;
  uint8_t magnitude = static_cast<uint8_t>(semanticPositive ? quantizedPercent : -quantizedPercent);

  if (rawForward) {
    motorControl_->forward(motorId, magnitude);
  } else {
    motorControl_->reverse(motorId, magnitude);
  }
  appliedPercent = quantizedPercent;
}

bool TeleopAdapter::submitLightToggle() {
  if (commandBus_ == nullptr || dispatcher_ == nullptr) {
    return false;
  }

  Command command;
  command.id = commandBus_->allocateId();
  command.type = CommandType::LIGHT_TOGGLE;
  command.source = CommandSource::INTERNAL;
  command.createdAtMs = millis();

  CommandResult result;
  if (!dispatcher_->execute(command, result)) {
    DebugLog::warn("Teleop LED toggle failed: %s", result.message);
    return false;
  }
  return result.success;
}

float TeleopAdapter::clampUnit(float value) {
  if (value > 1.0f) return 1.0f;
  if (value < -1.0f) return -1.0f;
  return value;
}

float TeleopAdapter::absf(float value) {
  return value < 0.0f ? -value : value;
}

float TeleopAdapter::normalizeAxis(float value, float deadzone, float fullScale) {
  if (fullScale <= deadzone) {
    return 0.0f;
  }

  float magnitude = absf(value);
  if (magnitude <= deadzone) {
    return 0.0f;
  }

  float normalized = (magnitude - deadzone) / (fullScale - deadzone);
  if (normalized > 1.0f) {
    normalized = 1.0f;
  }

  return value >= 0.0f ? normalized : -normalized;
}

int8_t TeleopAdapter::quantizeNormalized(float value, uint8_t capPercent) {
  float clamped = clampUnit(value);
  float magnitude = absf(clamped);
  if (magnitude < 0.05f) {
    return 0;
  }

  uint8_t percent = static_cast<uint8_t>(magnitude * 100.0f + 0.5f);
  if (percent > capPercent) {
    percent = capPercent;
  }
  uint8_t quantized = quantizePercentMagnitude(percent);
  if (quantized == 0) {
    return 0;
  }
  return clamped >= 0.0f ? static_cast<int8_t>(quantized) : -static_cast<int8_t>(quantized);
}

int8_t TeleopAdapter::quantizePercentMagnitude(uint8_t percent) {
  if (percent < OUTPUT_QUANT_STEP_PERCENT) {
    return 0;
  }
  uint8_t quantized = static_cast<uint8_t>(((percent + (OUTPUT_QUANT_STEP_PERCENT / 2)) / OUTPUT_QUANT_STEP_PERCENT) * OUTPUT_QUANT_STEP_PERCENT);
  if (quantized > 100) {
    quantized = 100;
  }
  return static_cast<int8_t>(quantized);
}

void TeleopAdapter::warnParserDrop(const char* reason, const char* line) {
  uint32_t now = millis();
  if (lastParserWarningMs_ != 0 && (now - lastParserWarningMs_) < PARSER_WARNING_INTERVAL_MS) {
    suppressedParserWarnings_++;
    return;
  }

  uint32_t suppressed = suppressedParserWarnings_;
  suppressedParserWarnings_ = 0;
  lastParserWarningMs_ = now;

  if (line == nullptr || line[0] == '\0') {
    DebugLog::warn("Teleop adapter %s (parseErrors=%lu, suppressed=%lu)",
                   reason,
                   parseErrors_,
                   suppressed);
    return;
  }

  char preview[81];
  size_t lineLength = strlen(line);
  size_t previewLength = lineLength < (sizeof(preview) - 1) ? lineLength : (sizeof(preview) - 1);
  memcpy(preview, line, previewLength);
  preview[previewLength] = '\0';

  DebugLog::warn("Teleop adapter %s len=%u preview=%s%s (parseErrors=%lu, suppressed=%lu)",
                 reason,
                 static_cast<unsigned>(lineLength),
                 preview,
                 lineLength > previewLength ? "..." : "",
                 parseErrors_,
                 suppressed);
}
