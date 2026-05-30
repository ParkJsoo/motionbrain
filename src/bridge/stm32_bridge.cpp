#include "bridge/stm32_bridge.h"
#include "debug/debug_log.h"
#include <stdlib.h>
#include <string.h>

namespace {

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

Stm32Bridge::Stm32Bridge()
  : serial_(&Serial2)
  , snapshot_()
  , lineBuffer_{0}
  , lineIndex_(0)
  , overflowDropping_(false)
  , packetsReceived_(0)
  , parseErrors_(0)
  , simulationMode_(SensorSimulationMode::OFF)
  , simulatedSnapshot_()
  , simulatedPacketPeriodMs_(DEFAULT_SIM_PACKET_PERIOD_MS)
  , lastSimulatedEmitMs_(0)
  , nextSimulatedSourceTimestampMs_(0) {}

bool Stm32Bridge::init() {
  serial_->begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  DebugLog::info("STM32 bridge initialized (Serial2 RX=%d, expects STM32 USART2 TX=PD5/D1 @ %lu)", RX_PIN, BAUD_RATE);
  return true;
}

void Stm32Bridge::update() {
  if (simulationMode_ == SensorSimulationMode::AUTO) {
    uint32_t now = millis();
    if (lastSimulatedEmitMs_ == 0 || (now - lastSimulatedEmitMs_) >= simulatedPacketPeriodMs_) {
      emitSimulatedSnapshot(now);
    }
    return;
  }

  while (serial_ != nullptr && serial_->available() > 0) {
    processIncomingByte(static_cast<char>(serial_->read()));
  }
}

const SensorSnapshot& Stm32Bridge::getSnapshot() const {
  return snapshot_;
}

bool Stm32Bridge::isConnected() const {
  if (!snapshot_.connected) {
    return false;
  }
  return getLastPacketAgeMs() <= LINK_TIMEOUT_MS;
}

uint32_t Stm32Bridge::getLastPacketAgeMs() const {
  if (!snapshot_.connected) {
    return 0;
  }
  return millis() - snapshot_.lastUpdateMs;
}

uint32_t Stm32Bridge::getPacketsReceived() const {
  return packetsReceived_;
}

uint32_t Stm32Bridge::getParseErrors() const {
  return parseErrors_;
}

bool Stm32Bridge::isSimulationEnabled() const {
  return simulationMode_ != SensorSimulationMode::OFF;
}

SensorSimulationMode Stm32Bridge::getSimulationMode() const {
  return simulationMode_;
}

const char* Stm32Bridge::getSimulationModeString() const {
  switch (simulationMode_) {
    case SensorSimulationMode::OFF:    return "OFF";
    case SensorSimulationMode::AUTO:   return "AUTO";
    case SensorSimulationMode::FROZEN: return "FROZEN";
    default:                           return "UNKNOWN";
  }
}

void Stm32Bridge::setSimulatedSnapshot(const SensorSnapshot& snapshot, uint32_t packetPeriodMs) {
  simulatedSnapshot_ = snapshot;
  simulatedSnapshot_.connected = true;
  simulatedSnapshot_.imuOk = snapshot.imuOk;
  simulatedSnapshot_.rangeOk = snapshot.rangeOk;
  simulatedPacketPeriodMs_ = packetPeriodMs > 0 ? packetPeriodMs : DEFAULT_SIM_PACKET_PERIOD_MS;
  simulationMode_ = SensorSimulationMode::AUTO;
  lastSimulatedEmitMs_ = 0;
  nextSimulatedSourceTimestampMs_ = snapshot.sourceTimestampMs;
  if (nextSimulatedSourceTimestampMs_ == 0) {
    nextSimulatedSourceTimestampMs_ = millis();
  }
  DebugLog::info("STM32 bridge simulation enabled (%s, period=%lums)",
                 getSimulationModeString(), simulatedPacketPeriodMs_);
}

void Stm32Bridge::freezeSimulation() {
  if (simulationMode_ == SensorSimulationMode::OFF) {
    return;
  }
  simulationMode_ = SensorSimulationMode::FROZEN;
  DebugLog::info("STM32 bridge simulation frozen");
}

void Stm32Bridge::clearSimulation(bool clearSnapshot) {
  simulationMode_ = SensorSimulationMode::OFF;
  simulatedSnapshot_ = SensorSnapshot();
  simulatedPacketPeriodMs_ = DEFAULT_SIM_PACKET_PERIOD_MS;
  lastSimulatedEmitMs_ = 0;
  nextSimulatedSourceTimestampMs_ = 0;
  if (clearSnapshot) {
    snapshot_ = SensorSnapshot();
  }
  DebugLog::info("STM32 bridge simulation disabled%s", clearSnapshot ? " and snapshot cleared" : "");
}

void Stm32Bridge::processIncomingByte(char c) {
  if (c == '\r') {
    return;
  }

  if (c == '\n') {
    if (overflowDropping_) {
      overflowDropping_ = false;
      lineIndex_ = 0;
      lineBuffer_[0] = '\0';
      return;
    }

    if (lineIndex_ == 0) {
      return;
    }

    lineBuffer_[lineIndex_] = '\0';

    SensorSnapshot parsedSnapshot;
    if (parseSensorLine(lineBuffer_, parsedSnapshot)) {
      bool wasConnected = snapshot_.connected;
      snapshot_ = parsedSnapshot;
      packetsReceived_++;

      if (!wasConnected) {
        DebugLog::info("STM32 bridge connected - first sensor packet received");
      }
    } else {
      parseErrors_++;
      DebugLog::warn("STM32 bridge parse failed: %s", lineBuffer_);
    }

    lineIndex_ = 0;
    lineBuffer_[0] = '\0';
    return;
  }

  if (overflowDropping_) {
    return;
  }

  if (lineIndex_ >= LINE_BUFFER_SIZE - 1) {
    overflowDropping_ = true;
    lineIndex_ = 0;
    lineBuffer_[0] = '\0';
    parseErrors_++;
    DebugLog::warn("STM32 bridge line overflow - dropping until newline");
    return;
  }

  lineBuffer_[lineIndex_++] = c;
}

bool Stm32Bridge::parseSensorLine(const char* line, SensorSnapshot& parsedSnapshot) const {
  if (line == nullptr || line[0] == '\0') {
    return false;
  }

  String json(line);
  json.trim();
  if (json.length() == 0 || json[0] != '{') {
    return false;
  }

  String packetType;
  if (extractRawValue(json, "type", packetType)) {
    packetType.toLowerCase();
    if (packetType != "sensor") {
      return false;
    }
  }

  SensorSnapshot candidate;
  candidate.connected = true;
  candidate.lastUpdateMs = millis();

  bool hasTimestamp = extractUInt32(json, "ts_ms", candidate.sourceTimestampMs);
  bool hasImuOk = extractBool(json, "imu_ok", candidate.imuOk);
  bool hasRangeOk = extractBool(json, "range_ok", candidate.rangeOk);
  bool hasRoll = extractFloat(json, "roll", candidate.roll);
  bool hasPitch = extractFloat(json, "pitch", candidate.pitch);
  bool hasGyroX = extractFloat(json, "gyro_x", candidate.gyroX);
  bool hasGyroY = extractFloat(json, "gyro_y", candidate.gyroY);
  bool hasGyroZ = extractFloat(json, "gyro_z", candidate.gyroZ);
  bool hasVibe = extractFloat(json, "vibe", candidate.vibe);
  bool hasDistance = extractFloat(json, "dist_cm", candidate.distanceCm);

  if (!hasTimestamp && !hasRoll && !hasPitch && !hasGyroX && !hasGyroY && !hasGyroZ && !hasVibe && !hasDistance) {
    return false;
  }

  parsedSnapshot = candidate;
  return true;
}

void Stm32Bridge::emitSimulatedSnapshot(uint32_t now) {
  snapshot_ = simulatedSnapshot_;
  snapshot_.connected = true;
  snapshot_.lastUpdateMs = now;
  snapshot_.sourceTimestampMs = nextSimulatedSourceTimestampMs_;
  packetsReceived_++;
  lastSimulatedEmitMs_ = now;
  nextSimulatedSourceTimestampMs_ += simulatedPacketPeriodMs_;
}
