#ifndef STM32_BRIDGE_H
#define STM32_BRIDGE_H

#include <Arduino.h>
#include <stdint.h>
#include "safety/sensor_snapshot.h"

enum class SensorSimulationMode : uint8_t {
  OFF = 0,
  AUTO,
  FROZEN
};

class Stm32Bridge {
public:
  static const uint32_t BAUD_RATE = 115200;
  static const uint32_t LINK_TIMEOUT_MS = 1000;
  static const uint32_t DEFAULT_SIM_PACKET_PERIOD_MS = 100;
  static const int RX_PIN = 35;
  static const int TX_PIN = -1;

  Stm32Bridge();

  bool init();
  void update();

  const SensorSnapshot& getSnapshot() const;
  bool isConnected() const;
  uint32_t getLastPacketAgeMs() const;
  uint32_t getPacketsReceived() const;
  uint32_t getParseErrors() const;
  bool isSimulationEnabled() const;
  SensorSimulationMode getSimulationMode() const;
  const char* getSimulationModeString() const;

  void setSimulatedSnapshot(const SensorSnapshot& snapshot,
                            uint32_t packetPeriodMs = DEFAULT_SIM_PACKET_PERIOD_MS);
  void freezeSimulation();
  void clearSimulation(bool clearSnapshot = true);

private:
  static const size_t LINE_BUFFER_SIZE = 256;

  HardwareSerial* serial_;
  SensorSnapshot snapshot_;
  char lineBuffer_[LINE_BUFFER_SIZE];
  size_t lineIndex_;
  bool overflowDropping_;
  uint32_t packetsReceived_;
  uint32_t parseErrors_;
  SensorSimulationMode simulationMode_;
  SensorSnapshot simulatedSnapshot_;
  uint32_t simulatedPacketPeriodMs_;
  uint32_t lastSimulatedEmitMs_;
  uint32_t nextSimulatedSourceTimestampMs_;

  void processIncomingByte(char c);
  bool parseSensorLine(const char* line, SensorSnapshot& parsedSnapshot) const;
  void emitSimulatedSnapshot(uint32_t now);
};

#endif // STM32_BRIDGE_H
