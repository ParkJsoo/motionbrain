#ifndef STM32_BRIDGE_H
#define STM32_BRIDGE_H

#include <Arduino.h>
#include <stdint.h>
#include "safety/sensor_snapshot.h"

class Stm32Bridge {
public:
  static const uint32_t BAUD_RATE = 115200;
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

private:
  static const size_t LINE_BUFFER_SIZE = 256;

  HardwareSerial* serial_;
  SensorSnapshot snapshot_;
  char lineBuffer_[LINE_BUFFER_SIZE];
  size_t lineIndex_;
  bool overflowDropping_;
  uint32_t packetsReceived_;
  uint32_t parseErrors_;

  void processIncomingByte(char c);
  bool parseSensorLine(const char* line, SensorSnapshot& parsedSnapshot) const;
};

#endif // STM32_BRIDGE_H
