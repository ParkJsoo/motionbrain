#ifndef SENSOR_SNAPSHOT_H
#define SENSOR_SNAPSHOT_H

#include <Arduino.h>
#include <stdint.h>

struct SensorSnapshot {
  bool connected;
  bool imuOk;
  bool rangeOk;
  uint32_t sourceTimestampMs;
  uint32_t lastUpdateMs;
  float roll;
  float pitch;
  float gyroX;
  float gyroY;
  float gyroZ;
  float vibe;
  float distanceCm;

  SensorSnapshot()
    : connected(false)
    , imuOk(false)
    , rangeOk(false)
    , sourceTimestampMs(0)
    , lastUpdateMs(0)
    , roll(0.0f)
    , pitch(0.0f)
    , gyroX(0.0f)
    , gyroY(0.0f)
    , gyroZ(0.0f)
    , vibe(0.0f)
    , distanceCm(0.0f) {}
};

#endif // SENSOR_SNAPSHOT_H
