#ifndef PERIPHERAL_SHOULDER_ANGLE_SENSOR_H
#define PERIPHERAL_SHOULDER_ANGLE_SENSOR_H

#include <Arduino.h>
#include <stdint.h>

#ifndef MOTIONBRAIN_SHOULDER_I2C_SDA_PIN
#define MOTIONBRAIN_SHOULDER_I2C_SDA_PIN 0
#endif

#ifndef MOTIONBRAIN_SHOULDER_I2C_SCL_PIN
#define MOTIONBRAIN_SHOULDER_I2C_SCL_PIN 15
#endif

#ifndef MOTIONBRAIN_SHOULDER_ADC_PIN
#define MOTIONBRAIN_SHOULDER_ADC_PIN 36
#endif

/**
 * AS5600 absolute-angle feedback reader for the shoulder joint.
 *
 * I2C is the control input. The analog OUT pin remains connected to
 * ESP32 VP/GPIO36 (ADC1_CH0) for diagnostic comparison only because the
 * observed analog signal saturated during bring-up.
 */
class ShoulderAngleSensor {
public:
  static constexpr uint8_t ADC_PIN = MOTIONBRAIN_SHOULDER_ADC_PIN;
  static constexpr uint8_t I2C_SDA_PIN = MOTIONBRAIN_SHOULDER_I2C_SDA_PIN;
  static constexpr uint8_t I2C_SCL_PIN = MOTIONBRAIN_SHOULDER_I2C_SCL_PIN;
  static constexpr uint8_t I2C_ADDRESS = 0x36;
  // Trial-mount calibration: raw 258.93 deg corresponds to the previously
  // established shoulder coordinate 234.58 deg after the 2026-06-28 remount.
  static constexpr float MOUNT_OFFSET_DEGREES = -24.35f;

  ShoulderAngleSensor();

  void init();
  void update(int16_t shoulderMotorSpeed);
  void setWatchEnabled(bool enabled);
  bool isWatchEnabled() const;

  uint16_t getRaw() const;
  uint32_t getMilliVolts() const;
  float getEstimatedDegrees() const;
  bool isI2cConnected() const;
  bool isI2cFresh(uint32_t maxAgeMs = 150) const;
  bool isReadyForMotion(uint32_t maxAgeMs = 150) const;
  bool isMagnetDetected() const;
  bool isMagnetTooWeak() const;
  bool isMagnetTooStrong() const;
  uint16_t getI2cRawAngle() const;
  float getI2cRawDegrees() const;
  float getI2cDegrees() const;
  uint8_t getAgc() const;
  uint16_t getMagnitude() const;
  uint32_t getI2cLastUpdateMs() const;
  uint32_t getI2cAgeMs() const;

private:
  static constexpr uint32_t SAMPLE_INTERVAL_MS = 20;
  static constexpr uint32_t MOVING_LOG_INTERVAL_MS = 100;
  static constexpr uint32_t I2C_POLL_INTERVAL_MS = 20;

  bool initialized_;
  bool wasMoving_;
  bool watchEnabled_;
  uint16_t raw_;
  uint32_t milliVolts_;
  uint32_t lastSampleMs_;
  uint32_t lastLogMs_;
  bool i2cConnected_;
  uint8_t magnetStatus_;
  uint16_t i2cRawAngle_;
  uint8_t agc_;
  uint16_t magnitude_;
  uint32_t lastI2cPollMs_;
  uint32_t lastI2cUpdateMs_;

  void sample(uint32_t now);
  void pollI2c(uint32_t now);
  bool readI2c(uint8_t reg, uint8_t* data, size_t size);
  void logSample(const char* phase, int16_t shoulderMotorSpeed) const;
};

#endif // PERIPHERAL_SHOULDER_ANGLE_SENSOR_H
