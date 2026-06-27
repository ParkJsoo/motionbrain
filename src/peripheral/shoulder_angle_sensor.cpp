#include "peripheral/shoulder_angle_sensor.h"

#include "debug/debug_log.h"
#include <Wire.h>

namespace {

constexpr uint8_t AS5600_REG_STATUS = 0x0B;
constexpr uint8_t AS5600_REG_RAW_ANGLE = 0x0C;
constexpr uint8_t AS5600_REG_AGC = 0x1A;
constexpr uint8_t AS5600_REG_MAGNITUDE = 0x1B;
constexpr uint8_t AS5600_STATUS_MAGNET_DETECTED = 0x20;
constexpr uint8_t AS5600_STATUS_MAGNET_TOO_WEAK = 0x10;
constexpr uint8_t AS5600_STATUS_MAGNET_TOO_STRONG = 0x08;

} // namespace

ShoulderAngleSensor::ShoulderAngleSensor()
  : initialized_(false)
  , wasMoving_(false)
  , watchEnabled_(false)
  , raw_(0)
  , milliVolts_(0)
  , lastSampleMs_(0)
  , lastLogMs_(0)
  , i2cConnected_(false)
  , magnetStatus_(0)
  , i2cRawAngle_(0)
  , agc_(0)
  , magnitude_(0)
  , lastI2cPollMs_(0)
  , lastI2cUpdateMs_(0) {}

void ShoulderAngleSensor::init() {
  pinMode(ADC_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(ADC_PIN, ADC_11db);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000);
  Wire.setTimeOut(20);

  const uint32_t now = millis();
  sample(now);
  initialized_ = true;

  DebugLog::info("Shoulder angle sensor initialized (AS5600 OUT -> VP/GPIO%d)", ADC_PIN);
  DebugLog::info("AS5600 I2C diagnostic initialized (SDA=GPIO%d SCL=GPIO%d addr=0x%02X)",
                 I2C_SDA_PIN, I2C_SCL_PIN, I2C_ADDRESS);
  logSample("IDLE", 0);
}

void ShoulderAngleSensor::update(int16_t shoulderMotorSpeed) {
  if (!initialized_) {
    init();
  }

  const uint32_t now = millis();
  if ((now - lastSampleMs_) >= SAMPLE_INTERVAL_MS) {
    sample(now);
  }
  if ((now - lastI2cPollMs_) >= I2C_POLL_INTERVAL_MS) {
    pollI2c(now);
  }

  const bool moving = shoulderMotorSpeed != 0;
  const bool movementChanged = moving != wasMoving_;
  const bool movingLogDue = moving &&
    (lastLogMs_ == 0 || (now - lastLogMs_) >= MOVING_LOG_INTERVAL_MS);
  const bool watchLogDue = watchEnabled_ &&
    (lastLogMs_ == 0 || (now - lastLogMs_) >= MOVING_LOG_INTERVAL_MS);

  if (movementChanged || movingLogDue || watchLogDue) {
    const char* phase = moving ? "MOVING" : (watchEnabled_ ? "WATCH" : "STOPPED");
    logSample(phase, shoulderMotorSpeed);
    lastLogMs_ = now;
  }

  wasMoving_ = moving;
}

void ShoulderAngleSensor::setWatchEnabled(bool enabled) {
  watchEnabled_ = enabled;
  lastLogMs_ = 0;
  if (enabled) {
    lastI2cPollMs_ = 0;
  }
}

bool ShoulderAngleSensor::isWatchEnabled() const {
  return watchEnabled_;
}

uint16_t ShoulderAngleSensor::getRaw() const {
  return raw_;
}

uint32_t ShoulderAngleSensor::getMilliVolts() const {
  return milliVolts_;
}

float ShoulderAngleSensor::getEstimatedDegrees() const {
  return static_cast<float>(raw_) * 360.0f / 4096.0f;
}

bool ShoulderAngleSensor::isI2cConnected() const {
  return i2cConnected_;
}

bool ShoulderAngleSensor::isI2cFresh(uint32_t maxAgeMs) const {
  return i2cConnected_ && lastI2cUpdateMs_ != 0 && getI2cAgeMs() <= maxAgeMs;
}

bool ShoulderAngleSensor::isReadyForMotion(uint32_t maxAgeMs) const {
  return isI2cFresh(maxAgeMs) && isMagnetDetected() &&
         !isMagnetTooWeak() && !isMagnetTooStrong();
}

bool ShoulderAngleSensor::isMagnetDetected() const {
  return (magnetStatus_ & AS5600_STATUS_MAGNET_DETECTED) != 0;
}

bool ShoulderAngleSensor::isMagnetTooWeak() const {
  return (magnetStatus_ & AS5600_STATUS_MAGNET_TOO_WEAK) != 0;
}

bool ShoulderAngleSensor::isMagnetTooStrong() const {
  return (magnetStatus_ & AS5600_STATUS_MAGNET_TOO_STRONG) != 0;
}

uint16_t ShoulderAngleSensor::getI2cRawAngle() const {
  return i2cRawAngle_;
}

float ShoulderAngleSensor::getI2cRawDegrees() const {
  return static_cast<float>(i2cRawAngle_) * 360.0f / 4096.0f;
}

float ShoulderAngleSensor::getI2cDegrees() const {
  float degrees = getI2cRawDegrees() + MOUNT_OFFSET_DEGREES;
  while (degrees < 0.0f) {
    degrees += 360.0f;
  }
  while (degrees >= 360.0f) {
    degrees -= 360.0f;
  }
  return degrees;
}

uint8_t ShoulderAngleSensor::getAgc() const {
  return agc_;
}

uint16_t ShoulderAngleSensor::getMagnitude() const {
  return magnitude_;
}

uint32_t ShoulderAngleSensor::getI2cLastUpdateMs() const {
  return lastI2cUpdateMs_;
}

uint32_t ShoulderAngleSensor::getI2cAgeMs() const {
  return lastI2cUpdateMs_ == 0 ? UINT32_MAX : millis() - lastI2cUpdateMs_;
}

void ShoulderAngleSensor::sample(uint32_t now) {
  raw_ = static_cast<uint16_t>(analogRead(ADC_PIN));
  milliVolts_ = analogReadMilliVolts(ADC_PIN);
  lastSampleMs_ = now;
}

void ShoulderAngleSensor::pollI2c(uint32_t now) {
  uint8_t status = 0;
  const bool wasConnected = i2cConnected_;
  uint8_t angleBytes[2] = {0, 0};
  const bool statusRead = readI2c(AS5600_REG_STATUS, &status, 1);
  const bool angleRead = statusRead &&
    readI2c(AS5600_REG_RAW_ANGLE, angleBytes, sizeof(angleBytes));
  i2cConnected_ = statusRead && angleRead;
  lastI2cPollMs_ = now;

  if (!i2cConnected_) {
    magnetStatus_ = 0;
    if (wasConnected) {
      DebugLog::warn("AS5600 I2C disconnected (addr=0x%02X)", I2C_ADDRESS);
    }
    return;
  }

  magnetStatus_ = status;
  i2cRawAngle_ = static_cast<uint16_t>(
    ((static_cast<uint16_t>(angleBytes[0]) << 8) | angleBytes[1]) & 0x0FFF);
  lastI2cUpdateMs_ = now;

  readI2c(AS5600_REG_AGC, &agc_, 1);

  uint8_t magnitudeBytes[2] = {0, 0};
  if (readI2c(AS5600_REG_MAGNITUDE, magnitudeBytes, sizeof(magnitudeBytes))) {
    magnitude_ = static_cast<uint16_t>(
      ((static_cast<uint16_t>(magnitudeBytes[0]) << 8) | magnitudeBytes[1]) & 0x0FFF);
  }

  if (!wasConnected) {
    DebugLog::info("AS5600 I2C connected (addr=0x%02X)", I2C_ADDRESS);
  }
}

bool ShoulderAngleSensor::readI2c(uint8_t reg, uint8_t* data, size_t size) {
  if (data == nullptr || size == 0 || size > 255) {
    return false;
  }

  // Probe with a complete transaction first. This avoids issuing a read on an
  // unpopulated bus while the temporary diagnostic wires are disconnected.
  Wire.beginTransmission(I2C_ADDRESS);
  if (Wire.endTransmission(true) != 0) {
    return false;
  }

  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const size_t received = Wire.requestFrom(I2C_ADDRESS, static_cast<uint8_t>(size));
  if (received != size) {
    while (Wire.available() > 0) {
      Wire.read();
    }
    return false;
  }

  for (size_t i = 0; i < size; ++i) {
    data[i] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

void ShoulderAngleSensor::logSample(const char* phase,
                                    int16_t shoulderMotorSpeed) const {
  if (!i2cConnected_) {
    DebugLog::info("[SHOULDER_AS5600] %s speed=%d adc_raw=%u mV=%lu i2c=NO_DEVICE",
                   phase,
                   shoulderMotorSpeed,
                   raw_,
                   static_cast<unsigned long>(milliVolts_));
    return;
  }

  DebugLog::info(
    "[SHOULDER_AS5600] %s speed=%d adc_raw=%u mV=%lu i2c_raw=%u raw_angle=%.2fdeg angle=%.2fdeg MD=%s ML=%s MH=%s AGC=%u MAG=%u",
    phase,
    shoulderMotorSpeed,
    raw_,
    static_cast<unsigned long>(milliVolts_),
    i2cRawAngle_,
    getI2cRawDegrees(),
    getI2cDegrees(),
    isMagnetDetected() ? "YES" : "NO",
    isMagnetTooWeak() ? "YES" : "NO",
    isMagnetTooStrong() ? "YES" : "NO",
    agc_,
    magnitude_);
}
