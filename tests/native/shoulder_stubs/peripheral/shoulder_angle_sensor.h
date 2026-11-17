#pragma once
#include <Arduino.h>
class ShoulderAngleSensor {
public:
  static constexpr uint8_t I2C_ADDRESS=0x36,I2C_SDA_PIN=0,I2C_SCL_PIN=15,ADC_PIN=36;
  static constexpr float MOUNT_OFFSET_DEGREES=-24.35f;
  float angle=195.11f;
  uint32_t sampleAt=0;
  bool ready=true;
  float getI2cDegrees() const{return angle;}
  float getI2cRawDegrees() const{return angle-MOUNT_OFFSET_DEGREES;}
  uint16_t getI2cRawAngle() const{return (angle-MOUNT_OFFSET_DEGREES)*4096/360;}
  uint32_t getI2cLastUpdateMs() const{return sampleAt;}
  uint32_t getI2cAgeMs() const{return millis()-sampleAt;}
  bool isI2cFresh(uint32_t age) const{return getI2cAgeMs()<=age;}
  bool isI2cConnected() const{return ready;}
  bool isReadyForMotion(uint32_t age) const{return ready && isI2cFresh(age);}
  bool isMagnetDetected() const{return ready;}
  bool isMagnetTooWeak() const{return false;}
  bool isMagnetTooStrong() const{return false;}
  uint8_t getAgc() const{return 100;}
  uint16_t getMagnitude() const{return 2000;}
};
