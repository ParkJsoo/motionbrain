#pragma once
#include <stdint.h>
#include <stdlib.h>
static const uint8_t OUTPUT = 1;
static const uint8_t HIGH = 1;
static const uint8_t LOW = 0;
uint32_t millis();
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
void ledcSetup(uint8_t channel, uint32_t frequency, uint8_t resolution);
void ledcAttachPin(uint8_t pin, uint8_t channel);
void ledcWrite(uint8_t channel, uint32_t duty);
