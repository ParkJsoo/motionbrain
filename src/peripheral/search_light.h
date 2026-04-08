#ifndef SEARCH_LIGHT_H
#define SEARCH_LIGHT_H

#include <Arduino.h>

/**
 * SearchLight — OWI-535 서치라이트 LED 제어
 *
 * 핀: GPIO5 (100Ω 저항 직렬)
 * 시스템 상태(ARMED/IDLE)와 무관하게 항상 제어 가능
 */
class SearchLight {
public:
  static const uint8_t PIN_LIGHT = 5;  // GPIO5

  SearchLight();

  void init();
  void on();
  void off();
  void toggle();
  bool isOn() const;

private:
  bool on_;
};

#endif // SEARCH_LIGHT_H
