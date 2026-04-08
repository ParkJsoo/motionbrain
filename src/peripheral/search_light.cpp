#include "search_light.h"
#include "debug/debug_log.h"

SearchLight::SearchLight() : on_(false) {}

void SearchLight::init() {
  pinMode(PIN_LIGHT, OUTPUT);
  digitalWrite(PIN_LIGHT, LOW);
  on_ = false;
  DebugLog::info("SearchLight initialized (GPIO%d, OFF)", PIN_LIGHT);
}

void SearchLight::on() {
  digitalWrite(PIN_LIGHT, HIGH);
  on_ = true;
  DebugLog::info("SearchLight: ON");
}

void SearchLight::off() {
  digitalWrite(PIN_LIGHT, LOW);
  on_ = false;
  DebugLog::info("SearchLight: OFF");
}

void SearchLight::toggle() {
  if (on_) off(); else on();
}

bool SearchLight::isOn() const {
  return on_;
}
