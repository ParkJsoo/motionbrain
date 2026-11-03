#pragma once
#include "../stubs/Arduino.h"
#include <stdio.h>
#include <string>

// Only Arduino formatting/IO is substituted; state and safety logic are real.
class String {
public:
  std::string value;
  String() {}
  String(const char* text) : value(text) {}
  template<class Number> String(Number number) : value(std::to_string(number)) {}
  String(float number, unsigned precision) {
    char buffer[80];
    snprintf(buffer, sizeof(buffer), "%.*f", static_cast<int>(precision), number);
    value = buffer;
  }
  const char* c_str() const { return value.c_str(); }
};
inline String operator+(const char* prefix, const String& suffix) {
  String result(prefix); result.value += suffix.value; return result;
}
