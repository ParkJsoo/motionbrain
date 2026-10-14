#pragma once
#include "../stubs/Arduino.h"
#include <stdio.h>
#include <string>
class String {
public:
  std::string value;
  String() {}
  String(float number, unsigned precision) {
    char buffer[80];
    snprintf(buffer, sizeof(buffer), "%.*f", static_cast<int>(precision), number);
    value = buffer;
  }
  String& operator+=(const char* rhs) { value += rhs; return *this; }
  String& operator+=(const String& rhs) { value += rhs.value; return *this; }
  String& operator+=(char rhs) { value += rhs; return *this; }
  template<class Number> String& operator+=(Number rhs) { value += std::to_string(rhs); return *this; }
  const char* c_str() const { return value.c_str(); }
};
