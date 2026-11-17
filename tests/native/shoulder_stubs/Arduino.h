#pragma once
#include "../stubs/Arduino.h"
#include <string>
#include <sstream>
#include <iomanip>
// Only formatting/IO boundaries are fake. Controllers and PWM ramp are real.
class String : public std::string {
public:
  String() = default;
  String(const char* s):std::string(s){}
  String(const std::string& s):std::string(s){}
  template<class T> String(T v):std::string(std::to_string(v)){}
  String(float v, int digits) { std::ostringstream s; s<<std::fixed<<std::setprecision(digits)<<v; assign(s.str()); }
};
inline String operator+(const String& a,const String& b){return String(static_cast<const std::string&>(a)+static_cast<const std::string&>(b));}
inline String operator+(const char* a,const String& b){return String(std::string(a)+static_cast<const std::string&>(b));}
inline String operator+(const String& a,const char* b){return String(static_cast<const std::string&>(a)+std::string(b));}
