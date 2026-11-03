#pragma once
#include "../stubs/Arduino.h"
#include <string>
#include <cctype>
#include <cstdio>
class String {
  std::string value_;
public:
  String() {}
  String(const char* value) : value_(value) {}
  String(const std::string& value) : value_(value) {}
  String& operator+=(const char* value) { value_ += value; return *this; }
  int length() const { return static_cast<int>(value_.size()); }
  char operator[](int i) const { return value_[static_cast<size_t>(i)]; }
  int indexOf(const String& value, int start = 0) const {
    auto at = value_.find(value.value_, static_cast<size_t>(start));
    return at == std::string::npos ? -1 : static_cast<int>(at);
  }
  int indexOf(char value, int start = 0) const {
    auto at = value_.find(value, static_cast<size_t>(start));
    return at == std::string::npos ? -1 : static_cast<int>(at);
  }
  String substring(int begin, int end) const { return String(value_.substr(begin, end - begin)); }
  void trim() {
    auto begin = value_.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) { value_.clear(); return; }
    value_ = value_.substr(begin, value_.find_last_not_of(" \t\r\n") - begin + 1);
  }
  void toLowerCase() { for (char& c : value_) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c))); }
  bool operator==(const char* value) const { return value_ == value; }
  bool operator!=(const char* value) const { return value_ != value; }
  const char* c_str() const { return value_.c_str(); }
};
static const unsigned SERIAL_8N1 = 0;
class HardwareSerial {
  std::string input_;
  size_t cursor_ = 0;
public:
  void setRxBufferSize(size_t) {}
  void begin(uint32_t, unsigned, int, int) {}
  int available() const { return static_cast<int>(input_.size() - cursor_); }
  int read() { return available() ? static_cast<unsigned char>(input_[cursor_++]) : -1; }
  void append(const std::string& text) { input_ += text; }
};
extern HardwareSerial Serial1;
