#pragma once
class DebugLog {
public:
  static void info(const char*, ...) {}
  static void debug(const char*, ...) {}
  static void warn(const char*, ...) {}
  static void error(const char*, ...) {}
  static void motor(const char*, ...) {}
  static void safety(const char*, ...) {}
};
