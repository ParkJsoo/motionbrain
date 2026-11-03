#pragma once
enum class EventSeverity { INFO, WARN, ERROR };
class EventLog {
public:
  unsigned events = 0;
  void push(const char*, const char*, EventSeverity, const char*) { ++events; }
};
