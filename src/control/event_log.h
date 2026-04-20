#ifndef CONTROL_EVENT_LOG_H
#define CONTROL_EVENT_LOG_H

#include <Arduino.h>
#include <stdint.h>

enum class EventSeverity : uint8_t {
  INFO = 0,
  WARN,
  ERROR
};

struct MotionEvent {
  uint32_t id;
  uint32_t tsMs;
  EventSeverity severity;
  char category[16];
  char code[24];
  char detail[96];

  MotionEvent()
    : id(0)
    , tsMs(0)
    , severity(EventSeverity::INFO)
    , category{0}
    , code{0}
    , detail{0} {
  }
};

class EventLog {
public:
  static const uint8_t MAX_EVENTS = 16;

  EventLog();

  void push(const char* category, const char* code, EventSeverity severity, const char* detail = nullptr);
  uint8_t size() const;
  bool getOldestFirst(uint8_t index, MotionEvent& outEvent) const;

  static const char* severityToString(EventSeverity severity);

private:
  MotionEvent events_[MAX_EVENTS];
  uint8_t writeIndex_;
  uint8_t count_;
  uint32_t nextId_;
};

#endif // CONTROL_EVENT_LOG_H
