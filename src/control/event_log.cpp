#include "control/event_log.h"

EventLog::EventLog()
  : writeIndex_(0)
  , count_(0)
  , nextId_(1) {
}

void EventLog::push(const char* category, const char* code, EventSeverity severity, const char* detail) {
  MotionEvent& event = events_[writeIndex_];
  event.id = nextId_++;
  event.tsMs = millis();
  event.severity = severity;
  strlcpy(event.category, category != nullptr ? category : "unknown", sizeof(event.category));
  strlcpy(event.code, code != nullptr ? code : "UNKNOWN", sizeof(event.code));
  strlcpy(event.detail, detail != nullptr ? detail : "", sizeof(event.detail));

  writeIndex_ = (writeIndex_ + 1) % MAX_EVENTS;
  if (count_ < MAX_EVENTS) {
    count_++;
  }
}

uint8_t EventLog::size() const {
  return count_;
}

bool EventLog::getOldestFirst(uint8_t index, MotionEvent& outEvent) const {
  if (index >= count_) {
    return false;
  }

  uint8_t oldestIndex = (writeIndex_ + MAX_EVENTS - count_) % MAX_EVENTS;
  uint8_t actualIndex = (oldestIndex + index) % MAX_EVENTS;
  outEvent = events_[actualIndex];
  return true;
}

const char* EventLog::severityToString(EventSeverity severity) {
  switch (severity) {
    case EventSeverity::INFO:  return "INFO";
    case EventSeverity::WARN:  return "WARN";
    case EventSeverity::ERROR: return "ERROR";
    default:                   return "UNKNOWN";
  }
}
