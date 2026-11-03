#pragma once
enum class EventSeverity { INFO, WARN };
class EventLog { public: void push(const char*, const char*, EventSeverity, const char*) {} };
