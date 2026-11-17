#pragma once
#include <string>
#include <vector>
enum class EventSeverity { INFO, WARN, ERROR };
class EventLog {
public:
  struct Entry { std::string code, detail; };
  std::vector<Entry> entries;
  void push(const char*,const char* code,EventSeverity,const char* detail){entries.push_back({code,detail ? detail : ""});}
};
