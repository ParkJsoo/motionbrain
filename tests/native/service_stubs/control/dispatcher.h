#pragma once
class Dispatcher {
public:
  bool ready = true;
  bool isReady() const { return ready; }
};
