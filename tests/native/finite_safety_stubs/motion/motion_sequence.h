#pragma once
class MotionSequence {
public:
  unsigned stops = 0;
  void stop() { ++stops; }
};
