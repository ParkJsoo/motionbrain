#include "motionbrain_control/source_observation_freshness.hpp"
#include <cassert>
#include <limits>
#include <iomanip>
#include <sstream>
#include <string>
using motionbrain_control::SourceObservation;
using motionbrain_control::SourceObservationFreshness;
SourceObservation observation(const std::string & boot = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", uint64_t captured = 10) {
  SourceObservation s;
  s.boot_id = boot; s.captured_us = captured;
  s.frame_id = boot + ":" + std::to_string(captured);
  s.detection_frame_id = s.frame_id;
  s.clock_domain = "camera_boot_us"; s.timestamp_semantics = "first_dma_buffer";
  s.time_known = s.source_fresh = s.control_eligible = true; s.age_ms = 100;
  return s;
}
std::string numbered_boot(unsigned number) {
  std::ostringstream out; out << std::hex << std::setfill('0') << std::setw(32) << number;
  return out.str();
}
int main() {
  SourceObservationFreshness f;
  auto s = observation();
  assert(f.observe(s, 100, 0.2));
  assert(f.fresh(100.5, 1));
  // Republishing with a new ROS stamp / lower age cannot reset source age.
  s.age_ms = 0;
  assert(f.observe(s, 101, 0));
  assert(!f.fresh(101, 1));
  assert(f.age_seconds(101) >= 1.29);
  s = observation("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", 11);
  assert(f.observe(s, 102, 0)); assert(f.fresh(102, 1));
  s.held = true; assert(!f.observe(s, 102.1, 0));
  s.held = false; assert(!f.observe(s, 102.2, 0));
  s = observation("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", 12); assert(f.observe(s, 103, 0));
  s = observation("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", 11); assert(!f.observe(s, 104, 0));
  s = observation("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb", 1); assert(f.observe(s, 105, 0));
  s = observation("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", 100); assert(!f.observe(s, 106, 0));
  s = observation("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb", 2); assert(f.observe(s, 107, 0));
  f.invalidate(); assert(!f.observe(s, 108, 0));
  s = observation("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb", 3); assert(f.observe(s, 109, 0));
  assert(!f.fresh(108, 10)); // steady clock rollback fails closed
  assert(!f.observe(s, 108, 0));
  s = observation("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb", 4);
  assert(!f.observe(s, 110, -0.001)); // ROS future stamp / backward jump
  assert(!f.observe(s, 110, std::numeric_limits<double>::infinity()));
  assert(!f.observe(s, std::numeric_limits<double>::quiet_NaN(), 0));
  s.age_ms = std::numeric_limits<double>::quiet_NaN(); assert(!f.observe(s, 110, 0));
  s.age_ms = -1; assert(!f.observe(s, 110, 0));
  s.age_ms = 0; s.detection_frame_id = "other"; assert(!f.observe(s, 110, 0));
  s = observation("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb", 4); s.time_known = false; assert(!f.observe(s, 110, 0));
  s = observation("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb", 4); s.control_eligible = false; assert(!f.observe(s, 110, 0));
  s = observation("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb", 4); s.source_fresh = false; assert(!f.observe(s, 110, 0));
  s = observation("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb", 4); s.timestamp_semantics = "response"; assert(!f.observe(s, 110, 0));
  s = observation("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb", 4); s.frame_id = "invented"; assert(!f.observe(s, 110, 0));
  s = observation("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb", 4); assert(f.observe(s, 110, 20));
  assert(!f.fresh(110, 1)); // large transport delay / forward ROS jump
  SourceObservationFreshness malformed;
  assert(!malformed.observe(observation("short"), 1, 0));
  assert(!malformed.observe(observation("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"), 1, 0));
  assert(!malformed.observe(observation("zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz"), 1, 0));
  assert(!malformed.observe(observation("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", 0), 1, 0));
  SourceObservationFreshness capacity;
  for (unsigned i = 0; i < 32; ++i) assert(capacity.observe(observation(numbered_boot(i)), i, 0));
  assert(!capacity.observe(observation(numbered_boot(32)), 33, 0));
  assert(!capacity.observe(observation(numbered_boot(0), 100), 34, 0));
  return 0;
}
