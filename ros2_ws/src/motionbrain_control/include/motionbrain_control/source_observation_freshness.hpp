#ifndef MOTIONBRAIN_CONTROL_SOURCE_OBSERVATION_FRESHNESS_HPP_
#define MOTIONBRAIN_CONTROL_SOURCE_OBSERVATION_FRESHNESS_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace motionbrain_control {
struct SourceObservation {
  std::string frame_id, boot_id, detection_frame_id, clock_domain, timestamp_semantics;
  uint64_t captured_us{0};
  double age_ms{0.0};
  bool time_known{false}, source_fresh{false}, held{false}, control_eligible{false};
};

// ROS publication and receipt must use a shared ROS clock. Its transit duration
// is checked once; all subsequent age growth uses a separate steady clock.
// Repeated IDs can increase age, but never reduce it or revive invalid evidence.
class SourceObservationFreshness {
public:
  bool observe(const SourceObservation & source, double steady_sec, double transit_sec) {
    if (!std::isfinite(steady_sec) || !std::isfinite(transit_sec) || transit_sec < 0 ||
        !std::isfinite(source.age_ms) || source.age_ms < 0 ||
        !source.time_known || !source.source_fresh || source.held || !source.control_eligible ||
        !valid_boot(source.boot_id) || source.captured_us == 0 || source.frame_id != source.boot_id + ":" + std::to_string(source.captured_us) ||
        source.frame_id != source.detection_frame_id || source.clock_domain != "camera_boot_us" ||
        source.timestamp_semantics != "first_dma_buffer") {
      invalidate(); return false;
    }
    const double incoming_age = source.age_ms / 1000.0 + transit_sec;
    if (!std::isfinite(incoming_age)) { invalidate(); return false; }
    if (source.boot_id == boot_id_) {
      if (source.captured_us < captured_us_) { invalidate(); return false; }
      if (source.captured_us == captured_us_) {
        const double accumulated_age = age_seconds(steady_sec);
        if (!valid_ || !std::isfinite(accumulated_age)) { invalidate(); return false; }
        age_at_receipt_ = std::max(accumulated_age, incoming_age);
        received_at_ = steady_sec;
        return true;
      }
    } else {
      for (size_t i = 0; i < boot_count_; ++i) {
        if (boots_[i] == source.boot_id) { invalidate(); return false; }
      }
      if (boot_count_ == boots_.size()) { invalidate(); return false; }
      boots_[boot_count_++] = source.boot_id;
    }
    boot_id_ = source.boot_id;
    captured_us_ = source.captured_us;
    received_at_ = steady_sec;
    age_at_receipt_ = incoming_age;
    valid_ = true;
    return true;
  }
  double age_seconds(double steady_sec) const {
    if (!valid_ || !std::isfinite(steady_sec) || steady_sec < received_at_) {
      return std::numeric_limits<double>::infinity();
    }
    return age_at_receipt_ + steady_sec - received_at_;
  }
  bool fresh(double steady_sec, double maximum_age_sec) const {
    return std::isfinite(maximum_age_sec) && maximum_age_sec >= 0 &&
           age_seconds(steady_sec) <= maximum_age_sec;
  }
  // Lifecycle cleanup discards validity, never the replay/retired-boot history.
  void invalidate() { valid_ = false; }
private:
  static bool valid_boot(const std::string & boot) {
    return boot.size() == 32 && std::all_of(boot.begin(), boot.end(), [](char ch) {
      return (ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f');
    });
  }
  std::array<std::string, 32> boots_{};
  size_t boot_count_{0};
  std::string boot_id_;
  uint64_t captured_us_{0};
  double age_at_receipt_{0}, received_at_{0};
  bool valid_{false};
};
}  // namespace motionbrain_control
#endif
