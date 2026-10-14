#pragma once

#include <array>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>

namespace motionbrain_hardware_interface
{

// This identifies a producer session, not an ESP32 boot or proof of execution.
// No timestamp, PID, or deterministic fallback is used if entropy is unavailable.
class ProposalCommandIds
{
public:
  using SessionWords = std::array<std::uint32_t, 4>;

  bool initialize()
  {
    return initialize_with([]() {
      std::random_device source;
      if (source.entropy() <= 0.0) {
        throw std::runtime_error("random source does not report entropy");
      }
      std::uniform_int_distribution<std::uint32_t> distribution;
      SessionWords words{};
      for (auto & word : words) {
        word = distribution(source);
      }
      return words;
    });
  }

  // Inject the entropy boundary for native tests without ROS or hardware.
  template<typename WordSource>
  bool initialize_with(WordSource source)
  {
    session_.clear();
    sequence_ = 0;
    try {
      const SessionWords words = source();
      if (words == SessionWords{}) {
        return false;
      }
      std::ostringstream encoded;
      encoded << std::hex << std::setfill('0');
      for (const auto word : words) {
        encoded << std::setw(8) << word;
      }
      session_ = encoded.str();
      return session_.size() == 32;
    } catch (...) {
      session_.clear();
      return false;
    }
  }

  bool next(std::string & command_id)
  {
    command_id.clear();
    if (session_.empty() || sequence_ == std::numeric_limits<std::uint64_t>::max()) {
      return false;
    }
    command_id = "ros2-control-" + session_ + "-" + std::to_string(++sequence_);
    return true;
  }

private:
  std::string session_;
  std::uint64_t sequence_{0};
};

}  // namespace motionbrain_hardware_interface
