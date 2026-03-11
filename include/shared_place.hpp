#pragma once
#include <atomic>
#include <cstdint>

struct SharedState {
  std::atomic<uint8_t> last_button{0};
  std::atomic<double> left_sensor{0.0};
  std::atomic<double> right_sensor{0.0};
};