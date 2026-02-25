#pragma once
#include <atomic>
#include <cstdint>

struct SharedState {
  std::atomic<uint8_t> last_button{0};
  std::atomic<bool> have_data{false};
};