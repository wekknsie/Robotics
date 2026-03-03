#pragma once
#include <atomic>
#include <cstdint>

struct SharedState {
  std::atomic<uint8_t> last_button{0};
};