#pragma once
#include <atomic>
#include <cstdint>

struct SharedState {
  // buttons
  std::atomic<uint8_t> last_button{0};

  // line sensors
  std::atomic<double> left_sensor{0.0};
  std::atomic<double> right_sensor{0.0};

  // lidar sensors
  std::atomic<double> lidarLeft{0.0};
  std::atomic<double> lidarRight{0.0};
  std::atomic<double> lidarFront{0.0};
};