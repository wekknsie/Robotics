#pragma once
#include <atomic>
#include <cstdint>
#include <limits>

enum class CorridorState : uint32_t {
    IDLE = 0,
    USING_LIDAR = 1,
    USING_IMU = 2,
};

struct SharedState {
  // buttons
  std::atomic<uint8_t> last_button{0};

  // line sensors
  std::atomic<double> left_sensor{0.0};
  std::atomic<double> right_sensor{0.0};

  // lidar sensors
  std::atomic<double> lidarLeft{0.0};
  std::atomic<double> lidarRight{0.0};
  std::atomic<double> lidarFront{std::numeric_limits<double>::infinity()};

  // IMU sensor
  std::atomic<double> imuAngle{0.0};
  std::atomic<bool> imuReady{false};

  // data from motor node
  std::atomic<CorridorState> corridorState{CorridorState::IDLE};
};