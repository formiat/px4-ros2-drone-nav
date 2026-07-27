#pragma once

#include <cstdint>

namespace drone_city_nav::mppi {

enum class RiskTier : std::uint8_t {
  kPreferred = 0,
  kPlanning = 1,
  kCritical = 2,
  kCollision = 3,
};

struct State {
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float vx{0.0F};
  float vy{0.0F};
  float vz{0.0F};
  float yaw{0.0F};
  float yaw_rate{0.0F};
};

struct Control {
  float ax{0.0F};
  float ay{0.0F};
  float az{0.0F};
  float yaw_accel{0.0F};
};

struct CostBreakdown {
  float head_progress{0.0F};
  float progress{0.0F};
  float speed_tracking{0.0F};
  float guide_deviation{0.0F};
  float acceleration{0.0F};
  float jerk{0.0F};
  float yaw_change{0.0F};
  float control_effort{0.0F};
  float terminal{0.0F};
};

struct RolloutMetrics {
  State terminal_state{};
  CostBreakdown costs{};
  float soft_cost{0.0F};
  float critical_exposure_m{0.0F};
  float planning_exposure_m{0.0F};
  float minimum_clearance_m{0.0F};
  RiskTier worst_tier{RiskTier::kPreferred};
  bool collision{false};
};

struct EsdfGrid {
  int width{0};
  int height{0};
  float resolution_m{0.0F};
  float origin_x_m{0.0F};
  float origin_y_m{0.0F};
};

} // namespace drone_city_nav::mppi
