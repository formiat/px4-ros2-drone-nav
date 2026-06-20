#pragma once

#include "drone_city_nav/types.hpp"

namespace drone_city_nav {

struct VelocityLimiterConfig {
  double max_vector_accel_mps2{3.0};
  double max_heading_rate_radps{1.5};
};

struct VelocityLimiterOutput {
  Point2 velocity_mps{};
  double raw_delta_mps{0.0};
  double applied_delta_mps{0.0};
  bool vector_delta_limited{false};
  bool heading_rate_limited{false};
};

class OffboardVelocityLimiter {
public:
  explicit OffboardVelocityLimiter(
      const VelocityLimiterConfig& config = VelocityLimiterConfig{});

  void setConfig(const VelocityLimiterConfig& config);
  [[nodiscard]] const VelocityLimiterConfig& config() const noexcept;

  void reset() noexcept;
  [[nodiscard]] VelocityLimiterOutput update(Point2 desired_velocity_mps, double dt_s);

private:
  VelocityLimiterConfig config_{};
  Point2 previous_velocity_mps_{};
  bool previous_velocity_valid_{false};
};

} // namespace drone_city_nav
