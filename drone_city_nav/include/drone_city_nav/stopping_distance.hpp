#pragma once

namespace drone_city_nav {

struct JerkLimitedAxisStoppingConfig {
  double guaranteed_deceleration_mps2{0.0};
  double maximum_acceleration_mps2{0.0};
  double maximum_jerk_mps3{0.0};
  double reaction_latency_s{0.0};
};

[[nodiscard]] bool jerkLimitedAxisStoppingConfigIsValid(
    const JerkLimitedAxisStoppingConfig& config) noexcept;

[[nodiscard]] double
jerkLimitedAxisStoppingDistanceM(double speed_mps, double forward_acceleration_mps2,
                                 const JerkLimitedAxisStoppingConfig& config) noexcept;

// The distance over which the speed falls to `terminal_speed_mps`, under the
// same reaction latency, jerk ramp and guaranteed deceleration.
[[nodiscard]] double
jerkLimitedAxisSlowdownDistanceM(double speed_mps, double terminal_speed_mps,
                                 double forward_acceleration_mps2,
                                 const JerkLimitedAxisStoppingConfig& config) noexcept;

} // namespace drone_city_nav
