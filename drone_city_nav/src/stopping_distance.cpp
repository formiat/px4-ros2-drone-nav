#include "drone_city_nav/stopping_distance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace drone_city_nav {

bool jerkLimitedAxisStoppingConfigIsValid(
    const JerkLimitedAxisStoppingConfig& config) noexcept {
  return std::isfinite(config.guaranteed_deceleration_mps2) &&
         config.guaranteed_deceleration_mps2 > 0.0 &&
         std::isfinite(config.maximum_acceleration_mps2) &&
         config.maximum_acceleration_mps2 > 0.0 &&
         std::isfinite(config.maximum_jerk_mps3) && config.maximum_jerk_mps3 > 0.0 &&
         std::isfinite(config.reaction_latency_s) && config.reaction_latency_s >= 0.0;
}

double
jerkLimitedAxisStoppingDistanceM(const double speed_mps,
                                 const double forward_acceleration_mps2,
                                 const JerkLimitedAxisStoppingConfig& config) noexcept {
  if (!std::isfinite(speed_mps) || speed_mps < 0.0 ||
      !std::isfinite(forward_acceleration_mps2) ||
      !jerkLimitedAxisStoppingConfigIsValid(config)) {
    return std::numeric_limits<double>::infinity();
  }
  if (!(speed_mps > 0.0)) {
    return 0.0;
  }

  const double forward_acceleration = std::clamp(
      std::max(0.0, forward_acceleration_mps2), 0.0, config.maximum_acceleration_mps2);
  const double reaction_distance_m =
      speed_mps * config.reaction_latency_s + 0.5 * forward_acceleration *
                                                  config.reaction_latency_s *
                                                  config.reaction_latency_s;
  const double speed_after_reaction_mps =
      speed_mps + forward_acceleration * config.reaction_latency_s;
  const double ramp_time_s =
      (forward_acceleration + config.guaranteed_deceleration_mps2) /
      config.maximum_jerk_mps3;
  const double stop_during_ramp_s =
      (forward_acceleration +
       std::sqrt(forward_acceleration * forward_acceleration +
                 2.0 * config.maximum_jerk_mps3 * speed_after_reaction_mps)) /
      config.maximum_jerk_mps3;
  const double applied_ramp_time_s = std::min(ramp_time_s, stop_during_ramp_s);
  const double ramp_time_squared_s2 = applied_ramp_time_s * applied_ramp_time_s;
  const double ramp_distance_m =
      speed_after_reaction_mps * applied_ramp_time_s +
      0.5 * forward_acceleration * ramp_time_squared_s2 -
      config.maximum_jerk_mps3 * ramp_time_squared_s2 * applied_ramp_time_s / 6.0;
  if (stop_during_ramp_s <= ramp_time_s) {
    return std::max(0.0, reaction_distance_m + ramp_distance_m);
  }
  const double speed_after_ramp_mps =
      speed_after_reaction_mps + forward_acceleration * ramp_time_s -
      0.5 * config.maximum_jerk_mps3 * ramp_time_s * ramp_time_s;
  const double constant_deceleration_distance_m =
      speed_after_ramp_mps * speed_after_ramp_mps /
      (2.0 * config.guaranteed_deceleration_mps2);
  return std::max(0.0, reaction_distance_m + ramp_distance_m +
                           constant_deceleration_distance_m);
}

} // namespace drone_city_nav
