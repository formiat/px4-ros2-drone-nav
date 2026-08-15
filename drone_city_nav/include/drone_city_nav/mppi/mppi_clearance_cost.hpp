#pragma once

#include <cmath>

namespace drone_city_nav::mppi {

#if defined(__CUDACC__)
#define DRONE_CITY_NAV_MPPI_CLEARANCE_HOST_DEVICE __host__ __device__
#else
#define DRONE_CITY_NAV_MPPI_CLEARANCE_HOST_DEVICE
#endif

[[nodiscard]] DRONE_CITY_NAV_MPPI_CLEARANCE_HOST_DEVICE inline float
criticalClearanceProximitySeverity(const float clearance_m,
                                   const float critical_distance_m) noexcept {
  if (!(critical_distance_m > 0.0F) || !(clearance_m < critical_distance_m)) {
    return 0.0F;
  }
  const float bounded_clearance_m = clearance_m > 0.0F ? clearance_m : 0.0F;
  const float normalized_deficit =
      (critical_distance_m - bounded_clearance_m) / critical_distance_m;
  return normalized_deficit * normalized_deficit;
}

[[nodiscard]] DRONE_CITY_NAV_MPPI_CLEARANCE_HOST_DEVICE inline bool
mppiClearanceFinite(const float value) noexcept {
#if defined(__CUDA_ARCH__)
  return isfinite(value);
#else
  return std::isfinite(value);
#endif
}

[[nodiscard]] DRONE_CITY_NAV_MPPI_CLEARANCE_HOST_DEVICE inline float
obstacleApproachSeverityM2(const float previous_clearance_m, const float clearance_m,
                           const float segment_speed_mps, const float dt_s,
                           const float minimum_clearance_m, const float response_time_s,
                           const float deceleration_mps2) noexcept {
  if (!mppiClearanceFinite(previous_clearance_m) || !mppiClearanceFinite(clearance_m) ||
      !mppiClearanceFinite(segment_speed_mps) || !(dt_s > 0.0F) ||
      !(minimum_clearance_m >= 0.0F) || !(response_time_s >= 0.0F) ||
      !(deceleration_mps2 > 0.0F)) {
    return 0.0F;
  }
  const float clearance_decrease_m = previous_clearance_m - clearance_m;
  if (!(clearance_decrease_m > 0.0F)) {
    return 0.0F;
  }
  const float estimated_approach_speed_mps = clearance_decrease_m / dt_s;
  const float bounded_segment_speed_mps =
      segment_speed_mps > 0.0F ? segment_speed_mps : 0.0F;
  const float approach_speed_mps =
      estimated_approach_speed_mps < bounded_segment_speed_mps
          ? estimated_approach_speed_mps
          : bounded_segment_speed_mps;
  const float required_clearance_m =
      minimum_clearance_m + approach_speed_mps * response_time_s +
      approach_speed_mps * approach_speed_mps / (2.0F * deceleration_mps2);
  const float shortfall_m = required_clearance_m - clearance_m;
  return shortfall_m > 0.0F ? shortfall_m * shortfall_m : 0.0F;
}

#undef DRONE_CITY_NAV_MPPI_CLEARANCE_HOST_DEVICE

} // namespace drone_city_nav::mppi
