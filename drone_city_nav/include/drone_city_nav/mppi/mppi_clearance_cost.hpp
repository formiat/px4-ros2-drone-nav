#pragma once

#include <cmath>

namespace drone_city_nav::mppi {

#if defined(__CUDACC__)
#define DRONE_CITY_NAV_MPPI_CLEARANCE_HOST_DEVICE __host__ __device__
#else
#define DRONE_CITY_NAV_MPPI_CLEARANCE_HOST_DEVICE
#endif

// The clearance a body moving at `speed_mps` needs before occupied evidence:
// the margin it keeps, the distance it covers while it reacts, and its braking
// distance.
[[nodiscard]] DRONE_CITY_NAV_MPPI_CLEARANCE_HOST_DEVICE inline float
requiredStoppingClearanceM(const float speed_mps, const float margin_m,
                           const float response_time_s,
                           const float deceleration_mps2) noexcept {
  const float speed = speed_mps > 0.0F ? speed_mps : 0.0F;
  if (!(deceleration_mps2 > 0.0F)) {
    return margin_m;
  }
  return margin_m + speed * response_time_s +
         speed * speed / (2.0F * deceleration_mps2);
}

// The inverse: the fastest a body may travel and still have `clearance_m`
// satisfy requiredStoppingClearanceM. This and the deficit below are one law
// read in two directions — the speed policy reads it as a speed cap, the
// rollout cost reads it as a shortfall — so the reference speed and the
// optimiser cannot disagree about what "too close, too fast" means.
[[nodiscard]] DRONE_CITY_NAV_MPPI_CLEARANCE_HOST_DEVICE inline float
stoppingAdmissibleSpeedMps(const float clearance_m, const float margin_m,
                           const float response_time_s,
                           const float deceleration_mps2) noexcept {
  if (!(deceleration_mps2 > 0.0F)) {
    return 0.0F;
  }
  const float usable_m = clearance_m - margin_m;
  if (!(usable_m > 0.0F)) {
    return 0.0F;
  }
  const float latency_speed = deceleration_mps2 * response_time_s;
#if defined(__CUDA_ARCH__)
  const float root =
      sqrtf(latency_speed * latency_speed + 2.0F * deceleration_mps2 * usable_m);
#else
  const float root =
      std::sqrt(latency_speed * latency_speed + 2.0F * deceleration_mps2 * usable_m);
#endif
  const float speed = root - latency_speed;
  return speed > 0.0F ? speed : 0.0F;
}

[[nodiscard]] DRONE_CITY_NAV_MPPI_CLEARANCE_HOST_DEVICE inline bool
mppiClearanceFinite(const float value) noexcept {
#if defined(__CUDA_ARCH__)
  return isfinite(value);
#else
  return std::isfinite(value);
#endif
}

// The squared shortfall between the clearance a rollout keeps and the
// clearance its own speed needs to stop within. Charging it at the rollout's
// speed rather than only at the rate it closes on the obstacle is what makes
// it the same law the speed policy applies: running fast along a wall is
// priced even when the wall never gets nearer, which is the case a
// closing-rate law prices at nothing.
[[nodiscard]] DRONE_CITY_NAV_MPPI_CLEARANCE_HOST_DEVICE inline float
stoppingClearanceDeficitM2(const float clearance_m, const float speed_mps,
                           const float margin_m, const float response_time_s,
                           const float deceleration_mps2) noexcept {
  if (!mppiClearanceFinite(clearance_m) || !mppiClearanceFinite(speed_mps) ||
      !(margin_m >= 0.0F) || !(response_time_s >= 0.0F) ||
      !(deceleration_mps2 > 0.0F)) {
    return 0.0F;
  }
  const float required_clearance_m = requiredStoppingClearanceM(
      speed_mps, margin_m, response_time_s, deceleration_mps2);
  const float shortfall_m = required_clearance_m - clearance_m;
  return shortfall_m > 0.0F ? shortfall_m * shortfall_m : 0.0F;
}

#undef DRONE_CITY_NAV_MPPI_CLEARANCE_HOST_DEVICE

} // namespace drone_city_nav::mppi
