#pragma once

#include <cmath>

namespace drone_city_nav::mppi {

#if defined(__CUDACC__)
#define DRONE_CITY_NAV_MPPI_CLEARANCE_HOST_DEVICE __host__ __device__
#else
#define DRONE_CITY_NAV_MPPI_CLEARANCE_HOST_DEVICE
#endif

[[nodiscard]] DRONE_CITY_NAV_MPPI_CLEARANCE_HOST_DEVICE inline bool
mppiClearanceFinite(const float value) noexcept {
#if defined(__CUDA_ARCH__)
  return isfinite(value);
#else
  return std::isfinite(value);
#endif
}

// Two laws price a body moving near known occupied evidence, and each answers
// a different physical question.
//
// The tracking-error tube law answers for evidence *beside* the motion: the
// error the controller can accumulate within its response time, `speed *
// response_time`, must fit inside the clearance the body keeps. A wall the
// vehicle flies along never gets nearer, so no braking distance is owed to it;
// the tube is the same law that certifies a route's speed profile, so the
// route, the reference speed and the rollout cost read one expression.
//
// The stopping law answers for evidence *ahead* on the motion: the distance
// the body covers while it reacts and brakes must fit inside the free path to
// the point where its envelope enters occupied evidence. It is charged along
// the rollout's own future, so a control that will turn the vehicle into a
// wall is priced before the wall is beside it.

// The fastest the tube law admits at `clearance_m`.
[[nodiscard]] DRONE_CITY_NAV_MPPI_CLEARANCE_HOST_DEVICE inline float
tubeAdmissibleSpeedMps(const float clearance_m, const float response_time_s) noexcept {
  if (!(response_time_s > 0.0F) || !(clearance_m > 0.0F)) {
    return 0.0F;
  }
  return clearance_m / response_time_s;
}

// The squared shortfall between the clearance a body keeps and the tracking
// error its speed can accumulate: zero at or below the admissible speed.
[[nodiscard]] DRONE_CITY_NAV_MPPI_CLEARANCE_HOST_DEVICE inline float
tubeClearanceDeficitM2(const float clearance_m, const float speed_mps,
                       const float response_time_s) noexcept {
  if (!mppiClearanceFinite(clearance_m) || !mppiClearanceFinite(speed_mps) ||
      !(response_time_s >= 0.0F)) {
    return 0.0F;
  }
  const float speed = speed_mps > 0.0F ? speed_mps : 0.0F;
  const float clearance = clearance_m > 0.0F ? clearance_m : 0.0F;
  const float shortfall_m = speed * response_time_s - clearance;
  return shortfall_m > 0.0F ? shortfall_m * shortfall_m : 0.0F;
}

// The path a body moving at `speed_mps` needs before evidence ahead: the
// distance it covers while it reacts, and its braking distance.
[[nodiscard]] DRONE_CITY_NAV_MPPI_CLEARANCE_HOST_DEVICE inline float
requiredStoppingDistanceM(const float speed_mps, const float response_time_s,
                          const float deceleration_mps2) noexcept {
  const float speed = speed_mps > 0.0F ? speed_mps : 0.0F;
  if (!(deceleration_mps2 > 0.0F)) {
    return speed * response_time_s;
  }
  return speed * response_time_s + speed * speed / (2.0F * deceleration_mps2);
}

// The squared shortfall between the free path ahead and the path the speed
// needs to stop within: zero when the body can stop before the contact.
[[nodiscard]] DRONE_CITY_NAV_MPPI_CLEARANCE_HOST_DEVICE inline float
stoppingDistanceDeficitM2(const float available_distance_m, const float speed_mps,
                          const float response_time_s,
                          const float deceleration_mps2) noexcept {
  if (!mppiClearanceFinite(available_distance_m) || !mppiClearanceFinite(speed_mps) ||
      !(response_time_s >= 0.0F) || !(deceleration_mps2 > 0.0F)) {
    return 0.0F;
  }
  const float available_m = available_distance_m > 0.0F ? available_distance_m : 0.0F;
  const float shortfall_m =
      requiredStoppingDistanceM(speed_mps, response_time_s, deceleration_mps2) -
      available_m;
  return shortfall_m > 0.0F ? shortfall_m * shortfall_m : 0.0F;
}

// Depth into the clearance preference band, normalised to [0, 1]: one at the
// evidence, zero at `preferred_distance_m` and beyond. It prices position, not
// motion, so it is integrated over time and a body hovering beside a wall pays
// as much as one flying past it.
[[nodiscard]] DRONE_CITY_NAV_MPPI_CLEARANCE_HOST_DEVICE inline float
clearancePreferenceDepth(const float clearance_m,
                         const float preferred_distance_m) noexcept {
  if (!mppiClearanceFinite(clearance_m) || !(preferred_distance_m > 0.0F) ||
      clearance_m >= preferred_distance_m) {
    return 0.0F;
  }
  const float clearance = clearance_m > 0.0F ? clearance_m : 0.0F;
  return (preferred_distance_m - clearance) / preferred_distance_m;
}

#undef DRONE_CITY_NAV_MPPI_CLEARANCE_HOST_DEVICE

} // namespace drone_city_nav::mppi
