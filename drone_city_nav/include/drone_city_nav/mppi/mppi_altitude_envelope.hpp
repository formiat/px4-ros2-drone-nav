#pragma once

#include "drone_city_nav/mppi/mppi_config.hpp"

#include <cmath>

namespace drone_city_nav::mppi {

#if defined(__CUDACC__)
#define DRONE_CITY_NAV_MPPI_ALTITUDE_HOST_DEVICE __host__ __device__
#else
#define DRONE_CITY_NAV_MPPI_ALTITUDE_HOST_DEVICE
#endif

[[nodiscard]] DRONE_CITY_NAV_MPPI_ALTITUDE_HOST_DEVICE inline bool
mppiAltitudeFinite(const float value) noexcept {
#if defined(__CUDA_ARCH__)
  return isfinite(value);
#else
  return std::isfinite(value);
#endif
}

[[nodiscard]] DRONE_CITY_NAV_MPPI_ALTITUDE_HOST_DEVICE inline float
verticalStoppingDistanceM(const float vertical_speed_mps,
                          const float vertical_acceleration_mps2,
                          const DynamicsConfig& dynamics) noexcept {
  const float speed_mps =
      vertical_speed_mps < 0.0F ? -vertical_speed_mps : vertical_speed_mps;
  if (!(speed_mps > 0.0F)) {
    return 0.0F;
  }
  const float maximum_acceleration_mps2 = dynamics.maximum_vertical_acceleration_mps2;
  const float maximum_jerk_mps3 = dynamics.maximum_control_jerk_mps3;
  if (!(maximum_acceleration_mps2 > 0.0F) || !(maximum_jerk_mps3 > 0.0F)) {
    return 3.402823466e+38F;
  }

  const float direction = vertical_speed_mps < 0.0F ? -1.0F : 1.0F;
  const float aligned_acceleration_mps2 =
      direction * vertical_acceleration_mps2 < -maximum_acceleration_mps2
          ? -maximum_acceleration_mps2
          : (direction * vertical_acceleration_mps2 > maximum_acceleration_mps2
                 ? maximum_acceleration_mps2
                 : direction * vertical_acceleration_mps2);
  const float ramp_time_s =
      (aligned_acceleration_mps2 + maximum_acceleration_mps2) / maximum_jerk_mps3;
  const float ramp_time_squared_s2 = ramp_time_s * ramp_time_s;
  const float speed_after_ramp_mps = speed_mps +
                                     aligned_acceleration_mps2 * ramp_time_s -
                                     0.5F * maximum_jerk_mps3 * ramp_time_squared_s2;
  if (!(speed_after_ramp_mps > 0.0F)) {
    const float nonnegative_acceleration_mps2 =
        aligned_acceleration_mps2 > 0.0F ? aligned_acceleration_mps2 : 0.0F;
    return speed_mps * ramp_time_s +
           0.5F * nonnegative_acceleration_mps2 * ramp_time_squared_s2;
  }

  const float ramp_distance_m =
      speed_mps * ramp_time_s +
      0.5F * aligned_acceleration_mps2 * ramp_time_squared_s2 -
      maximum_jerk_mps3 * ramp_time_squared_s2 * ramp_time_s / 6.0F;
  const float constant_acceleration_distance_m =
      speed_after_ramp_mps * speed_after_ramp_mps / (2.0F * maximum_acceleration_mps2);
  return ramp_distance_m + constant_acceleration_distance_m;
}

[[nodiscard]] DRONE_CITY_NAV_MPPI_ALTITUDE_HOST_DEVICE inline bool
altitudeEnvelopeDynamicallyRecoverable(
    const State& state, const Control& control, const DynamicsConfig& dynamics,
    const AltitudeEnvelopeConfig& envelope) noexcept {
  if (!mppiAltitudeFinite(state.z) || !mppiAltitudeFinite(state.vz) ||
      !mppiAltitudeFinite(control.az) || state.z < envelope.minimum_z_m ||
      state.z >= envelope.maximum_z_m) {
    return false;
  }
  if (state.vz < 0.0F) {
    return verticalStoppingDistanceM(state.vz, control.az, dynamics) <=
           state.z - envelope.minimum_z_m;
  }
  if (state.vz > 0.0F) {
    return verticalStoppingDistanceM(state.vz, control.az, dynamics) <
           envelope.maximum_z_m - state.z;
  }
  return true;
}

#undef DRONE_CITY_NAV_MPPI_ALTITUDE_HOST_DEVICE

} // namespace drone_city_nav::mppi
