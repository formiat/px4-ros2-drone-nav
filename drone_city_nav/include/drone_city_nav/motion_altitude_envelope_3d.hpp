#pragma once

#include "drone_city_nav/control_contracts_3d.hpp"

#include <cmath>

namespace drone_city_nav {

#if defined(__CUDACC__)
#define DRONE_CITY_NAV_MOTION_ALTITUDE_HOST_DEVICE __host__ __device__
#else
#define DRONE_CITY_NAV_MOTION_ALTITUDE_HOST_DEVICE
#endif

[[nodiscard]] DRONE_CITY_NAV_MOTION_ALTITUDE_HOST_DEVICE inline bool
motionAltitudeFinite(const float value) noexcept {
#if defined(__CUDA_ARCH__)
  return isfinite(value);
#else
  return std::isfinite(value);
#endif
}

[[nodiscard]] DRONE_CITY_NAV_MOTION_ALTITUDE_HOST_DEVICE inline float
verticalStoppingDistanceM3D(const float vertical_speed_mps,
                            const float vertical_acceleration_mps2,
                            const MotionDynamicsConfig3D& dynamics,
                            const float guaranteed_vertical_deceleration_mps2,
                            const float reaction_latency_s) noexcept {
  const float speed_mps =
      vertical_speed_mps < 0.0F ? -vertical_speed_mps : vertical_speed_mps;
  if (!(speed_mps > 0.0F)) {
    return 0.0F;
  }
  const float guaranteed_deceleration_mps2 =
      guaranteed_vertical_deceleration_mps2 <
              dynamics.maximum_vertical_acceleration_mps2
          ? guaranteed_vertical_deceleration_mps2
          : dynamics.maximum_vertical_acceleration_mps2;
  const float maximum_jerk_mps3 = dynamics.maximum_control_jerk_mps3;
  if (!(guaranteed_deceleration_mps2 > 0.0F) || !(maximum_jerk_mps3 > 0.0F) ||
      !(reaction_latency_s >= 0.0F)) {
    return 3.402823466e+38F;
  }

  const float direction = vertical_speed_mps < 0.0F ? -1.0F : 1.0F;
  const float aligned_acceleration_mps2 =
      direction * vertical_acceleration_mps2 < -guaranteed_deceleration_mps2
          ? -guaranteed_deceleration_mps2
          : (direction * vertical_acceleration_mps2 >
                     dynamics.maximum_vertical_acceleration_mps2
                 ? dynamics.maximum_vertical_acceleration_mps2
                 : direction * vertical_acceleration_mps2);
  const float latency_speed_mps =
      speed_mps + aligned_acceleration_mps2 * reaction_latency_s;
  if (!(latency_speed_mps > 0.0F)) {
    return aligned_acceleration_mps2 < 0.0F
               ? speed_mps * speed_mps / (-2.0F * aligned_acceleration_mps2)
               : 0.0F;
  }
  const float latency_distance_m =
      speed_mps * reaction_latency_s +
      0.5F * aligned_acceleration_mps2 * reaction_latency_s * reaction_latency_s;
  const float ramp_time_s =
      (aligned_acceleration_mps2 + guaranteed_deceleration_mps2) / maximum_jerk_mps3;
  const float ramp_time_squared_s2 = ramp_time_s * ramp_time_s;
  const float speed_after_ramp_mps = latency_speed_mps +
                                     aligned_acceleration_mps2 * ramp_time_s -
                                     0.5F * maximum_jerk_mps3 * ramp_time_squared_s2;
  if (!(speed_after_ramp_mps > 0.0F)) {
    const float nonnegative_acceleration_mps2 =
        aligned_acceleration_mps2 > 0.0F ? aligned_acceleration_mps2 : 0.0F;
    return latency_distance_m + latency_speed_mps * ramp_time_s +
           0.5F * nonnegative_acceleration_mps2 * ramp_time_squared_s2;
  }

  const float ramp_distance_m =
      latency_speed_mps * ramp_time_s +
      0.5F * aligned_acceleration_mps2 * ramp_time_squared_s2 -
      maximum_jerk_mps3 * ramp_time_squared_s2 * ramp_time_s / 6.0F;
  const float constant_acceleration_distance_m = speed_after_ramp_mps *
                                                 speed_after_ramp_mps /
                                                 (2.0F * guaranteed_deceleration_mps2);
  return latency_distance_m + ramp_distance_m + constant_acceleration_distance_m;
}

[[nodiscard]] DRONE_CITY_NAV_MOTION_ALTITUDE_HOST_DEVICE inline float
verticalStoppingDistanceM3D(const float vertical_speed_mps,
                            const float vertical_acceleration_mps2,
                            const MotionDynamicsConfig3D& dynamics) noexcept {
  return verticalStoppingDistanceM3D(vertical_speed_mps, vertical_acceleration_mps2,
                                     dynamics,
                                     dynamics.maximum_vertical_acceleration_mps2, 0.0F);
}

[[nodiscard]] DRONE_CITY_NAV_MOTION_ALTITUDE_HOST_DEVICE inline bool
motionAltitudeEnvelopeDynamicallyRecoverable3D(
    const MotionState3D& state, const MotionControl3D& control,
    const MotionDynamicsConfig3D& dynamics,
    const MotionAltitudeEnvelopeConfig3D& envelope) noexcept {
  if (!motionAltitudeFinite(state.z) || !motionAltitudeFinite(state.vz) ||
      !motionAltitudeFinite(control.az) || state.z < envelope.minimum_z_m ||
      state.z >= envelope.maximum_z_m) {
    return false;
  }
  if (state.vz < 0.0F) {
    return verticalStoppingDistanceM3D(state.vz, control.az, dynamics,
                                       envelope.guaranteed_vertical_deceleration_mps2,
                                       envelope.reaction_latency_s) <=
           state.z - envelope.minimum_z_m;
  }
  if (state.vz > 0.0F) {
    return verticalStoppingDistanceM3D(state.vz, control.az, dynamics,
                                       envelope.guaranteed_vertical_deceleration_mps2,
                                       envelope.reaction_latency_s) <
           envelope.maximum_z_m - state.z;
  }
  return true;
}

#undef DRONE_CITY_NAV_MOTION_ALTITUDE_HOST_DEVICE

} // namespace drone_city_nav
