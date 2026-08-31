#pragma once

#include "drone_city_nav/motion_altitude_envelope_3d.hpp"
#include "drone_city_nav/mppi/mppi_config.hpp"
#include "drone_city_nav/mppi/mppi_types.hpp"

namespace drone_city_nav::mppi {

#if defined(__CUDACC__)
#define DRONE_CITY_NAV_MPPI_ALTITUDE_HOST_DEVICE __host__ __device__
#else
#define DRONE_CITY_NAV_MPPI_ALTITUDE_HOST_DEVICE
#endif

[[nodiscard]] DRONE_CITY_NAV_MPPI_ALTITUDE_HOST_DEVICE inline bool
mppiAltitudeFinite(const float value) noexcept {
  return motionAltitudeFinite(value);
}

[[nodiscard]] DRONE_CITY_NAV_MPPI_ALTITUDE_HOST_DEVICE inline float
verticalStoppingDistanceM(const float vertical_speed_mps,
                          const float vertical_acceleration_mps2,
                          const DynamicsConfig& dynamics,
                          const float guaranteed_vertical_deceleration_mps2,
                          const float reaction_latency_s) noexcept {
  return verticalStoppingDistanceM3D(vertical_speed_mps, vertical_acceleration_mps2,
                                     dynamics, guaranteed_vertical_deceleration_mps2,
                                     reaction_latency_s);
}

[[nodiscard]] DRONE_CITY_NAV_MPPI_ALTITUDE_HOST_DEVICE inline float
verticalStoppingDistanceM(const float vertical_speed_mps,
                          const float vertical_acceleration_mps2,
                          const DynamicsConfig& dynamics) noexcept {
  return verticalStoppingDistanceM3D(vertical_speed_mps, vertical_acceleration_mps2,
                                     dynamics);
}

[[nodiscard]] DRONE_CITY_NAV_MPPI_ALTITUDE_HOST_DEVICE inline bool
altitudeEnvelopeDynamicallyRecoverable(
    const State& state, const Control& control, const DynamicsConfig& dynamics,
    const AltitudeEnvelopeConfig& envelope) noexcept {
  return motionAltitudeEnvelopeDynamicallyRecoverable3D(state, control, dynamics,
                                                        envelope);
}

#undef DRONE_CITY_NAV_MPPI_ALTITUDE_HOST_DEVICE

} // namespace drone_city_nav::mppi
