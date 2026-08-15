#pragma once

#include "drone_city_nav/mppi/mppi_engine.hpp"

#include <cfloat>
#include <cmath>
#include <cstddef>

namespace drone_city_nav::mppi {

#if defined(__CUDACC__)
#define DRONE_CITY_NAV_MPPI_HOST_DEVICE __host__ __device__
#else
#define DRONE_CITY_NAV_MPPI_HOST_DEVICE
#endif

struct MppiRouteProjection3D {
  float station_m{0.0F};
  float distance_m{0.0F};
  float reference_x_m{0.0F};
  float reference_y_m{0.0F};
  float reference_z_m{0.0F};
  float reference_speed_mps{0.0F};
  float tangent_x{0.0F};
  float tangent_y{0.0F};
  float tangent_z{0.0F};
  bool valid{false};
};

[[nodiscard]] DRONE_CITY_NAV_MPPI_HOST_DEVICE inline float
routeTrackingSpeedMps(const State& state,
                      const MppiRouteProjection3D& projection) noexcept {
  const float along_route_speed_mps = state.vx * projection.tangent_x +
                                      state.vy * projection.tangent_y +
                                      state.vz * projection.tangent_z;
  return fmaxf(0.0F, along_route_speed_mps);
}

[[nodiscard]] DRONE_CITY_NAV_MPPI_HOST_DEVICE inline MppiRouteProjection3D
projectOntoMppiRoute3D(const State& state, const RouteSample3D* route_points,
                       const std::size_t route_point_count,
                       const float minimum_station_m) noexcept {
  MppiRouteProjection3D result;
  float best_squared_distance = FLT_MAX;
  for (std::size_t index = 0U; index + 1U < route_point_count; ++index) {
    const RouteSample3D first = route_points[index];
    const RouteSample3D second = route_points[index + 1U];
    if (second.station_m + 1.0e-5F < minimum_station_m) {
      continue;
    }
    const float dx = second.x_m - first.x_m;
    const float dy = second.y_m - first.y_m;
    const float dz = second.z_m - first.z_m;
    const float squared_length = dx * dx + dy * dy + dz * dz;
    const float station_length_m = second.station_m - first.station_m;
    if (!(squared_length > 1.0e-8F) || !(station_length_m > 1.0e-5F)) {
      continue;
    }
    const float unclamped_minimum_ratio =
        (minimum_station_m - first.station_m) / station_length_m;
    const float minimum_ratio = fminf(1.0F, fmaxf(0.0F, unclamped_minimum_ratio));
    const float projected_ratio =
        ((state.x - first.x_m) * dx + (state.y - first.y_m) * dy +
         (state.z - first.z_m) * dz) /
        squared_length;
    const float ratio = fminf(1.0F, fmaxf(minimum_ratio, projected_ratio));
    const float reference_x_m = first.x_m + ratio * dx;
    const float reference_y_m = first.y_m + ratio * dy;
    const float reference_z_m = first.z_m + ratio * dz;
    const float offset_x = state.x - reference_x_m;
    const float offset_y = state.y - reference_y_m;
    const float offset_z = state.z - reference_z_m;
    const float squared_distance =
        offset_x * offset_x + offset_y * offset_y + offset_z * offset_z;
    if (squared_distance < best_squared_distance) {
      best_squared_distance = squared_distance;
      result.station_m = first.station_m + ratio * station_length_m;
      result.distance_m = sqrtf(squared_distance);
      result.reference_x_m = reference_x_m;
      result.reference_y_m = reference_y_m;
      result.reference_z_m = reference_z_m;
      result.reference_speed_mps =
          first.reference_speed_mps +
          ratio * (second.reference_speed_mps - first.reference_speed_mps);
      const float inverse_length = 1.0F / sqrtf(squared_length);
      result.tangent_x = dx * inverse_length;
      result.tangent_y = dy * inverse_length;
      result.tangent_z = dz * inverse_length;
      result.valid = true;
    }
  }
  return result;
}

[[nodiscard]] inline MppiRouteProjection3D
projectOntoMppiRoute3D(const State& state, const std::span<const RouteSample3D> route,
                       const float minimum_station_m) noexcept {
  return projectOntoMppiRoute3D(state, route.data(), route.size(), minimum_station_m);
}

#undef DRONE_CITY_NAV_MPPI_HOST_DEVICE

} // namespace drone_city_nav::mppi
