#pragma once

#include "drone_city_nav/mppi/mppi_config.hpp"
#include "drone_city_nav/mppi/mppi_types.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <span>

namespace drone_city_nav::mppi::detail {

struct DeviceRouteWindow3D {
  std::span<const RouteSample3D> points;
  std::size_t begin_index{0U};

  [[nodiscard]] bool coversThroughStation(const float station_m) const noexcept {
    constexpr float kStationToleranceM{1.0e-3F};
    return points.size() >= 2U &&
           points.back().station_m + kStationToleranceM >= station_m;
  }
};

[[nodiscard]] inline DeviceRouteWindow3D
selectDeviceRouteWindow3D(const std::span<const RouteSample3D> route,
                          const float minimum_station_m,
                          const std::size_t maximum_point_count) noexcept {
  if (route.size() < 2U || maximum_point_count < 2U) {
    return {};
  }
  if (route.size() <= maximum_point_count) {
    return DeviceRouteWindow3D{.points = route};
  }
  const auto first_not_behind =
      std::lower_bound(route.begin(), route.end(), minimum_station_m,
                       [](const RouteSample3D& sample, const float station_m) {
                         return sample.station_m < station_m;
                       });
  const std::size_t first_not_behind_index =
      static_cast<std::size_t>(std::distance(route.begin(), first_not_behind));
  const std::size_t begin_index =
      std::min(first_not_behind_index == 0U ? 0U : first_not_behind_index - 1U,
               route.size() - 2U);
  return DeviceRouteWindow3D{
      .points = route.subspan(
          begin_index, std::min(maximum_point_count, route.size() - begin_index)),
      .begin_index = begin_index,
  };
}

[[nodiscard]] inline float
maximumFiniteHorizonTravelM(const State& state,
                            const BenchmarkConfig& config) noexcept {
  const float current_horizontal_speed_mps = std::hypot(state.vx, state.vy);
  const float current_vertical_speed_mps = std::abs(state.vz);
  const float current_speed_mps =
      std::hypot(current_horizontal_speed_mps, current_vertical_speed_mps);
  const float component_speed_bound_mps = std::hypot(
      std::max(current_horizontal_speed_mps,
               config.dynamics.maximum_horizontal_speed_mps),
      std::max(current_vertical_speed_mps, config.dynamics.maximum_vertical_speed_mps));
  const float translational_speed_bound_mps =
      std::max(current_speed_mps, config.dynamics.maximum_translational_speed_mps);
  const float speed_bound_mps =
      std::min(component_speed_bound_mps, translational_speed_bound_mps);
  return speed_bound_mps * config.dynamics.dt_s * static_cast<float>(config.steps);
}

} // namespace drone_city_nav::mppi::detail
