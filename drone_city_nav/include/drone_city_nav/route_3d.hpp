#pragma once

#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/types.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace drone_city_nav {

struct RouteSample3D {
  Point3 position{};
  Vec3 tangent{};
  double station_m{0.0};
  double reference_speed_mps{0.0};
};

struct RouteEnvelopeSample {
  double station_m{0.0};
  double lateral_free_left_m{0.0};
  double lateral_free_right_m{0.0};
  double min_z_m{0.0};
  double max_z_m{0.0};
  double reference_z_m{0.0};
  double reference_speed_mps{0.0};
};

struct ConstrainedRouteSpan {
  std::uint64_t route_generation{0U};
  double begin_station_m{0.0};
  double end_station_m{0.0};
  std::vector<RouteEnvelopeSample> envelope;
};

struct RouteEnvelopeConfig {
  double sample_step_m{0.5};
  double maximum_probe_distance_m{12.0};
  double constrained_lateral_width_m{8.0};
  double constrained_vertical_height_m{8.0};
  double minimum_span_length_m{2.0};
  double unconstrained_speed_mps{20.0};
  double constrained_speed_mps{10.0};
};

[[nodiscard]] std::vector<RouteSample3D> sampleRoute3D(std::span<const Point3> points,
                                                       double sample_step_m,
                                                       double reference_speed_mps);

[[nodiscard]] std::vector<ConstrainedRouteSpan>
analyzeConstrainedRouteSpans(std::span<const RouteSample3D> route,
                             const mppi::EsdfGrid& grid, std::span<const float> esdf_m,
                             std::uint64_t route_generation,
                             const RouteEnvelopeConfig& config);

} // namespace drone_city_nav
