#include "drone_city_nav/static_route_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <ranges>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool
protectedStation(const double station_m,
                 const std::span<const ConstrainedRouteSpan> spans) noexcept {
  return std::ranges::any_of(spans, [station_m](const ConstrainedRouteSpan& span) {
    return station_m + 1.0e-9 >= span.begin_station_m &&
           station_m <= span.end_station_m + 1.0e-9;
  });
}

[[nodiscard]] Point3 lerpPoint(const Point3& first, const Point3& second,
                               const double ratio) noexcept {
  return Point3{std::lerp(first.x, second.x, ratio),
                std::lerp(first.y, second.y, ratio),
                std::lerp(first.z, second.z, ratio)};
}

[[nodiscard]] std::optional<std::vector<Point3>>
smoothCorner(const Point3& previous, const Point3& corner, const Point3& next,
             const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
             const SweptFootprintConfig& footprint_config,
             const StaticRouteGeometryConfig& geometry_config) {
  const double incoming_length = distance3D(previous, corner);
  const double outgoing_length = distance3D(corner, next);
  const double smoothing_m = std::min({geometry_config.corner_smoothing_distance_m,
                                       incoming_length * 0.4, outgoing_length * 0.4});
  if (!(smoothing_m > geometry_config.sample_step_m)) {
    return std::nullopt;
  }
  const Point3 entry = lerpPoint(corner, previous, smoothing_m / incoming_length);
  const Point3 exit = lerpPoint(corner, next, smoothing_m / outgoing_length);
  const std::size_t samples =
      std::max<std::size_t>(2U, geometry_config.corner_curve_samples);
  std::vector<Point3> curve;
  curve.reserve(samples + 1U);
  for (std::size_t sample = 0U; sample <= samples; ++sample) {
    const double t = static_cast<double>(sample) / static_cast<double>(samples);
    const double one_minus_t = 1.0 - t;
    curve.push_back(Point3{
        one_minus_t * one_minus_t * entry.x + 2.0 * one_minus_t * t * corner.x +
            t * t * exit.x,
        one_minus_t * one_minus_t * entry.y + 2.0 * one_minus_t * t * corner.y +
            t * t * exit.y,
        one_minus_t * one_minus_t * entry.z + 2.0 * one_minus_t * t * corner.z +
            t * t * exit.z,
    });
  }
  for (std::size_t index = 1U; index < curve.size(); ++index) {
    if (!validateSweptFootprint(grid, esdf_m, curve[index - 1U], curve[index],
                                footprint_config)
             .accepted()) {
      return std::nullopt;
    }
  }
  return curve;
}

} // namespace

StaticRouteGeometryResult optimizeStaticRouteGeometry(
    const std::span<const RouteSample3D> route,
    const std::span<const ConstrainedRouteSpan> constrained_spans,
    const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
    const SweptFootprintConfig& footprint_config,
    const StaticRouteGeometryConfig& geometry_config,
    const RouteEnvelopeConfig& envelope_config) {
  StaticRouteGeometryResult result;
  if (route.size() < 2U) {
    result.route.assign(route.begin(), route.end());
    return result;
  }

  std::vector<Point3> anchors;
  anchors.reserve(route.size());
  anchors.push_back(route.front().position);
  std::size_t current = 0U;
  while (current + 1U < route.size()) {
    std::size_t selected = current + 1U;
    if (!protectedStation(route[current].station_m, constrained_spans)) {
      for (std::size_t candidate = current + 2U; candidate < route.size();
           ++candidate) {
        if (route[candidate].station_m - route[current].station_m >
            geometry_config.maximum_shortcut_length_m) {
          break;
        }
        bool crosses_protected = false;
        for (std::size_t index = current + 1U; index <= candidate; ++index) {
          if (protectedStation(route[index].station_m, constrained_spans)) {
            crosses_protected = true;
            break;
          }
        }
        if (crosses_protected) {
          break;
        }
        if (validateSweptFootprint(grid, esdf_m, route[current].position,
                                   route[candidate].position, footprint_config)
                .accepted()) {
          selected = candidate;
        }
      }
    }
    if (selected > current + 1U) {
      ++result.shortcuts_applied;
    }
    anchors.push_back(route[selected].position);
    current = selected;
  }

  std::vector<Point3> smoothed;
  smoothed.reserve(anchors.size() * 2U);
  smoothed.push_back(anchors.front());
  for (std::size_t index = 1U; index + 1U < anchors.size(); ++index) {
    const RouteProjection3D projection = projectOntoRoute3D(route, anchors[index]);
    const RouteProjection3D previous_projection =
        projectOntoRoute3D(route, anchors[index - 1U]);
    const RouteProjection3D next_projection =
        projectOntoRoute3D(route, anchors[index + 1U]);
    const bool protected_corner =
        (projection.valid &&
         protectedStation(projection.station_m, constrained_spans)) ||
        (previous_projection.valid &&
         protectedStation(previous_projection.station_m, constrained_spans)) ||
        (next_projection.valid &&
         protectedStation(next_projection.station_m, constrained_spans));
    const std::optional<std::vector<Point3>> curve =
        protected_corner
            ? std::nullopt
            : smoothCorner(anchors[index - 1U], anchors[index], anchors[index + 1U],
                           grid, esdf_m, footprint_config, geometry_config);
    if (!curve.has_value()) {
      smoothed.push_back(anchors[index]);
      continue;
    }
    for (const Point3& point : *curve) {
      if (distance3D(smoothed.back(), point) > 1.0e-6) {
        smoothed.push_back(point);
      }
    }
    ++result.corners_smoothed;
  }
  if (distance3D(smoothed.back(), anchors.back()) > 1.0e-6) {
    smoothed.push_back(anchors.back());
  }
  result.route = sampleRoute3D(smoothed, geometry_config.sample_step_m,
                               route.front().reference_speed_mps);

  std::vector<SelectedChannelTraversal> traversals;
  traversals.reserve(constrained_spans.size());
  for (const ConstrainedRouteSpan& span : constrained_spans) {
    if (span.envelope.empty()) {
      continue;
    }
    const Point3 old_entry =
        sampleRoute3DAtStation(route, span.begin_station_m).position;
    const Point3 old_exit = sampleRoute3DAtStation(route, span.end_station_m).position;
    const RouteProjection3D new_entry = projectOntoRoute3D(result.route, old_entry);
    const RouteProjection3D new_exit = projectOntoRoute3D(result.route, old_exit);
    if (!new_entry.valid || !new_exit.valid ||
        new_exit.station_m <= new_entry.station_m) {
      continue;
    }
    const RouteEnvelopeSample& envelope = span.envelope.front();
    traversals.push_back(SelectedChannelTraversal{
        .channel_id = span.channel_id,
        .begin_station_m = new_entry.station_m,
        .end_station_m = new_exit.station_m,
        .min_z_m = envelope.min_z_m,
        .max_z_m = envelope.max_z_m,
        .minimum_clearance_m =
            0.5 * (envelope.lateral_free_left_m + envelope.lateral_free_right_m),
        .speed_limit_mps = envelope.reference_speed_mps,
    });
  }
  result.constrained_spans =
      makeConstrainedRouteSpans(result.route, traversals, 0U, envelope_config);
  return result;
}

} // namespace drone_city_nav
