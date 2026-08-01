#include "drone_city_nav/route_3d.hpp"

#include "drone_city_nav/esdf_query.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace drone_city_nav {
namespace {

[[nodiscard]] Vec3 normalized(const Point3& from, const Point3& to) noexcept {
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const double dz = to.z - from.z;
  const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
  return length > 1.0e-9 ? Vec3{dx / length, dy / length, dz / length} : Vec3{};
}

[[nodiscard]] double probeFreeDistance(const mppi::EsdfGrid& grid,
                                       const std::span<const float> esdf_m,
                                       const Point3& origin, const Vec3& direction,
                                       const RouteEnvelopeConfig& config) {
  double distance_m = 0.0;
  const std::size_t sample_count = static_cast<std::size_t>(
      std::floor(config.maximum_probe_distance_m / config.sample_step_m));
  for (std::size_t sample = 1U; sample <= sample_count; ++sample) {
    const double probe_m = static_cast<double>(sample) * config.sample_step_m;
    const EsdfQueryResult query = queryConservativeEsdf3D(
        grid, esdf_m, static_cast<float>(origin.x + direction.x * probe_m),
        static_cast<float>(origin.y + direction.y * probe_m),
        static_cast<float>(origin.z + direction.z * probe_m));
    if (query.status != EsdfQueryStatus::kValid || query.raw_occupied) {
      break;
    }
    distance_m = probe_m;
  }
  return distance_m;
}

} // namespace

std::vector<RouteSample3D> sampleRoute3D(const std::span<const Point3> points,
                                         const double sample_step_m,
                                         const double reference_speed_mps) {
  std::vector<RouteSample3D> result;
  if (points.empty() || !(sample_step_m > 0.0)) {
    return result;
  }
  double station_m = 0.0;
  result.push_back(RouteSample3D{.position = points.front(),
                                 .reference_speed_mps = reference_speed_mps});
  for (std::size_t index = 0U; index + 1U < points.size(); ++index) {
    const Point3 first = points[index];
    const Point3 second = points[index + 1U];
    const Vec3 tangent = normalized(first, second);
    const double length = distance3D(first, second);
    if (!(length > 1.0e-9)) {
      continue;
    }
    const std::size_t samples = std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::ceil(length / sample_step_m)));
    for (std::size_t sample = 1U; sample <= samples; ++sample) {
      const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
      const double sample_station_m = station_m + ratio * length;
      result.push_back(RouteSample3D{
          .position = Point3{std::lerp(first.x, second.x, ratio),
                             std::lerp(first.y, second.y, ratio),
                             std::lerp(first.z, second.z, ratio)},
          .tangent = tangent,
          .station_m = sample_station_m,
          .reference_speed_mps = reference_speed_mps,
      });
    }
    station_m += length;
  }
  if (result.size() >= 2U) {
    result.front().tangent = result[1U].tangent;
  }
  return result;
}

std::vector<ConstrainedRouteSpan> analyzeConstrainedRouteSpans(
    const std::span<const RouteSample3D> route, const mppi::EsdfGrid& grid,
    const std::span<const float> esdf_m, const std::uint64_t route_generation,
    const RouteEnvelopeConfig& config) {
  std::vector<ConstrainedRouteSpan> spans;
  std::optional<ConstrainedRouteSpan> active;
  for (const RouteSample3D& sample : route) {
    const double horizontal_norm = std::hypot(sample.tangent.x, sample.tangent.y);
    const Vec3 left = horizontal_norm > 1.0e-9
                          ? Vec3{-sample.tangent.y / horizontal_norm,
                                 sample.tangent.x / horizontal_norm, 0.0}
                          : Vec3{1.0, 0.0, 0.0};
    RouteEnvelopeSample envelope{
        .station_m = sample.station_m,
        .lateral_free_left_m =
            probeFreeDistance(grid, esdf_m, sample.position, left, config),
        .lateral_free_right_m = probeFreeDistance(grid, esdf_m, sample.position,
                                                  Vec3{-left.x, -left.y, 0.0}, config),
        .min_z_m = sample.position.z - probeFreeDistance(grid, esdf_m, sample.position,
                                                         Vec3{0.0, 0.0, -1.0}, config),
        .max_z_m = sample.position.z + probeFreeDistance(grid, esdf_m, sample.position,
                                                         Vec3{0.0, 0.0, 1.0}, config),
        .reference_z_m = sample.position.z,
        .reference_speed_mps = config.unconstrained_speed_mps,
    };
    const double lateral_width =
        envelope.lateral_free_left_m + envelope.lateral_free_right_m;
    const double vertical_height = envelope.max_z_m - envelope.min_z_m;
    const bool constrained = lateral_width <= config.constrained_lateral_width_m ||
                             vertical_height <= config.constrained_vertical_height_m;
    if (constrained) {
      envelope.reference_speed_mps = config.constrained_speed_mps;
      if (!active.has_value()) {
        active = ConstrainedRouteSpan{.route_generation = route_generation,
                                      .begin_station_m = sample.station_m,
                                      .end_station_m = sample.station_m,
                                      .envelope = {}};
      }
      active->end_station_m = sample.station_m;
      active->envelope.push_back(envelope);
    } else if (active.has_value()) {
      if (active->end_station_m - active->begin_station_m >=
          config.minimum_span_length_m) {
        spans.push_back(std::move(*active));
      }
      active.reset();
    }
  }
  if (active.has_value() &&
      active->end_station_m - active->begin_station_m >= config.minimum_span_length_m) {
    spans.push_back(std::move(*active));
  }
  return spans;
}

} // namespace drone_city_nav
