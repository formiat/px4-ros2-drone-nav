#include "drone_city_nav/incremental_topological_lattice_adapter_3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace drone_city_nav {
namespace {

constexpr double kGeometryEpsilon{1.0e-9};

struct PolylineProjection3D {
  Point3 point{};
  std::size_t segment_index{0U};
  double segment_fraction{0.0};
  double station_m{0.0};
  double distance_m{std::numeric_limits<double>::infinity()};
};

[[nodiscard]] bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] Point3 interpolate(const Point3& from, const Point3& to,
                                 const double fraction) noexcept {
  return Point3{.x = std::lerp(from.x, to.x, fraction),
                .y = std::lerp(from.y, to.y, fraction),
                .z = std::lerp(from.z, to.z, fraction)};
}

[[nodiscard]] std::optional<PolylineProjection3D>
projectOntoPolyline(const std::vector<Point3>& points, const Point3& position) {
  if (points.size() < 2U || !finitePoint(position)) {
    return std::nullopt;
  }

  std::optional<PolylineProjection3D> best;
  double segment_start_station_m = 0.0;
  for (std::size_t index = 0U; index + 1U < points.size(); ++index) {
    const Point3& first = points[index];
    const Point3& second = points[index + 1U];
    if (!finitePoint(first) || !finitePoint(second)) {
      return std::nullopt;
    }
    const Vec3 segment{
        .x = second.x - first.x, .y = second.y - first.y, .z = second.z - first.z};
    const double length_squared =
        segment.x * segment.x + segment.y * segment.y + segment.z * segment.z;
    const double segment_length = std::sqrt(length_squared);
    if (segment_length <= kGeometryEpsilon) {
      continue;
    }
    const Vec3 offset{.x = position.x - first.x,
                      .y = position.y - first.y,
                      .z = position.z - first.z};
    const double fraction = std::clamp(
        (offset.x * segment.x + offset.y * segment.y + offset.z * segment.z) /
            length_squared,
        0.0, 1.0);
    const Point3 projection = interpolate(first, second, fraction);
    const double projection_distance = distance3D(position, projection);
    const double station = segment_start_station_m + fraction * segment_length;
    if (!best || projection_distance + kGeometryEpsilon < best->distance_m ||
        (std::abs(projection_distance - best->distance_m) <= kGeometryEpsilon &&
         station + kGeometryEpsilon < best->station_m)) {
      best = PolylineProjection3D{.point = projection,
                                  .segment_index = index,
                                  .segment_fraction = fraction,
                                  .station_m = station,
                                  .distance_m = projection_distance};
    }
    segment_start_station_m += segment_length;
  }
  return best;
}

[[nodiscard]] Lattice3DRoutePurpose
latticePurpose(const IncrementalTopologicalRoutePurpose3D purpose) noexcept {
  switch (purpose) {
    case IncrementalTopologicalRoutePurpose3D::kMissionTransit:
      return Lattice3DRoutePurpose::kMissionTransit;
    case IncrementalTopologicalRoutePurpose3D::kObservationFrontier:
      return Lattice3DRoutePurpose::kObservationFrontier;
    case IncrementalTopologicalRoutePurpose3D::kTopologicalBacktrack:
      return Lattice3DRoutePurpose::kTopologicalBacktrack;
  }
  return Lattice3DRoutePurpose::kMissionTransit;
}

} // namespace

bool incrementalTopologicalLatticeAdapter3DConfigIsValid(
    const IncrementalTopologicalLatticeAdapter3DConfig& config) noexcept {
  return std::isfinite(config.maximum_lookahead_m) &&
         config.maximum_lookahead_m > 0.0 &&
         std::isfinite(config.minimum_target_displacement_m) &&
         config.minimum_target_displacement_m >= 0.0 &&
         config.minimum_target_displacement_m < config.maximum_lookahead_m;
}

std::optional<IncrementalTopologicalLatticeDirective3D>
makeIncrementalTopologicalLatticeDirective3D(
    const IncrementalTopologicalPlan3D& plan, const Point3& position,
    const IncrementalTopologicalLatticeAdapter3DConfig& config) {
  if (!incrementalTopologicalLatticeAdapter3DConfigIsValid(config) ||
      !plan.executableTargetSelected()) {
    return std::nullopt;
  }
  const std::optional<PolylineProjection3D> projection =
      projectOntoPolyline(plan.guidance_points, position);
  if (!projection) {
    return std::nullopt;
  }

  Point3 cursor = projection->point;
  Point3 target = cursor;
  Vec3 preferred_direction{};
  double target_station_m = projection->station_m;
  double remaining_m = config.maximum_lookahead_m;
  bool reaches_target = true;
  for (std::size_t index = projection->segment_index + 1U;
       index < plan.guidance_points.size(); ++index) {
    const Point3& next = plan.guidance_points[index];
    const Vec3 segment{
        .x = next.x - cursor.x, .y = next.y - cursor.y, .z = next.z - cursor.z};
    const double segment_length = std::sqrt(
        segment.x * segment.x + segment.y * segment.y + segment.z * segment.z);
    if (segment_length <= kGeometryEpsilon) {
      cursor = next;
      continue;
    }
    if (preferred_direction.x == 0.0 && preferred_direction.y == 0.0 &&
        preferred_direction.z == 0.0) {
      preferred_direction = segment;
    }
    if (segment_length > remaining_m + kGeometryEpsilon) {
      target = interpolate(cursor, next, remaining_m / segment_length);
      target_station_m += remaining_m;
      reaches_target = false;
      break;
    }
    target = next;
    target_station_m += segment_length;
    remaining_m -= segment_length;
    cursor = next;
  }

  if (distance3D(position, target) + kGeometryEpsilon <
      config.minimum_target_displacement_m) {
    return std::nullopt;
  }

  return IncrementalTopologicalLatticeDirective3D{
      .lattice =
          Lattice3DStrategicDirective{
              .planning_goal = target,
              .preferred_direction = preferred_direction,
              .route_purpose = latticePurpose(plan.purpose),
              .observation_frontier = plan.selected_frontier,
              .selection_score = plan.selection_score,
              .reaches_mission_goal = plan.reaches_mission_goal && reaches_target,
          },
      .source_segment_index = projection->segment_index,
      .source_station_m = projection->station_m,
      .target_station_m = target_station_m,
      .projection_distance_m = projection->distance_m,
      .reaches_topological_target = reaches_target,
  };
}

} // namespace drone_city_nav
