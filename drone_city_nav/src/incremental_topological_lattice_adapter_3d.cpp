#include "drone_city_nav/incremental_topological_lattice_adapter_3d.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav {
namespace {

constexpr double kGeometryEpsilon{1.0e-9};

[[nodiscard]] bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] Point3 interpolate(const Point3& from, const Point3& to,
                                 const double fraction) noexcept {
  return Point3{.x = std::lerp(from.x, to.x, fraction),
                .y = std::lerp(from.y, to.y, fraction),
                .z = std::lerp(from.z, to.z, fraction)};
}

} // namespace

std::optional<TopologicalPolylineProjection3D>
projectOntoTopologicalPolyline3D(const std::span<const Point3> points,
                                 const Point3& position) {
  if (points.size() < 2U || !finitePoint(position)) {
    return std::nullopt;
  }

  std::optional<TopologicalPolylineProjection3D> best;
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
      best = TopologicalPolylineProjection3D{.point = projection,
                                             .segment_index = index,
                                             .segment_fraction = fraction,
                                             .station_m = station,
                                             .remaining_m = 0.0,
                                             .distance_m = projection_distance};
    }
    segment_start_station_m += segment_length;
  }
  if (best.has_value()) {
    best->remaining_m = std::max(0.0, segment_start_station_m - best->station_m);
  }
  return best;
}

namespace {

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
         std::isfinite(config.segment_capture_radius_m) &&
         config.segment_capture_radius_m >= 0.0 &&
         config.segment_capture_radius_m < config.maximum_lookahead_m &&
         std::isfinite(config.minimum_collinear_direction_cosine) &&
         config.minimum_collinear_direction_cosine >= -1.0 &&
         config.minimum_collinear_direction_cosine <= 1.0;
}

std::optional<IncrementalTopologicalLatticeDirective3D>
makeIncrementalTopologicalLatticeDirective3D(
    const IncrementalTopologicalPlan3D& plan, const Point3& position,
    const IncrementalTopologicalLatticeAdapter3DConfig& config) {
  if (!incrementalTopologicalLatticeAdapter3DConfigIsValid(config) ||
      !plan.executableTargetSelected()) {
    return std::nullopt;
  }
  const std::optional<TopologicalPolylineProjection3D> projection =
      projectOntoTopologicalPolyline3D(plan.guidance_points, position);
  if (!projection) {
    return std::nullopt;
  }
  if (projection->remaining_m <= config.segment_capture_radius_m + kGeometryEpsilon) {
    return std::nullopt;
  }

  // A topological polyline is not merely a preferred heading. Its vertices
  // represent observed connectivity. Keep a useful local lookahead along a
  // straight voxel chain, but stop at the first real bend. This prevents the
  // local lattice from cutting across graph topology while avoiding terminal
  // micro-routes for the many collinear cells in an incremental graph.
  Point3 cursor = projection->point;
  Point3 target = cursor;
  Vec3 preferred_direction{};
  double target_station_m = projection->station_m;
  double remaining_m = config.maximum_lookahead_m;
  std::size_t captured_bends_skipped = 0U;
  bool reaches_target = false;
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
    if (preferred_direction.x != 0.0 || preferred_direction.y != 0.0 ||
        preferred_direction.z != 0.0) {
      const double preferred_length =
          std::sqrt(preferred_direction.x * preferred_direction.x +
                    preferred_direction.y * preferred_direction.y +
                    preferred_direction.z * preferred_direction.z);
      const double directional_cosine =
          (preferred_direction.x * segment.x + preferred_direction.y * segment.y +
           preferred_direction.z * segment.z) /
          (preferred_length * segment_length);
      if (directional_cosine + kGeometryEpsilon <
          config.minimum_collinear_direction_cosine) {
        if (distance3D(position, cursor) >
            config.segment_capture_radius_m + kGeometryEpsilon) {
          break;
        }
        // The preceding connector or graph step terminates inside the same
        // capture neighbourhood in which the lattice reports goal arrival.
        // Treat the bend as acquired and aim along the outgoing segment.
        preferred_direction = segment;
        ++captured_bends_skipped;
      }
    } else {
      preferred_direction = segment;
    }
    if (segment_length > remaining_m + kGeometryEpsilon) {
      target = interpolate(cursor, next, remaining_m / segment_length);
      target_station_m += remaining_m;
      break;
    }
    target = next;
    target_station_m += segment_length;
    remaining_m -= segment_length;
    cursor = next;
    if (index + 1U == plan.guidance_points.size()) {
      reaches_target = true;
    }
    if (remaining_m <= kGeometryEpsilon) {
      break;
    }
  }

  if (distance3D(position, target) <=
      config.segment_capture_radius_m + kGeometryEpsilon) {
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
      .captured_bends_skipped = captured_bends_skipped,
      .reaches_topological_target = reaches_target,
  };
}

} // namespace drone_city_nav
