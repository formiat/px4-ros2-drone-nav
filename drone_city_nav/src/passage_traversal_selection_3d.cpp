#include "drone_city_nav/passage_traversal_selection_3d.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <ranges>
#include <utility>

namespace drone_city_nav {
namespace {

struct TraversalMatch3D {
  SelectedPassageTraversal traversal{};
  double maximum_distance_m{0.0};
};

[[nodiscard]] double vectorLength(const Vec3& vector) noexcept {
  return std::hypot(std::hypot(vector.x, vector.y), vector.z);
}

[[nodiscard]] bool pointFinite(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool
routeGeometryFiniteAndOrdered(const std::span<const RouteSample3D> route) noexcept {
  if (route.size() < 2U) {
    return false;
  }
  double previous_station_m{-1.0};
  for (const RouteSample3D& sample : route) {
    if (!pointFinite(sample.position) || !std::isfinite(sample.station_m) ||
        sample.station_m < 0.0 || sample.station_m <= previous_station_m) {
      return false;
    }
    previous_station_m = sample.station_m;
  }
  return true;
}

[[nodiscard]] double directionAlignment(const Point3& first_begin,
                                        const Point3& first_end,
                                        const Point3& second_begin,
                                        const Point3& second_end) noexcept {
  const Vec3 first{first_end.x - first_begin.x, first_end.y - first_begin.y,
                   first_end.z - first_begin.z};
  const Vec3 second{second_end.x - second_begin.x, second_end.y - second_begin.y,
                    second_end.z - second_begin.z};
  const double denominator = vectorLength(first) * vectorLength(second);
  if (!(denominator > 0.0) || !std::isfinite(denominator)) {
    return -1.0;
  }
  return (first.x * second.x + first.y * second.y + first.z * second.z) / denominator;
}

[[nodiscard]] bool finiteTraversal(const PassageTraversalEdge& traversal) noexcept {
  return !traversal.id.empty() && routeGeometryFiniteAndOrdered(traversal.centerline) &&
         pointFinite(traversal.entry) && pointFinite(traversal.exit) &&
         std::isfinite(traversal.min_z_m) && std::isfinite(traversal.max_z_m) &&
         traversal.max_z_m > traversal.min_z_m && std::isfinite(traversal.width_m) &&
         traversal.width_m > 0.0 && std::isfinite(traversal.height_m) &&
         traversal.height_m > 0.0 && std::isfinite(traversal.minimum_clearance_m) &&
         traversal.minimum_clearance_m > 0.0 &&
         std::isfinite(traversal.speed_limit_mps) && traversal.speed_limit_mps > 0.0;
}

[[nodiscard]] std::optional<TraversalMatch3D>
matchTraversal(const std::span<const RouteSample3D> route,
               const PassageTraversalEdge& traversal,
               const PassageTraversalSelectionConfig3D& config) {
  if (!finiteTraversal(traversal)) {
    return std::nullopt;
  }
  const RouteProjection3D entry = projectOntoRoute3D(route, traversal.entry);
  const RouteProjection3D exit = projectOntoRoute3D(route, traversal.exit);
  if (!entry.valid || !exit.valid ||
      entry.distance_m > config.maximum_endpoint_distance_m ||
      exit.distance_m > config.maximum_endpoint_distance_m) {
    return std::nullopt;
  }
  const double begin_station_m = std::min(entry.station_m, exit.station_m);
  const double end_station_m = std::max(entry.station_m, exit.station_m);
  if (end_station_m - begin_station_m < config.minimum_traversal_length_m) {
    return std::nullopt;
  }
  const int direction_sign = entry.station_m < exit.station_m ? 1 : -1;
  const Point3 route_begin = sampleRoute3DAtStation(route, begin_station_m).position;
  const Point3 route_end = sampleRoute3DAtStation(route, end_station_m).position;
  const Point3 traversal_begin = direction_sign > 0 ? traversal.entry : traversal.exit;
  const Point3 traversal_end = direction_sign > 0 ? traversal.exit : traversal.entry;
  if (directionAlignment(route_begin, route_end, traversal_begin, traversal_end) <
      config.minimum_direction_alignment) {
    return std::nullopt;
  }

  double maximum_distance_m = std::max(entry.distance_m, exit.distance_m);
  for (const RouteSample3D& sample : traversal.centerline) {
    const RouteProjection3D projection = projectOntoRoute3DWithinStationWindow(
        route, sample.position, begin_station_m, end_station_m);
    if (!projection.valid ||
        projection.distance_m > config.maximum_centerline_distance_m) {
      return std::nullopt;
    }
    maximum_distance_m = std::max(maximum_distance_m, projection.distance_m);
  }
  const Point3 route_midpoint =
      sampleRoute3DAtStation(route, 0.5 * (begin_station_m + end_station_m)).position;
  const RouteProjection3D midpoint_projection =
      projectOntoRoute3D(traversal.centerline, route_midpoint);
  if (!midpoint_projection.valid ||
      midpoint_projection.distance_m > config.maximum_centerline_distance_m) {
    return std::nullopt;
  }
  maximum_distance_m = std::max(maximum_distance_m, midpoint_projection.distance_m);

  // The topology-to-route projection above prevents a sparse route from merely
  // crossing the traversal endpoints. This reciprocal check prevents a route
  // with a loop or detour from being decorated just because the topology
  // centerline still projects onto some other part of it.
  for (const RouteSample3D& sample : route) {
    if (sample.station_m + 1.0e-6 < begin_station_m ||
        sample.station_m - 1.0e-6 > end_station_m) {
      continue;
    }
    const RouteProjection3D projection =
        projectOntoRoute3D(traversal.centerline, sample.position);
    if (!projection.valid ||
        projection.distance_m > config.maximum_centerline_distance_m) {
      return std::nullopt;
    }
    maximum_distance_m = std::max(maximum_distance_m, projection.distance_m);
  }

  std::vector<PassageTraversalSegmentSpan> segment_spans;
  segment_spans.reserve(traversal.segment_spans.size());
  const double topology_end_station_m = traversal.centerline.back().station_m;
  for (const PassageTraversalSegmentSpan& source_span : traversal.segment_spans) {
    if (source_span.passage_segment_id.empty() ||
        !std::isfinite(source_span.begin_station_m) ||
        !std::isfinite(source_span.end_station_m) ||
        source_span.begin_station_m < 0.0 ||
        !(source_span.end_station_m > source_span.begin_station_m) ||
        source_span.end_station_m > topology_end_station_m + 1.0e-6) {
      return std::nullopt;
    }
    const Point3 source_begin =
        sampleRoute3DAtStation(traversal.centerline, source_span.begin_station_m)
            .position;
    const Point3 source_end =
        sampleRoute3DAtStation(traversal.centerline, source_span.end_station_m)
            .position;
    const RouteProjection3D mapped_begin = projectOntoRoute3DWithinStationWindow(
        route, source_begin, begin_station_m, end_station_m);
    const RouteProjection3D mapped_end = projectOntoRoute3DWithinStationWindow(
        route, source_end, begin_station_m, end_station_m);
    if (!mapped_begin.valid || !mapped_end.valid ||
        mapped_begin.distance_m > config.maximum_centerline_distance_m ||
        mapped_end.distance_m > config.maximum_centerline_distance_m) {
      return std::nullopt;
    }
    const double mapped_begin_station_m =
        std::min(mapped_begin.station_m, mapped_end.station_m);
    const double mapped_end_station_m =
        std::max(mapped_begin.station_m, mapped_end.station_m);
    if (mapped_end_station_m <= mapped_begin_station_m) {
      return std::nullopt;
    }
    segment_spans.push_back(PassageTraversalSegmentSpan{
        .passage_segment_id = source_span.passage_segment_id,
        .begin_station_m = mapped_begin_station_m,
        .end_station_m = mapped_end_station_m,
    });
  }
  std::ranges::sort(segment_spans, {}, &PassageTraversalSegmentSpan::begin_station_m);
  for (std::size_t index = 1U; index < segment_spans.size(); ++index) {
    if (segment_spans[index].begin_station_m <
        segment_spans[index - 1U].end_station_m - 1.0e-6) {
      return std::nullopt;
    }
  }

  return TraversalMatch3D{
      .traversal =
          SelectedPassageTraversal{
              .passage_traversal_id = traversal.id,
              .direction_sign = direction_sign,
              .begin_station_m = begin_station_m,
              .end_station_m = end_station_m,
              .min_z_m = traversal.min_z_m,
              .max_z_m = traversal.max_z_m,
              .width_m = traversal.width_m,
              .height_m = traversal.height_m,
              .minimum_clearance_m = traversal.minimum_clearance_m,
              .speed_limit_mps = traversal.speed_limit_mps,
              .segment_spans = std::move(segment_spans),
          },
      .maximum_distance_m = maximum_distance_m,
  };
}

[[nodiscard]] bool overlaps(const SelectedPassageTraversal& first,
                            const SelectedPassageTraversal& second) noexcept {
  constexpr double kStationToleranceM{1.0e-6};
  return first.begin_station_m < second.end_station_m - kStationToleranceM &&
         second.begin_station_m < first.end_station_m - kStationToleranceM;
}

} // namespace

bool passageTraversalSelectionConfig3DValid(
    const PassageTraversalSelectionConfig3D& config) noexcept {
  return std::isfinite(config.maximum_endpoint_distance_m) &&
         config.maximum_endpoint_distance_m >= 0.0 &&
         std::isfinite(config.maximum_centerline_distance_m) &&
         config.maximum_centerline_distance_m >= 0.0 &&
         std::isfinite(config.minimum_direction_alignment) &&
         config.minimum_direction_alignment >= -1.0 &&
         config.minimum_direction_alignment <= 1.0 &&
         std::isfinite(config.minimum_traversal_length_m) &&
         config.minimum_traversal_length_m > 0.0;
}

std::vector<SelectedPassageTraversal> selectRoutePassageTraversals3D(
    const std::span<const RouteSample3D> route,
    const std::span<const PassageTraversalEdge> topology_traversals,
    const PassageTraversalSelectionConfig3D& config) {
  if (route.size() < 2U || !passageTraversalSelectionConfig3DValid(config)) {
    return {};
  }
  if (!routeGeometryFiniteAndOrdered(route)) {
    return {};
  }
  std::vector<TraversalMatch3D> matches;
  matches.reserve(topology_traversals.size());
  for (const PassageTraversalEdge& traversal : topology_traversals) {
    std::optional<TraversalMatch3D> match = matchTraversal(route, traversal, config);
    if (match.has_value()) {
      matches.push_back(std::move(*match));
    }
  }
  std::ranges::sort(
      matches, [](const TraversalMatch3D& first, const TraversalMatch3D& second) {
        if (first.maximum_distance_m != second.maximum_distance_m) {
          return first.maximum_distance_m < second.maximum_distance_m;
        }
        const double first_length =
            first.traversal.end_station_m - first.traversal.begin_station_m;
        const double second_length =
            second.traversal.end_station_m - second.traversal.begin_station_m;
        if (first_length != second_length) {
          return first_length > second_length;
        }
        return first.traversal.passage_traversal_id.value() <
               second.traversal.passage_traversal_id.value();
      });

  std::vector<SelectedPassageTraversal> selected;
  selected.reserve(matches.size());
  for (TraversalMatch3D& match : matches) {
    if (std::ranges::none_of(selected, [&match](const auto& existing) {
          return overlaps(existing, match.traversal);
        })) {
      selected.push_back(std::move(match.traversal));
    }
  }
  std::ranges::sort(selected, {}, &SelectedPassageTraversal::begin_station_m);
  return selected;
}

} // namespace drone_city_nav
