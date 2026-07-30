#include "drone_city_nav/semantic_portal_route.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr double kEpsilon{1.0e-9};

[[nodiscard]] double dot(const Point2 first, const Point2 second) noexcept {
  return first.x * second.x + first.y * second.y;
}

[[nodiscard]] Point2 difference(const Point2 minuend,
                                const Point2 subtrahend) noexcept {
  return Point2{minuend.x - subtrahend.x, minuend.y - subtrahend.y};
}

[[nodiscard]] Point2 addScaled(const Point2 point, const Point2 direction,
                               const double scale) noexcept {
  return Point2{point.x + scale * direction.x, point.y + scale * direction.y};
}

[[nodiscard]] double longitudinal(const Point2 point,
                                  const PassageOpening& opening) noexcept {
  return dot(difference(point, Point2{opening.center.x, opening.center.y}),
             opening.normal_xy);
}

[[nodiscard]] double lateral(const Point2 point,
                             const PassageOpening& opening) noexcept {
  const Point2 offset = difference(point, Point2{opening.center.x, opening.center.y});
  return -offset.x * opening.normal_xy.y + offset.y * opening.normal_xy.x;
}

[[nodiscard]] std::vector<double>
pointStations(const std::span<const Point2> polyline) {
  std::vector<double> stations(polyline.size(), 0.0);
  for (std::size_t index = 1U; index < polyline.size(); ++index) {
    stations[index] =
        stations[index - 1U] + distance(polyline[index - 1U], polyline[index]);
  }
  return stations;
}

[[nodiscard]] Portal portalFromOpening(const PassageOpening& opening,
                                       const int direction) {
  const Point2 center{opening.center.x, opening.center.y};
  const Point2 travel_normal{static_cast<double>(direction) * opening.normal_xy.x,
                             static_cast<double>(direction) * opening.normal_xy.y};
  const Point2 lateral_axis{-travel_normal.y, travel_normal.x};
  const double half_depth = 0.5 * opening.depth_m;
  const double half_width = 0.5 * opening.width_m;
  const Point2 entry = addScaled(center, travel_normal, -half_depth);
  const Point2 exit = addScaled(center, travel_normal, half_depth);
  return Portal{
      .id = opening.id,
      .structure_id = opening.structure_id,
      .entry_plane = PortalPlane{entry, travel_normal},
      .exit_plane = PortalPlane{exit, travel_normal},
      .opening_polygon =
          {
              addScaled(entry, lateral_axis, -half_width),
              addScaled(entry, lateral_axis, half_width),
              addScaled(exit, lateral_axis, half_width),
              addScaled(exit, lateral_axis, -half_width),
          },
      .center = opening.center,
      .normal_xy = travel_normal,
      .width_m = opening.width_m,
      .depth_m = opening.depth_m,
      .min_z_m = opening.min_z_m,
      .max_z_m = opening.max_z_m,
  };
}

[[nodiscard]] bool openingGeometryIsValid(const PassageOpening& opening) noexcept {
  const double normal_length = std::hypot(opening.normal_xy.x, opening.normal_xy.y);
  return std::isfinite(opening.center.x) && std::isfinite(opening.center.y) &&
         std::isfinite(opening.min_z_m) && std::isfinite(opening.max_z_m) &&
         std::isfinite(opening.width_m) && std::isfinite(opening.depth_m) &&
         std::isfinite(opening.approach_distance_m) &&
         std::isfinite(opening.exit_distance_m) &&
         std::abs(normal_length - 1.0) <= 1.0e-6 && opening.width_m > 0.0 &&
         opening.depth_m > 0.0 && opening.max_z_m > opening.min_z_m &&
         opening.approach_distance_m > 0.0 && opening.exit_distance_m > 0.0;
}

struct PortalPlaneCrossing {
  std::size_t segment_index{0U};
  double station_m{0.0};
};

[[nodiscard]] std::optional<PortalPlaneCrossing> findDirectedPlaneCrossing(
    const std::span<const Point2> polyline, const std::span<const double> stations,
    const PassageOpening& opening, const double plane_longitudinal_m,
    const int direction, const double minimum_station_m,
    const double usable_half_width_m,
    const SemanticPortalRouteConfig& config) noexcept {
  for (std::size_t index = 0U; index + 1U < polyline.size(); ++index) {
    if (stations[index + 1U] < minimum_station_m - kEpsilon) {
      continue;
    }
    const Point2 delta = difference(polyline[index + 1U], polyline[index]);
    const double segment_length = std::hypot(delta.x, delta.y);
    if (!(segment_length > kEpsilon)) {
      continue;
    }
    const double signed_alignment = dot(delta, opening.normal_xy) / segment_length;
    if (static_cast<double>(direction) * signed_alignment <
        config.minimum_normal_alignment) {
      continue;
    }
    const double first_longitudinal = longitudinal(polyline[index], opening);
    const double second_longitudinal = longitudinal(polyline[index + 1U], opening);
    const double first_side =
        static_cast<double>(direction) * (first_longitudinal - plane_longitudinal_m);
    const double second_side =
        static_cast<double>(direction) * (second_longitudinal - plane_longitudinal_m);
    if (first_side > kEpsilon || second_side < -kEpsilon ||
        std::abs(second_longitudinal - first_longitudinal) <= kEpsilon) {
      continue;
    }
    const double ratio = std::clamp((plane_longitudinal_m - first_longitudinal) /
                                        (second_longitudinal - first_longitudinal),
                                    0.0, 1.0);
    const double station = stations[index] + ratio * segment_length;
    if (station < minimum_station_m - kEpsilon) {
      continue;
    }
    const Point2 crossing{
        std::lerp(polyline[index].x, polyline[index + 1U].x, ratio),
        std::lerp(polyline[index].y, polyline[index + 1U].y, ratio),
    };
    if (std::abs(lateral(crossing, opening)) > usable_half_width_m) {
      continue;
    }
    return PortalPlaneCrossing{.segment_index = index, .station_m = station};
  }
  return std::nullopt;
}

[[nodiscard]] bool traversalStaysInsidePortal(
    const std::span<const Point2> polyline, const std::span<const double> stations,
    const PassageOpening& opening, const double entry_station_m,
    const double exit_station_m, const double usable_half_width_m) noexcept {
  const double half_depth_m = 0.5 * opening.depth_m;
  for (std::size_t index = 0U; index < polyline.size(); ++index) {
    if (stations[index] <= entry_station_m + kEpsilon ||
        stations[index] >= exit_station_m - kEpsilon) {
      continue;
    }
    if (std::abs(longitudinal(polyline[index], opening)) > half_depth_m + kEpsilon ||
        std::abs(lateral(polyline[index], opening)) > usable_half_width_m + kEpsilon) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] std::optional<RoutePassageEvent> routeEventForDirection(
    const std::span<const Point2> polyline, const std::span<const double> stations,
    const PassageOpening& opening, const double passage_speed_limit_mps,
    const SemanticPortalRouteConfig& config, const int direction,
    const double usable_half_width_m) {
  const double half_depth_m = 0.5 * opening.depth_m;
  const double entry_longitudinal_m = -static_cast<double>(direction) * half_depth_m;
  const double exit_longitudinal_m = static_cast<double>(direction) * half_depth_m;
  const double initial_longitudinal = longitudinal(polyline.front(), opening);
  const bool starts_inside =
      std::abs(initial_longitudinal) <= half_depth_m + kEpsilon &&
      std::abs(lateral(polyline.front(), opening)) <= usable_half_width_m;

  double entry_station_m = 0.0;
  if (!starts_inside) {
    const std::optional<PortalPlaneCrossing> entry =
        findDirectedPlaneCrossing(polyline, stations, opening, entry_longitudinal_m,
                                  direction, 0.0, usable_half_width_m, config);
    if (!entry.has_value()) {
      return std::nullopt;
    }
    entry_station_m = entry->station_m;
  }
  const std::optional<PortalPlaneCrossing> exit = findDirectedPlaneCrossing(
      polyline, stations, opening, exit_longitudinal_m, direction,
      entry_station_m + kEpsilon, usable_half_width_m, config);
  if (!exit.has_value() ||
      !traversalStaysInsidePortal(polyline, stations, opening, entry_station_m,
                                  exit->station_m, usable_half_width_m)) {
    return std::nullopt;
  }

  return RoutePassageEvent{
      .portal = portalFromOpening(opening, direction),
      .approach_station_m =
          std::max(0.0, entry_station_m - opening.approach_distance_m),
      .entry_station_m = entry_station_m,
      .exit_station_m = exit->station_m,
      .departure_station_m =
          std::min(stations.back(), exit->station_m + opening.exit_distance_m),
      .traversal_direction = direction,
      .preferred_z_m = 0.5 * (opening.min_z_m + opening.max_z_m),
      .speed_limit_mps = passage_speed_limit_mps,
  };
}

[[nodiscard]] std::optional<RoutePassageEvent> routeEventForOpening(
    const std::span<const Point2> polyline, const std::span<const double> stations,
    const PassageOpening& opening, const double passage_speed_limit_mps,
    const SemanticPortalRouteConfig& config) {
  const double usable_half_width =
      0.5 * opening.width_m - config.crossing_lateral_margin_m;
  if (!(usable_half_width > 0.0)) {
    return std::nullopt;
  }
  std::optional<RoutePassageEvent> forward =
      routeEventForDirection(polyline, stations, opening, passage_speed_limit_mps,
                             config, 1, usable_half_width);
  std::optional<RoutePassageEvent> reverse =
      routeEventForDirection(polyline, stations, opening, passage_speed_limit_mps,
                             config, -1, usable_half_width);
  if (!forward.has_value()) {
    return reverse;
  }
  if (!reverse.has_value()) {
    return forward;
  }
  return forward->entry_station_m <= reverse->entry_station_m ? forward : reverse;
}

[[nodiscard]] SemanticRouteSegmentType
segmentTypeAt(const std::span<const RoutePassageEvent> events, const double station,
              std::optional<PortalId>& portal_id) {
  for (const RoutePassageEvent& event : events) {
    if (station < event.approach_station_m || station > event.departure_station_m) {
      continue;
    }
    portal_id = event.portal.id;
    if (station < event.entry_station_m) {
      return SemanticRouteSegmentType::kPortalApproach;
    }
    if (station <= event.exit_station_m) {
      return SemanticRouteSegmentType::kPortalTraversal;
    }
    return SemanticRouteSegmentType::kPortalExit;
  }
  portal_id.reset();
  return SemanticRouteSegmentType::kNormal;
}

[[nodiscard]] std::vector<SemanticRouteSegment>
buildSegments(const double total_length_m,
              const std::span<const RoutePassageEvent> events) {
  std::vector<double> boundaries{0.0, total_length_m};
  boundaries.reserve(2U + 4U * events.size());
  for (const RoutePassageEvent& event : events) {
    boundaries.push_back(event.approach_station_m);
    boundaries.push_back(event.entry_station_m);
    boundaries.push_back(event.exit_station_m);
    boundaries.push_back(event.departure_station_m);
  }
  std::ranges::sort(boundaries);
  const auto unique_end =
      std::ranges::unique(boundaries, [](const double first, const double second) {
        return std::abs(first - second) <= kEpsilon;
      });
  boundaries.erase(unique_end.begin(), unique_end.end());

  std::vector<SemanticRouteSegment> segments;
  for (std::size_t index = 0U; index + 1U < boundaries.size(); ++index) {
    if (!(boundaries[index + 1U] > boundaries[index] + kEpsilon)) {
      continue;
    }
    std::optional<PortalId> portal_id;
    const SemanticRouteSegmentType type = segmentTypeAt(
        events, 0.5 * (boundaries[index] + boundaries[index + 1U]), portal_id);
    segments.push_back(SemanticRouteSegment{
        .type = type,
        .start_station_m = boundaries[index],
        .end_station_m = boundaries[index + 1U],
        .portal_id = std::move(portal_id),
    });
  }
  return segments;
}

} // namespace

SemanticPortalRouteBuildResult buildSemanticPortalRoute(
    std::shared_ptr<const std::vector<Point2>> polyline, const std::uint64_t generation,
    const KnownPassageMap& passage_map, const double passage_speed_limit_mps,
    const SemanticPortalRouteConfig& config) {
  if (!polyline || polyline->size() < 2U ||
      !(config.crossing_lateral_margin_m >= 0.0) ||
      !(config.minimum_normal_alignment >= 0.0) ||
      !(config.minimum_normal_alignment <= 1.0) || !(passage_speed_limit_mps >= 0.0) ||
      !std::isfinite(passage_speed_limit_mps)) {
    throw std::invalid_argument{"invalid semantic portal route input"};
  }

  SemanticPortalRouteBuildResult result;
  std::vector<double> stations = pointStations(*polyline);
  std::vector<RoutePassageEvent> events;
  for (const PassageStructure& structure : passage_map.structures) {
    for (const PassageOpening& opening : structure.openings) {
      ++result.portals_considered;
      if (!openingGeometryIsValid(opening)) {
        ++result.rejected_invalid_geometry;
        continue;
      }
      const std::optional<RoutePassageEvent> event = routeEventForOpening(
          *polyline, stations, opening, passage_speed_limit_mps, config);
      if (!event.has_value()) {
        ++result.rejected_route_miss;
        continue;
      }
      events.push_back(*event);
    }
  }
  std::ranges::sort(
      events, [](const RoutePassageEvent& first, const RoutePassageEvent& second) {
        if (first.entry_station_m != second.entry_station_m) {
          return first.entry_station_m < second.entry_station_m;
        }
        return first.portal.id < second.portal.id;
      });

  std::vector<RoutePassageEvent> non_overlapping;
  non_overlapping.reserve(events.size());
  for (RoutePassageEvent& event : events) {
    if (!non_overlapping.empty() &&
        event.approach_station_m <
            non_overlapping.back().departure_station_m - kEpsilon) {
      ++result.rejected_overlap;
      continue;
    }
    non_overlapping.push_back(std::move(event));
  }

  auto route = std::make_shared<SemanticPortalRoute>();
  route->generation = generation;
  route->polyline = std::move(polyline);
  route->point_stations_m = std::move(stations);
  route->passage_events = std::move(non_overlapping);
  route->total_length_m = route->point_stations_m.back();
  route->segments = buildSegments(route->total_length_m, route->passage_events);
  result.portal_events_created = route->passage_events.size();
  result.route = std::move(route);
  return result;
}

std::vector<SemanticPortalPrimitive>
semanticPortalPrimitives(const KnownPassageMap& passage_map) {
  std::vector<SemanticPortalPrimitive> primitives;
  for (const PassageStructure& structure : passage_map.structures) {
    for (const PassageOpening& opening : structure.openings) {
      if (!openingGeometryIsValid(opening)) {
        continue;
      }
      primitives.push_back(SemanticPortalPrimitive{
          .id = opening.id,
          .center = Point2{opening.center.x, opening.center.y},
          .normal_xy = opening.normal_xy,
          .width_m = opening.width_m,
          .depth_m = opening.depth_m,
      });
    }
  }
  return primitives;
}

double semanticRouteZReference(const SemanticPortalRoute& route, const double station_m,
                               const double normal_flight_z_m) noexcept {
  const double station = std::clamp(station_m, 0.0, route.total_length_m);
  for (const RoutePassageEvent& event : route.passage_events) {
    if (station < event.approach_station_m || station > event.departure_station_m) {
      continue;
    }
    return routePassageZReference(event, station, normal_flight_z_m,
                                  event.approach_station_m);
  }
  return normal_flight_z_m;
}

double routePassageZReference(const RoutePassageEvent& event, const double station_m,
                              const double normal_flight_z_m,
                              const double approach_station_m) noexcept {
  return routePassageZReference(event, station_m, normal_flight_z_m, approach_station_m,
                                event.entry_station_m);
}

double routePassageZReference(const RoutePassageEvent& event, const double station_m,
                              const double normal_flight_z_m,
                              const double approach_station_m,
                              const double alignment_station_m) noexcept {
  const double approach = std::min(approach_station_m, event.entry_station_m);
  const double alignment =
      std::clamp(alignment_station_m, approach, event.entry_station_m);
  if (station_m < approach || station_m > event.departure_station_m) {
    return normal_flight_z_m;
  }
  const auto smoothStep = [](const double ratio) noexcept {
    const double value = std::clamp(ratio, 0.0, 1.0);
    return value * value * value * (value * (value * 6.0 - 15.0) + 10.0);
  };
  if (station_m < alignment) {
    const double length = alignment - approach;
    const double ratio = length > kEpsilon ? (station_m - approach) / length : 1.0;
    return std::lerp(normal_flight_z_m, event.preferred_z_m, smoothStep(ratio));
  }
  if (station_m <= event.exit_station_m) {
    return event.preferred_z_m;
  }
  const double length = event.departure_station_m - event.exit_station_m;
  const double ratio =
      length > kEpsilon ? (station_m - event.exit_station_m) / length : 1.0;
  return std::lerp(event.preferred_z_m, normal_flight_z_m, smoothStep(ratio));
}

const RoutePassageEvent*
nextRoutePassageEvent(const SemanticPortalRoute& route, const double station_m,
                      const std::size_t minimum_event_index) noexcept {
  for (std::size_t index = minimum_event_index; index < route.passage_events.size();
       ++index) {
    if (station_m <= route.passage_events[index].exit_station_m) {
      return &route.passage_events[index];
    }
  }
  return nullptr;
}

const char* semanticRouteSegmentTypeName(const SemanticRouteSegmentType type) noexcept {
  switch (type) {
    case SemanticRouteSegmentType::kNormal:
      return "normal";
    case SemanticRouteSegmentType::kPortalApproach:
      return "portal_approach";
    case SemanticRouteSegmentType::kPortalTraversal:
      return "portal_traversal";
    case SemanticRouteSegmentType::kPortalExit:
      return "portal_exit";
  }
  return "unknown";
}

} // namespace drone_city_nav
