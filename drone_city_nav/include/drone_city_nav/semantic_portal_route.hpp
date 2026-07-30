#pragma once

#include "drone_city_nav/known_passage_map.hpp"
#include "drone_city_nav/types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace drone_city_nav {

using PortalId = std::string;

struct PortalPlane {
  Point2 point{};
  Point2 normal{1.0, 0.0};
};

struct Portal {
  PortalId id;
  std::string structure_id;
  PortalPlane entry_plane{};
  PortalPlane exit_plane{};
  std::array<Point2, 4U> opening_polygon{};
  Point3 center{};
  Point2 normal_xy{1.0, 0.0};
  double width_m{0.0};
  double depth_m{0.0};
  double min_z_m{0.0};
  double max_z_m{0.0};
};

struct SemanticPortalPrimitive {
  PortalId id;
  Point2 center{};
  Point2 normal_xy{1.0, 0.0};
  double width_m{0.0};
  double depth_m{0.0};
};

struct RoutePassageEvent {
  Portal portal{};
  double approach_station_m{0.0};
  double entry_station_m{0.0};
  double exit_station_m{0.0};
  double departure_station_m{0.0};
  int traversal_direction{1};
  double preferred_z_m{0.0};
  double speed_limit_mps{0.0};
};

enum class SemanticRouteSegmentType : std::uint8_t {
  kNormal,
  kPortalApproach,
  kPortalTraversal,
  kPortalExit,
};

struct SemanticRouteSegment {
  SemanticRouteSegmentType type{SemanticRouteSegmentType::kNormal};
  double start_station_m{0.0};
  double end_station_m{0.0};
  std::optional<PortalId> portal_id;
};

struct SemanticPortalRoute {
  std::uint64_t generation{0U};
  std::shared_ptr<const std::vector<Point2>> polyline;
  std::vector<double> point_stations_m;
  std::vector<RoutePassageEvent> passage_events;
  std::vector<SemanticRouteSegment> segments;
  double total_length_m{0.0};
};

struct SemanticPortalRouteConfig {
  double crossing_lateral_margin_m{0.5};
  double minimum_normal_alignment{0.35};
};

struct SemanticPortalRouteBuildResult {
  std::shared_ptr<const SemanticPortalRoute> route;
  std::size_t portals_considered{0U};
  std::size_t portal_events_created{0U};
  std::size_t rejected_invalid_geometry{0U};
  std::size_t rejected_route_miss{0U};
  std::size_t rejected_overlap{0U};
};

[[nodiscard]] SemanticPortalRouteBuildResult
buildSemanticPortalRoute(std::shared_ptr<const std::vector<Point2>> polyline,
                         std::uint64_t generation, const KnownPassageMap& passage_map,
                         double passage_speed_limit_mps,
                         const SemanticPortalRouteConfig& config = {});

[[nodiscard]] std::vector<SemanticPortalPrimitive>
semanticPortalPrimitives(const KnownPassageMap& passage_map);

[[nodiscard]] double semanticRouteZReference(const SemanticPortalRoute& route,
                                             double station_m,
                                             double normal_flight_z_m) noexcept;

[[nodiscard]] double routePassageZReference(const RoutePassageEvent& event,
                                            double station_m, double normal_flight_z_m,
                                            double approach_station_m) noexcept;

[[nodiscard]] double routePassageZReference(const RoutePassageEvent& event,
                                            double station_m, double normal_flight_z_m,
                                            double approach_station_m,
                                            double alignment_station_m) noexcept;

[[nodiscard]] const RoutePassageEvent*
nextRoutePassageEvent(const SemanticPortalRoute& route, double station_m,
                      std::size_t minimum_event_index = 0U) noexcept;

[[nodiscard]] const char*
semanticRouteSegmentTypeName(SemanticRouteSegmentType type) noexcept;

} // namespace drone_city_nav
