#pragma once

#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <limits>
#include <optional>
#include <span>

namespace drone_city_nav {

struct OffboardPathFollowerConfig {
  double acceptance_radius_m{1.5};
  double turn_preview_distance_m{32.0};
};

struct OffboardPathProjection {
  std::size_t segment_start_index{0U};
  double segment_t{0.0};
  double distance_sq{std::numeric_limits<double>::infinity()};
  Point2 point{};
};

struct UpcomingTurn {
  bool valid{false};
  std::size_t waypoint_index{0U};
  double distance_to_turn_m{std::numeric_limits<double>::infinity()};
  double angle_rad{0.0};
  Point2 turn_point{};
};

[[nodiscard]] std::optional<OffboardPathProjection>
closestOffboardPathProjection(std::span<const Point2> path, Point2 current_position,
                              std::size_t minimum_segment_start_index = 0U);

[[nodiscard]] std::size_t
advanceWaypointIndex(std::span<const Point2> path, Point2 current_position,
                     std::size_t waypoint_index,
                     const OffboardPathFollowerConfig& config);

[[nodiscard]] UpcomingTurn
upcomingTurnAtWaypoint(std::span<const Point2> path, std::size_t index,
                       Point2 current_position, bool local_position_valid,
                       const OffboardPathFollowerConfig& config);

} // namespace drone_city_nav
