#pragma once

#include "drone_city_nav/offboard_path_follower.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <limits>
#include <span>

namespace drone_city_nav {

struct UpcomingTurn {
  bool valid{false};
  std::size_t waypoint_index{0U};
  double distance_to_turn_m{std::numeric_limits<double>::infinity()};
  double angle_rad{0.0};
  Point2 turn_point{};
};

[[nodiscard]] UpcomingTurn
upcomingTurnAtWaypoint(std::span<const Point2> path, std::size_t index,
                       Point2 current_position, bool local_position_valid,
                       const OffboardPathFollowerConfig& config);

} // namespace drone_city_nav
