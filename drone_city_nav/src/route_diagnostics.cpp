#include "drone_city_nav/route_diagnostics.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;

[[nodiscard]] bool
turnWaypointIsWithinPreviewDistance(const double distance_to_turn_m,
                                    const OffboardPathFollowerConfig& config) {
  return distance_to_turn_m <= config.diagnostic_turn_preview_distance_m;
}

[[nodiscard]] double turnAngleRad(const Point2 previous, const Point2 current,
                                  const Point2 next) noexcept {
  const Point2 incoming{current.x - previous.x, current.y - previous.y};
  const Point2 outgoing{next.x - current.x, next.y - current.y};
  const double incoming_length = std::hypot(incoming.x, incoming.y);
  const double outgoing_length = std::hypot(outgoing.x, outgoing.y);
  if (incoming_length <= kTinyDistanceM || outgoing_length <= kTinyDistanceM) {
    return 0.0;
  }

  const double cosine = std::clamp((incoming.x * outgoing.x + incoming.y * outgoing.y) /
                                       (incoming_length * outgoing_length),
                                   -1.0, 1.0);
  return std::acos(cosine);
}

} // namespace

UpcomingTurn upcomingTurnAtWaypoint(const std::span<const Point2> path,
                                    const std::size_t index,
                                    const Point2 current_position,
                                    const bool local_position_valid,
                                    const OffboardPathFollowerConfig& config) {
  UpcomingTurn turn{};
  if (path.size() < 3U || index >= path.size()) {
    return turn;
  }

  double distance_to_candidate_m =
      local_position_valid ? distance(current_position, path[index]) : 0.0;
  for (std::size_t candidate_index = index; candidate_index + 1U < path.size();
       ++candidate_index) {
    const Point2 previous =
        candidate_index == 0U && local_position_valid
            ? current_position
            : path[candidate_index == 0U ? 0U : candidate_index - 1U];
    const Point2 current = path[candidate_index];
    const Point2 next = path[candidate_index + 1U];
    const double angle = turnAngleRad(previous, current, next);
    if (angle > kTinyDistanceM) {
      if (!turnWaypointIsWithinPreviewDistance(distance_to_candidate_m, config)) {
        return turn;
      }
      turn.valid = true;
      turn.waypoint_index = candidate_index;
      turn.distance_to_turn_m = distance_to_candidate_m;
      turn.angle_rad = angle;
      turn.turn_point = current;
      return turn;
    }

    distance_to_candidate_m += distance(path[candidate_index], next);
  }

  return turn;
}

} // namespace drone_city_nav
