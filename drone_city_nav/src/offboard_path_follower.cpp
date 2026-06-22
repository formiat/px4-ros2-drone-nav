#include "drone_city_nav/offboard_path_follower.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;

[[nodiscard]] bool finite2D(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

[[nodiscard]] bool
turnWaypointIsWithinPreviewDistance(const double distance_to_turn_m,
                                    const OffboardPathFollowerConfig& config) {
  return distance_to_turn_m <= config.turn_preview_distance_m;
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

std::optional<OffboardPathProjection>
closestOffboardPathProjection(const std::span<const Point2> path,
                              const Point2 current_position,
                              const std::size_t minimum_segment_start_index) {
  if (path.empty() || !finite2D(current_position)) {
    return std::nullopt;
  }
  if (path.size() == 1U) {
    return OffboardPathProjection{
        0U, 0.0, squaredDistance(current_position, path.front()), path.front()};
  }

  OffboardPathProjection best{};
  const std::size_t first_segment_index =
      std::min(minimum_segment_start_index, path.size() - 2U);
  for (std::size_t i = first_segment_index; i + 1U < path.size(); ++i) {
    const Point2 segment_start = path[i];
    const Point2 segment_end = path[i + 1U];
    const Point2 segment{segment_end.x - segment_start.x,
                         segment_end.y - segment_start.y};
    const double segment_length_sq = squaredDistance(segment_start, segment_end);
    const double segment_t =
        segment_length_sq > kTinyDistanceM
            ? std::clamp(((current_position.x - segment_start.x) * segment.x +
                          (current_position.y - segment_start.y) * segment.y) /
                             segment_length_sq,
                         0.0, 1.0)
            : 0.0;
    const Point2 projected{segment_start.x + segment.x * segment_t,
                           segment_start.y + segment.y * segment_t};
    const double distance_sq = squaredDistance(current_position, projected);
    if (distance_sq < best.distance_sq) {
      best = OffboardPathProjection{i, segment_t, distance_sq, projected};
    }
  }

  return best;
}

std::size_t continuityWaypointIndex(const std::span<const Point2> path,
                                    const Point2 current_position,
                                    const Point2 previous_target,
                                    const std::size_t candidate_index,
                                    const bool had_active_target,
                                    const OffboardPathFollowerConfig& config) {
  if (!had_active_target || path.empty() || config.path_switch_hysteresis_m <= 0.0 ||
      config.path_continuity_reuse_radius_m <= 0.0 || candidate_index >= path.size()) {
    return candidate_index;
  }
  if (distance(current_position, previous_target) >
      config.path_continuity_max_target_distance_m) {
    return candidate_index;
  }

  if (distance(previous_target, path[candidate_index]) <=
      config.path_switch_hysteresis_m) {
    return candidate_index;
  }

  std::size_t closest_index = candidate_index;
  double closest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0U; i < path.size(); ++i) {
    const double waypoint_distance = distance(previous_target, path[i]);
    if (waypoint_distance < closest_distance) {
      closest_distance = waypoint_distance;
      closest_index = i;
    }
  }

  if (closest_distance <= config.path_continuity_reuse_radius_m) {
    return closest_index;
  }
  return candidate_index;
}

std::size_t advanceWaypointIndex(const std::span<const Point2> path,
                                 const Point2 current_position,
                                 const std::size_t waypoint_index,
                                 const OffboardPathFollowerConfig& config) {
  if (path.empty() || !finite2D(current_position)) {
    return 0U;
  }

  std::size_t next_index = std::min(waypoint_index, path.size() - 1U);
  const std::size_t minimum_segment_index = next_index > 0U ? next_index - 1U : 0U;
  if (const auto projection =
          closestOffboardPathProjection(path, current_position, minimum_segment_index);
      projection.has_value()) {
    next_index = std::max(
        next_index, std::min(projection->segment_start_index + 1U, path.size() - 1U));
  }
  while (next_index + 1U < path.size()) {
    if (distance(current_position, path[next_index]) > config.acceptance_radius_m) {
      break;
    }
    ++next_index;
  }

  return next_index;
}

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
