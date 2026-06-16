#include "drone_city_nav/offboard_path_follower.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;

[[nodiscard]] bool finite2D(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

[[nodiscard]] bool turnWaypointIsCloseEnoughForSlowdown(
    const Point2 turn_waypoint, const Point2 current_position,
    const bool local_position_valid, const OffboardPathFollowerConfig& config,
    const double desired_speed_mps) {
  if (!local_position_valid) {
    return true;
  }

  const double activation_distance_m =
      std::max(4.0 * effectiveLookaheadDistanceM(config, desired_speed_mps),
               config.max_setpoint_distance_m + config.acceptance_radius_m);
  return distance(current_position, turn_waypoint) <= activation_distance_m;
}

} // namespace

double effectiveLookaheadDistanceM(const OffboardPathFollowerConfig& config,
                                   const double desired_speed_mps) noexcept {
  double lookahead_m = config.lookahead_distance_m;
  if (config.dynamic_lookahead_enabled) {
    lookahead_m = std::max(lookahead_m,
                           std::max(0.0, desired_speed_mps) * config.lookahead_time_s);
  }
  return std::clamp(lookahead_m, config.min_lookahead_distance_m,
                    config.max_lookahead_distance_m);
}

std::size_t closestWaypointIndex(const std::span<const Point2> path,
                                 const Point2 current_position) {
  if (path.empty() || !finite2D(current_position)) {
    return 0U;
  }

  std::size_t closest_index = 0U;
  double closest_distance_sq = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0U; i < path.size(); ++i) {
    const double distance_sq = squaredDistance(current_position, path[i]);
    if (distance_sq < closest_distance_sq) {
      closest_distance_sq = distance_sq;
      closest_index = i;
    }
  }
  return closest_index;
}

std::size_t lookaheadWaypointIndex(const std::span<const Point2> path,
                                   const Point2 current_position,
                                   const Point2 mission_goal,
                                   const OffboardPathFollowerConfig& config,
                                   const double desired_speed_mps) {
  (void)mission_goal;
  if (path.empty() || !finite2D(current_position)) {
    return 0U;
  }

  const double lookahead_distance =
      effectiveLookaheadDistanceM(config, desired_speed_mps);
  const auto projection = closestOffboardPathProjection(path, current_position);
  if (!projection.has_value()) {
    return closestWaypointIndex(path, current_position);
  }

  double remaining_lookahead_m = lookahead_distance;
  Point2 segment_start = projection->point;
  for (std::size_t i = projection->segment_start_index; i + 1U < path.size(); ++i) {
    const Point2 segment_end = path[i + 1U];
    const double segment_length_m = distance(segment_start, segment_end);
    if (segment_length_m > kTinyDistanceM) {
      if (remaining_lookahead_m <= segment_length_m) {
        return i + 1U;
      }
      remaining_lookahead_m -= segment_length_m;
    }
    segment_start = segment_end;
  }

  return path.size() - 1U;
}

std::optional<OffboardPathProjection>
closestOffboardPathProjection(const std::span<const Point2> path,
                              const Point2 current_position) {
  if (path.empty() || !finite2D(current_position)) {
    return std::nullopt;
  }
  if (path.size() == 1U) {
    return OffboardPathProjection{
        0U, 0.0, squaredDistance(current_position, path.front()), path.front()};
  }

  OffboardPathProjection best{};
  for (std::size_t i = 0U; i + 1U < path.size(); ++i) {
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

Point2 lookaheadTargetOnPath(const std::span<const Point2> path,
                             const Point2 current_position,
                             const std::size_t waypoint_index,
                             const OffboardPathFollowerConfig& config,
                             const double desired_speed_mps) {
  if (path.empty()) {
    return current_position;
  }
  if (waypoint_index >= path.size()) {
    return path.back();
  }

  return targetOnPathAtDistance(path, current_position,
                                effectiveLookaheadDistanceM(config, desired_speed_mps));
}

Point2 targetOnPathAtDistance(const std::span<const Point2> path,
                              const Point2 current_position,
                              const double path_distance_m) {
  if (path.empty()) {
    return current_position;
  }
  if (path.size() == 1U) {
    return path.front();
  }

  const auto projection = closestOffboardPathProjection(path, current_position);
  if (!projection.has_value()) {
    return path.front();
  }

  double remaining_lookahead_m = std::max(0.0, path_distance_m);
  Point2 segment_start = projection->point;
  for (std::size_t i = projection->segment_start_index; i + 1U < path.size(); ++i) {
    const Point2 segment_end = path[i + 1U];
    const double segment_length_m = distance(segment_start, segment_end);
    if (segment_length_m > kTinyDistanceM) {
      if (remaining_lookahead_m <= segment_length_m) {
        const double ratio = remaining_lookahead_m / segment_length_m;
        return Point2{segment_start.x + (segment_end.x - segment_start.x) * ratio,
                      segment_start.y + (segment_end.y - segment_start.y) * ratio};
      }
      remaining_lookahead_m -= segment_length_m;
    }
    segment_start = segment_end;
  }

  return path.back();
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
  if (const auto projection = closestOffboardPathProjection(path, current_position);
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

Point2 limitedTarget(const Point2 target, const Point2 current_position,
                     const bool local_position_valid,
                     const double max_setpoint_distance_m) {
  if (!local_position_valid) {
    return target;
  }

  const double dx = target.x - current_position.x;
  const double dy = target.y - current_position.y;
  const double target_distance = std::hypot(dx, dy);
  if (target_distance <= max_setpoint_distance_m || !(target_distance > 0.0)) {
    return target;
  }

  const double scale = max_setpoint_distance_m / target_distance;
  return Point2{current_position.x + dx * scale, current_position.y + dy * scale};
}

Point2 smoothedCommandTarget(const Point2 desired_target, const double target_step_m,
                             const bool snap_to_desired_target,
                             const Point2 current_position,
                             const bool local_position_valid,
                             const double max_setpoint_distance_m,
                             CommandTargetState& state) {
  if (!local_position_valid) {
    state.valid = false;
    return desired_target;
  }
  if (snap_to_desired_target) {
    state.target = limitedTarget(desired_target, current_position, local_position_valid,
                                 max_setpoint_distance_m);
    state.valid = true;
    return limitedTarget(state.target, current_position, local_position_valid,
                         max_setpoint_distance_m);
  }
  if (!state.valid) {
    state.target = current_position;
    state.valid = true;
  }

  const double dx = desired_target.x - state.target.x;
  const double dy = desired_target.y - state.target.y;
  const double target_step = std::hypot(dx, dy);
  if (!(target_step_m > 0.0)) {
    state.target = current_position;
    state.valid = true;
    return state.target;
  }
  if (target_step <= target_step_m || !(target_step > 0.0)) {
    state.target = desired_target;
    return limitedTarget(state.target, current_position, local_position_valid,
                         max_setpoint_distance_m);
  }

  const double scale = target_step_m / target_step;
  state.target = Point2{state.target.x + dx * scale, state.target.y + dy * scale};
  return limitedTarget(state.target, current_position, local_position_valid,
                       max_setpoint_distance_m);
}

Point2 enforceMinimumTargetLead(const Point2 command_target,
                                const Point2 desired_target,
                                const Point2 current_position,
                                const bool local_position_valid,
                                const double minimum_target_lead_m,
                                const double max_setpoint_distance_m) {
  if (!local_position_valid || !(minimum_target_lead_m > 0.0) ||
      !(max_setpoint_distance_m > 0.0)) {
    return command_target;
  }

  const Point2 desired_vector{desired_target.x - current_position.x,
                              desired_target.y - current_position.y};
  const double desired_distance_m = std::hypot(desired_vector.x, desired_vector.y);
  if (desired_distance_m <= kTinyDistanceM) {
    return command_target;
  }

  const double required_lead_m =
      std::min({minimum_target_lead_m, max_setpoint_distance_m, desired_distance_m});
  const Point2 command_vector{command_target.x - current_position.x,
                              command_target.y - current_position.y};
  const double projected_command_lead_m =
      (command_vector.x * desired_vector.x + command_vector.y * desired_vector.y) /
      desired_distance_m;
  if (projected_command_lead_m >= required_lead_m) {
    return command_target;
  }

  const double scale = required_lead_m / desired_distance_m;
  return Point2{current_position.x + (desired_target.x - current_position.x) * scale,
                current_position.y + (desired_target.y - current_position.y) * scale};
}

double pathTurnAngleAtWaypoint(const std::span<const Point2> path,
                               const std::size_t index, const Point2 current_position,
                               const bool local_position_valid,
                               const OffboardPathFollowerConfig& config,
                               const double desired_speed_mps) {
  if (path.size() < 3U || index >= path.size()) {
    return 0.0;
  }

  const Point2 previous = index == 0U && local_position_valid
                              ? current_position
                              : path[index == 0U ? 0U : index - 1U];
  const Point2 current = path[index];
  if (!turnWaypointIsCloseEnoughForSlowdown(
          current, current_position, local_position_valid, config, desired_speed_mps)) {
    return 0.0;
  }
  const Point2 next = path[std::min(index + 1U, path.size() - 1U)];

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

} // namespace drone_city_nav
