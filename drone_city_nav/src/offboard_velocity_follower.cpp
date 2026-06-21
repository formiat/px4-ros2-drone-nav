#include "drone_city_nav/offboard_velocity_follower.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;

[[nodiscard]] bool finite2D(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

[[nodiscard]] Point2 operator+(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x + rhs.x, lhs.y + rhs.y};
}

[[nodiscard]] Point2 operator-(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x - rhs.x, lhs.y - rhs.y};
}

[[nodiscard]] Point2 operator*(const Point2 point, const double scale) noexcept {
  return Point2{point.x * scale, point.y * scale};
}

[[nodiscard]] double norm(const Point2 point) noexcept {
  return std::hypot(point.x, point.y);
}

[[nodiscard]] Point2 normalized(const Point2 point) noexcept {
  const double length = norm(point);
  if (!(length > kTinyDistanceM)) {
    return Point2{};
  }
  return Point2{point.x / length, point.y / length};
}

[[nodiscard]] bool pathIsFinite(const std::span<const Point2> path) noexcept {
  return std::ranges::all_of(path, finite2D);
}

[[nodiscard]] std::optional<OffboardPathProjection>
closestUsableVelocityProjection(const std::span<const Point2> path,
                                const Point2 current_position,
                                const std::size_t waypoint_index) {
  if (path.size() < 2U || !finite2D(current_position) || !pathIsFinite(path)) {
    return std::nullopt;
  }

  const std::size_t first_segment =
      waypoint_index > 0U ? std::min(waypoint_index - 1U, path.size() - 2U) : 0U;
  std::optional<OffboardPathProjection> best;
  for (std::size_t index = first_segment; index + 1U < path.size(); ++index) {
    const Point2 segment_start = path[index];
    const Point2 segment_end = path[index + 1U];
    const Point2 segment = segment_end - segment_start;
    const double segment_length_sq = squaredDistance(segment_start, segment_end);
    if (segment_length_sq <= kTinyDistanceM * kTinyDistanceM) {
      continue;
    }

    const double segment_t =
        std::clamp(((current_position.x - segment_start.x) * segment.x +
                    (current_position.y - segment_start.y) * segment.y) /
                       segment_length_sq,
                   0.0, 1.0);
    const Point2 projected{segment_start.x + segment.x * segment_t,
                           segment_start.y + segment.y * segment_t};
    const double distance_sq = squaredDistance(current_position, projected);
    if (!best.has_value() || distance_sq < best->distance_sq) {
      best = OffboardPathProjection{index, segment_t, distance_sq, projected};
    }
  }

  return best;
}

[[nodiscard]] double turnAngleRad(const Point2 previous, const Point2 current,
                                  const Point2 next) noexcept {
  const Point2 incoming = current - previous;
  const Point2 outgoing = next - current;
  const double incoming_length = norm(incoming);
  const double outgoing_length = norm(outgoing);
  if (incoming_length <= kTinyDistanceM || outgoing_length <= kTinyDistanceM) {
    return 0.0;
  }

  const double cosine = std::clamp((incoming.x * outgoing.x + incoming.y * outgoing.y) /
                                       (incoming_length * outgoing_length),
                                   -1.0, 1.0);
  return std::acos(cosine);
}

[[nodiscard]] double sanitizedPositive(const double value, const double fallback,
                                       const double min_value,
                                       const double max_value) noexcept {
  if (!std::isfinite(value)) {
    return fallback;
  }
  return std::clamp(value, min_value, max_value);
}

[[nodiscard]] double currentSpeedMps(const Point2 current_velocity,
                                     const bool current_velocity_valid,
                                     const VelocityFollowerState& previous_state) {
  if (current_velocity_valid && finite2D(current_velocity)) {
    return norm(current_velocity);
  }
  if (previous_state.previous_velocity_setpoint_valid &&
      finite2D(previous_state.previous_velocity_setpoint)) {
    return norm(previous_state.previous_velocity_setpoint);
  }
  return 0.0;
}

[[nodiscard]] double
effectiveVelocityDeltaAccelMps2(const VelocityFollowerConfig& config) {
  const double max_accel = sanitizedPositive(config.max_accel_mps2, 3.0, 1.0e-6, 100.0);
  const double max_lateral =
      sanitizedPositive(config.max_lateral_accel_mps2, 3.0, 1.0e-6, 100.0);
  return std::min(max_accel, max_lateral);
}

[[nodiscard]] double
effectiveVelocityDeltaDecelMps2(const VelocityFollowerConfig& config) {
  const double max_decel = sanitizedPositive(config.max_decel_mps2, 4.0, 1.0e-6, 100.0);
  return std::min(max_decel, effectiveVelocityDeltaAccelMps2(config));
}

[[nodiscard]] double targetTurnSpeedMps(const double angle_rad,
                                        const VelocityFollowerConfig& config) {
  const double cruise_speed =
      sanitizedPositive(config.cruise_speed_mps, 12.0, 0.0, 100.0);
  const double min_turn_speed = std::min(
      sanitizedPositive(config.min_turn_speed_mps, 2.0, 0.0, 100.0), cruise_speed);
  const double min_angle =
      sanitizedPositive(config.turn_slowdown_min_angle_rad, 0.25, 0.0, 3.2);
  const double sharp_angle =
      std::max(sanitizedPositive(config.sharp_turn_angle_rad, 1.5707963267948966,
                                 min_angle + 1.0e-6, 3.2),
               min_angle + 1.0e-6);

  if (angle_rad <= min_angle) {
    return cruise_speed;
  }
  if (angle_rad >= sharp_angle) {
    return min_turn_speed;
  }

  const double ratio = (angle_rad - min_angle) / (sharp_angle - min_angle);
  return cruise_speed + (min_turn_speed - cruise_speed) * ratio;
}

[[nodiscard]] Point2 boundedCrossTrackCorrection(Point2 correction_velocity,
                                                 const double base_speed_mps,
                                                 const VelocityFollowerConfig& config) {
  const double max_angle =
      sanitizedPositive(config.max_cross_track_correction_angle_rad, 0.35, 0.0, 1.4);
  const double max_correction_mps = std::max(base_speed_mps, 1.0) * std::tan(max_angle);
  const double correction_speed = norm(correction_velocity);
  if (correction_speed <= max_correction_mps || !(correction_speed > kTinyDistanceM)) {
    return correction_velocity;
  }
  return correction_velocity * (max_correction_mps / correction_speed);
}

[[nodiscard]] VelocitySetpointReason
reasonFromTurnPlan(const TurnSpeedPlan& turn, const VelocityFollowerConfig& config) {
  if (!turn.valid) {
    return VelocitySetpointReason::kStraight;
  }
  if (turn.angle_rad < config.turn_slowdown_min_angle_rad) {
    return VelocitySetpointReason::kGentleTurn;
  }
  if (turn.raw_speed_limit_mps + 1.0e-6 < config.cruise_speed_mps) {
    return VelocitySetpointReason::kBrakingForTurn;
  }
  return VelocitySetpointReason::kStraight;
}

} // namespace

const char* velocitySetpointReasonName(const VelocitySetpointReason reason) noexcept {
  switch (reason) {
    case VelocitySetpointReason::kInvalidPath:
      return "invalid_path";
    case VelocitySetpointReason::kHold:
      return "hold";
    case VelocitySetpointReason::kStraight:
      return "straight";
    case VelocitySetpointReason::kGentleTurn:
      return "gentle_turn";
    case VelocitySetpointReason::kBrakingForTurn:
      return "braking_for_turn";
  }
  return "unknown";
}

double distanceFromProjectionToWaypoint(const std::span<const Point2> path,
                                        const OffboardPathProjection& projection,
                                        const std::size_t waypoint_index) {
  if (path.size() < 2U || waypoint_index >= path.size() ||
      projection.segment_start_index + 1U >= path.size() ||
      !finite2D(projection.point) || !pathIsFinite(path)) {
    return std::numeric_limits<double>::infinity();
  }
  if (waypoint_index <= projection.segment_start_index) {
    return 0.0;
  }

  double result = distance(projection.point, path[projection.segment_start_index + 1U]);
  for (std::size_t index = projection.segment_start_index + 1U;
       index + 1U <= waypoint_index && index + 1U < path.size(); ++index) {
    result += distance(path[index], path[index + 1U]);
  }
  return result;
}

TurnSpeedPlan speedLimitForUpcomingTurn(const std::span<const Point2> path,
                                        const OffboardPathProjection& projection,
                                        const double current_speed_mps,
                                        const VelocityFollowerConfig& config) {
  TurnSpeedPlan plan{};
  if (path.size() < 3U || projection.segment_start_index + 1U >= path.size() ||
      !pathIsFinite(path)) {
    return plan;
  }

  const double cruise_speed =
      sanitizedPositive(config.cruise_speed_mps, 12.0, 0.0, 100.0);
  const double preview_distance =
      sanitizedPositive(config.turn_preview_distance_m, 32.0, 0.0, 1000.0);
  const double min_angle =
      sanitizedPositive(config.turn_slowdown_min_angle_rad, 0.25, 0.0, 3.2);
  const double current_speed = std::max(0.0, current_speed_mps);

  const std::size_t first_turn_index =
      std::max<std::size_t>(1U, projection.segment_start_index + 1U);
  for (std::size_t index = first_turn_index; index + 1U < path.size(); ++index) {
    const double angle = turnAngleRad(path[index - 1U], path[index], path[index + 1U]);
    const double distance_to_turn =
        distanceFromProjectionToWaypoint(path, projection, index);
    if (distance_to_turn > preview_distance) {
      return plan;
    }
    if (angle <= kTinyDistanceM) {
      continue;
    }

    plan.valid = true;
    plan.waypoint_index = index;
    plan.angle_rad = angle;
    plan.distance_to_turn_m = distance_to_turn;
    plan.target_turn_speed_mps = targetTurnSpeedMps(angle, config);

    const double max_decel = effectiveVelocityDeltaDecelMps2(config);
    const double braking_margin =
        sanitizedPositive(config.braking_margin_m, 2.0, 0.0, 100.0);
    plan.braking_distance_m =
        std::max(0.0, (current_speed * current_speed -
                       plan.target_turn_speed_mps * plan.target_turn_speed_mps) /
                          (2.0 * max_decel)) +
        braking_margin;
    if (angle < min_angle || distance_to_turn > plan.braking_distance_m) {
      plan.raw_speed_limit_mps = cruise_speed;
      return plan;
    }

    const double braking_distance_without_margin =
        std::max(0.0, distance_to_turn - braking_margin);
    plan.raw_speed_limit_mps = std::min(
        cruise_speed, std::sqrt(std::max(
                          0.0, plan.target_turn_speed_mps * plan.target_turn_speed_mps +
                                   2.0 * max_decel * braking_distance_without_margin)));
    return plan;
  }

  return plan;
}

VelocityVectorLimitResult limitVelocityVectorDelta(const Point2 desired_velocity,
                                                   const Point2 previous_velocity,
                                                   const bool previous_velocity_valid,
                                                   const double dt_s,
                                                   const double max_delta_mps2) {
  VelocityVectorLimitResult result{};
  result.velocity = desired_velocity;
  if (!previous_velocity_valid || !finite2D(previous_velocity) ||
      !finite2D(desired_velocity)) {
    result.delta_mps = std::numeric_limits<double>::quiet_NaN();
    return result;
  }

  const double dt = sanitizedPositive(dt_s, 0.1, 0.0, 10.0);
  const double max_delta = sanitizedPositive(max_delta_mps2, 3.0, 0.0, 100.0) * dt;
  const Point2 delta = desired_velocity - previous_velocity;
  const double delta_norm = norm(delta);
  result.delta_mps = delta_norm;
  if (delta_norm <= max_delta || !(delta_norm > kTinyDistanceM)) {
    return result;
  }

  result.velocity = previous_velocity + delta * (max_delta / delta_norm);
  result.delta_mps = max_delta;
  return result;
}

bool velocityCruisePathIsUsable(const std::span<const Point2> path,
                                const Point2 current_position,
                                const std::size_t waypoint_index) {
  return closestUsableVelocityProjection(path, current_position, waypoint_index)
      .has_value();
}

VelocitySetpointPlan
planVelocitySetpoint(const std::span<const Point2> path, const Point2 current_position,
                     const Point2 current_velocity, const bool current_velocity_valid,
                     const std::size_t waypoint_index, const double dt_s,
                     const VelocityFollowerState& previous_state,
                     const VelocityFollowerConfig& config) {
  VelocitySetpointPlan plan{};
  if (path.size() < 2U || !finite2D(current_position) || !pathIsFinite(path)) {
    return plan;
  }

  const double final_acceptance =
      sanitizedPositive(config.final_acceptance_radius_m, 1.0, 0.0, 100.0);
  if (distance(current_position, path.back()) <= final_acceptance) {
    plan.valid = true;
    plan.final_goal_reached = true;
    plan.reason = VelocitySetpointReason::kHold;
    plan.projection = path.back();
    return plan;
  }

  const auto projection =
      closestUsableVelocityProjection(path, current_position, waypoint_index);
  if (!projection.has_value() || projection->segment_start_index + 1U >= path.size()) {
    return plan;
  }

  const Point2 segment_start = path[projection->segment_start_index];
  const Point2 segment_end = path[projection->segment_start_index + 1U];
  const Point2 tangent = normalized(segment_end - segment_start);
  if (!(norm(tangent) > kTinyDistanceM)) {
    return plan;
  }

  const double current_speed =
      currentSpeedMps(current_velocity, current_velocity_valid, previous_state);
  const TurnSpeedPlan turn_plan =
      speedLimitForUpcomingTurn(path, *projection, current_speed, config);
  const double cruise_speed =
      sanitizedPositive(config.cruise_speed_mps, 12.0, 0.0, 100.0);
  const double raw_speed_limit =
      turn_plan.valid && std::isfinite(turn_plan.raw_speed_limit_mps)
          ? std::clamp(turn_plan.raw_speed_limit_mps, 0.0, cruise_speed)
          : cruise_speed;

  const double previous_speed =
      previous_state.previous_velocity_setpoint_valid &&
              finite2D(previous_state.previous_velocity_setpoint)
          ? norm(previous_state.previous_velocity_setpoint)
          : current_speed;
  const double dt = sanitizedPositive(dt_s, 0.1, 0.0, 10.0);
  const double max_accel = effectiveVelocityDeltaAccelMps2(config);
  const double max_decel = effectiveVelocityDeltaDecelMps2(config);
  const double max_speed_delta =
      (raw_speed_limit >= previous_speed ? max_accel : max_decel) * dt;
  const double accel_limited_speed =
      previous_speed +
      std::clamp(raw_speed_limit - previous_speed, -max_speed_delta, max_speed_delta);

  const Point2 cross_track = projection->point - current_position;
  const double cross_track_gain =
      sanitizedPositive(config.cross_track_gain, 0.25, 0.0, 10.0);
  const Point2 correction = boundedCrossTrackCorrection(cross_track * cross_track_gain,
                                                        accel_limited_speed, config);
  const Point2 desired_direction =
      normalized(tangent * std::max(accel_limited_speed, 1.0) + correction);
  if (!(norm(desired_direction) > kTinyDistanceM)) {
    return plan;
  }

  const Point2 desired_velocity = desired_direction * accel_limited_speed;
  const double vector_limit = effectiveVelocityDeltaAccelMps2(config);
  const VelocityVectorLimitResult limited_velocity = limitVelocityVectorDelta(
      desired_velocity, previous_state.previous_velocity_setpoint,
      previous_state.previous_velocity_setpoint_valid, dt, vector_limit);

  plan.valid = true;
  plan.reason = reasonFromTurnPlan(turn_plan, config);
  plan.velocity_xy = limited_velocity.velocity;
  plan.path_tangent = tangent;
  plan.projection = projection->point;
  plan.cross_track_correction_velocity = correction;
  plan.speed_mps = norm(plan.velocity_xy);
  plan.raw_speed_limit_mps = raw_speed_limit;
  plan.accel_limited_speed_mps = accel_limited_speed;
  plan.velocity_delta_mps = limited_velocity.delta_mps;
  plan.cross_track_correction_mps = norm(correction);
  plan.path_projection = *projection;
  plan.turn = turn_plan;
  return plan;
}

} // namespace drone_city_nav
