#include "drone_city_nav/offboard_velocity_follower.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;
constexpr double kTinyAngleRad = 1.0e-6;
constexpr double kMinTurnSeverity = 1.0e-3;
constexpr double kDefaultTurnRadiusBaseM = 10.0;

struct TurnKinematics {
  double radius_m{std::numeric_limits<double>::infinity()};
  double target_speed_mps{std::numeric_limits<double>::quiet_NaN()};
};

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

[[nodiscard]] double plannedBrakingDistanceM(const double cruise_speed_mps,
                                             const double target_speed_mps,
                                             const double max_decel_mps2,
                                             const double braking_margin_m) {
  const double cruise_speed = sanitizedPositive(cruise_speed_mps, 12.0, 0.0, 100.0);
  const double target_speed =
      std::min(sanitizedPositive(target_speed_mps, 0.0, 0.0, 100.0), cruise_speed);
  const double max_decel = sanitizedPositive(max_decel_mps2, 4.0, 1.0e-6, 100.0);
  const double braking_margin = sanitizedPositive(braking_margin_m, 0.0, 0.0, 100.0);
  const double speed_delta_sq =
      std::max(0.0, cruise_speed * cruise_speed - target_speed * target_speed);
  if (speed_delta_sq <= kTinyDistanceM) {
    return 0.0;
  }

  const double physics_braking_distance = speed_delta_sq / (2.0 * max_decel);
  return physics_braking_distance + braking_margin;
}

[[nodiscard]] double smoothBrakingSpeedLimitMps(const double remaining_distance_m,
                                                const double target_speed_mps,
                                                const double cruise_speed_mps,
                                                const double max_decel_mps2,
                                                const double braking_margin_m) {
  const double cruise_speed = sanitizedPositive(cruise_speed_mps, 12.0, 0.0, 100.0);
  const double target_speed =
      std::min(sanitizedPositive(target_speed_mps, 0.0, 0.0, 100.0), cruise_speed);
  const double remaining_distance = std::max(0.0, remaining_distance_m);
  const double speed_delta_sq =
      std::max(0.0, cruise_speed * cruise_speed - target_speed * target_speed);
  if (speed_delta_sq <= kTinyDistanceM) {
    return target_speed;
  }

  const double planned_braking_distance = plannedBrakingDistanceM(
      cruise_speed, target_speed, max_decel_mps2, braking_margin_m);
  if (planned_braking_distance <= kTinyDistanceM ||
      remaining_distance >= planned_braking_distance) {
    return cruise_speed;
  }

  const double profile_decel = speed_delta_sq / (2.0 * planned_braking_distance);
  return std::min(cruise_speed, std::sqrt(std::max(0.0, target_speed * target_speed +
                                                            2.0 * profile_decel *
                                                                remaining_distance)));
}

[[nodiscard]] TurnKinematics turnKinematics(const double angle_rad,
                                            const VelocityFollowerConfig& config) {
  const double cruise_speed =
      sanitizedPositive(config.cruise_speed_mps, 12.0, 0.0, 100.0);
  const double min_turn_speed = std::min(
      sanitizedPositive(config.min_turn_speed_mps, 2.0, 0.0, 100.0), cruise_speed);
  const double max_lateral_accel =
      sanitizedPositive(config.max_lateral_accel_mps2, 3.0, 1.0e-6, 100.0);
  const double turn_radius_base = sanitizedPositive(
      config.turn_radius_base_m, kDefaultTurnRadiusBaseM, 0.1, 1000.0);
  const double angle = std::clamp(angle_rad, 0.0, std::numbers::pi);
  const double severity = std::sin(angle * 0.5);

  if (severity <= kTinyAngleRad) {
    return TurnKinematics{std::numeric_limits<double>::infinity(), cruise_speed};
  }
  const double radius = turn_radius_base / std::max(severity, kMinTurnSeverity);
  const double speed = std::sqrt(max_lateral_accel * radius);
  return TurnKinematics{radius, std::clamp(speed, min_turn_speed, cruise_speed)};
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

} // namespace

const char* velocitySetpointReasonName(const VelocitySetpointReason reason) noexcept {
  switch (reason) {
    case VelocitySetpointReason::kInvalidPath:
      return "invalid_path";
    case VelocitySetpointReason::kHold:
      return "hold";
    case VelocitySetpointReason::kStraight:
      return "straight";
    case VelocitySetpointReason::kBrakingForTurn:
      return "braking_for_turn";
    case VelocitySetpointReason::kFinalApproach:
      return "final_approach";
  }
  return "unknown";
}

const char*
speedConstraintTypeName(const SpeedConstraintType constraint_type) noexcept {
  switch (constraint_type) {
    case SpeedConstraintType::kNone:
      return "none";
    case SpeedConstraintType::kTurn:
      return "turn";
    case SpeedConstraintType::kGoal:
      return "goal";
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
                                        const VelocityFollowerConfig& config) {
  TurnSpeedPlan best_plan{};
  if (path.size() < 3U || projection.segment_start_index + 1U >= path.size() ||
      !pathIsFinite(path)) {
    return best_plan;
  }

  const double cruise_speed =
      sanitizedPositive(config.cruise_speed_mps, 12.0, 0.0, 100.0);
  const double preview_distance =
      sanitizedPositive(config.turn_preview_distance_m, 32.0, 0.0, 1000.0);
  const double max_decel = effectiveVelocityDeltaDecelMps2(config);
  const double braking_margin =
      sanitizedPositive(config.braking_margin_m, 2.0, 0.0, 100.0);

  const std::size_t first_turn_index =
      std::max<std::size_t>(1U, projection.segment_start_index + 1U);
  for (std::size_t index = first_turn_index; index + 1U < path.size(); ++index) {
    const double angle = turnAngleRad(path[index - 1U], path[index], path[index + 1U]);
    const double distance_to_turn =
        distanceFromProjectionToWaypoint(path, projection, index);
    if (distance_to_turn > preview_distance) {
      return best_plan;
    }
    if (angle <= kTinyAngleRad) {
      continue;
    }

    const TurnKinematics kinematics = turnKinematics(angle, config);
    TurnSpeedPlan candidate{};
    candidate.valid = true;
    candidate.waypoint_index = index;
    candidate.angle_rad = angle;
    candidate.turn_radius_m = kinematics.radius_m;
    candidate.distance_to_turn_m = distance_to_turn;
    candidate.target_turn_speed_mps = kinematics.target_speed_mps;
    candidate.braking_distance_m = plannedBrakingDistanceM(
        cruise_speed, candidate.target_turn_speed_mps, max_decel, braking_margin);
    candidate.raw_speed_limit_mps =
        smoothBrakingSpeedLimitMps(distance_to_turn, candidate.target_turn_speed_mps,
                                   cruise_speed, max_decel, braking_margin);

    if (!best_plan.valid ||
        candidate.raw_speed_limit_mps < best_plan.raw_speed_limit_mps) {
      best_plan = candidate;
    }
  }

  return best_plan;
}

StopSpeedPlan speedLimitForFinalStop(const std::span<const Point2> path,
                                     const OffboardPathProjection& projection,
                                     const VelocityFollowerConfig& config) {
  StopSpeedPlan plan{};
  if (path.size() < 2U || !pathIsFinite(path)) {
    return plan;
  }

  const double cruise_speed =
      sanitizedPositive(config.cruise_speed_mps, 12.0, 0.0, 100.0);
  const double max_decel = effectiveVelocityDeltaDecelMps2(config);
  const double braking_margin =
      sanitizedPositive(config.braking_margin_m, 2.0, 0.0, 100.0);
  const double final_acceptance =
      sanitizedPositive(config.final_acceptance_radius_m, 1.0, 0.0, 100.0);

  plan.valid = true;
  plan.distance_to_stop_m =
      distanceFromProjectionToWaypoint(path, projection, path.size() - 1U);
  plan.braking_distance_m =
      final_acceptance +
      plannedBrakingDistanceM(cruise_speed, 0.0, max_decel, braking_margin);

  if (plan.distance_to_stop_m <= final_acceptance) {
    plan.raw_speed_limit_mps = 0.0;
    return plan;
  }

  plan.raw_speed_limit_mps =
      smoothBrakingSpeedLimitMps(plan.distance_to_stop_m - final_acceptance, 0.0,
                                 cruise_speed, max_decel, braking_margin);
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
  const TurnSpeedPlan turn_plan = speedLimitForUpcomingTurn(path, *projection, config);
  const StopSpeedPlan final_stop_plan =
      speedLimitForFinalStop(path, *projection, config);
  const double cruise_speed =
      sanitizedPositive(config.cruise_speed_mps, 12.0, 0.0, 100.0);
  const double turn_speed_limit =
      turn_plan.valid && std::isfinite(turn_plan.raw_speed_limit_mps)
          ? std::clamp(turn_plan.raw_speed_limit_mps, 0.0, cruise_speed)
          : cruise_speed;
  const double final_stop_speed_limit =
      final_stop_plan.valid && std::isfinite(final_stop_plan.raw_speed_limit_mps)
          ? std::clamp(final_stop_plan.raw_speed_limit_mps, 0.0, cruise_speed)
          : cruise_speed;
  const double raw_speed_limit = std::min(turn_speed_limit, final_stop_speed_limit);

  SpeedConstraintType limiting_constraint_type = SpeedConstraintType::kNone;
  std::size_t limiting_constraint_index = 0U;
  double limiting_constraint_distance = std::numeric_limits<double>::quiet_NaN();
  double limiting_turn_angle = std::numeric_limits<double>::quiet_NaN();
  double limiting_turn_radius = std::numeric_limits<double>::quiet_NaN();
  double limiting_constraint_speed = std::numeric_limits<double>::quiet_NaN();
  double limiting_allowed_speed_now = std::numeric_limits<double>::quiet_NaN();
  if (turn_plan.valid && turn_speed_limit + 1.0e-6 < cruise_speed) {
    limiting_constraint_type = SpeedConstraintType::kTurn;
    limiting_constraint_index = turn_plan.waypoint_index;
    limiting_constraint_distance = turn_plan.distance_to_turn_m;
    limiting_turn_angle = turn_plan.angle_rad;
    limiting_turn_radius = turn_plan.turn_radius_m;
    limiting_constraint_speed = turn_plan.target_turn_speed_mps;
    limiting_allowed_speed_now = turn_speed_limit;
  }
  if (final_stop_plan.valid &&
      final_stop_speed_limit + 1.0e-6 < (std::isfinite(limiting_allowed_speed_now)
                                             ? limiting_allowed_speed_now
                                             : cruise_speed)) {
    limiting_constraint_type = SpeedConstraintType::kGoal;
    limiting_constraint_index = path.size() - 1U;
    limiting_constraint_distance = final_stop_plan.distance_to_stop_m;
    limiting_turn_angle = std::numeric_limits<double>::quiet_NaN();
    limiting_turn_radius = std::numeric_limits<double>::quiet_NaN();
    limiting_constraint_speed = 0.0;
    limiting_allowed_speed_now = final_stop_speed_limit;
  }

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
  plan.reason = VelocitySetpointReason::kStraight;
  if (limiting_constraint_type == SpeedConstraintType::kTurn) {
    plan.reason = VelocitySetpointReason::kBrakingForTurn;
  } else if (limiting_constraint_type == SpeedConstraintType::kGoal) {
    plan.reason = VelocitySetpointReason::kFinalApproach;
  }
  plan.velocity_xy = limited_velocity.velocity;
  plan.path_tangent = tangent;
  plan.projection = projection->point;
  plan.cross_track_correction_velocity = correction;
  plan.speed_mps = norm(plan.velocity_xy);
  plan.raw_speed_limit_mps = raw_speed_limit;
  plan.accel_limited_speed_mps = accel_limited_speed;
  plan.velocity_delta_mps = limited_velocity.delta_mps;
  plan.cross_track_correction_mps = norm(correction);
  plan.limiting_constraint_type = limiting_constraint_type;
  plan.limiting_constraint_index = limiting_constraint_index;
  plan.limiting_constraint_distance_m = limiting_constraint_distance;
  plan.limiting_turn_angle_rad = limiting_turn_angle;
  plan.limiting_turn_radius_m = limiting_turn_radius;
  plan.limiting_constraint_speed_mps = limiting_constraint_speed;
  plan.limiting_allowed_speed_now_mps = limiting_allowed_speed_now;
  plan.path_projection = *projection;
  plan.turn = turn_plan;
  plan.final_stop = final_stop_plan;
  return plan;
}

} // namespace drone_city_nav
