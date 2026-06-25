#include "drone_city_nav/offboard_velocity_follower.hpp"

#include "drone_city_nav/velocity_command_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;

struct VectorRateLimitResult {
  Point2 value{};
  double delta{std::numeric_limits<double>::quiet_NaN()};
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

[[nodiscard]] double dot(const Point2 lhs, const Point2 rhs) noexcept {
  return lhs.x * rhs.x + lhs.y * rhs.y;
}

[[nodiscard]] double sanitizedPositive(const double value, const double fallback,
                                       const double min_value,
                                       const double max_value) noexcept {
  if (!std::isfinite(value)) {
    return fallback;
  }
  return std::clamp(value, min_value, max_value);
}

[[nodiscard]] double sanitizedCruiseSpeed(const VelocityFollowerConfig& config) {
  return sanitizedPositive(config.cruise_speed_mps, 12.0, 0.0, 100.0);
}

[[nodiscard]] double
effectiveSpeedProfileDecelMps2(const VelocityFollowerConfig& config) {
  const double fallback = sanitizedPositive(config.max_decel_mps2, 4.0, 1.0e-6, 100.0);
  return sanitizedPositive(config.speed_profile_decel_mps2, fallback, 1.0e-6, 100.0);
}

[[nodiscard]] double finalHoldMaxSpeedMps(const VelocityFollowerConfig& config) {
  return sanitizedPositive(config.final_hold_max_speed_mps, 0.8, 0.0, 100.0);
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
previousCommandSpeedMps(const VelocityFollowerState& previous_state,
                        const double current_speed_mps) {
  if (previous_state.previous_velocity_setpoint_valid &&
      finite2D(previous_state.previous_velocity_setpoint)) {
    return norm(previous_state.previous_velocity_setpoint);
  }
  return current_speed_mps;
}

[[nodiscard]] std::size_t
segmentIndexForS(const std::span<const TrajectorySegment> trajectory,
                 const double s_m) {
  if (trajectory.empty()) {
    return 0U;
  }
  for (std::size_t i = 0U; i < trajectory.size(); ++i) {
    const double segment_end_s =
        trajectory[i].s_start_m + std::max(0.0, trajectory[i].length_m);
    if (s_m <= segment_end_s) {
      return i;
    }
  }
  return trajectory.size() - 1U;
}

[[nodiscard]] VectorRateLimitResult
limitVectorRate(const Point2 desired, const Point2 previous, const bool previous_valid,
                const double dt_s, const double max_rate) {
  VectorRateLimitResult result{};
  result.value = desired;
  if (!previous_valid || !finite2D(previous) || !finite2D(desired)) {
    return result;
  }
  const double sanitized_rate = sanitizedPositive(max_rate, 0.0, 0.0, 1000.0);
  const Point2 delta = desired - previous;
  const double delta_norm = norm(delta);
  result.delta = delta_norm;
  if (!(sanitized_rate > 0.0)) {
    return result;
  }
  const double dt = sanitizedPositive(dt_s, 0.1, 0.0, 10.0);
  const double max_delta = sanitized_rate * dt;
  if (delta_norm <= max_delta || !(delta_norm > kTinyDistanceM)) {
    return result;
  }
  result.value = previous + delta * (max_delta / delta_norm);
  result.delta = max_delta;
  return result;
}

void fillFeedforwardAcceleration(VelocitySetpointPlan& plan,
                                 const TrajectoryProjection& projection,
                                 const VelocityFollowerState& previous_state,
                                 const double dt_s,
                                 const VelocityFollowerConfig& config) {
  const Point2 left_normal{-projection.tangent.y, projection.tangent.x};
  const double feedforward_scale =
      sanitizedPositive(config.acceleration_feedforward_scale, 1.0, 0.0, 10.0);
  const double feedforward_accel = plan.trajectory_curvature_1pm * plan.speed_mps *
                                   plan.speed_mps * feedforward_scale;
  const double max_feedforward_accel = sanitizedPositive(
      config.max_feedforward_accel_mps2, config.max_accel_mps2, 0.0, 100.0);
  const double limited_feedforward_accel =
      std::clamp(feedforward_accel, -max_feedforward_accel, max_feedforward_accel);
  const Point2 raw_acceleration_xy = left_normal * limited_feedforward_accel;
  const VectorRateLimitResult limited_acceleration = limitVectorRate(
      raw_acceleration_xy, previous_state.previous_feedforward_acceleration_setpoint,
      previous_state.previous_feedforward_acceleration_setpoint_valid, dt_s,
      config.max_feedforward_jerk_mps3);

  plan.acceleration_xy = limited_acceleration.value;
  plan.raw_acceleration_xy = raw_acceleration_xy;
  plan.acceleration_xy_mps2 = norm(plan.acceleration_xy);
  plan.raw_acceleration_xy_mps2 = norm(plan.raw_acceleration_xy);
  plan.acceleration_delta_mps2 = limited_acceleration.delta;
  plan.acceleration_jerk_mps3 = std::isfinite(limited_acceleration.delta)
                                    ? limited_acceleration.delta / dt_s
                                    : std::numeric_limits<double>::quiet_NaN();
  plan.curvature_feedforward_accel_mps2 = limited_feedforward_accel;
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
    case VelocitySetpointReason::kTrajectorySpeedProfile:
      return "trajectory_profile";
    case VelocitySetpointReason::kFinalApproach:
      return "final_approach";
  }
  return "unknown";
}

VelocitySetpointPlan planVelocitySetpoint(
    const std::span<const TrajectorySegment> trajectory,
    const TrajectorySpeedProfile& speed_profile, const Point2 current_position,
    const Point2 current_velocity, const bool current_velocity_valid, const double dt_s,
    const VelocityFollowerState& previous_state, const VelocityFollowerConfig& config) {
  VelocitySetpointPlan plan{};
  if (!finite2D(current_position) || !trajectoryIsUsable(trajectory) ||
      !speed_profile.valid || speed_profile.samples.empty()) {
    return plan;
  }

  const double dt = sanitizedPositive(dt_s, 0.1, 0.0, 10.0);
  const Point2 final_point = trajectory.back().end;
  const double final_acceptance =
      sanitizedPositive(config.final_acceptance_radius_m, 1.0, 0.0, 100.0);
  const double current_speed =
      currentSpeedMps(current_velocity, current_velocity_valid, previous_state);
  if (distance(current_position, final_point) <= final_acceptance &&
      current_speed <= finalHoldMaxSpeedMps(config)) {
    plan.valid = true;
    plan.final_goal_reached = true;
    plan.reason = VelocitySetpointReason::kHold;
    plan.projection = final_point;
    return plan;
  }

  const std::optional<TrajectoryProjection> projection =
      projectOnTrajectory(trajectory, current_position);
  if (!projection.has_value() || !(norm(projection->tangent) > kTinyDistanceM)) {
    return plan;
  }

  const double cross_track_error = std::sqrt(projection->distance_sq);
  const ScalarSpeedPlan scalar_speed = planScalarSpeed(
      speed_profile,
      ScalarSpeedQuery{.trajectory_s_m = projection->s_m,
                       .cross_track_error_m = cross_track_error,
                       .previous_command_speed_mps =
                           previousCommandSpeedMps(previous_state, current_speed),
                       .current_speed_mps = current_speed,
                       .dt_s = dt},
      config);
  if (!scalar_speed.valid) {
    return plan;
  }

  const VelocityCommandPlan command = planVelocityCommand(
      VelocityCommandQuery{
          .projection = *projection,
          .current_position = current_position,
          .current_velocity = current_velocity,
          .current_velocity_valid = current_velocity_valid,
          .scalar_speed_mps = scalar_speed.final_scalar_speed_mps,
          .dt_s = dt,
          .previous_cross_track_correction_velocity =
              previous_state.previous_cross_track_correction_velocity,
          .previous_cross_track_correction_velocity_valid =
              previous_state.previous_cross_track_correction_velocity_valid},
      config);
  if (!command.valid) {
    return plan;
  }

  const VelocitySmootherPlan smoothed = smoothVelocityCommand(
      VelocitySmootherInput{
          .desired_velocity_xy = command.desired_velocity_xy,
          .previous_velocity_setpoint = previous_state.previous_velocity_setpoint,
          .previous_velocity_acceleration_setpoint =
              previous_state.previous_velocity_acceleration_setpoint,
          .previous_velocity_setpoint_valid =
              previous_state.previous_velocity_setpoint_valid,
          .previous_velocity_acceleration_setpoint_valid =
              previous_state.previous_velocity_acceleration_setpoint_valid,
          .dt_s = dt},
      config);
  if (!smoothed.valid) {
    return plan;
  }

  const double cruise_speed = sanitizedCruiseSpeed(config);
  plan.valid = true;
  plan.reason = VelocitySetpointReason::kStraight;
  if (scalar_speed.constraint_type == SpeedConstraintType::kArc &&
      scalar_speed.cross_track_limited_speed_mps + 1.0e-6 < cruise_speed) {
    plan.reason = VelocitySetpointReason::kTrajectorySpeedProfile;
  } else if (scalar_speed.constraint_type == SpeedConstraintType::kGoal &&
             scalar_speed.cross_track_limited_speed_mps + 1.0e-6 < cruise_speed) {
    plan.reason = VelocitySetpointReason::kFinalApproach;
  }

  plan.velocity_xy = smoothed.velocity_xy;
  plan.desired_velocity_xy = command.desired_velocity_xy;
  plan.speed_mps = norm(plan.velocity_xy);
  plan.final_command_speed_mps = plan.speed_mps;
  plan.desired_speed_mps = norm(plan.desired_velocity_xy);
  plan.velocity_setpoint_acceleration_xy = smoothed.velocity_setpoint_acceleration_xy;
  plan.velocity_setpoint_acceleration_mps2 =
      smoothed.velocity_setpoint_acceleration_mps2;
  plan.velocity_setpoint_jerk_mps3 = smoothed.velocity_setpoint_jerk_mps3;
  plan.path_tangent = projection->tangent;
  plan.projection = projection->point;
  plan.raw_cross_track_correction_velocity =
      command.raw_cross_track_correction_velocity;
  plan.cross_track_correction_velocity = command.cross_track_correction_velocity;
  plan.raw_speed_limit_mps = scalar_speed.cross_track_limited_speed_mps;
  plan.profile_speed_limit_mps = scalar_speed.profile_speed_limit_mps;
  plan.speed_lookahead_distance_m = scalar_speed.lookahead_distance_m;
  plan.lookahead_speed_limit_mps = scalar_speed.lookahead_speed_limit_mps;
  plan.lookahead_limiting_constraint_type = scalar_speed.lookahead_constraint_type;
  plan.lookahead_limiting_constraint_index = scalar_speed.lookahead_constraint_index;
  plan.lookahead_limiting_constraint_distance_m =
      scalar_speed.lookahead_constraint_distance_m;
  plan.speed_after_lookahead_mps = scalar_speed.speed_after_lookahead_mps;
  plan.cross_track_speed_factor = scalar_speed.cross_track_speed_factor;
  plan.cross_track_limited_speed_mps = scalar_speed.cross_track_limited_speed_mps;
  plan.accel_limited_speed_mps = scalar_speed.accel_limited_speed_mps;
  plan.velocity_delta_mps = smoothed.velocity_delta_mps;
  plan.desired_velocity_delta_mps = smoothed.desired_velocity_delta_mps;
  plan.velocity_tracking_error_mps =
      current_velocity_valid && finite2D(current_velocity)
          ? norm(plan.velocity_xy - current_velocity)
          : std::numeric_limits<double>::quiet_NaN();
  const Point2 left_normal{-projection->tangent.y, projection->tangent.x};
  plan.current_velocity_tangent_mps =
      current_velocity_valid && finite2D(current_velocity)
          ? dot(current_velocity, projection->tangent)
          : std::numeric_limits<double>::quiet_NaN();
  plan.current_velocity_normal_mps =
      current_velocity_valid && finite2D(current_velocity)
          ? dot(current_velocity, left_normal)
          : std::numeric_limits<double>::quiet_NaN();
  plan.desired_velocity_tangent_mps = command.desired_velocity_tangent_mps;
  plan.desired_velocity_normal_mps = command.desired_velocity_normal_mps;
  plan.setpoint_velocity_tangent_mps = dot(plan.velocity_xy, projection->tangent);
  plan.setpoint_velocity_normal_mps = dot(plan.velocity_xy, left_normal);
  plan.raw_cross_track_correction_mps = command.raw_cross_track_correction_mps;
  plan.cross_track_correction_mps = command.cross_track_correction_mps;
  plan.cross_track_correction_delta_mps = command.cross_track_correction_delta_mps;
  plan.cross_track_lateral_velocity_mps = command.cross_track_lateral_velocity_mps;
  plan.trajectory_cross_track_error_m = cross_track_error;
  plan.limiting_constraint_type = scalar_speed.constraint_type;
  plan.limiting_constraint_index = scalar_speed.constraint_index;
  plan.limiting_constraint_distance_m = scalar_speed.limiting_constraint_distance_m;
  plan.limiting_constraint_speed_mps = scalar_speed.limiting_constraint_speed_mps;
  plan.limiting_allowed_speed_now_mps = scalar_speed.limiting_allowed_speed_now_mps;
  plan.limiting_curve_radius_m = scalar_speed.limiting_curve_radius_m;
  plan.trajectory_s_m = projection->s_m;
  plan.trajectory_segment_index = projection->segment_index;
  plan.trajectory_segment_kind =
      trajectory[segmentIndexForS(trajectory, projection->s_m)].kind;
  plan.trajectory_curvature_1pm = scalar_speed.limiting_curvature_1pm;
  plan.trajectory_arc_radius_m =
      std::abs(scalar_speed.limiting_curvature_1pm) > kTinyDistanceM
          ? 1.0 / std::abs(scalar_speed.limiting_curvature_1pm)
          : std::numeric_limits<double>::quiet_NaN();
  plan.trajectory_projection = *projection;

  plan.final_stop.valid = true;
  plan.final_stop.distance_to_stop_m =
      distanceFromTrajectorySToEnd(trajectory, projection->s_m);
  plan.final_stop.braking_distance_m =
      current_speed * current_speed / (2.0 * effectiveSpeedProfileDecelMps2(config));
  plan.final_stop.raw_speed_limit_mps = scalar_speed.limiting_allowed_speed_now_mps;

  fillFeedforwardAcceleration(plan, *projection, previous_state, dt, config);
  return plan;
}

} // namespace drone_city_nav
