#include "drone_city_nav/offboard_velocity_follower.hpp"

#include "drone_city_nav/velocity_command_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;

[[nodiscard]] bool finite2D(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

[[nodiscard]] Point2 operator-(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x - rhs.x, lhs.y - rhs.y};
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

namespace {

VelocitySetpointPlan planVelocitySetpointFromProjection(
    const TrajectoryProjection& projection, const Point2 final_point,
    const double trajectory_length_m, const TrajectorySegmentKind segment_kind,
    const TrajectorySpeedProfile& speed_profile, const Point2 current_position,
    const Point2 current_velocity, const bool current_velocity_valid, const double dt_s,
    const VelocityFollowerState& previous_state, const VelocityFollowerConfig& config) {
  VelocitySetpointPlan plan{};
  if (!finite2D(current_position) || !finite2D(final_point) || !projection.valid ||
      !(norm(projection.tangent) > kTinyDistanceM) ||
      !std::isfinite(trajectory_length_m) || !speed_profile.valid ||
      speed_profile.samples.empty()) {
    return plan;
  }

  const double dt = sanitizedPositive(dt_s, 0.1, 0.0, 10.0);
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

  const ScalarSpeedPlan scalar_speed = planScalarSpeed(
      speed_profile,
      ScalarSpeedQuery{.trajectory_s_m = projection.s_m,
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
          .projection = projection,
          .current_position = current_position,
          .current_velocity = current_velocity,
          .current_velocity_valid = current_velocity_valid,
          .scalar_speed_mps = scalar_speed.final_scalar_speed_mps,
          .dt_s = dt,
          .previous_cross_track_correction_velocity =
              previous_state.previous_cross_track_correction_velocity,
          .previous_cross_track_correction_velocity_valid =
              previous_state.previous_cross_track_correction_velocity_valid,
          .previous_curvature_anticipation_velocity =
              previous_state.previous_curvature_anticipation_velocity,
          .previous_curvature_anticipation_velocity_valid =
              previous_state.previous_curvature_anticipation_velocity_valid},
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
  plan.path_tangent = projection.tangent;
  plan.projection = projection.point;
  plan.raw_cross_track_correction_velocity =
      command.raw_cross_track_correction_velocity;
  plan.cross_track_correction_velocity = command.cross_track_correction_velocity;
  plan.raw_curvature_anticipation_velocity =
      command.raw_curvature_anticipation_velocity;
  plan.curvature_anticipation_velocity = command.curvature_anticipation_velocity;
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
  const Point2 left_normal{-projection.tangent.y, projection.tangent.x};
  plan.current_velocity_tangent_mps =
      current_velocity_valid && finite2D(current_velocity)
          ? dot(current_velocity, projection.tangent)
          : std::numeric_limits<double>::quiet_NaN();
  plan.current_velocity_normal_mps =
      current_velocity_valid && finite2D(current_velocity)
          ? dot(current_velocity, left_normal)
          : std::numeric_limits<double>::quiet_NaN();
  plan.desired_velocity_tangent_mps = command.desired_velocity_tangent_mps;
  plan.desired_velocity_normal_mps = command.desired_velocity_normal_mps;
  plan.setpoint_velocity_tangent_mps = dot(plan.velocity_xy, projection.tangent);
  plan.setpoint_velocity_normal_mps = dot(plan.velocity_xy, left_normal);
  plan.raw_cross_track_correction_mps = command.raw_cross_track_correction_mps;
  plan.cross_track_correction_mps = command.cross_track_correction_mps;
  plan.cross_track_correction_delta_mps = command.cross_track_correction_delta_mps;
  plan.cross_track_lateral_velocity_mps = command.cross_track_lateral_velocity_mps;
  plan.raw_curvature_anticipation_mps = command.raw_curvature_anticipation_mps;
  plan.curvature_anticipation_mps = command.curvature_anticipation_mps;
  plan.curvature_anticipation_delta_mps = command.curvature_anticipation_delta_mps;
  plan.curvature_anticipation_angle_rad = command.curvature_anticipation_angle_rad;
  plan.trajectory_cross_track_error_m = std::sqrt(projection.distance_sq);
  plan.limiting_constraint_type = scalar_speed.constraint_type;
  plan.limiting_constraint_index = scalar_speed.constraint_index;
  plan.limiting_constraint_distance_m = scalar_speed.limiting_constraint_distance_m;
  plan.limiting_constraint_speed_mps = scalar_speed.limiting_constraint_speed_mps;
  plan.limiting_allowed_speed_now_mps = scalar_speed.limiting_allowed_speed_now_mps;
  plan.limiting_curve_radius_m = scalar_speed.limiting_curve_radius_m;
  plan.trajectory_s_m = projection.s_m;
  plan.trajectory_segment_index = projection.segment_index;
  plan.trajectory_segment_kind = segment_kind;
  plan.trajectory_curvature_1pm = projection.curvature_1pm;
  plan.trajectory_arc_radius_m =
      std::abs(plan.trajectory_curvature_1pm) > kTinyDistanceM
          ? 1.0 / std::abs(plan.trajectory_curvature_1pm)
          : std::numeric_limits<double>::quiet_NaN();
  plan.trajectory_projection = projection;

  plan.final_stop.valid = true;
  plan.final_stop.distance_to_stop_m =
      std::max(0.0, trajectory_length_m - std::max(0.0, projection.s_m));
  plan.final_stop.braking_distance_m =
      current_speed * current_speed / (2.0 * effectiveSpeedProfileDecelMps2(config));
  plan.final_stop.raw_speed_limit_mps = scalar_speed.limiting_allowed_speed_now_mps;

  return plan;
}

} // namespace

VelocitySetpointPlan planVelocitySetpoint(
    const std::span<const TrajectorySegment> trajectory,
    const TrajectorySpeedProfile& speed_profile, const Point2 current_position,
    const Point2 current_velocity, const bool current_velocity_valid, const double dt_s,
    const VelocityFollowerState& previous_state, const VelocityFollowerConfig& config) {
  if (!trajectoryIsUsable(trajectory)) {
    return VelocitySetpointPlan{};
  }
  const std::optional<TrajectoryProjection> projection =
      projectOnTrajectory(trajectory, current_position);
  if (!projection.has_value()) {
    return VelocitySetpointPlan{};
  }
  const TrajectorySegment& segment =
      trajectory[std::min(projection->segment_index, trajectory.size() - 1U)];
  return planVelocitySetpointFromProjection(
      *projection, trajectory.back().end, trajectoryLengthM(trajectory), segment.kind,
      speed_profile, current_position, current_velocity, current_velocity_valid, dt_s,
      previous_state, config);
}

VelocitySetpointPlan planVelocitySetpoint(
    const std::span<const TrajectoryPointSample> trajectory_samples,
    const TrajectorySpeedProfile& speed_profile, const Point2 current_position,
    const Point2 current_velocity, const bool current_velocity_valid, const double dt_s,
    const VelocityFollowerState& previous_state, const VelocityFollowerConfig& config) {
  if (!trajectorySamplesAreUsable(trajectory_samples)) {
    return VelocitySetpointPlan{};
  }
  const std::optional<TrajectoryProjection> projection =
      projectOnTrajectorySamples(trajectory_samples, current_position);
  if (!projection.has_value()) {
    return VelocitySetpointPlan{};
  }
  const TrajectorySegmentKind segment_kind =
      std::abs(projection->curvature_1pm) > kTinyDistanceM
          ? TrajectorySegmentKind::kArc
          : TrajectorySegmentKind::kLine;
  return planVelocitySetpointFromProjection(
      *projection, trajectory_samples.back().point, trajectory_samples.back().s_m,
      segment_kind, speed_profile, current_position, current_velocity,
      current_velocity_valid, dt_s, previous_state, config);
}

} // namespace drone_city_nav
