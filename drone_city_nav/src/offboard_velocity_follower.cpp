#include "drone_city_nav/offboard_velocity_follower.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

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

[[nodiscard]] double sanitizedMinTurnSpeed(const VelocityFollowerConfig& config) {
  return std::min(sanitizedPositive(config.min_turn_speed_mps, 2.0, 0.0, 100.0),
                  sanitizedCruiseSpeed(config));
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

[[nodiscard]] double segmentEndS(const TrajectorySegment& segment) noexcept {
  return segment.s_start_m + std::max(0.0, segment.length_m);
}

[[nodiscard]] std::size_t
segmentIndexForS(const std::span<const TrajectorySegment> trajectory,
                 const double s_m) {
  if (trajectory.empty()) {
    return 0U;
  }
  for (std::size_t i = 0U; i < trajectory.size(); ++i) {
    if (s_m <= segmentEndS(trajectory[i])) {
      return i;
    }
  }
  return trajectory.size() - 1U;
}

[[nodiscard]] double segmentCurvature(const TrajectorySegment& segment) noexcept {
  if (segment.kind != TrajectorySegmentKind::kArc ||
      !(segment.radius_m > kTinyDistanceM)) {
    return 0.0;
  }
  return (segment.sweep_rad >= 0.0 ? 1.0 : -1.0) / segment.radius_m;
}

[[nodiscard]] TrajectorySpeedSample
geometricSpeedSampleForSegment(const std::span<const TrajectorySegment> trajectory,
                               const std::size_t requested_segment_index,
                               const double requested_s_m,
                               const VelocityFollowerConfig& config) {
  TrajectorySpeedSample sample{};
  if (trajectory.empty()) {
    return sample;
  }
  const double cruise_speed = sanitizedCruiseSpeed(config);
  const double min_turn_speed = sanitizedMinTurnSpeed(config);
  const double max_lateral_accel =
      sanitizedPositive(config.max_lateral_accel_mps2, 3.0, 1.0e-6, 100.0);

  const double clamped_s =
      std::clamp(requested_s_m, 0.0, trajectoryLengthM(trajectory));
  const std::size_t segment_index =
      std::min(requested_segment_index, trajectory.size() - 1U);
  const TrajectorySegment& segment = trajectory[segment_index];
  sample.s_m = clamped_s;
  sample.segment_index = segment_index;
  sample.curvature_1pm = segmentCurvature(segment);
  sample.radius_m = std::numeric_limits<double>::quiet_NaN();
  sample.reason = SpeedConstraintType::kNone;
  sample.geometric_limit_mps = cruise_speed;
  if (std::abs(sample.curvature_1pm) > kTinyDistanceM) {
    sample.radius_m = 1.0 / std::abs(sample.curvature_1pm);
    sample.reason = SpeedConstraintType::kArc;
    sample.geometric_limit_mps = std::clamp(
        std::sqrt(max_lateral_accel * sample.radius_m), min_turn_speed, cruise_speed);
  }

  sample.profiled_limit_mps = sample.geometric_limit_mps;
  sample.constraint_s_m = sample.s_m;
  sample.constraint_limit_mps = sample.geometric_limit_mps;
  return sample;
}

[[nodiscard]] TrajectorySpeedSample
geometricSpeedSampleAtS(const std::span<const TrajectorySegment> trajectory,
                        const double s_m, const VelocityFollowerConfig& config) {
  return geometricSpeedSampleForSegment(trajectory, segmentIndexForS(trajectory, s_m),
                                        s_m, config);
}

void mergeSpeedSample(std::vector<TrajectorySpeedSample>& samples,
                      const TrajectorySpeedSample& candidate) {
  constexpr double kMergeToleranceM = 1.0e-6;
  if (samples.empty() ||
      std::abs(samples.back().s_m - candidate.s_m) > kMergeToleranceM) {
    samples.push_back(candidate);
    return;
  }
  TrajectorySpeedSample& existing = samples.back();
  if (candidate.geometric_limit_mps < existing.geometric_limit_mps) {
    existing.geometric_limit_mps = candidate.geometric_limit_mps;
    existing.profiled_limit_mps = candidate.profiled_limit_mps;
    existing.reason = candidate.reason;
    existing.segment_index = candidate.segment_index;
    existing.curvature_1pm = candidate.curvature_1pm;
    existing.radius_m = candidate.radius_m;
    existing.constraint_s_m = candidate.constraint_s_m;
    existing.constraint_limit_mps = candidate.constraint_limit_mps;
  }
}

[[nodiscard]] double limitedSpeedForDistance(const double next_speed,
                                             const double distance_m,
                                             const double acceleration_mps2) {
  return std::sqrt(
      std::max(0.0, next_speed * next_speed +
                        2.0 * acceleration_mps2 * std::max(0.0, distance_m)));
}

[[nodiscard]] std::size_t
firstProfileSampleNotBefore(const TrajectorySpeedProfile& profile, const double s_m) {
  if (profile.samples.empty()) {
    return 0U;
  }
  const auto it = std::ranges::lower_bound(
      profile.samples, s_m, {},
      [](const TrajectorySpeedSample& sample) { return sample.s_m; });
  if (it == profile.samples.end()) {
    return profile.samples.size() - 1U;
  }
  return static_cast<std::size_t>(std::distance(profile.samples.begin(), it));
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
      return "trajectory_profile";
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
    case SpeedConstraintType::kArc:
      return "arc";
    case SpeedConstraintType::kGoal:
      return "goal";
  }
  return "unknown";
}

double distanceFromTrajectorySToEnd(const std::span<const TrajectorySegment> trajectory,
                                    const double s_m) {
  return std::max(0.0, trajectoryLengthM(trajectory) - std::max(0.0, s_m));
}

TrajectorySpeedProfile
buildTrajectorySpeedProfile(const std::span<const TrajectorySegment> trajectory,
                            const VelocityFollowerConfig& config) {
  TrajectorySpeedProfile profile{};
  if (!trajectoryIsUsable(trajectory)) {
    return profile;
  }

  const double length_m = trajectoryLengthM(trajectory);
  const double step_m =
      sanitizedPositive(config.speed_profile_sample_step_m, 1.0, 0.1, 100.0);
  const std::size_t regular_sample_count =
      static_cast<std::size_t>(std::ceil(length_m / step_m));
  std::vector<TrajectorySpeedSample> candidates;
  candidates.reserve(regular_sample_count + trajectory.size() * 3U + 2U);
  for (std::size_t i = 0U; i <= regular_sample_count; ++i) {
    const double s_m = std::min(length_m, static_cast<double>(i) * step_m);
    candidates.push_back(geometricSpeedSampleAtS(trajectory, s_m, config));
  }
  for (std::size_t i = 0U; i < trajectory.size(); ++i) {
    const TrajectorySegment& segment = trajectory[i];
    candidates.push_back(
        geometricSpeedSampleForSegment(trajectory, i, segment.s_start_m, config));
    candidates.push_back(
        geometricSpeedSampleForSegment(trajectory, i, segmentEndS(segment), config));
    if (segment.kind == TrajectorySegmentKind::kArc) {
      candidates.push_back(geometricSpeedSampleForSegment(
          trajectory, i, segment.s_start_m + segment.length_m * 0.5, config));
    }
  }

  std::ranges::sort(candidates, {}, &TrajectorySpeedSample::s_m);
  profile.samples.reserve(candidates.size());
  for (const TrajectorySpeedSample& candidate : candidates) {
    mergeSpeedSample(profile.samples, candidate);
  }

  if (profile.samples.empty()) {
    return profile;
  }
  profile.samples.back().profiled_limit_mps = 0.0;
  profile.samples.back().geometric_limit_mps = 0.0;
  profile.samples.back().reason = SpeedConstraintType::kGoal;
  profile.samples.back().constraint_s_m = profile.samples.back().s_m;
  profile.samples.back().constraint_limit_mps = 0.0;

  const double max_decel = effectiveVelocityDeltaDecelMps2(config);
  for (std::size_t i = profile.samples.size() - 1U; i > 0U; --i) {
    const double ds = profile.samples[i].s_m - profile.samples[i - 1U].s_m;
    const double allowed =
        limitedSpeedForDistance(profile.samples[i].profiled_limit_mps, ds, max_decel);
    if (allowed < profile.samples[i - 1U].profiled_limit_mps) {
      profile.samples[i - 1U].profiled_limit_mps = allowed;
      profile.samples[i - 1U].reason = profile.samples[i].reason;
      profile.samples[i - 1U].segment_index = profile.samples[i].segment_index;
      profile.samples[i - 1U].curvature_1pm = profile.samples[i].curvature_1pm;
      profile.samples[i - 1U].radius_m = profile.samples[i].radius_m;
      profile.samples[i - 1U].constraint_s_m = profile.samples[i].constraint_s_m;
      profile.samples[i - 1U].constraint_limit_mps =
          profile.samples[i].constraint_limit_mps;
    }
  }

  const double max_accel = effectiveVelocityDeltaAccelMps2(config);
  for (std::size_t i = 1U; i < profile.samples.size(); ++i) {
    const double ds = profile.samples[i].s_m - profile.samples[i - 1U].s_m;
    const double allowed = limitedSpeedForDistance(
        profile.samples[i - 1U].profiled_limit_mps, ds, max_accel);
    if (allowed < profile.samples[i].profiled_limit_mps) {
      profile.samples[i].profiled_limit_mps = allowed;
    }
  }

  profile.valid = true;
  return profile;
}

TrajectorySpeedSample speedProfileSampleAtS(const TrajectorySpeedProfile& profile,
                                            const double s_m) {
  if (!profile.valid || profile.samples.empty()) {
    return TrajectorySpeedSample{};
  }
  return profile.samples[firstProfileSampleNotBefore(profile, s_m)];
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
                                const std::size_t /*waypoint_index*/) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectoryFromPoints(path);
  return projectOnTrajectory(trajectory, current_position).has_value();
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

  const Point2 final_point = trajectory.back().end;
  const double final_acceptance =
      sanitizedPositive(config.final_acceptance_radius_m, 1.0, 0.0, 100.0);
  if (distance(current_position, final_point) <= final_acceptance) {
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

  const double current_speed =
      currentSpeedMps(current_velocity, current_velocity_valid, previous_state);
  const TrajectorySpeedSample speed_sample =
      speedProfileSampleAtS(speed_profile, projection->s_m);
  const double cruise_speed = sanitizedCruiseSpeed(config);
  const double raw_speed_limit =
      std::clamp(speed_sample.profiled_limit_mps, 0.0, cruise_speed);

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
      normalized(projection->tangent * std::max(accel_limited_speed, 1.0) + correction);
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
  if (speed_sample.reason == SpeedConstraintType::kArc &&
      raw_speed_limit + 1.0e-6 < cruise_speed) {
    plan.reason = VelocitySetpointReason::kBrakingForTurn;
  } else if (speed_sample.reason == SpeedConstraintType::kGoal &&
             raw_speed_limit + 1.0e-6 < cruise_speed) {
    plan.reason = VelocitySetpointReason::kFinalApproach;
  }
  plan.velocity_xy = limited_velocity.velocity;
  plan.path_tangent = projection->tangent;
  plan.projection = projection->point;
  plan.cross_track_correction_velocity = correction;
  plan.speed_mps = norm(plan.velocity_xy);
  plan.raw_speed_limit_mps = raw_speed_limit;
  plan.accel_limited_speed_mps = accel_limited_speed;
  plan.velocity_delta_mps = limited_velocity.delta_mps;
  plan.cross_track_correction_mps = norm(correction);
  plan.limiting_constraint_type = speed_sample.reason;
  plan.limiting_constraint_index = speed_sample.segment_index;
  plan.limiting_constraint_distance_m =
      speed_sample.reason == SpeedConstraintType::kNone
          ? std::numeric_limits<double>::quiet_NaN()
          : std::max(0.0, speed_sample.constraint_s_m - projection->s_m);
  plan.limiting_constraint_speed_mps = speed_sample.constraint_limit_mps;
  plan.limiting_allowed_speed_now_mps = speed_sample.profiled_limit_mps;
  plan.limiting_turn_angle_rad = std::numeric_limits<double>::quiet_NaN();
  plan.limiting_turn_radius_m = speed_sample.radius_m;
  plan.trajectory_s_m = projection->s_m;
  plan.trajectory_segment_index = projection->segment_index;
  plan.trajectory_segment_kind =
      trajectory[segmentIndexForS(trajectory, projection->s_m)].kind;
  plan.trajectory_curvature_1pm = projection->curvature_1pm;
  plan.trajectory_arc_radius_m = std::abs(projection->curvature_1pm) > kTinyDistanceM
                                     ? 1.0 / std::abs(projection->curvature_1pm)
                                     : std::numeric_limits<double>::quiet_NaN();
  plan.trajectory_projection = *projection;

  plan.turn.valid = speed_sample.reason == SpeedConstraintType::kArc;
  plan.turn.waypoint_index = speed_sample.segment_index;
  plan.turn.angle_rad = std::numeric_limits<double>::quiet_NaN();
  plan.turn.turn_radius_m = speed_sample.radius_m;
  plan.turn.distance_to_turn_m = plan.limiting_constraint_distance_m;
  plan.turn.target_turn_speed_mps = speed_sample.geometric_limit_mps;
  plan.turn.braking_distance_m = std::numeric_limits<double>::quiet_NaN();
  plan.turn.raw_speed_limit_mps = speed_sample.profiled_limit_mps;

  plan.final_stop.valid = true;
  plan.final_stop.distance_to_stop_m =
      distanceFromTrajectorySToEnd(trajectory, projection->s_m);
  plan.final_stop.braking_distance_m = std::numeric_limits<double>::quiet_NaN();
  plan.final_stop.raw_speed_limit_mps = speed_sample.profiled_limit_mps;

  return plan;
}

VelocitySetpointPlan
planVelocitySetpoint(const std::span<const Point2> path, const Point2 current_position,
                     const Point2 current_velocity, const bool current_velocity_valid,
                     const std::size_t /*waypoint_index*/, const double dt_s,
                     const VelocityFollowerState& previous_state,
                     const VelocityFollowerConfig& config) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectoryFromPoints(path);
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, config);
  return planVelocitySetpoint(trajectory, profile, current_position, current_velocity,
                              current_velocity_valid, dt_s, previous_state, config);
}

} // namespace drone_city_nav
