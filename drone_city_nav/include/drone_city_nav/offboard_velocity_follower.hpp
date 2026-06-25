#pragma once

#include "drone_city_nav/trajectory.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <limits>
#include <span>
#include <vector>

namespace drone_city_nav {

enum class VelocitySetpointReason {
  kInvalidPath,
  kHold,
  kStraight,
  kTrajectorySpeedProfile,
  kFinalApproach,
};

enum class SpeedConstraintType {
  kNone,
  kArc,
  kGoal,
};

struct VelocityFollowerConfig {
  double cruise_speed_mps{12.0};
  double min_turn_speed_mps{2.0};
  double max_accel_mps2{3.0};
  double max_decel_mps2{4.0};
  double max_lateral_accel_mps2{3.0};
  double speed_profile_decel_mps2{2.0};
  double speed_profile_sample_step_m{1.0};
  double cross_track_gain{0.5};
  double cross_track_derivative_gain{0.8};
  double max_cross_track_correction_angle_rad{0.7853981633974483};
  double max_cross_track_correction_rate_mps2{4.0};
  double cross_track_speed_guard_start_m{2.0};
  double cross_track_speed_guard_full_m{6.0};
  double cross_track_speed_guard_min_factor{0.35};
  double max_feedforward_accel_mps2{3.0};
  double max_feedforward_jerk_mps3{12.0};
  double max_velocity_jerk_mps3{12.0};
  double acceleration_feedforward_scale{1.0};
  double final_acceptance_radius_m{1.0};
  double final_hold_max_speed_mps{0.8};
};

struct VelocityFollowerState {
  Point2 previous_velocity_setpoint{};
  bool previous_velocity_setpoint_valid{false};
  Point2 previous_velocity_acceleration_setpoint{};
  bool previous_velocity_acceleration_setpoint_valid{false};
  Point2 previous_feedforward_acceleration_setpoint{};
  bool previous_feedforward_acceleration_setpoint_valid{false};
  Point2 previous_cross_track_correction_velocity{};
  bool previous_cross_track_correction_velocity_valid{false};
};

struct StopSpeedPlan {
  bool valid{false};
  double distance_to_stop_m{std::numeric_limits<double>::infinity()};
  double braking_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double raw_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
};

struct TrajectorySpeedSample {
  double s_m{0.0};
  double geometric_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  double profiled_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  SpeedConstraintType reason{SpeedConstraintType::kNone};
  std::size_t segment_index{0U};
  double curvature_1pm{0.0};
  double radius_m{std::numeric_limits<double>::quiet_NaN()};
  double constraint_s_m{std::numeric_limits<double>::quiet_NaN()};
  double constraint_limit_mps{std::numeric_limits<double>::quiet_NaN()};
};

struct TrajectorySpeedProfile {
  std::vector<TrajectorySpeedSample> samples;
  bool valid{false};
};

struct TraversalTimeEstimate {
  bool valid{false};
  double estimated_time_s{std::numeric_limits<double>::quiet_NaN()};
  double min_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  double max_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  std::size_t curvature_limited_samples{0U};
};

struct VelocityVectorLimitResult {
  Point2 velocity{};
  double delta_mps{0.0};
};

struct VelocitySetpointPlan {
  bool valid{false};
  bool final_goal_reached{false};
  VelocitySetpointReason reason{VelocitySetpointReason::kInvalidPath};
  Point2 velocity_xy{};
  Point2 desired_velocity_xy{};
  Point2 acceleration_xy{};
  Point2 raw_acceleration_xy{};
  Point2 velocity_setpoint_acceleration_xy{};
  Point2 path_tangent{};
  Point2 projection{};
  Point2 raw_cross_track_correction_velocity{};
  Point2 cross_track_correction_velocity{};
  double speed_mps{0.0};
  double desired_speed_mps{0.0};
  double acceleration_xy_mps2{0.0};
  double raw_acceleration_xy_mps2{0.0};
  double velocity_setpoint_acceleration_mps2{std::numeric_limits<double>::quiet_NaN()};
  double velocity_setpoint_jerk_mps3{std::numeric_limits<double>::quiet_NaN()};
  double acceleration_delta_mps2{std::numeric_limits<double>::quiet_NaN()};
  double acceleration_jerk_mps3{std::numeric_limits<double>::quiet_NaN()};
  double curvature_feedforward_accel_mps2{0.0};
  double raw_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  double profile_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  double cross_track_speed_factor{1.0};
  double cross_track_limited_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double final_command_speed_mps{0.0};
  double accel_limited_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double velocity_delta_mps{std::numeric_limits<double>::quiet_NaN()};
  double desired_velocity_delta_mps{std::numeric_limits<double>::quiet_NaN()};
  double velocity_tracking_error_mps{std::numeric_limits<double>::quiet_NaN()};
  double current_velocity_tangent_mps{std::numeric_limits<double>::quiet_NaN()};
  double current_velocity_normal_mps{std::numeric_limits<double>::quiet_NaN()};
  double desired_velocity_tangent_mps{std::numeric_limits<double>::quiet_NaN()};
  double desired_velocity_normal_mps{std::numeric_limits<double>::quiet_NaN()};
  double setpoint_velocity_tangent_mps{std::numeric_limits<double>::quiet_NaN()};
  double setpoint_velocity_normal_mps{std::numeric_limits<double>::quiet_NaN()};
  double raw_cross_track_correction_mps{0.0};
  double cross_track_correction_mps{0.0};
  double cross_track_correction_delta_mps{std::numeric_limits<double>::quiet_NaN()};
  double cross_track_lateral_velocity_mps{std::numeric_limits<double>::quiet_NaN()};
  double trajectory_cross_track_error_m{std::numeric_limits<double>::quiet_NaN()};
  SpeedConstraintType limiting_constraint_type{SpeedConstraintType::kNone};
  std::size_t limiting_constraint_index{0U};
  double limiting_constraint_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double limiting_curve_radius_m{std::numeric_limits<double>::quiet_NaN()};
  double limiting_constraint_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double limiting_allowed_speed_now_mps{std::numeric_limits<double>::quiet_NaN()};
  double trajectory_s_m{std::numeric_limits<double>::quiet_NaN()};
  std::size_t trajectory_segment_index{0U};
  TrajectorySegmentKind trajectory_segment_kind{TrajectorySegmentKind::kLine};
  double trajectory_curvature_1pm{0.0};
  double trajectory_arc_radius_m{std::numeric_limits<double>::quiet_NaN()};
  TrajectoryProjection trajectory_projection{};
  StopSpeedPlan final_stop{};
};

[[nodiscard]] const char*
velocitySetpointReasonName(VelocitySetpointReason reason) noexcept;

[[nodiscard]] const char*
speedConstraintTypeName(SpeedConstraintType constraint_type) noexcept;

[[nodiscard]] double
distanceFromTrajectorySToEnd(std::span<const TrajectorySegment> trajectory, double s_m);

[[nodiscard]] TrajectorySpeedProfile
buildTrajectorySpeedProfile(std::span<const TrajectorySegment> trajectory,
                            const VelocityFollowerConfig& config);

[[nodiscard]] TrajectorySpeedProfile
buildTrajectorySpeedProfile(std::span<const TrajectoryPointSample> trajectory_samples,
                            const VelocityFollowerConfig& config);

[[nodiscard]] TrajectorySpeedSample
speedProfileSampleAtS(const TrajectorySpeedProfile& profile, double s_m);

[[nodiscard]] TraversalTimeEstimate
estimateTraversalTime(std::span<const TrajectoryPointSample> trajectory_samples,
                      const VelocityFollowerConfig& config,
                      bool use_forward_backward_profile);

[[nodiscard]] VelocityVectorLimitResult
limitVelocityVectorDelta(Point2 desired_velocity, Point2 previous_velocity,
                         bool previous_velocity_valid, double dt_s,
                         double max_delta_mps2);

[[nodiscard]] VelocityVectorLimitResult
limitVelocityVectorDelta(Point2 desired_velocity, Point2 previous_velocity,
                         bool previous_velocity_valid, double dt_s,
                         double max_accel_mps2, double max_decel_mps2);

[[nodiscard]] VelocitySetpointPlan planVelocitySetpoint(
    std::span<const TrajectorySegment> trajectory,
    const TrajectorySpeedProfile& speed_profile, Point2 current_position,
    Point2 current_velocity, bool current_velocity_valid, double dt_s,
    const VelocityFollowerState& previous_state, const VelocityFollowerConfig& config);

} // namespace drone_city_nav
