#pragma once

#include "drone_city_nav/trajectory.hpp"
#include "drone_city_nav/trajectory_speed_planner.hpp"
#include "drone_city_nav/types.hpp"
#include "drone_city_nav/velocity_control_config.hpp"
#include "drone_city_nav/velocity_smoother.hpp"

#include <cstddef>
#include <limits>
#include <span>

namespace drone_city_nav {

enum class VelocitySetpointReason {
  kInvalidPath,
  kHold,
  kStraight,
  kTrajectorySpeedProfile,
  kFinalApproach,
  kTerminalCapture,
};

struct VelocityFollowerState {
  Point2 previous_velocity_setpoint{};
  bool previous_velocity_setpoint_valid{false};
  Point2 previous_velocity_acceleration_setpoint{};
  bool previous_velocity_acceleration_setpoint_valid{false};
  Point2 previous_lateral_control_velocity{};
  bool previous_lateral_control_velocity_valid{false};
  double previous_scalar_speed_command_mps{std::numeric_limits<double>::quiet_NaN()};
  bool previous_scalar_speed_command_valid{false};
};

struct StopSpeedPlan {
  bool valid{false};
  double distance_to_stop_m{std::numeric_limits<double>::infinity()};
  double braking_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double raw_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
};

struct VelocitySetpointPlan {
  bool valid{false};
  bool final_goal_reached{false};
  VelocitySetpointReason reason{VelocitySetpointReason::kInvalidPath};
  Point2 velocity_xy{};
  Point2 desired_velocity_xy{};
  Point2 velocity_setpoint_acceleration_xy{};
  Point2 path_tangent{};
  Point2 control_tangent_raw{};
  Point2 projection{};
  Point2 current_projection{};
  Point2 predicted_position{};
  Point2 predicted_projection{};
  Point2 cross_track_feedback_velocity{};
  Point2 cross_track_derivative_damping_velocity{};
  Point2 curvature_feedforward_velocity{};
  Point2 raw_lateral_control_velocity{};
  Point2 lateral_control_velocity{};
  double speed_mps{0.0};
  double desired_speed_mps{0.0};
  double velocity_setpoint_acceleration_mps2{std::numeric_limits<double>::quiet_NaN()};
  double velocity_setpoint_jerk_mps3{std::numeric_limits<double>::quiet_NaN()};
  bool path_frame_lateral_smoothing_applied{false};
  double lateral_smoothing_factor{1.0};
  double smoother_lateral_response_accel_mps2{std::numeric_limits<double>::quiet_NaN()};
  double raw_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  double profile_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  double speed_lookahead_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double lookahead_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  SpeedConstraintType lookahead_limiting_constraint_type{SpeedConstraintType::kNone};
  std::size_t lookahead_limiting_constraint_index{0U};
  double lookahead_limiting_constraint_distance_m{
      std::numeric_limits<double>::quiet_NaN()};
  double speed_after_lookahead_mps{std::numeric_limits<double>::quiet_NaN()};
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
  double desired_to_setpoint_tangent_error_mps{
      std::numeric_limits<double>::quiet_NaN()};
  double desired_to_setpoint_normal_error_mps{std::numeric_limits<double>::quiet_NaN()};
  double setpoint_to_actual_tangent_error_mps{std::numeric_limits<double>::quiet_NaN()};
  double setpoint_to_actual_normal_error_mps{std::numeric_limits<double>::quiet_NaN()};
  double desired_to_actual_tangent_error_mps{std::numeric_limits<double>::quiet_NaN()};
  double desired_to_actual_normal_error_mps{std::numeric_limits<double>::quiet_NaN()};
  double cross_track_feedback_mps{0.0};
  double cross_track_derivative_damping_mps{0.0};
  double cross_track_derivative_damping_factor{1.0};
  double cross_track_derivative_gain_effective{0.0};
  double cross_track_lateral_velocity_mps{std::numeric_limits<double>::quiet_NaN()};
  bool control_tangent_smoothed{false};
  double control_tangent_smoothing_heading_span_rad{
      std::numeric_limits<double>::quiet_NaN()};
  double control_tangent_smoothing_max_abs_curvature_1pm{
      std::numeric_limits<double>::quiet_NaN()};
  double control_tangent_smoothing_window_start_s_m{
      std::numeric_limits<double>::quiet_NaN()};
  double control_tangent_smoothing_window_end_s_m{
      std::numeric_limits<double>::quiet_NaN()};
  double curvature_feedforward_mps{0.0};
  double curvature_feedforward_angle_rad{0.0};
  double curvature_feedforward_raw_angle_rad{0.0};
  double curvature_feedforward_scale{1.0};
  double raw_lateral_control_mps{0.0};
  double lateral_control_mps{0.0};
  double lateral_control_delta_mps{std::numeric_limits<double>::quiet_NaN()};
  double adaptive_lateral_response_factor{1.0};
  bool terminal_capture_active{false};
  double terminal_goal_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double terminal_remaining_trajectory_distance_m{
      std::numeric_limits<double>::quiet_NaN()};
  double terminal_acceptance_radius_m{std::numeric_limits<double>::quiet_NaN()};
  double terminal_hold_max_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  bool terminal_hold_distance_met{false};
  bool terminal_hold_speed_met{false};
  bool terminal_capture_goal_distance_triggered{false};
  bool terminal_capture_remaining_distance_triggered{false};
  double terminal_capture_gain_speed_limit_mps{
      std::numeric_limits<double>::quiet_NaN()};
  double terminal_capture_max_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double terminal_capture_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  double trajectory_cross_track_error_m{std::numeric_limits<double>::quiet_NaN()};
  double current_cross_track_error_m{std::numeric_limits<double>::quiet_NaN()};
  double predicted_cross_track_error_m{std::numeric_limits<double>::quiet_NaN()};
  double prediction_horizon_s{0.0};
  double prediction_distance_m{0.0};
  double response_delay_distance_m{std::numeric_limits<double>::quiet_NaN()};
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

[[nodiscard]] VelocitySetpointPlan planVelocitySetpoint(
    std::span<const TrajectorySegment> trajectory,
    const TrajectorySpeedProfile& speed_profile, Point2 current_position,
    Point2 current_velocity, bool current_velocity_valid, double dt_s,
    const VelocityFollowerState& previous_state, const VelocityFollowerConfig& config);

[[nodiscard]] VelocitySetpointPlan planVelocitySetpoint(
    std::span<const TrajectoryPointSample> trajectory_samples,
    const TrajectorySpeedProfile& speed_profile, Point2 current_position,
    Point2 current_velocity, bool current_velocity_valid, double dt_s,
    const VelocityFollowerState& previous_state, const VelocityFollowerConfig& config);

} // namespace drone_city_nav
