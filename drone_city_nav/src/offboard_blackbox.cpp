#include "drone_city_nav/offboard_blackbox.hpp"

#include "drone_city_nav/trajectory_diagnostics_io.hpp"

#include <cmath>
#include <numbers>
#include <ostream>

namespace drone_city_nav {

void writeBlackboxJsonBool(std::ostream& stream, const bool value) {
  stream << (value ? "true" : "false");
}

void writeBlackboxJsonNumberOrNull(std::ostream& stream, const double value) {
  if (std::isfinite(value)) {
    stream << value;
    return;
  }
  stream << "null";
}

void writeBlackboxPathId(std::ostream& stream, const OffboardBlackboxPathId& path_id) {
  stream << "\"path_id\":{\"local_update\":" << path_id.local_update
         << ",\"planner\":" << path_id.planner << ",\"planner_seen\":";
  writeBlackboxJsonBool(stream, path_id.planner_seen);
  stream << ",\"stamp_ns\":" << path_id.stamp_ns << "}";
}

void writeBlackboxStringField(std::ostream& stream, const std::string_view key,
                              const std::string_view value) {
  stream << "\"" << key << "\":\"" << value << "\"";
}

void writeOffboardBlackboxRecord(std::ostream& stream,
                                 const OffboardBlackboxRecord& record) {
  const VelocitySetpointPlan& velocity_plan = record.velocity_plan;
  const TrajectoryPlannerStats& planner_stats = record.trajectory_planner_stats;
  const TrajectoryShapeDiagnostics& shape = record.trajectory_shape_diagnostics;

  stream << "{\"time_ns\":" << record.time_ns << ",";
  writeBlackboxPathId(stream, record.path_id);
  stream << ",\"pose\":{\"fresh\":";
  writeBlackboxJsonBool(stream, record.pose_fresh);
  stream << ",\"age_s\":";
  writeBlackboxJsonNumberOrNull(stream, record.pose_age_s);
  stream << ",\"x\":";
  writeBlackboxJsonNumberOrNull(stream, record.current_position.x);
  stream << ",\"y\":";
  writeBlackboxJsonNumberOrNull(stream, record.current_position.y);
  stream << ",\"altitude_m\":";
  writeBlackboxJsonNumberOrNull(stream, record.current_altitude_m);
  stream << ",\"heading_rad\":";
  writeBlackboxJsonNumberOrNull(stream, record.current_heading_rad);
  stream << "}";
  stream << ",\"attitude\":{\"valid\":";
  writeBlackboxJsonBool(stream, record.attitude_valid);
  stream << ",\"age_s\":";
  writeBlackboxJsonNumberOrNull(stream, record.attitude_age_s);
  stream << ",\"roll_rad\":";
  writeBlackboxJsonNumberOrNull(stream, record.current_attitude.roll_rad);
  stream << ",\"pitch_rad\":";
  writeBlackboxJsonNumberOrNull(stream, record.current_attitude.pitch_rad);
  stream << ",\"yaw_rad\":";
  writeBlackboxJsonNumberOrNull(stream, record.current_attitude.yaw_rad);
  stream << ",\"tilt_deg\":";
  writeBlackboxJsonNumberOrNull(stream, std::hypot(record.current_attitude.roll_rad,
                                                   record.current_attitude.pitch_rad) *
                                            180.0 / std::numbers::pi);
  stream << "}";
  stream << ",\"velocity\":{\"valid\":";
  writeBlackboxJsonBool(stream, record.current_velocity_valid);
  stream << ",\"x\":";
  writeBlackboxJsonNumberOrNull(stream, record.current_velocity.x);
  stream << ",\"y\":";
  writeBlackboxJsonNumberOrNull(stream, record.current_velocity.y);
  stream << ",\"speed_mps\":";
  writeBlackboxJsonNumberOrNull(stream, record.current_speed_mps);
  stream << "}";
  stream << ",\"target\":{\"x\":";
  writeBlackboxJsonNumberOrNull(stream, record.target.x);
  stream << ",\"y\":";
  writeBlackboxJsonNumberOrNull(stream, record.target.y);
  stream << ",\"distance_m\":";
  writeBlackboxJsonNumberOrNull(stream, record.target_distance_m);
  stream << ",\"delta_m\":";
  writeBlackboxJsonNumberOrNull(stream, record.last_commanded_target_delta_m);
  stream << "}";
  stream << ",\"command\":{\"yaw_rad\":";
  writeBlackboxJsonNumberOrNull(stream, record.last_commanded_yaw_rad);
  stream << "}";

  stream << ",\"velocity_command\":{\"control_mode\":\"" << record.control_mode
         << "\",\"setpoint_x\":";
  writeBlackboxJsonNumberOrNull(stream, record.last_velocity_setpoint.x);
  stream << ",\"setpoint_y\":";
  writeBlackboxJsonNumberOrNull(stream, record.last_velocity_setpoint.y);
  stream << ",\"setpoint_z\":";
  writeBlackboxJsonNumberOrNull(stream, record.last_vertical_velocity_setpoint_mps);
  stream << ",\"setpoint_speed_mps\":";
  writeBlackboxJsonNumberOrNull(stream, record.last_velocity_setpoint_speed_mps);
  stream << ",\"final_command_speed_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.final_command_speed_mps);
  stream << ",\"smoother_reset_reason\":\"" << record.velocity_smoother_reset_reason
         << "\"";
  stream << ",\"path_update_reset_count\":"
         << record.path_update_velocity_smoother_reset_count;
  stream << ",\"desired_setpoint_x\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.desired_velocity_xy.x);
  stream << ",\"desired_setpoint_y\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.desired_velocity_xy.y);
  stream << ",\"desired_setpoint_speed_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.desired_speed_mps);
  stream << ",\"cross_track_feedback_x\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.cross_track_feedback_velocity.x);
  stream << ",\"cross_track_feedback_y\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.cross_track_feedback_velocity.y);
  stream << ",\"cross_track_feedback_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.cross_track_feedback_mps);
  stream << ",\"cross_track_feedback_scale\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.cross_track_feedback_scale);
  stream << ",\"cross_track_closing_speed_target_mps\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.cross_track_closing_speed_target_mps);
  stream << ",\"cross_track_derivative_damping_x\":";
  writeBlackboxJsonNumberOrNull(
      stream, velocity_plan.cross_track_derivative_damping_velocity.x);
  stream << ",\"cross_track_derivative_damping_y\":";
  writeBlackboxJsonNumberOrNull(
      stream, velocity_plan.cross_track_derivative_damping_velocity.y);
  stream << ",\"cross_track_derivative_damping_mps\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.cross_track_derivative_damping_mps);
  stream << ",\"cross_track_overshoot_damping_x\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.cross_track_overshoot_damping_velocity.x);
  stream << ",\"cross_track_overshoot_damping_y\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.cross_track_overshoot_damping_velocity.y);
  stream << ",\"cross_track_overshoot_damping_mps\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.cross_track_overshoot_damping_mps);
  stream << ",\"actual_signed_cross_track_error_m\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.actual_signed_cross_track_error_m);
  stream << ",\"actual_cross_track_lateral_velocity_mps\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.actual_cross_track_lateral_velocity_mps);
  stream << ",\"actual_cross_track_closing_speed_mps\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.actual_cross_track_closing_speed_mps);
  stream << ",\"actual_cross_track_closing_speed_limit_mps\":";
  writeBlackboxJsonNumberOrNull(
      stream, velocity_plan.actual_cross_track_closing_speed_limit_mps);
  stream << ",\"control_tangent_smoothed\":";
  writeBlackboxJsonBool(stream, velocity_plan.control_tangent_smoothed);
  stream << ",\"control_tangent_raw_x\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.control_tangent_raw.x);
  stream << ",\"control_tangent_raw_y\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.control_tangent_raw.y);
  stream << ",\"control_tangent_smoothing_heading_span_rad\":";
  writeBlackboxJsonNumberOrNull(
      stream, velocity_plan.control_tangent_smoothing_heading_span_rad);
  stream << ",\"control_tangent_smoothing_max_abs_curvature_1pm\":";
  writeBlackboxJsonNumberOrNull(
      stream, velocity_plan.control_tangent_smoothing_max_abs_curvature_1pm);
  stream << ",\"control_tangent_smoothing_window_start_s_m\":";
  writeBlackboxJsonNumberOrNull(
      stream, velocity_plan.control_tangent_smoothing_window_start_s_m);
  stream << ",\"control_tangent_smoothing_window_end_s_m\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.control_tangent_smoothing_window_end_s_m);
  stream << ",\"cross_track_derivative_damping_factor\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.cross_track_derivative_damping_factor);
  stream << ",\"cross_track_derivative_gain_effective\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.cross_track_derivative_gain_effective);
  stream << ",\"curvature_feedforward_x\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.curvature_feedforward_velocity.x);
  stream << ",\"curvature_feedforward_y\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.curvature_feedforward_velocity.y);
  stream << ",\"curvature_feedforward_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.curvature_feedforward_mps);
  stream << ",\"curvature_feedforward_angle_rad\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.curvature_feedforward_angle_rad);
  stream << ",\"curvature_feedforward_raw_angle_rad\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.curvature_feedforward_raw_angle_rad);
  stream << ",\"curvature_feedforward_scale\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.curvature_feedforward_scale);
  stream << ",\"raw_lateral_control_x\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.raw_lateral_control_velocity.x);
  stream << ",\"raw_lateral_control_y\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.raw_lateral_control_velocity.y);
  stream << ",\"raw_lateral_control_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.raw_lateral_control_mps);
  stream << ",\"lateral_control_x\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.lateral_control_velocity.x);
  stream << ",\"lateral_control_y\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.lateral_control_velocity.y);
  stream << ",\"lateral_control_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.lateral_control_mps);
  stream << ",\"lateral_control_delta_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.lateral_control_delta_mps);
  stream << ",\"adaptive_lateral_response_factor\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.adaptive_lateral_response_factor);
  stream << ",\"velocity_setpoint_accel_x\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.velocity_setpoint_acceleration_xy.x);
  stream << ",\"velocity_setpoint_accel_y\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.velocity_setpoint_acceleration_xy.y);
  stream << ",\"velocity_setpoint_accel_norm_mps2\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.velocity_setpoint_acceleration_mps2);
  stream << ",\"velocity_setpoint_jerk_mps3\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.velocity_setpoint_jerk_mps3);
  stream << ",\"path_frame_lateral_smoothing_applied\":";
  writeBlackboxJsonBool(stream, velocity_plan.path_frame_lateral_smoothing_applied);
  stream << ",\"lateral_smoothing_factor\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.lateral_smoothing_factor);
  stream << ",\"smoother_lateral_response_accel_mps2\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.smoother_lateral_response_accel_mps2);
  stream << ",\"speed_limit_reason\":\""
         << velocitySetpointReasonName(velocity_plan.reason)
         << "\",\"terminal_capture_active\":";
  writeBlackboxJsonBool(stream, velocity_plan.terminal_capture_active);
  stream << ",\"terminal_goal_distance_m\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.terminal_goal_distance_m);
  stream << ",\"terminal_signed_along_track_distance_m\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.terminal_signed_along_track_distance_m);
  stream << ",\"terminal_remaining_trajectory_distance_m\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.terminal_remaining_trajectory_distance_m);
  stream << ",\"terminal_acceptance_radius_m\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.terminal_acceptance_radius_m);
  stream << ",\"terminal_hold_max_speed_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.terminal_hold_max_speed_mps);
  stream << ",\"terminal_hold_distance_met\":";
  writeBlackboxJsonBool(stream, velocity_plan.terminal_hold_distance_met);
  stream << ",\"terminal_hold_speed_met\":";
  writeBlackboxJsonBool(stream, velocity_plan.terminal_hold_speed_met);
  stream << ",\"terminal_capture_goal_distance_triggered\":";
  writeBlackboxJsonBool(stream, velocity_plan.terminal_capture_goal_distance_triggered);
  stream << ",\"terminal_capture_remaining_distance_triggered\":";
  writeBlackboxJsonBool(stream,
                        velocity_plan.terminal_capture_remaining_distance_triggered);
  stream << ",\"terminal_capture_gain_speed_limit_mps\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.terminal_capture_gain_speed_limit_mps);
  stream << ",\"terminal_capture_max_speed_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.terminal_capture_max_speed_mps);
  stream << ",\"terminal_capture_decel_mps2\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.terminal_capture_decel_mps2);
  stream << ",\"terminal_capture_braking_margin_m\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.terminal_capture_braking_margin_m);
  stream << ",\"terminal_capture_braking_distance_m\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.terminal_capture_braking_distance_m);
  stream << ",\"terminal_capture_activation_distance_m\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.terminal_capture_activation_distance_m);
  stream << ",\"terminal_capture_braking_speed_limit_mps\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.terminal_capture_braking_speed_limit_mps);
  stream << ",\"terminal_capture_speed_limit_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.terminal_capture_speed_limit_mps);
  stream << ",\"raw_speed_limit_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.raw_speed_limit_mps);
  stream << ",\"profile_speed_limit_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.profile_speed_limit_mps);
  stream << ",\"lookahead_distance_m\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.speed_lookahead_distance_m);
  stream << ",\"lookahead_speed_limit_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.lookahead_speed_limit_mps);
  stream << ",\"speed_after_lookahead_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.speed_after_lookahead_mps);
  stream << ",\"lookahead_limiting_constraint_type\":\""
         << speedConstraintTypeName(velocity_plan.lookahead_limiting_constraint_type)
         << "\",\"lookahead_limiting_constraint_index\":"
         << velocity_plan.lookahead_limiting_constraint_index;
  stream << ",\"lookahead_limiting_constraint_distance_m\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.lookahead_limiting_constraint_distance_m);
  stream << ",\"cross_track_speed_factor\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.cross_track_speed_factor);
  stream << ",\"cross_track_limited_speed_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.cross_track_limited_speed_mps);
  stream << ",\"accel_limited_speed_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.accel_limited_speed_mps);
  stream << ",\"limiting_constraint_type\":\""
         << speedConstraintTypeName(velocity_plan.limiting_constraint_type)
         << "\",\"limiting_constraint_index\":"
         << velocity_plan.limiting_constraint_index;
  stream << ",\"limiting_constraint_distance_m\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.limiting_constraint_distance_m);
  stream << ",\"limiting_constraint_speed_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.limiting_constraint_speed_mps);
  stream << ",\"limiting_allowed_speed_now_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.limiting_allowed_speed_now_mps);
  stream << ",\"limiting_curve_radius_m\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.limiting_curve_radius_m);
  stream << ",\"final_stop_distance_m\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.final_stop.distance_to_stop_m);
  stream << ",\"final_stop_braking_distance_m\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.final_stop.braking_distance_m);
  stream << ",\"velocity_delta_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.velocity_delta_mps);
  stream << ",\"desired_velocity_delta_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.desired_velocity_delta_mps);
  stream << ",\"velocity_tracking_error_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.velocity_tracking_error_mps);
  stream << ",\"cross_track_lateral_velocity_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.cross_track_lateral_velocity_mps);
  stream << ",\"current_velocity_tangent_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.current_velocity_tangent_mps);
  stream << ",\"current_velocity_normal_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.current_velocity_normal_mps);
  stream << ",\"desired_velocity_tangent_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.desired_velocity_tangent_mps);
  stream << ",\"desired_velocity_normal_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.desired_velocity_normal_mps);
  stream << ",\"setpoint_velocity_tangent_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.setpoint_velocity_tangent_mps);
  stream << ",\"setpoint_velocity_normal_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.setpoint_velocity_normal_mps);
  stream << ",\"desired_to_setpoint_tangent_error_mps\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.desired_to_setpoint_tangent_error_mps);
  stream << ",\"desired_to_setpoint_normal_error_mps\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.desired_to_setpoint_normal_error_mps);
  stream << ",\"setpoint_to_actual_tangent_error_mps\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.setpoint_to_actual_tangent_error_mps);
  stream << ",\"setpoint_to_actual_normal_error_mps\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.setpoint_to_actual_normal_error_mps);
  stream << ",\"desired_to_actual_tangent_error_mps\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.desired_to_actual_tangent_error_mps);
  stream << ",\"desired_to_actual_normal_error_mps\":";
  writeBlackboxJsonNumberOrNull(stream,
                                velocity_plan.desired_to_actual_normal_error_mps);
  stream << ",\"trajectory_cross_track_error_m\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.trajectory_cross_track_error_m);
  stream << ",\"altitude_error_m\":";
  writeBlackboxJsonNumberOrNull(stream, record.last_altitude_error_m);
  stream << ",\"path_tangent_x\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.path_tangent.x);
  stream << ",\"path_tangent_y\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.path_tangent.y);
  stream << ",\"projection_x\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.projection.x);
  stream << ",\"projection_y\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.projection.y);
  stream << ",\"tracking_prediction_horizon_s\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.prediction_horizon_s);
  stream << ",\"tracking_prediction_distance_m\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.prediction_distance_m);
  stream << ",\"tracking_predicted_x\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.predicted_position.x);
  stream << ",\"tracking_predicted_y\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.predicted_position.y);
  stream << ",\"current_projection_x\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.current_projection.x);
  stream << ",\"current_projection_y\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.current_projection.y);
  stream << ",\"predicted_projection_x\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.predicted_projection.x);
  stream << ",\"predicted_projection_y\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.predicted_projection.y);
  stream << ",\"current_cross_track_error_m\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.current_cross_track_error_m);
  stream << ",\"predicted_cross_track_error_m\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.predicted_cross_track_error_m);
  stream << ",\"response_delay_distance_m\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.response_delay_distance_m);
  stream << ",\"trajectory_valid\":";
  writeBlackboxJsonBool(stream, record.trajectory_valid);
  stream << ",\"trajectory_s_m\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.trajectory_s_m);
  stream << ",\"trajectory_segment_index\":" << velocity_plan.trajectory_segment_index;
  stream << ",\"trajectory_segment_type\":\""
         << trajectorySegmentKindName(velocity_plan.trajectory_segment_kind) << "\"";
  stream << ",\"trajectory_curvature_1pm\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.trajectory_curvature_1pm);
  stream << ",\"trajectory_arc_radius_m\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.trajectory_arc_radius_m);
  stream << ",\"trajectory_total_length_m\":";
  writeBlackboxJsonNumberOrNull(stream, record.trajectory_metrics.length_m);
  stream << ",\"trajectory_line_segments\":" << record.trajectory_metrics.line_segments;
  stream << ",\"trajectory_arc_segments\":" << record.trajectory_metrics.arc_segments;
  stream << ",\"speed_profile_limit_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.profile_speed_limit_mps);
  stream << ",\"speed_profile_lookahead_distance_m\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.speed_lookahead_distance_m);
  stream << ",\"speed_profile_lookahead_limit_mps\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.lookahead_speed_limit_mps);
  stream << ",\"speed_profile_reason\":\""
         << speedConstraintTypeName(velocity_plan.limiting_constraint_type) << "\"";
  stream << ",\"speed_profile_distance_to_constraint_m\":";
  writeBlackboxJsonNumberOrNull(stream, velocity_plan.limiting_constraint_distance_m);
  stream << ",\"final_trajectory_samples\":" << record.final_trajectory_samples;
  stream << ",\"trajectory_planner_status\":\""
         << trajectoryPlannerStatusName(planner_stats.status) << "\"";
  stream << ",\"trajectory_quality\":\"" << trajectoryQualityName(planner_stats.quality)
         << "\"";
  stream << "," << trajectoryTimingDiagnosticsJsonFields(planner_stats);
  stream << ",\"corridor_samples\":" << planner_stats.corridor.samples;
  stream << ",\"corridor_width_min_m\":";
  writeBlackboxJsonNumberOrNull(stream, planner_stats.corridor.min_width_m);
  stream << ",\"corridor_width_mean_m\":";
  writeBlackboxJsonNumberOrNull(stream, planner_stats.corridor.mean_width_m);
  stream << ",\"corridor_width_max_m\":";
  writeBlackboxJsonNumberOrNull(stream, planner_stats.corridor.max_width_m);
  stream << ",\"corridor_lateral_limited_samples\":"
         << planner_stats.corridor.lateral_limited_samples;
  stream << ",\"corridor_center_recovered_samples\":"
         << planner_stats.corridor.center_recovered_samples;
  stream << ",\"corridor_center_unrecoverable_samples\":"
         << planner_stats.corridor.center_unrecoverable_samples;
  stream << ",\"corridor_center_recovery_max_m\":";
  writeBlackboxJsonNumberOrNull(stream, planner_stats.corridor.max_center_recovery_m);
  stream << ",\"corridor_lateral_reduction_max_m\":";
  writeBlackboxJsonNumberOrNull(stream,
                                planner_stats.corridor.max_lateral_bound_reduction_m);
  stream << ",\"corridor_parallel_workers_used\":"
         << planner_stats.corridor.parallel_workers_used;
  stream << ",\"corridor_sample_build_duration_ms\":";
  writeBlackboxJsonNumberOrNull(stream,
                                planner_stats.corridor.sample_build_duration_ms);
  stream << ",\"corridor_raycast_duration_ms\":";
  writeBlackboxJsonNumberOrNull(stream, planner_stats.corridor.raycast_duration_ms);
  stream << ",\"corridor_lateral_limit_duration_ms\":";
  writeBlackboxJsonNumberOrNull(stream,
                                planner_stats.corridor.lateral_limit_duration_ms);
  stream << ",\"corridor_clearance_field_build_ms\":";
  writeBlackboxJsonNumberOrNull(
      stream, planner_stats.corridor.clearance_field_build_duration_ms);
  stream << ",\"clearance_field_reused_by_corridor\":"
         << (planner_stats.corridor.clearance_field_reused ? "true" : "false");
  stream << ",\"corridor_clearance_field_cache_hit\":"
         << (planner_stats.corridor.clearance_field_cache_hit ? "true" : "false");
  stream << ",\"trajectory_optimizer_iterations\":"
         << planner_stats.trajectory_optimizer.iterations;
  stream << ",\"trajectory_optimizer_optimizer_samples\":"
         << planner_stats.trajectory_optimizer.optimizer_samples;
  stream << ",\"trajectory_optimizer_cost_initial\":";
  writeBlackboxJsonNumberOrNull(stream,
                                planner_stats.trajectory_optimizer.initial_cost);
  stream << ",\"trajectory_optimizer_cost_final\":";
  writeBlackboxJsonNumberOrNull(stream, planner_stats.trajectory_optimizer.final_cost);
  stream << ",\"trajectory_optimizer_max_offset_m\":";
  writeBlackboxJsonNumberOrNull(stream,
                                planner_stats.trajectory_optimizer.max_abs_offset_m);
  stream << "," << trajectoryOptimizerDiagnosticsJsonFields(planner_stats);
  stream << "," << turnSmoothingDiagnosticsJsonFields(planner_stats);
  stream << ",\"trajectory_optimizer_time_final_s\":";
  writeBlackboxJsonNumberOrNull(stream,
                                planner_stats.trajectory_optimizer.estimated_time_s);
  stream << ",\"trajectory_optimizer_best_candidate_score\":";
  writeBlackboxJsonNumberOrNull(
      stream, planner_stats.trajectory_optimizer.best_candidate_score);
  stream << ",\"trajectory_optimizer_speed_limit_min_mps\":";
  writeBlackboxJsonNumberOrNull(stream,
                                planner_stats.trajectory_optimizer.min_speed_limit_mps);
  stream << ",\"trajectory_optimizer_speed_limit_max_mps\":";
  writeBlackboxJsonNumberOrNull(stream,
                                planner_stats.trajectory_optimizer.max_speed_limit_mps);
  stream << ",\"trajectory_optimizer_curvature_limited_samples\":"
         << planner_stats.trajectory_optimizer.curvature_limited_samples;
  stream << ",\"trajectory_optimizer_regularization_applied\":";
  writeBlackboxJsonBool(stream,
                        planner_stats.trajectory_optimizer.regularization_applied);
  stream << ",\"trajectory_optimizer_regularization_iterations\":"
         << planner_stats.trajectory_optimizer.regularization_iterations;
  stream << ",\"trajectory_optimizer_pre_regularization_curvature_jump_1pm\":";
  writeBlackboxJsonNumberOrNull(
      stream,
      planner_stats.trajectory_optimizer.pre_regularization_max_curvature_jump_1pm);
  stream << ",\"trajectory_optimizer_post_regularization_curvature_jump_1pm\":";
  writeBlackboxJsonNumberOrNull(
      stream,
      planner_stats.trajectory_optimizer.post_regularization_max_curvature_jump_1pm);
  stream << ",\"curvature_min_1pm\":";
  writeBlackboxJsonNumberOrNull(stream, planner_stats.curvature_min_1pm);
  stream << ",\"curvature_max_1pm\":";
  writeBlackboxJsonNumberOrNull(stream, planner_stats.curvature_max_1pm);
  stream << ",\"curvature_mean_abs_1pm\":";
  writeBlackboxJsonNumberOrNull(stream, planner_stats.curvature_mean_abs_1pm);
  stream << ",\"speed_profile_min_mps\":";
  writeBlackboxJsonNumberOrNull(stream, planner_stats.speed_profile_min_mps);
  stream << ",\"speed_profile_max_mps\":";
  writeBlackboxJsonNumberOrNull(stream, planner_stats.speed_profile_max_mps);
  stream << ",\"speed_profile_mean_mps\":";
  writeBlackboxJsonNumberOrNull(stream, planner_stats.speed_profile_mean_mps);
  stream << ",\"speed_profile_limited_by_curvature_count\":"
         << planner_stats.speed_profile_curvature_limited_samples;
  stream << "," << speedProfileConstraintDiagnosticsJsonFields(planner_stats);
  stream << ",\"trajectory_shape_segment_count\":" << shape.segment_count;
  stream << ",\"trajectory_shape_segment_len_min_m\":";
  writeBlackboxJsonNumberOrNull(stream, shape.min_segment_length_m);
  stream << ",\"trajectory_shape_segment_len_mean_m\":";
  writeBlackboxJsonNumberOrNull(stream, shape.mean_segment_length_m);
  stream << ",\"trajectory_shape_segment_len_max_m\":";
  writeBlackboxJsonNumberOrNull(stream, shape.max_segment_length_m);
  stream << ",\"trajectory_shape_segments_lt_0_5m\":"
         << shape.segments_shorter_than_0_5m;
  stream << ",\"trajectory_shape_segments_lt_1m\":" << shape.segments_shorter_than_1m;
  stream << ",\"trajectory_shape_segments_lt_2m\":" << shape.segments_shorter_than_2m;
  stream << ",\"trajectory_shape_max_heading_delta_rad\":";
  writeBlackboxJsonNumberOrNull(stream, shape.max_heading_delta_rad);
  stream << ",\"trajectory_shape_max_heading_delta_index\":"
         << shape.max_heading_delta_index;
  stream << ",\"trajectory_shape_max_heading_delta_x\":";
  writeBlackboxJsonNumberOrNull(stream, shape.max_heading_delta_point.x);
  stream << ",\"trajectory_shape_max_heading_delta_y\":";
  writeBlackboxJsonNumberOrNull(stream, shape.max_heading_delta_point.y);
  stream << ",\"trajectory_shape_max_curvature_jump_1pm\":";
  writeBlackboxJsonNumberOrNull(stream, shape.max_curvature_jump_1pm);
  stream << ",\"trajectory_shape_max_curvature_jump_index\":"
         << shape.max_curvature_jump_index;
  stream << ",\"trajectory_shape_max_curvature_jump_x\":";
  writeBlackboxJsonNumberOrNull(stream, shape.max_curvature_jump_point.x);
  stream << ",\"trajectory_shape_max_curvature_jump_y\":";
  writeBlackboxJsonNumberOrNull(stream, shape.max_curvature_jump_point.y);
  stream << ",\"trajectory_shape_max_offset_delta_m\":";
  writeBlackboxJsonNumberOrNull(stream, shape.max_offset_delta_m);
  stream << ",\"trajectory_shape_max_offset_delta_index\":"
         << shape.max_offset_delta_index;
  stream << ",\"trajectory_shape_max_offset_second_delta_m\":";
  writeBlackboxJsonNumberOrNull(stream, shape.max_offset_second_delta_m);
  stream << ",\"trajectory_shape_max_offset_second_delta_index\":"
         << shape.max_offset_second_delta_index;
  stream << "}";

  stream << ",\"path\":{\"valid\":";
  writeBlackboxJsonBool(stream, record.path_valid);
  stream << ",\"waypoint_index\":"
         << (record.path_valid ? record.waypoint_index + 1U : 0U)
         << ",\"waypoint_count\":" << record.waypoint_count
         << ",\"path_goal_distance_m\":";
  writeBlackboxJsonNumberOrNull(stream, record.path_goal_distance_m);
  stream << ",\"mission_goal_distance_m\":";
  writeBlackboxJsonNumberOrNull(stream, record.mission_goal_distance_m);
  stream << ",\"final_trajectory_debug_turn_angle_rad\":";
  writeBlackboxJsonNumberOrNull(stream, record.upcoming_turn.angle_rad);
  stream << ",\"final_trajectory_debug_turn_valid\":";
  writeBlackboxJsonBool(stream, record.upcoming_turn.valid);
  stream << ",\"final_trajectory_debug_turn_waypoint_index\":"
         << (record.upcoming_turn.valid ? record.upcoming_turn.waypoint_index + 1U
                                        : 0U);
  stream << ",\"final_trajectory_debug_turn_distance_m\":";
  writeBlackboxJsonNumberOrNull(stream, record.upcoming_turn.distance_to_turn_m);
  stream << ",\"final_trajectory_debug_turn_point_x\":";
  writeBlackboxJsonNumberOrNull(stream, record.upcoming_turn.turn_point.x);
  stream << ",\"final_trajectory_debug_turn_point_y\":";
  writeBlackboxJsonNumberOrNull(stream, record.upcoming_turn.turn_point.y);
  stream << ",\"final_trajectory_debug_segment_type\":\""
         << record.final_trajectory_segment_type << "\"";
  stream << ",\"tracking\":{\"valid\":";
  writeBlackboxJsonBool(stream, record.path_tracking.valid);
  stream << ",\"cross_track_error_m\":";
  writeBlackboxJsonNumberOrNull(stream, record.path_tracking.cross_track_error_m);
  stream << ",\"signed_cross_track_error_m\":";
  writeBlackboxJsonNumberOrNull(stream,
                                record.path_tracking.signed_cross_track_error_m);
  stream << ",\"heading_error_rad\":";
  writeBlackboxJsonNumberOrNull(stream, record.path_tracking.heading_error_rad);
  stream << ",\"path_heading_rad\":";
  writeBlackboxJsonNumberOrNull(stream, record.path_tracking.path_heading_rad);
  stream << ",\"segment_start_index\":" << record.path_tracking.segment_start_index
         << ",\"segment_t\":";
  writeBlackboxJsonNumberOrNull(stream, record.path_tracking.segment_t);
  stream << ",\"projection_x\":";
  writeBlackboxJsonNumberOrNull(stream, record.path_tracking.projection.x);
  stream << ",\"projection_y\":";
  writeBlackboxJsonNumberOrNull(stream, record.path_tracking.projection.y);
  stream << "}}";
  stream << ",\"control\":{\"motion_phase\":\"" << record.motion_phase
         << "\",\"final_goal_hold_active\":";
  writeBlackboxJsonBool(stream, record.final_goal_hold_active);
  stream << ",\"terminal_position_capture_active\":";
  writeBlackboxJsonBool(stream, record.terminal_position_capture_active);
  stream << ",\"terminal_position_capture_reason\":\""
         << record.terminal_position_capture_reason << "\"";
  stream << ",\"terminal_position_capture_goal_distance_m\":";
  writeBlackboxJsonNumberOrNull(stream,
                                record.terminal_position_capture_goal_distance_m);
  stream << ",\"terminal_position_capture_remaining_s_m\":";
  writeBlackboxJsonNumberOrNull(stream, record.terminal_position_capture_remaining_s_m);
  stream << ",\"terminal_position_capture_speed_mps\":";
  writeBlackboxJsonNumberOrNull(stream, record.terminal_position_capture_speed_mps);
  stream << ",\"terminal_position_capture_activation_radius_m\":";
  writeBlackboxJsonNumberOrNull(stream,
                                record.terminal_position_capture_activation_radius_m);
  stream << ",\"terminal_position_capture_max_entry_speed_mps\":";
  writeBlackboxJsonNumberOrNull(stream,
                                record.terminal_position_capture_max_entry_speed_mps);
  stream << ",\"terminal_position_capture_stuck_speed_mps\":";
  writeBlackboxJsonNumberOrNull(stream,
                                record.terminal_position_capture_stuck_speed_mps);
  stream << "}";
  stream << ",\"obstacle\":{\"prohibited_grid_clearance_m\":";
  writeBlackboxJsonNumberOrNull(stream, record.prohibited_grid_clearance_m);
  stream << ",\"nearest_prohibited_cell_valid\":";
  writeBlackboxJsonBool(stream, record.nearest_prohibited_cell.valid);
  stream << ",\"nearest_prohibited_grid_clearance_m\":";
  writeBlackboxJsonNumberOrNull(stream, record.nearest_prohibited_cell.clearance_m);
  stream << ",\"nearest_prohibited_cell_bearing_map_rad\":";
  writeBlackboxJsonNumberOrNull(stream, record.nearest_prohibited_cell.bearing_map_rad);
  stream << ",\"nearest_prohibited_cell_bearing_body_rad\":";
  writeBlackboxJsonNumberOrNull(stream,
                                record.nearest_prohibited_cell.bearing_body_rad);
  stream << ",\"nearest_prohibited_cell_bearing_body_deg\":";
  writeBlackboxJsonNumberOrNull(stream,
                                record.nearest_prohibited_cell.bearing_body_deg);
  stream << ",\"nearest_prohibited_cell_x\":";
  writeBlackboxJsonNumberOrNull(stream, record.nearest_prohibited_cell.point.x);
  stream << ",\"nearest_prohibited_cell_y\":";
  writeBlackboxJsonNumberOrNull(stream, record.nearest_prohibited_cell.point.y);
  stream << "}}\n";
}

} // namespace drone_city_nav
