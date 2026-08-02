#include "production_mppi_node.hpp"

#include "drone_city_nav/occupancy_grid_3d.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <filesystem>
#include <stdexcept>

namespace drone_city_nav {
const char*
productionMppiPlanningStateName(const ProductionMppiPlanningState state) noexcept {
  switch (state) {
    case ProductionMppiPlanningState::kPlanned:
      return "planned";
    case ProductionMppiPlanningState::kNoGuideBrakingHold:
      return "no_guide_braking_hold";
    case ProductionMppiPlanningState::kUnavailableWorldBrakingHold:
      return "unavailable_world_braking_hold";
    case ProductionMppiPlanningState::kMissionGoalPositionHold:
      return "mission_goal_position_hold";
  }
  return "unknown";
}

const char*
productionMppiExecutionModeName(const ProductionMppiExecutionMode mode) noexcept {
  switch (mode) {
    case ProductionMppiExecutionMode::kPlanned:
      return "planned";
    case ProductionMppiExecutionMode::kBraking:
      return "braking";
    case ProductionMppiExecutionMode::kPositionHold:
      return "position_hold";
  }
  return "unknown";
}

const char*
productionMppiExecutionReasonName(const ProductionMppiExecutionReason reason) noexcept {
  switch (reason) {
    case ProductionMppiExecutionReason::kNone:
      return "none";
    case ProductionMppiExecutionReason::kHorizonSafety:
      return "horizon_safety";
    case ProductionMppiExecutionReason::kGoalCapture:
      return "goal_capture";
    case ProductionMppiExecutionReason::kNoGuide:
      return "no_guide";
    case ProductionMppiExecutionReason::kUnavailableWorld:
      return "unavailable_world";
  }
  return "unknown";
}

ProductionMppiNode::ProductionMppiNode()
    : Node{"production_mppi_node"} {
  tick_rate_hz_ = declare_parameter<double>("tick_rate_hz", 50.0);
  rviz_rate_hz_ = declare_parameter<double>("rviz_rate_hz", 10.0);
  diagnostics_info_rate_hz_ =
      declare_parameter<double>("diagnostics_info_rate_hz", 5.0);
  deadline_ms_ = declare_parameter<double>("deadline_ms", 20.0);
  maximum_pose_age_ms_ = declare_parameter<double>("maximum_pose_age_ms", 150.0);
  maximum_pose_prediction_age_ms_ =
      declare_parameter<double>("maximum_pose_prediction_age_ms", 1000.0);
  maximum_esdf_age_ms_ = declare_parameter<double>("maximum_esdf_age_ms", 1000.0);
  maximum_control_feedback_age_ms_ =
      declare_parameter<double>("maximum_control_feedback_age_ms", 200.0);
  use_static_map_ = declare_parameter<bool>("use_static_map", true);
  no_static_guide_lookahead_m_ =
      declare_parameter<double>("no_static_guide_lookahead_m", 30.0);
  const std::int64_t risk_recovery_stable_cycles =
      declare_parameter<std::int64_t>("mppi_risk_recovery_stable_cycles", 20);
  if (risk_recovery_stable_cycles <= 0) {
    throw std::invalid_argument{"MPPI risk recovery stable cycles must be positive"};
  }
  const std::size_t risk_recovery_cycles =
      static_cast<std::size_t>(risk_recovery_stable_cycles);
  constrained_route_speed_limit_mps_ = static_cast<float>(
      declare_parameter<double>("constrained_route_speed_limit_mps", 10.0));
  route_constraint_diagnostics_distance_m_ =
      declare_parameter<double>("route_constraint_diagnostics_distance_m", 30.0);
  target_mode_ = declare_parameter<std::string>("target_mode", "active_route_guide");
  frame_id_ = declare_parameter<std::string>("frame_id", "map");
  diagnostics_output_dir_ =
      declare_parameter<std::string>("diagnostics_output_dir", "log/mppi");
  px4_local_origin_.x = declare_parameter<double>("px4_local_origin_x_m", 54.0);
  px4_local_origin_.y = declare_parameter<double>("px4_local_origin_y_m", 54.0);
  mission_start_.x = declare_parameter<double>("start_x_m", 54.0);
  mission_start_.y = declare_parameter<double>("start_y_m", 54.0);
  mission_start_.z = declare_parameter<double>("start_z_m", 0.0);
  mission_goal_.x = declare_parameter<double>("goal_x_m", 216.0);
  mission_goal_.y = declare_parameter<double>("goal_y_m", 378.0);
  mission_goal_.z = declare_parameter<double>("goal_z_m", 18.0);
  mission_goal_capture_config_.capture_radius_m =
      declare_parameter<double>("mission_goal_capture_radius_m", 2.0);
  mppi_config_.rollouts =
      static_cast<std::size_t>(declare_parameter<int>("rollouts", 8192));
  mppi_config_.dynamics.dt_s =
      static_cast<float>(declare_parameter<double>("dt_s", 0.05));
  const double static_horizon_duration_s =
      declare_parameter<double>("static_horizon_duration_s", 6.0);
  const double no_static_horizon_duration_s =
      declare_parameter<double>("no_static_horizon_duration_s", 4.0);
  const double active_horizon_duration_s =
      use_static_map_ ? static_horizon_duration_s : no_static_horizon_duration_s;
  const double static_stale_esdf_execution_window_s = declare_parameter<double>(
      "static_stale_esdf_execution_window_s", static_horizon_duration_s);
  const double no_static_stale_esdf_execution_window_s = declare_parameter<double>(
      "no_static_stale_esdf_execution_window_s", no_static_horizon_duration_s);
  stale_esdf_execution_window_ms_ =
      1000.0 * (use_static_map_ ? static_stale_esdf_execution_window_s
                                : no_static_stale_esdf_execution_window_s);
  mppi_config_.steps = static_cast<std::size_t>(
      std::ceil(active_horizon_duration_s / mppi_config_.dynamics.dt_s));
  MppiSpeedPolicyConfig static_speed_policy_config;
  static_speed_policy_config.horizon_duration_s = static_horizon_duration_s;
  static_speed_policy_config.cruise_speed_mps =
      declare_parameter<double>("static_cruise_speed_mps", 20.0);
  static_speed_policy_config.absolute_speed_limit_mps =
      declare_parameter<double>("static_absolute_speed_limit_mps", 20.0);
  static_speed_policy_config.maximum_lateral_acceleration_mps2 =
      declare_parameter<double>("static_maximum_lateral_acceleration_mps2", 5.0);
  static_speed_policy_config.maximum_braking_acceleration_mps2 =
      declare_parameter<double>("static_maximum_braking_acceleration_mps2", 8.0);
  static_speed_policy_config.reaction_latency_s =
      declare_parameter<double>("static_speed_reaction_latency_s", 0.10);
  static_speed_policy_config.observation_distance_m =
      declare_parameter<double>("static_observation_distance_m", 30.0);
  static_speed_policy_config.observation_margin_m =
      declare_parameter<double>("static_observation_margin_m", 3.0);
  static_speed_policy_config.goal_margin_m =
      declare_parameter<double>("static_goal_braking_margin_m", 2.0);
  static_speed_policy_config.curvature_preview_distance_m =
      declare_parameter<double>("static_curvature_preview_distance_m", 100.0);
  static_speed_policy_config.minimum_target_lookahead_m =
      declare_parameter<double>("static_minimum_target_lookahead_m", 30.0);
  static_speed_policy_config.maximum_target_lookahead_m =
      declare_parameter<double>("static_maximum_target_lookahead_m", 100.0);
  const double static_maximum_horizontal_acceleration_mps2 =
      declare_parameter<double>("static_maximum_horizontal_acceleration_mps2", 8.0);
  const double static_maximum_control_jerk_mps3 =
      declare_parameter<double>("static_maximum_control_jerk_mps3", 20.0);
  MppiSpeedPolicyConfig no_static_speed_policy_config = static_speed_policy_config;
  no_static_speed_policy_config.horizon_duration_s = no_static_horizon_duration_s;
  no_static_speed_policy_config.cruise_speed_mps =
      declare_parameter<double>("no_static_cruise_speed_mps", 10.0);
  no_static_speed_policy_config.absolute_speed_limit_mps =
      declare_parameter<double>("no_static_absolute_speed_limit_mps", 10.0);
  const double no_static_maximum_horizontal_acceleration_mps2 =
      declare_parameter<double>("no_static_maximum_horizontal_acceleration_mps2", 4.0);
  const double no_static_maximum_control_jerk_mps3 =
      declare_parameter<double>("no_static_maximum_control_jerk_mps3", 12.0);
  no_static_speed_policy_config.maximum_lateral_acceleration_mps2 =
      no_static_maximum_horizontal_acceleration_mps2;
  no_static_speed_policy_config.maximum_braking_acceleration_mps2 =
      no_static_maximum_horizontal_acceleration_mps2;
  no_static_speed_policy_config.curvature_preview_distance_m =
      declare_parameter<double>("no_static_curvature_preview_distance_m", 60.0);
  no_static_speed_policy_config.minimum_target_lookahead_m =
      no_static_guide_lookahead_m_;
  no_static_speed_policy_config.maximum_target_lookahead_m =
      no_static_guide_lookahead_m_;
  speed_policy_config_ =
      use_static_map_ ? static_speed_policy_config : no_static_speed_policy_config;
  mppi_config_.dynamics.maximum_horizontal_speed_mps =
      static_cast<float>(speed_policy_config_.absolute_speed_limit_mps);
  mppi_config_.dynamics.maximum_horizontal_acceleration_mps2 = static_cast<float>(
      use_static_map_ ? static_maximum_horizontal_acceleration_mps2
                      : no_static_maximum_horizontal_acceleration_mps2);
  mppi_config_.dynamics.maximum_control_jerk_mps3 =
      static_cast<float>(use_static_map_ ? static_maximum_control_jerk_mps3
                                         : no_static_maximum_control_jerk_mps3);
  mppi_config_.dynamics.maximum_vertical_acceleration_mps2 = static_cast<float>(
      declare_parameter<double>("maximum_vertical_acceleration_mps2", 4.0));
  mppi_config_.dynamics.maximum_vertical_speed_mps =
      static_cast<float>(declare_parameter<double>("maximum_vertical_speed_mps", 5.0));
  constrained_route_control_config_.maximum_vertical_acceleration_mps2 =
      mppi_config_.dynamics.maximum_vertical_acceleration_mps2;
  constrained_route_control_config_.maximum_vertical_speed_mps =
      mppi_config_.dynamics.maximum_vertical_speed_mps;
  constrained_route_control_config_.alignment_distance_buffer_m =
      declare_parameter<double>("constrained_route_alignment_distance_buffer_m", 5.0);
  constrained_route_control_config_.stationary_hold_distance_m =
      declare_parameter<double>("constrained_route_stationary_hold_distance_m", 2.0);
  constrained_route_control_config_.vertical_capture_margin_m =
      declare_parameter<double>("constrained_route_vertical_capture_margin_m", 0.5);
  constrained_route_control_config_.vertical_capture_speed_mps =
      declare_parameter<double>("constrained_route_vertical_capture_speed_mps", 0.75);
  safety_config_.physical_footprint_radius_m =
      declare_parameter<double>("physical_footprint_radius_m", 0.82);
  safety_config_.physical_footprint_samples = static_cast<std::size_t>(
      declare_parameter<std::int64_t>("physical_footprint_samples", 12));
  mppi_config_.costs.head_progress_horizon_s =
      static_cast<float>(declare_parameter<double>("head_progress_horizon_s", 0.4));
  mppi_config_.costs.head_progress_weight =
      static_cast<float>(declare_parameter<double>("head_progress_weight", 8.0));
  const double static_speed_tracking_weight =
      declare_parameter<double>("static_speed_tracking_weight", 1.0);
  const double no_static_speed_tracking_weight =
      declare_parameter<double>("no_static_speed_tracking_weight", 1.0);
  mppi_config_.costs.speed_tracking_weight = static_cast<float>(
      use_static_map_ ? static_speed_tracking_weight : no_static_speed_tracking_weight);
  mppi_config_.risk.critical_distance_m =
      static_cast<float>(declare_parameter<double>("critical_distance_m", 1.0));
  mppi_config_.risk.preferred_distance_m =
      static_cast<float>(declare_parameter<double>("preferred_distance_m", 6.0));
  mppi_config_.seed = static_cast<std::uint64_t>(declare_parameter<int>("seed", 42));
  lattice_config_.heading_bins = static_cast<int>(
      declare_parameter<std::int64_t>("global_lattice_heading_bins", 16));
  lattice_config_.primitive_length_m =
      declare_parameter<double>("global_lattice_primitive_length_m", 4.0);
  lattice_config_.short_primitive_length_m =
      declare_parameter<double>("global_lattice_short_primitive_length_m", 2.0);
  const double static_lattice_distance =
      declare_parameter<double>("static_global_lattice_window_m", 180.0);
  const double no_static_lattice_distance =
      declare_parameter<double>("no_static_global_lattice_window_m", 60.0);
  lattice_config_.receding_goal_distance_m =
      use_static_map_ ? static_lattice_distance : no_static_lattice_distance;
  const std::int64_t static_lattice_expansions = declare_parameter<std::int64_t>(
      "static_global_lattice_maximum_expansions", 120000);
  const std::int64_t no_static_lattice_expansions = declare_parameter<std::int64_t>(
      "no_static_global_lattice_maximum_expansions", 60000);
  lattice_config_.maximum_expansions = static_cast<std::size_t>(
      use_static_map_ ? static_lattice_expansions : no_static_lattice_expansions);
  const double static_lattice_deadline_ms =
      declare_parameter<double>("static_global_lattice_deadline_ms", 250.0);
  const double no_static_lattice_deadline_ms =
      declare_parameter<double>("no_static_global_lattice_deadline_ms", 100.0);
  lattice_config_.maximum_search_time_ms =
      use_static_map_ ? static_lattice_deadline_ms : no_static_lattice_deadline_ms;
  const double static_lattice_roi_halo_m =
      declare_parameter<double>("static_global_lattice_roi_halo_m", 90.0);
  const double no_static_lattice_roi_halo_m =
      declare_parameter<double>("no_static_global_lattice_roi_halo_m", 45.0);
  lattice_config_.maximum_search_roi_halo_m =
      use_static_map_ ? static_lattice_roi_halo_m : no_static_lattice_roi_halo_m;
  lattice_config_.maximum_frontier_candidates = static_cast<std::size_t>(
      declare_parameter<std::int64_t>("global_lattice_frontier_candidates", 64));
  lattice_config_.minimum_frontier_endpoint_displacement_m = declare_parameter<double>(
      "global_lattice_frontier_minimum_endpoint_displacement_m", 4.0);
  lattice_config_.minimum_frontier_reachable_depth_m =
      declare_parameter<double>("global_lattice_frontier_reachable_depth_m", 20.0);
  const std::int64_t frontier_validation_maximum_states =
      declare_parameter<std::int64_t>(
          "global_lattice_frontier_validation_maximum_states", 2048);
  if (frontier_validation_maximum_states <= 0) {
    throw std::invalid_argument{
        "global lattice frontier validation maximum states must be positive"};
  }
  lattice_config_.frontier_validation_maximum_states =
      static_cast<std::size_t>(frontier_validation_maximum_states);
  lattice_config_.frontier_goal_distance_weight =
      declare_parameter<double>("global_lattice_frontier_goal_distance_weight", 0.25);
  frontier_blacklist_enabled_ =
      declare_parameter<bool>("global_lattice_frontier_blacklist_enabled", false);
  lattice_config_.frontier_blacklist_radius_m =
      declare_parameter<double>("global_lattice_frontier_blacklist_radius_m", 6.0);
  lattice_config_.frontier_blacklist_heading_tolerance_bins =
      static_cast<int>(declare_parameter<std::int64_t>(
          "global_lattice_frontier_blacklist_heading_bins", 1));
  frontier_blacklist_ttl_s_ =
      declare_parameter<double>("global_lattice_frontier_blacklist_ttl_s", 15.0);
  const std::int64_t lattice_maximum_continuation_attempts =
      declare_parameter<std::int64_t>("global_lattice_maximum_continuation_attempts",
                                      4);
  if (lattice_maximum_continuation_attempts <= 0) {
    throw std::invalid_argument{
        "global lattice maximum continuation attempts must be positive"};
  }
  lattice_maximum_continuation_attempts_ =
      static_cast<std::size_t>(lattice_maximum_continuation_attempts);
  lattice_config_.planning_exposure_tie_break_per_m =
      declare_parameter<double>("global_lattice_planning_tie_break_per_m", 1.0);
  lattice_config_.critical_exposure_tie_break_per_m =
      declare_parameter<double>("global_lattice_critical_tie_break_per_m", 10.0);
  lattice_config_.critical_distance_m = mppi_config_.risk.critical_distance_m;
  lattice_config_.preferred_distance_m = mppi_config_.risk.preferred_distance_m;
  lattice_3d_config_.horizontal_step_m =
      declare_parameter<double>("global_lattice_3d_horizontal_step_m", 2.0);
  lattice_3d_config_.vertical_step_m =
      declare_parameter<double>("global_lattice_3d_vertical_step_m", 1.0);
  lattice_3d_config_.sample_step_m =
      declare_parameter<double>("global_lattice_3d_sample_step_m", 0.5);
  lattice_3d_config_.planning_goal_distance_m = static_lattice_distance;
  lattice_3d_config_.critical_distance_m = mppi_config_.risk.critical_distance_m;
  lattice_3d_config_.preferred_distance_m = mppi_config_.risk.preferred_distance_m;
  lattice_3d_config_.nominal_horizontal_speed_mps =
      speed_policy_config_.cruise_speed_mps;
  lattice_3d_config_.nominal_vertical_speed_mps =
      declare_parameter<double>("global_lattice_3d_nominal_vertical_speed_mps", 4.0);
  lattice_3d_config_.vertical_alignment_cost_weight = declare_parameter<double>(
      "global_lattice_3d_vertical_alignment_cost_weight", 0.0);
  lattice_3d_config_.turn_cost_per_rad =
      declare_parameter<double>("global_lattice_3d_turn_cost_per_rad", 0.10);
  lattice_3d_config_.planning_exposure_cost_per_m =
      declare_parameter<double>("global_lattice_3d_planning_exposure_cost_per_m", 0.05);
  lattice_3d_config_.critical_exposure_cost_per_m =
      declare_parameter<double>("global_lattice_3d_critical_exposure_cost_per_m", 0.50);
  lattice_3d_config_.channel_connection_distance_m =
      declare_parameter<double>("global_lattice_3d_channel_connection_distance_m", 3.0);
  lattice_3d_config_.frontier_minimum_reachable_depth_m =
      declare_parameter<double>("global_lattice_3d_frontier_reachable_depth_m", 8.0);
  lattice_3d_config_.frontier_validation_maximum_states =
      static_cast<std::size_t>(declare_parameter<std::int64_t>(
          "global_lattice_3d_frontier_validation_maximum_states", 2048));
  lattice_3d_config_.maximum_expansions =
      static_cast<std::size_t>(static_lattice_expansions);
  lattice_3d_config_.maximum_search_time_ms = static_lattice_deadline_ms;
  active_guide_config_.critical_distance_m = mppi_config_.risk.critical_distance_m;
  active_guide_config_.preferred_distance_m = mppi_config_.risk.preferred_distance_m;
  active_guide_config_.validation_sample_step_m =
      declare_parameter<double>("global_guide_validation_sample_step_m", 0.5);
  const double static_guide_replan_remaining_m =
      declare_parameter<double>("static_global_guide_replan_remaining_m", 45.0);
  const double no_static_guide_replan_remaining_m =
      declare_parameter<double>("no_static_global_guide_replan_remaining_m", 15.0);
  active_guide_config_.minimum_remaining_m = use_static_map_
                                                 ? static_guide_replan_remaining_m
                                                 : no_static_guide_replan_remaining_m;
  static_route_extension_config_.minimum_remaining_m = static_guide_replan_remaining_m;
  static_route_extension_config_.latency_margin_s =
      declare_parameter<double>("static_global_guide_extension_latency_margin_s", 0.5);
  static_route_extension_config_.maximum_latency_s =
      declare_parameter<double>("static_global_guide_extension_maximum_latency_s", 8.0);
  static_route_extension_config_.minimum_retry_progress_m =
      declare_parameter<double>("static_global_guide_extension_retry_progress_m", 15.0);
  static_route_extension_config_.minimum_retry_interval_s =
      declare_parameter<double>("static_global_guide_extension_retry_interval_s", 1.0);
  static_route_extension_config_.minimum_endpoint_improvement_m =
      declare_parameter<double>(
          "static_global_guide_extension_minimum_endpoint_improvement_m", 5.0);
  active_guide_config_.maximum_cross_track_m =
      declare_parameter<double>("global_guide_maximum_cross_track_m", 15.0);
  active_guide_config_.velocity_heading_low_speed_mps =
      declare_parameter<double>("global_guide_heading_low_speed_mps", 0.5);
  active_guide_config_.velocity_heading_high_speed_mps =
      declare_parameter<double>("global_guide_heading_high_speed_mps", 1.5);
  guide_progress_config_.observation_window_s =
      declare_parameter<double>("global_guide_stall_observation_window_s", 1.0);
  guide_progress_config_.minimum_progress_m =
      declare_parameter<double>("global_guide_stall_minimum_progress_m", 0.5);
  guide_progress_config_.minimum_predicted_head_progress_m = declare_parameter<double>(
      "global_guide_stall_minimum_predicted_head_progress_m", 0.5);
  guide_progress_config_.persistent_safety_rejection_window_s =
      declare_parameter<double>("persistent_safety_rejection_window_s", 1.0);
  mppi_config_.early_exit_on_collision = true;
  safety_config_.reaction_latency_s =
      declare_parameter<double>("safety_reaction_latency_s", 0.10);
  safety_config_.maximum_braking_acceleration_mps2 = std::min(
      declare_parameter<double>("safety_maximum_braking_acceleration_mps2", 8.0),
      static_cast<double>(mppi_config_.dynamics.maximum_horizontal_acceleration_mps2));
  safety_config_.minimum_time_to_collision_s =
      declare_parameter<double>("safety_minimum_time_to_collision_s", 0.50);
  safety_config_.swept_validation_step_m =
      declare_parameter<double>("safety_swept_validation_step_m", 0.25);
  safety_config_.position_hold_capture_speed_mps =
      declare_parameter<double>("safety_position_hold_capture_speed_mps", 0.20);
  const double static_safety_fallback_duration_s =
      declare_parameter<double>("static_safety_fallback_duration_s", 3.0);
  const double no_static_safety_fallback_duration_s =
      declare_parameter<double>("no_static_safety_fallback_duration_s", 3.0);
  const double configured_fallback_duration_s =
      use_static_map_ ? static_safety_fallback_duration_s
                      : no_static_safety_fallback_duration_s;
  const double minimum_fallback_duration_s =
      safety_config_.reaction_latency_s +
      speed_policy_config_.absolute_speed_limit_mps /
          std::max(1.0e-3, safety_config_.maximum_braking_acceleration_mps2) +
      mppi_config_.dynamics.dt_s;
  safety_config_.fallback_duration_s =
      std::max(configured_fallback_duration_s, minimum_fallback_duration_s);
  safety_config_.dt_s = mppi_config_.dynamics.dt_s;
  liveness_config_.enabled = declare_parameter<bool>("liveness_enabled", true);
  liveness_config_.observation_window_s =
      declare_parameter<double>("liveness_observation_window_s", 1.0);
  liveness_config_.minimum_actual_displacement_m =
      declare_parameter<double>("liveness_minimum_actual_displacement_m", 0.5);
  liveness_config_.minimum_predicted_terminal_progress_m =
      declare_parameter<double>("liveness_minimum_predicted_terminal_progress_m", 5.0);
  rviz_period_ns_ = static_cast<std::int64_t>(1.0e9 / std::max(0.1, rviz_rate_hz_));
  diagnostics_info_period_ns_ =
      static_cast<std::int64_t>(1.0e9 / std::max(0.1, diagnostics_info_rate_hz_));

  if (!(tick_rate_hz_ > 0.0) || !(rviz_rate_hz_ > 0.0) ||
      !(diagnostics_info_rate_hz_ > 0.0) || !(deadline_ms_ > 0.0) ||
      !(maximum_control_feedback_age_ms_ > 0.0) ||
      !std::isfinite(constrained_route_speed_limit_mps_) ||
      constrained_route_speed_limit_mps_ < 0.0F ||
      !(route_constraint_diagnostics_distance_m_ >= 0.0) ||
      !(lattice_3d_config_.nominal_horizontal_speed_mps > 0.0) ||
      !(lattice_3d_config_.nominal_vertical_speed_mps > 0.0) ||
      !(lattice_3d_config_.vertical_alignment_cost_weight >= 0.0) ||
      !(lattice_3d_config_.turn_cost_per_rad >= 0.0) ||
      !(lattice_3d_config_.planning_exposure_cost_per_m >= 0.0) ||
      !(lattice_3d_config_.critical_exposure_cost_per_m >= 0.0) ||
      !(lattice_3d_config_.channel_connection_distance_m > 0.0) ||
      !(lattice_3d_config_.frontier_minimum_reachable_depth_m > 0.0) ||
      lattice_3d_config_.frontier_validation_maximum_states == 0U ||
      !(safety_config_.swept_validation_step_m > 0.0) ||
      !(safety_config_.physical_footprint_radius_m >= 0.0) ||
      safety_config_.physical_footprint_samples == 0U ||
      !(safety_config_.position_hold_capture_speed_mps >= 0.0) ||
      !(frontier_blacklist_ttl_s_ > 0.0)) {
    throw std::invalid_argument{"invalid production MPPI configuration"};
  }

  liveness_supervisor_ = std::make_unique<MppiLivenessSupervisor>(liveness_config_);
  risk_escalation_ = std::make_unique<MppiRiskEscalation>(
      MppiRiskEscalationConfig{.recovery_stable_cycles = risk_recovery_cycles});
  active_guide_lifecycle_ =
      std::make_unique<ActiveGlobalGuideLifecycle>(active_guide_config_);
  guide_progress_tracker_ =
      std::make_unique<GlobalGuideProgressTracker>(guide_progress_config_);
  mission_goal_capture_latch_ =
      std::make_unique<MissionGoalCaptureLatch>(mission_goal_capture_config_);
  engine_ = std::make_unique<mppi::MppiCudaEngine>(mppi_config_);
  if (use_static_map_) {
    const auto package_share = std::filesystem::path{
        ament_index_cpp::get_package_share_directory("drone_city_nav")};
    std::filesystem::path occupancy_path = declare_parameter<std::string>(
        "static_occupancy_3d_path", "worlds/generated_city.occupancy3d");
    if (occupancy_path.is_relative()) {
      occupancy_path = package_share / occupancy_path;
    }
    static_occupancy_3d_ = OccupancyGrid3D::load(occupancy_path);
    static_channel_edges_ =
        std::make_shared<const std::vector<ConstrainedFreeSpaceEdge>>(
            static_occupancy_3d_->channelEdges());
    RCLCPP_INFO(get_logger(),
                "STATIC_WORLD_3D path=%s fingerprint=%" PRIu64
                " occupied_voxels=%zu channels=%zu dimensions=%dx%dx%d",
                occupancy_path.c_str(), static_occupancy_3d_->fingerprint(),
                static_occupancy_3d_->occupiedVoxelCount(),
                static_channel_edges_->size(),
                static_occupancy_3d_->bounds().width_cells,
                static_occupancy_3d_->bounds().height_cells,
                static_occupancy_3d_->bounds().depth_cells);
  }
  std::filesystem::create_directories(diagnostics_output_dir_);
  diagnostics_stream_.open(diagnostics_output_dir_ / "mppi_ticks.jsonl", std::ios::app);
  const auto sensor_qos = rclcpp::SensorDataQoS{};
  local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      declare_parameter<std::string>("px4_local_position_topic",
                                     "/fmu/out/vehicle_local_position_v1"),
      sensor_qos, [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr message) {
        onLocalPosition(*message);
      });
  raw_snapshot_sub_ = create_subscription<msg::RawObstacleSnapshot>(
      declare_parameter<std::string>("raw_obstacle_snapshot_topic",
                                     "/drone_city_nav/raw_obstacle_snapshot"),
      rclcpp::QoS{1}.reliable().transient_local(),
      [this](msg::RawObstacleSnapshot::ConstSharedPtr message) {
        onRawObstacleSnapshot(std::move(message));
      });
  memory_snapshot_sub_ = create_subscription<msg::ObstacleMemorySnapshot>(
      declare_parameter<std::string>("obstacle_memory_snapshot_topic",
                                     "/drone_city_nav/obstacle_memory_snapshot"),
      rclcpp::QoS{1}.reliable().transient_local(),
      [this](const msg::ObstacleMemorySnapshot::SharedPtr message) {
        onMemorySnapshot(*message);
      });
  applied_control_sub_ = create_subscription<msg::MppiControlFeedback>(
      declare_parameter<std::string>("applied_control_feedback_topic",
                                     "/drone_city_nav/mppi/applied_control"),
      rclcpp::QoS{10}.reliable(),
      [this](const msg::MppiControlFeedback::SharedPtr message) {
        onAppliedControl(*message);
      });
  path_pub_ = create_publisher<nav_msgs::msg::Path>(
      declare_parameter<std::string>("path_topic", "/drone_city_nav/mppi/path"),
      rclcpp::QoS{1}.reliable());
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      declare_parameter<std::string>("markers_topic", "/drone_city_nav/mppi/markers"),
      rclcpp::QoS{1}.reliable());
  status_pub_ = create_publisher<std_msgs::msg::String>(
      declare_parameter<std::string>("status_topic", "/drone_city_nav/mppi/status"),
      rclcpp::QoS{10}.best_effort());
  execution_horizon_pub_ = create_publisher<msg::MppiTrajectoryHorizon>(
      declare_parameter<std::string>("execution_horizon_topic",
                                     "/drone_city_nav/mppi/execution_horizon"),
      rclcpp::QoS{2}.reliable());
  diagnostics_worker_ =
      std::jthread([this](const std::stop_token token) { diagnosticsWorker(token); });
  esdf_worker_ =
      std::jthread([this](const std::stop_token token) { esdfWorker(token); });
  guide_worker_ =
      std::jthread([this](const std::stop_token token) { guideWorker(token); });
  planning_timer_ = create_wall_timer(
      std::chrono::duration<double>{1.0 / tick_rate_hz_}, [this]() { planningTick(); });
  RCLCPP_INFO(get_logger(),
              "Production MPPI ready: rollouts=%zu steps=%zu rate=%.1fHz "
              "deadline=%.1fms known_solids=%zu static_map=%s route3d=%s "
              "horizon=%.1fs guide_window=%.1fm cruise=%.1fmps speed_cap=%.1fmps "
              "acceleration_cap=%.1fmps2 jerk_cap=%.1fmps3 speed_tracking_weight=%.2f "
              "constrained_route_speed_limit=%.1fmps head_progress=%.2fs liveness=%s "
              "sticky_guide=true frontier_blacklist=%s guide_replan_remaining=%.1fm "
              "guide_heading_blend=(%.1f,%.1f)mps",
              mppi_config_.rollouts, mppi_config_.steps, tick_rate_hz_, deadline_ms_,
              0UL, use_static_map_ ? "true" : "false", "false",
              static_cast<double>(mppi_config_.steps) * mppi_config_.dynamics.dt_s,
              lattice_config_.receding_goal_distance_m,
              speed_policy_config_.cruise_speed_mps,
              mppi_config_.dynamics.maximum_horizontal_speed_mps,
              mppi_config_.dynamics.maximum_horizontal_acceleration_mps2,
              mppi_config_.dynamics.maximum_control_jerk_mps3,
              mppi_config_.costs.speed_tracking_weight,
              use_static_map_ ? constrained_route_speed_limit_mps_ : 0.0F,
              mppi_config_.costs.head_progress_horizon_s,
              liveness_config_.enabled ? "true" : "false",
              frontier_blacklist_enabled_ ? "true" : "false",
              active_guide_config_.minimum_remaining_m,
              active_guide_config_.velocity_heading_low_speed_mps,
              active_guide_config_.velocity_heading_high_speed_mps);
}

ProductionMppiNode::~ProductionMppiNode() {
  if (diagnostics_worker_.joinable()) {
    diagnostics_worker_.request_stop();
    diagnostics_mailbox_.notifyAll();
    diagnostics_worker_.join();
  }
  if (esdf_worker_.joinable()) {
    esdf_worker_.request_stop();
    raw_queue_condition_.notify_all();
    esdf_worker_.join();
  }
  if (guide_worker_.joinable()) {
    guide_worker_.request_stop();
    guide_queue_condition_.notify_all();
    guide_worker_.join();
  }
  publishSummary();
}

} // namespace drone_city_nav

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::ProductionMppiNode>());
  rclcpp::shutdown();
  return 0;
}
