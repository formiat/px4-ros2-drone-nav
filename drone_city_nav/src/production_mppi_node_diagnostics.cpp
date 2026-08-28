#include "drone_city_nav/json_output.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <iomanip>
#include <numeric>
#include <sstream>

#include "production_mppi_cooperative_diagnostics.hpp"
#include "production_mppi_execution_diagnostics.hpp"
#include "production_mppi_node.hpp"
#include "production_mppi_node_diagnostics_format.hpp"
#include "production_mppi_noncooperative_diagnostics.hpp"
#include "tracking_objective_diagnostics.hpp"

namespace drone_city_nav {

void ProductionMppiNode::processDiagnostics(
    const ProductionMppiDiagnosticsSnapshot& snapshot) {
  const std::shared_ptr<const ProductionNavigationObjective>& objective =
      snapshot.objective;
  const Point3 mission_goal = objective ? objective->goal : mission_goal_;
  const mppi::MppiTickInput& input = snapshot.input;
  const mppi::MppiTickResult& result = snapshot.result;
  const ProductionMppiPreparedEsdf& esdf = snapshot.esdf;
  const ProductionMppiStability& stability = snapshot.stability;
  const ProductionMppiPredictionError& prediction = snapshot.prediction;
  const MppiLivenessResult& liveness = snapshot.liveness;
  const MppiSpeedPolicyResult& speed_policy = snapshot.speed_policy;
  const detail::TrackingPursuitDiagnostics pursuit_diagnostics =
      detail::trackingPursuitDiagnostics(objective.get(), input, snapshot.execution);
  const ConstrainedRouteObservation route_constraint = diagnosticRouteConstraint(
      snapshot, route_envelope_config_, route_constraint_diagnostics_distance_m_);
  const ProductionMppiPlanningState planning_state = snapshot.planning_state;
  const std::string_view target_source = snapshot.target_source;
  const char* static_route_generation_matches = "not_attempted";
  if (esdf.static_route_generation_assessed) {
    static_route_generation_matches =
        esdf.static_route_generation_matches ? "true" : "false";
  }
  const auto rviz_started = std::chrono::steady_clock::now();
  publishRviz(snapshot);
  const double rviz_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - rviz_started)
                             .count();

  logDiagnosticsEvents(snapshot, route_constraint);

  std::ostringstream line;
  line
      << std::fixed << std::setprecision(3)
      << "PRODUCTION_MPPI_TICK tick=" << snapshot.tick_sequence
      << " pose_revision=" << input.pose_revision
      << " raw_revision=" << input.obstacle_revision
      << " esdf_revision=" << result.esdf_revision
      << " memory_sequence=" << snapshot.memory_sequence
      << " pose_age_ms=" << snapshot.pose_age_ms
      << " observation_age_ms=" << snapshot.observation_age_ms
      << " esdf_content_age_ms=" << snapshot.esdf_age_ms
      << " local_world_generation=" << esdf.local_world_generation.generation
      << " control_feedback_age_ms=" << snapshot.control_feedback_age_ms
      << " state_position=(" << input.initial_state.x << ',' << input.initial_state.y
      << ',' << input.initial_state.z << ") state_velocity=(" << input.initial_state.vx
      << ',' << input.initial_state.vy << ',' << input.initial_state.vz << ')'
      << " planning_mode=" << (use_static_map_ ? "static" : "no_static")
      << " planning_state=" << productionMppiPlanningStateName(planning_state)
      << detail::executionInfoFields(snapshot.execution) << " horizon_s="
      << static_cast<double>(mppi_config_.steps) * mppi_config_.dynamics.dt_s
      << " target_source=" << target_source << " target=(" << input.target.x << ','
      << input.target.y << ',' << input.target.z << ")"
      << " route_generation=" << esdf.route_generation
      << " route_objective_epoch=" << esdf.route_objective.mission_epoch
      << " route_objective_sample=" << esdf.route_objective.sample_sequence
      << " route_assignment_generation=" << esdf.route_objective.assignment_generation
      << " route_target_detection_id=" << esdf.route_objective.target_detection_id
      << " route_target_track_id=" << esdf.route_objective.target_track_id
      << " search_objective_epoch=" << esdf.search_objective.mission_epoch
      << " search_objective_sample=" << esdf.search_objective.sample_sequence
      << " search_assignment_generation=" << esdf.search_objective.assignment_generation
      << " search_target_detection_id=" << esdf.search_objective.target_detection_id
      << " search_target_track_id=" << esdf.search_objective.target_track_id
      << " route_reaches_mission_goal="
      << (esdf.route_reaches_mission_goal ? "true" : "false")
      << " route_intent_id=" << esdf.route_intent.id
      << " route_intent_planned_on=" << esdf.route_intent.planned_on_revision
      << " route_validated_through="
      << esdf.route_segment_evidence.validated_through_revision
      << " route_segment_evidence="
      << segmentEvidenceStatus3DName(esdf.route_segment_evidence.status)
      << " route_unknown_exposure="
      << (esdf.route_segment_evidence.unknown_exposure ? "true" : "false")
      << " route_known_clearance="
      << (esdf.route_segment_evidence.known_clearance_observed ? "true" : "false")
      << " goal_capture_latched=" << (snapshot.goal_capture.latched ? "true" : "false")
      << " goal_distance_m=" << snapshot.goal_capture.distance_m
      << " route_release=" << routeReleaseReason3DName(esdf.route_release_reason)
      << " route_station_m=" << snapshot.route_station_m
      << " route_remaining_m=" << snapshot.route_remaining_m
      << " route_constraint_phase=" << constrainedRoutePhaseName(route_constraint.phase)
      << " route_constraint_passage="
      << (route_constraint.passage_traversal_id.empty()
              ? "none"
              : route_constraint.passage_traversal_id)
      << " route_constraint_span_index="
      << (route_constraint.span_available
              ? static_cast<std::ptrdiff_t>(route_constraint.span_index)
              : static_cast<std::ptrdiff_t>(-1))
      << " route_constraint_span_count=" << route_constraint.span_count
      << " route_constraint_distance_to_entry_m="
      << route_constraint.distance_to_entry_m
      << " route_constraint_distance_to_exit_m=" << route_constraint.distance_to_exit_m
      << " route_constraint_reference_z_m=" << route_constraint.reference_z_m
      << " route_constraint_vertical_error_m=" << route_constraint.vertical_error_m
      << " route_constraint_lateral_width_m=" << route_constraint.lateral_width_m
      << " route_constraint_vertical_height_m=" << route_constraint.vertical_height_m
      << " route_constraint_lateral="
      << (route_constraint.lateral_constrained ? "true" : "false")
      << " route_constraint_vertical="
      << (route_constraint.vertical_constrained ? "true" : "false")
      << " route_constraint_cross_track_error_m="
      << route_constraint.cross_track_error_m << " route_constraint_vertical_window_ok="
      << (route_constraint.within_vertical_window ? "true" : "false")
      << " route_progress_action="
      << routeProgressAction3DName(snapshot.route_progress.action)
      << " route_local_reseed_generation="
      << snapshot.route_progress.local_reseed_generation << " planning_search_kind="
      << productionPlanningSearchKindName(esdf.planning_search_kind)
      << " planning_search_base_route_instance_id="
      << esdf.planning_search_base_route_instance_id.value
      << " planning_search_base_stitch_station_m="
      << esdf.planning_search_base_stitch_station_m.value_or(-1.0)
      << " required_splice_base_route_instance_id="
      << esdf.required_splice_base_route_instance_id.value << " planning_search_start=("
      << esdf.planning_search_start.x << ',' << esdf.planning_search_start.y << ','
      << esdf.planning_search_start.z << ')' << " planning_search_goal=("
      << esdf.planning_search_goal.x << ',' << esdf.planning_search_goal.y << ','
      << esdf.planning_search_goal.z << ')' << " planning_candidate_endpoint=("
      << esdf.planning_candidate_endpoint.x << ',' << esdf.planning_candidate_endpoint.y
      << ',' << esdf.planning_candidate_endpoint.z << ')'
      << " planning_search_direction=(" << esdf.planning_search_direction.x << ','
      << esdf.planning_search_direction.y << ',' << esdf.planning_search_direction.z
      << ')' << " planning_candidate_points=" << esdf.planning_candidate_points
      << " planning_candidate_samples=" << esdf.planning_candidate_samples
      << persistentPlannerInfoFields(esdf) << " static_route_candidate="
      << staticRouteCandidateStatusName(esdf.static_route_candidate_status)
      << certifiedRouteReserveInfoFields(esdf) << trackingErrorTubeInfoFields(esdf)
      << " static_route_activation="
      << staticRouteActivationStatusName(esdf.static_route_activation_status)
      << " static_route_publication_status="
      << routePublicationStatus3DName(esdf.static_route_publication_status)
      << " static_route_world_compatible="
      << (esdf.static_route_world_compatible ? "true" : "false")
      << " static_route_generation_matches=" << static_route_generation_matches
      << " route_selected_passage_traversals="
      << (esdf.selected_passage_traversal_ids
              ? esdf.selected_passage_traversal_ids->size()
              : 0U)
      << " pose_predicted=" << (snapshot.pose_predicted ? "true" : "false")
      << " target_lookahead_m=" << speed_policy.target_lookahead_m
      << " reference_speed_mps=" << input.reference_speed_mps
      << detail::trackingPursuitInfoFields(pursuit_diagnostics, speed_policy, result)
      << " curvature_speed_limit_mps="
      << finiteOrNegative(speed_policy.curvature_limit_mps)
      << " observation_speed_limit_mps="
      << finiteOrNegative(speed_policy.observation_limit_mps)
      << " goal_speed_limit_mps=" << finiteOrNegative(speed_policy.goal_limit_mps)
      << " route_endpoint_speed_limit_mps="
      << finiteOrNegative(speed_policy.route_endpoint_limit_mps)
      << " active_rollouts=" << result.active_rollouts << " rollout_budget_reason="
      << mppiRolloutBudgetReasonName(snapshot.rollout_budget.reason)
      << detail::cooperativeInfoFields(snapshot.cooperative, result)
      << detail::nonCooperativeInfoFields(snapshot.noncooperative, result)
      << " gpu_warm_start_ms=" << result.timings.warm_start_ms
      << " gpu_noise_generation_ms=" << result.timings.noise_generation_ms
      << " gpu_rollout_simulation_ms=" << result.timings.rollout_simulation_ms
      << " gpu_risk_reduction_ms=" << result.timings.risk_reduction_ms
      << " gpu_weight_calculation_ms=" << result.timings.weight_calculation_ms
      << " gpu_control_update_ms=" << result.timings.control_update_ms
      << " gpu_repair_validation_ms=" << result.timings.repair_validation_ms
      << " post_update_evaluation_ms=" << result.timings.post_update_evaluation_ms
      << " gpu_ms=" << result.timings.gpu_total_ms
      << " horizon_reconstruction_ms=" << result.timings.horizon_reconstruction_ms
      << " total_ms=" << result.timings.host_total_ms
      << " snapshot_ms=" << snapshot.snapshot_ms
      << " stability_ms=" << snapshot.stability_ms << " rviz_ms=" << rviz_ms
      << " deadline_missed="
      << (result.timings.host_total_ms > deadline_ms_ ? "true" : "false")
      << " risk_tier=" << mppi::mppiRiskTierName(result.selected_tier)
      << " altitude_envelope_violation="
      << (result.altitude_envelope_violation ? "true" : "false")
      << " raw_collision=" << (result.raw_collision ? "true" : "false")
      << " known_solid_collision=" << (result.known_solid_collision ? "true" : "false")
      << " route_terminal_cross_track_violation="
      << (result.route_terminal_cross_track_violation ? "true" : "false")
      << " terminal_route_cross_track_m=" << result.terminal_route_cross_track_m
      << " route_terminal_arrival_shaping_attempts="
      << result.route_terminal_arrival_shaping_attempts
      << " route_terminal_nominal_prefix_controls="
      << result.route_terminal_nominal_prefix_control_count
      << " critical_exposure_m=" << result.critical_exposure_m
      << " planning_exposure_m=" << result.planning_exposure_m
      << " critical_clearance_proximity_s=" << result.critical_clearance_proximity_s
      << " obstacle_approach_m2_s=" << result.obstacle_approach_m2_s
      << " feasible_available="
      << (result.feasibility_contract.available ? "true" : "false")
      << " feasible_weight_sum="
      << finiteOrNegative(result.feasibility_contract.weight_sum)
      << " post_update_classification="
      << mppi::mppiPostUpdateClassificationName(
             result.post_update_classification.classification)
      << " control_selection="
      << mppi::mppiControlSelectionName(result.control_selection)
      << " post_update_executable="
      << (result.post_update_classification.executable ? "true" : "false")
      << " post_update_repair="
      << mppi::mppiPostUpdateRepairName(result.post_update_repair)
      << " post_update_backtrack_ratio=" << result.post_update_backtrack_ratio
      << " minimum_esdf_m=" << result.minimum_esdf_distance_m
      << " head_progress_m=" << result.head_progress_m
      << " terminal_progress_m=" << result.terminal_progress_m
      << " route_progress_integral_m_s=" << result.route_progress_integral_m_s
      << " warm_start_shift_ms=" << result.warm_start_shift_s * 1000.0
      << " previous_control_source="
      << productionMppiPreviousControlSourceName(snapshot.previous_control_source)
      << " nominal_reseeded=" << (result.nominal_reseeded ? "true" : "false")
      << " direct_maneuver_reseed="
      << (snapshot.direct_tracking_maneuver.reseed_requested ? "true" : "false")
      << " direct_maneuver_reason="
      << directTrackingReseedReasonName(snapshot.direct_tracking_maneuver.reason)
      << " direct_bearing_change_deg="
      << snapshot.direct_tracking_maneuver.bearing_change_rad * 180.0 / std::acos(-1.0)
      << " direct_closing_speed_mps="
      << snapshot.direct_tracking_maneuver.closing_speed_mps
      << " direct_no_closing_duration_s="
      << snapshot.direct_tracking_maneuver.no_closing_duration_s
      << " target_directed_candidate_injected="
      << (result.target_directed_candidate_injected ? "true" : "false")
      << " target_directed_candidate_raw_safe="
      << (result.target_directed_candidate_raw_safe ? "true" : "false")
      << " target_directed_candidate_best_feasible="
      << (result.target_directed_candidate_best_feasible ? "true" : "false")
      << " target_directed_candidate_weight=" << result.target_directed_candidate_weight
      << " route_directed_candidate_injected="
      << (result.route_directed_candidate_injected ? "true" : "false")
      << " route_directed_candidate_raw_safe="
      << (result.route_directed_candidate_raw_safe ? "true" : "false")
      << " route_directed_candidate_best_feasible="
      << (result.route_directed_candidate_best_feasible ? "true" : "false")
      << " route_directed_candidate_weight=" << result.route_directed_candidate_weight
      << " route_directed_candidate_generation="
      << result.route_directed_candidate_generation << " local_route_stop_is_terminal="
      << (snapshot.local_route_stop_is_terminal ? "true" : "false")
      << detail::rollingRouteInfoFields(snapshot.rolling_route) << " no_eligible_phase="
      << mppiNoEligiblePhaseName(snapshot.no_eligible_recovery.phase)
      << " no_eligible_recovery_generation="
      << snapshot.no_eligible_recovery.no_eligible_recovery_generation
      << " no_eligible_route_replan="
      << (snapshot.no_eligible_recovery.route_replan_requested ? "true" : "false")
      << " liveness_state=" << mppiLivenessStateName(liveness.state)
      << " liveness_recovery_active=" << (liveness.recovery_active ? "true" : "false")
      << " liveness_window_s=" << liveness.observation_age_s
      << " liveness_actual_displacement_m=" << liveness.actual_displacement_m
      << " liveness_actual_route_progress_m=" << liveness.actual_route_progress_m
      << " liveness_route_progress_used="
      << (liveness.used_route_progress ? "true" : "false")
      << " liveness_reseed_generation=" << liveness.reseed_generation
      << " route_required_risk_tier="
      << mppi::mppiRiskTierName(snapshot.route_required_risk_tier)
      << " maximum_acceleration_mps2=" << result.maximum_acceleration_mps2
      << " maximum_jerk_mps3=" << result.maximum_jerk_mps3
      << " first_control_delta=" << result.first_control_delta
      << " horizon_stability_rms="
      << (stability.valid ? stability.position_rms_m : -1.0)
      << " shifted_horizon_first_control_delta="
      << (stability.valid ? stability.first_control_delta : -1.0)
      << " prediction_position_error_m="
      << (prediction.valid ? prediction.position_m : -1.0)
      << " esdf_build_ms=" << esdf.build_ms << " esdf_x_pass_ms=" << esdf.esdf_x_pass_ms
      << " esdf_y_pass_ms=" << esdf.esdf_y_pass_ms
      << " esdf_z_pass_ms=" << esdf.esdf_z_pass_ms
      << " esdf_finalize_ms=" << esdf.esdf_finalize_ms
      << " route_search_ms=" << esdf.route_search_ms
      << " continuation_validation_ms=" << esdf.continuation_validation_ms
      << " route_smoothing_ms=" << esdf.route_smoothing_ms
      << " route_shortcuts_applied=" << esdf.route_shortcuts_applied
      << " route_corners_smoothed=" << esdf.route_corners_smoothed
      << " candidate_validation_ms=" << esdf.candidate_validation_ms
      << " route_fingerprint=" << esdf.route_fingerprint
      << " esdf_upload_ms=" << esdf.upload_ms << " dropped_diagnostics="
      << dropped_diagnostics_snapshots_.load(std::memory_order_relaxed);
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  if (now_ns - last_diagnostics_info_stamp_ns_ >= diagnostics_info_period_ns_) {
    RCLCPP_INFO(get_logger(), "%s", line.str().c_str());
    std_msgs::msg::String status;
    status.data = line.str();
    status_pub_->publish(status);
    last_diagnostics_info_stamp_ns_ = now_ns;
  }
  const bool diagnostics_error = result.altitude_envelope_violation ||
                                 result.raw_collision || result.known_solid_collision;
  const bool new_error_episode = diagnostics_error && !diagnostics_error_active_;
  if (!diagnostics_error) {
    diagnostics_error_active_ = false;
  }
  const bool diagnostics_file_due =
      last_diagnostics_file_stamp_ns_ <= 0 ||
      now_ns < last_diagnostics_file_stamp_ns_ ||
      now_ns - last_diagnostics_file_stamp_ns_ >= diagnostics_file_period_ns_;
  if (diagnostics_stream_ && (diagnostics_file_due || new_error_episode)) {
    JsonOutputStream json;
    json << "{\"tick\":" << snapshot.tick_sequence
         << ",\"pose_revision\":" << input.pose_revision
         << ",\"raw_revision\":" << input.obstacle_revision
         << ",\"esdf_revision\":" << result.esdf_revision
         << ",\"pose_age_ms\":" << snapshot.pose_age_ms
         << ",\"observation_age_ms\":" << snapshot.observation_age_ms
         << ",\"esdf_content_age_ms\":" << snapshot.esdf_age_ms
         << ",\"local_world_generation\":" << esdf.local_world_generation.generation
         << ",\"control_feedback_age_ms\":" << snapshot.control_feedback_age_ms
         << ",\"previous_control_source\":\""
         << productionMppiPreviousControlSourceName(snapshot.previous_control_source)
         << '"' << ",\"previous_control_ax_mps2\":"
         << (input.previous_applied_control ? input.previous_applied_control->ax : 0.0F)
         << ",\"previous_control_ay_mps2\":"
         << (input.previous_applied_control ? input.previous_applied_control->ay : 0.0F)
         << ",\"previous_control_az_mps2\":"
         << (input.previous_applied_control ? input.previous_applied_control->az : 0.0F)
         << ",\"mppi_target_x_m\":" << input.target.x
         << ",\"mppi_target_y_m\":" << input.target.y
         << ",\"mppi_target_z_m\":" << input.target.z << ",\"planning_mode\":\""
         << (use_static_map_ ? "static" : "no_static") << '"'
         << ",\"esdf_build_ms\":" << esdf.build_ms
         << ",\"esdf_x_pass_ms\":" << esdf.esdf_x_pass_ms
         << ",\"esdf_y_pass_ms\":" << esdf.esdf_y_pass_ms
         << ",\"esdf_z_pass_ms\":" << esdf.esdf_z_pass_ms
         << ",\"esdf_finalize_ms\":" << esdf.esdf_finalize_ms
         << ",\"route_search_ms\":" << esdf.route_search_ms
         << ",\"continuation_validation_ms\":" << esdf.continuation_validation_ms
         << ",\"route_smoothing_ms\":" << esdf.route_smoothing_ms
         << ",\"route_shortcuts_applied\":" << esdf.route_shortcuts_applied
         << ",\"route_corners_smoothed\":" << esdf.route_corners_smoothed
         << ",\"candidate_validation_ms\":" << esdf.candidate_validation_ms
         << ",\"route_fingerprint\":" << esdf.route_fingerprint
         << ",\"planning_state\":\"" << productionMppiPlanningStateName(planning_state)
         << '"' << detail::executionJsonFields(snapshot.execution)
         << ",\"state_x_m\":" << input.initial_state.x
         << ",\"state_y_m\":" << input.initial_state.y
         << ",\"state_z_m\":" << input.initial_state.z
         << ",\"state_vx_mps\":" << input.initial_state.vx
         << ",\"state_vy_mps\":" << input.initial_state.vy
         << ",\"state_vz_mps\":" << input.initial_state.vz << ",\"target_source\":\""
         << target_source << '"'
         << detail::trackingObjectiveJsonFields(objective.get(), mission_goal, now_ns)
         << ",\"horizon_s\":"
         << static_cast<double>(mppi_config_.steps) * mppi_config_.dynamics.dt_s
         << ",\"speed_cap_mps\":" << mppi_config_.dynamics.maximum_horizontal_speed_mps
         << ",\"acceleration_cap_mps2\":"
         << mppi_config_.dynamics.maximum_horizontal_acceleration_mps2
         << ",\"jerk_cap_mps3\":" << mppi_config_.dynamics.maximum_control_jerk_mps3
         << ",\"speed_tracking_weight\":" << mppi_config_.costs.speed_tracking_weight
         << ",\"route_generation\":" << esdf.route_generation
         << ",\"route_objective_epoch\":" << esdf.route_objective.mission_epoch
         << ",\"route_objective_sample\":" << esdf.route_objective.sample_sequence
         << ",\"route_assignment_generation\":"
         << esdf.route_objective.assignment_generation
         << ",\"route_target_detection_id\":"
         << esdf.route_objective.target_detection_id
         << ",\"route_target_track_id\":" << esdf.route_objective.target_track_id
         << ",\"search_objective_epoch\":" << esdf.search_objective.mission_epoch
         << ",\"search_objective_sample\":" << esdf.search_objective.sample_sequence
         << ",\"search_assignment_generation\":"
         << esdf.search_objective.assignment_generation
         << ",\"search_target_detection_id\":"
         << esdf.search_objective.target_detection_id
         << ",\"search_target_track_id\":" << esdf.search_objective.target_track_id
         << ",\"route_reaches_mission_goal\":"
         << (esdf.route_reaches_mission_goal ? "true" : "false")
         << ",\"route_intent_id\":" << esdf.route_intent.id
         << ",\"route_intent_planned_on\":" << esdf.route_intent.planned_on_revision
         << ",\"route_validated_through\":"
         << esdf.route_segment_evidence.validated_through_revision
         << ",\"route_segment_evidence\":\""
         << segmentEvidenceStatus3DName(esdf.route_segment_evidence.status) << '"'
         << ",\"route_unknown_exposure\":"
         << (esdf.route_segment_evidence.unknown_exposure ? "true" : "false")
         << ",\"route_known_clearance\":"
         << (esdf.route_segment_evidence.known_clearance_observed ? "true" : "false")
         << ",\"goal_capture_latched\":"
         << (snapshot.goal_capture.latched ? "true" : "false")
         << ",\"goal_distance_m\":" << snapshot.goal_capture.distance_m
         << ",\"route_release\":\""
         << routeReleaseReason3DName(esdf.route_release_reason) << '"'
         << ",\"route_station_m\":" << snapshot.route_station_m
         << ",\"route_remaining_m\":" << snapshot.route_remaining_m
         << ",\"route_constraint_phase\":\""
         << constrainedRoutePhaseName(route_constraint.phase) << '"'
         << ",\"route_constraint_passage\":\""
         << (route_constraint.passage_traversal_id.empty()
                 ? "none"
                 : route_constraint.passage_traversal_id)
         << '"' << ",\"route_constraint_span_available\":"
         << (route_constraint.span_available ? "true" : "false")
         << ",\"route_constraint_span_index\":"
         << (route_constraint.span_available
                 ? static_cast<std::ptrdiff_t>(route_constraint.span_index)
                 : static_cast<std::ptrdiff_t>(-1))
         << ",\"route_constraint_span_count\":" << route_constraint.span_count
         << ",\"route_constraint_station_m\":" << route_constraint.station_m
         << ",\"route_constraint_begin_station_m\":" << route_constraint.begin_station_m
         << ",\"route_constraint_end_station_m\":" << route_constraint.end_station_m
         << ",\"route_constraint_distance_to_entry_m\":"
         << route_constraint.distance_to_entry_m
         << ",\"route_constraint_distance_to_exit_m\":"
         << route_constraint.distance_to_exit_m
         << ",\"route_constraint_entry_x_m\":" << route_constraint.entry_position.x
         << ",\"route_constraint_entry_y_m\":" << route_constraint.entry_position.y
         << ",\"route_constraint_entry_z_m\":" << route_constraint.entry_position.z
         << ",\"route_constraint_exit_x_m\":" << route_constraint.exit_position.x
         << ",\"route_constraint_exit_y_m\":" << route_constraint.exit_position.y
         << ",\"route_constraint_exit_z_m\":" << route_constraint.exit_position.z
         << ",\"route_constraint_reference_z_m\":" << route_constraint.reference_z_m
         << ",\"route_constraint_min_z_m\":" << route_constraint.min_z_m
         << ",\"route_constraint_max_z_m\":" << route_constraint.max_z_m
         << ",\"route_constraint_lateral_free_left_m\":"
         << route_constraint.lateral_free_left_m
         << ",\"route_constraint_lateral_free_right_m\":"
         << route_constraint.lateral_free_right_m
         << ",\"route_constraint_lateral_width_m\":" << route_constraint.lateral_width_m
         << ",\"route_constraint_vertical_height_m\":"
         << route_constraint.vertical_height_m << ",\"route_constraint_lateral\":"
         << (route_constraint.lateral_constrained ? "true" : "false")
         << ",\"route_constraint_vertical\":"
         << (route_constraint.vertical_constrained ? "true" : "false")
         << ",\"route_constraint_vertical_error_m\":"
         << route_constraint.vertical_error_m
         << ",\"route_constraint_cross_track_error_m\":"
         << route_constraint.cross_track_error_m
         << ",\"route_constraint_vertical_window_ok\":"
         << (route_constraint.within_vertical_window ? "true" : "false")
         << ",\"route_constraint_reference_speed_mps\":"
         << route_constraint.reference_speed_mps
         << ",\"route_constraint_actual_horizontal_speed_mps\":"
         << route_constraint.actual_horizontal_speed_mps
         << ",\"route_constraint_actual_vertical_speed_mps\":"
         << route_constraint.actual_vertical_speed_mps
         << ",\"route_progress_action\":\""
         << routeProgressAction3DName(snapshot.route_progress.action) << '"'
         << ",\"route_local_reseed_generation\":"
         << snapshot.route_progress.local_reseed_generation
         << ",\"planning_search_kind\":\""
         << productionPlanningSearchKindName(esdf.planning_search_kind) << '"'
         << ",\"planning_search_base_route_instance_id\":"
         << esdf.planning_search_base_route_instance_id.value
         << ",\"planning_search_base_stitch_station_m\":"
         << esdf.planning_search_base_stitch_station_m.value_or(-1.0)
         << ",\"required_splice_base_route_instance_id\":"
         << esdf.required_splice_base_route_instance_id.value
         << ",\"planning_search_start_x\":" << esdf.planning_search_start.x
         << ",\"planning_search_start_y\":" << esdf.planning_search_start.y
         << ",\"planning_search_start_z\":" << esdf.planning_search_start.z
         << ",\"planning_search_goal_x\":" << esdf.planning_search_goal.x
         << ",\"planning_search_goal_y\":" << esdf.planning_search_goal.y
         << ",\"planning_search_goal_z\":" << esdf.planning_search_goal.z
         << ",\"planning_candidate_endpoint_x\":" << esdf.planning_candidate_endpoint.x
         << ",\"planning_candidate_endpoint_y\":" << esdf.planning_candidate_endpoint.y
         << ",\"planning_candidate_endpoint_z\":" << esdf.planning_candidate_endpoint.z
         << ",\"planning_search_direction_x\":" << esdf.planning_search_direction.x
         << ",\"planning_search_direction_y\":" << esdf.planning_search_direction.y
         << ",\"planning_search_direction_z\":" << esdf.planning_search_direction.z
         << ",\"planning_candidate_points\":" << esdf.planning_candidate_points
         << ",\"planning_candidate_samples\":" << esdf.planning_candidate_samples
         << persistentPlannerJsonFields(esdf) << ",\"static_route_candidate\":\""
         << staticRouteCandidateStatusName(esdf.static_route_candidate_status) << '"'
         << certifiedRouteReserveJsonFields(esdf) << trackingErrorTubeJsonFields(esdf)
         << ",\"static_route_activation\":\""
         << staticRouteActivationStatusName(esdf.static_route_activation_status) << '"'
         << ",\"static_route_publication_status\":\""
         << routePublicationStatus3DName(esdf.static_route_publication_status) << '"'
         << ",\"static_route_world_compatible\":"
         << (esdf.static_route_world_compatible ? "true" : "false")
         << ",\"static_route_generation_matches\":";
    if (esdf.static_route_generation_assessed) {
      json << (esdf.static_route_generation_matches ? "true" : "false");
    } else {
      json << "null";
    }
    json << ",\"route_selected_passage_count\":"
         << (esdf.selected_passage_traversal_ids
                 ? esdf.selected_passage_traversal_ids->size()
                 : 0U)
         << ",\"pose_predicted\":" << (snapshot.pose_predicted ? "true" : "false")
         << ",\"target_lookahead_m\":" << speed_policy.target_lookahead_m
         << ",\"reference_speed_mps\":" << input.reference_speed_mps
         << detail::trackingPursuitJsonFields(pursuit_diagnostics, speed_policy, result)
         << ",\"curvature_speed_limit_mps\":"
         << finiteOrNegative(speed_policy.curvature_limit_mps)
         << ",\"observation_speed_limit_mps\":"
         << finiteOrNegative(speed_policy.observation_limit_mps)
         << ",\"goal_speed_limit_mps\":"
         << finiteOrNegative(speed_policy.goal_limit_mps)
         << ",\"route_endpoint_speed_limit_mps\":"
         << finiteOrNegative(speed_policy.route_endpoint_limit_mps)
         << ",\"active_rollouts\":" << result.active_rollouts
         << ",\"rollout_budget_reason\":\""
         << mppiRolloutBudgetReasonName(snapshot.rollout_budget.reason) << '"'
         << detail::cooperativeJsonFields(snapshot.cooperative, result)
         << detail::nonCooperativeJsonFields(snapshot.noncooperative, result)
         << ",\"gpu_warm_start_ms\":" << result.timings.warm_start_ms
         << ",\"gpu_noise_generation_ms\":" << result.timings.noise_generation_ms
         << ",\"gpu_rollout_simulation_ms\":" << result.timings.rollout_simulation_ms
         << ",\"gpu_risk_reduction_ms\":" << result.timings.risk_reduction_ms
         << ",\"gpu_weight_calculation_ms\":" << result.timings.weight_calculation_ms
         << ",\"gpu_control_update_ms\":" << result.timings.control_update_ms
         << ",\"gpu_repair_validation_ms\":" << result.timings.repair_validation_ms
         << ",\"post_update_evaluation_ms\":"
         << result.timings.post_update_evaluation_ms
         << ",\"gpu_ms\":" << result.timings.gpu_total_ms
         << ",\"horizon_reconstruction_ms\":"
         << result.timings.horizon_reconstruction_ms
         << ",\"total_ms\":" << result.timings.host_total_ms
         << ",\"snapshot_ms\":" << snapshot.snapshot_ms
         << ",\"stability_ms\":" << snapshot.stability_ms << ",\"rviz_ms\":" << rviz_ms
         << ",\"altitude_envelope_violation\":"
         << (result.altitude_envelope_violation ? "true" : "false")
         << ",\"raw_collision\":" << (result.raw_collision ? "true" : "false")
         << ",\"known_solid_collision\":"
         << (result.known_solid_collision ? "true" : "false")
         << ",\"route_terminal_cross_track_violation\":"
         << (result.route_terminal_cross_track_violation ? "true" : "false")
         << ",\"terminal_route_cross_track_m\":" << result.terminal_route_cross_track_m
         << ",\"route_terminal_arrival_shaping_attempts\":"
         << result.route_terminal_arrival_shaping_attempts
         << ",\"route_terminal_nominal_prefix_controls\":"
         << result.route_terminal_nominal_prefix_control_count << ",\"risk_tier\":\""
         << mppi::mppiRiskTierName(result.selected_tier) << '"'
         << ",\"feasible_available\":"
         << (result.feasibility_contract.available ? "true" : "false")
         << ",\"feasible_weight_sum\":"
         << finiteOrNegative(result.feasibility_contract.weight_sum)
         << ",\"post_update_classification\":\""
         << mppi::mppiPostUpdateClassificationName(
                result.post_update_classification.classification)
         << '"' << ",\"control_selection\":\""
         << mppi::mppiControlSelectionName(result.control_selection) << '"'
         << ",\"post_update_executable\":"
         << (result.post_update_classification.executable ? "true" : "false")
         << ",\"post_update_repair\":\""
         << mppi::mppiPostUpdateRepairName(result.post_update_repair) << '"'
         << ",\"post_update_backtrack_ratio\":" << result.post_update_backtrack_ratio
         << ",\"critical_exposure_m\":" << result.critical_exposure_m
         << ",\"planning_exposure_m\":" << result.planning_exposure_m
         << ",\"critical_clearance_proximity_s\":"
         << result.critical_clearance_proximity_s
         << ",\"obstacle_approach_m2_s\":" << result.obstacle_approach_m2_s
         << ",\"head_progress_m\":" << result.head_progress_m
         << ",\"terminal_progress_m\":" << result.terminal_progress_m
         << ",\"route_progress_integral_m_s\":" << result.route_progress_integral_m_s
         << ",\"warm_start_shift_ms\":" << result.warm_start_shift_s * 1000.0
         << ",\"nominal_reseeded\":" << (result.nominal_reseeded ? "true" : "false")
         << ",\"direct_maneuver_reseed\":"
         << (snapshot.direct_tracking_maneuver.reseed_requested ? "true" : "false")
         << ",\"direct_maneuver_reason\":\""
         << directTrackingReseedReasonName(snapshot.direct_tracking_maneuver.reason)
         << '"' << ",\"direct_bearing_change_rad\":"
         << snapshot.direct_tracking_maneuver.bearing_change_rad
         << ",\"direct_closing_speed_mps\":"
         << snapshot.direct_tracking_maneuver.closing_speed_mps
         << ",\"direct_no_closing_duration_s\":"
         << snapshot.direct_tracking_maneuver.no_closing_duration_s
         << ",\"target_directed_candidate_injected\":"
         << (result.target_directed_candidate_injected ? "true" : "false")
         << ",\"target_directed_candidate_raw_safe\":"
         << (result.target_directed_candidate_raw_safe ? "true" : "false")
         << ",\"target_directed_candidate_best_feasible\":"
         << (result.target_directed_candidate_best_feasible ? "true" : "false")
         << ",\"target_directed_candidate_weight\":"
         << result.target_directed_candidate_weight
         << ",\"route_directed_candidate_injected\":"
         << (result.route_directed_candidate_injected ? "true" : "false")
         << ",\"route_directed_candidate_raw_safe\":"
         << (result.route_directed_candidate_raw_safe ? "true" : "false")
         << ",\"route_directed_candidate_best_feasible\":"
         << (result.route_directed_candidate_best_feasible ? "true" : "false")
         << ",\"route_directed_candidate_weight\":"
         << result.route_directed_candidate_weight
         << ",\"route_directed_candidate_generation\":"
         << result.route_directed_candidate_generation
         << ",\"local_route_stop_is_terminal\":"
         << (snapshot.local_route_stop_is_terminal ? "true" : "false")
         << detail::rollingRouteJsonFields(snapshot.rolling_route)
         << ",\"no_eligible_phase\":\""
         << mppiNoEligiblePhaseName(snapshot.no_eligible_recovery.phase) << '"'
         << ",\"no_eligible_recovery_generation\":"
         << snapshot.no_eligible_recovery.no_eligible_recovery_generation
         << ",\"no_eligible_route_replan\":"
         << (snapshot.no_eligible_recovery.route_replan_requested ? "true" : "false")
         << ",\"liveness_state\":\"" << mppiLivenessStateName(liveness.state) << '"'
         << ",\"liveness_recovery_active\":"
         << (liveness.recovery_active ? "true" : "false")
         << ",\"liveness_actual_displacement_m\":" << liveness.actual_displacement_m
         << ",\"liveness_actual_route_progress_m\":" << liveness.actual_route_progress_m
         << ",\"liveness_route_progress_used\":"
         << (liveness.used_route_progress ? "true" : "false")
         << ",\"liveness_reseed_generation\":" << liveness.reseed_generation
         << ",\"route_required_risk_tier\":\""
         << mppi::mppiRiskTierName(snapshot.route_required_risk_tier) << '"'
         << ",\"maximum_acceleration_mps2\":" << result.maximum_acceleration_mps2
         << ",\"maximum_jerk_mps3\":" << result.maximum_jerk_mps3
         << ",\"first_control_delta\":" << result.first_control_delta
         << ",\"stability_rms_m\":"
         << (stability.valid ? stability.position_rms_m : -1.0)
         << ",\"dropped_diagnostics\":"
         << dropped_diagnostics_snapshots_.load(std::memory_order_relaxed) << "}\n";
    std::string json_line = json.str();
    diagnostics_stream_ << json_line;
    last_diagnostics_file_stamp_ns_ = now_ns;
    diagnostics_error_ring_.push_back(std::move(json_line));
    while (diagnostics_error_ring_.size() > diagnostics_error_ring_capacity_) {
      diagnostics_error_ring_.pop_front();
    }
    if (new_error_episode && diagnostics_error_stream_) {
      diagnostics_error_stream_
          << "{\"event\":\"diagnostics_error_context\",\"trigger_tick\":"
          << snapshot.tick_sequence << ",\"records\":" << diagnostics_error_ring_.size()
          << "}\n";
      for (const std::string& record : diagnostics_error_ring_) {
        diagnostics_error_stream_ << record;
      }
      diagnostics_error_stream_.flush();
      diagnostics_stream_.flush();
      last_diagnostics_flush_time_ = std::chrono::steady_clock::now();
    }
    diagnostics_error_active_ = diagnostics_error;
  }
  const auto flush_now = std::chrono::steady_clock::now();
  if (diagnostics_stream_ &&
      flush_now - last_diagnostics_flush_time_ >=
          std::chrono::duration<double>{diagnostics_flush_period_s_}) {
    diagnostics_stream_.flush();
    last_diagnostics_flush_time_ = flush_now;
  }
  if (now_ns - last_summary_stamp_ns_ >= 5000000000LL) {
    publishSummary();
    last_summary_stamp_ns_ = now_ns;
  }
}

} // namespace drone_city_nav
