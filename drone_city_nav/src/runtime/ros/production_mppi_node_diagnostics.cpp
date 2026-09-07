#include "drone_city_nav/json_output.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <iomanip>
#include <numeric>
#include <sstream>

#include "navigation_diagnostics_sink.hpp"
#include "production_mppi_cooperative_diagnostics.hpp"
#include "production_mppi_diagnostics_snapshot.hpp"
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
  const WorldSnapshot3D empty_world;
  const WorldSnapshot3D& world =
      snapshot.world != nullptr ? *snapshot.world : empty_world;
  const ProductionRouteActivationResult3D empty_route_pipeline;
  const ProductionRouteActivationResult3D& route_pipeline =
      snapshot.route_pipeline != nullptr ? *snapshot.route_pipeline
                                         : empty_route_pipeline;
  const MaterializedRoute3D& route_candidate = route_pipeline.materialized;
  const ProductionRoutePipelineTelemetry3D& route_telemetry = route_pipeline.telemetry;
  const ProductionRouteMaterializationTelemetry3D& materialization_telemetry =
      route_telemetry.materialization;
  const ProductionPersistentPlannerTelemetry3D& planner_telemetry =
      route_telemetry.planner;
  const RouteAdmissionReport3D& admission = route_pipeline.admission;
  const CertifiedRouteSuffix3D* const execution_route = snapshot.execution_route.get();
  const ProductionMppiStability& stability = snapshot.stability;
  const ProductionMppiPredictionError& prediction = snapshot.prediction;
  const MppiLivenessResult& liveness = snapshot.liveness;
  const MppiSpeedPolicyResult& speed_policy = snapshot.speed_policy;
  const SensorBrakingAssessment3D& sensor_braking =
      speed_policy.sensor_braking_assessment;
  const detail::TrackingPursuitDiagnostics pursuit_diagnostics =
      detail::trackingPursuitDiagnostics(objective.get(), input, snapshot.execution);
  const ConstrainedRouteObservation route_constraint =
      diagnosticRouteConstraint(snapshot, config_.planning.route_envelope,
                                config_.diagnostics.route_constraint_distance_m);
  const ProductionMppiPlanningState planning_state = snapshot.planning_state;
  const std::string_view target_source = snapshot.target_source;
  const char* static_route_generation_matches = "not_attempted";
  if (admission.generation_assessed) {
    static_route_generation_matches = admission.generation_matches ? "true" : "false";
  }
  const auto rviz_started = std::chrono::steady_clock::now();
  publishRviz(snapshot);
  const double rviz_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - rviz_started)
                             .count();

  logDiagnosticsEvents(snapshot, route_constraint);

  std::ostringstream line;
  line << std::fixed << std::setprecision(3)
       << "PRODUCTION_MPPI_TICK tick=" << snapshot.tick_sequence
       << " pose_revision=" << input.pose_revision
       << " raw_revision=" << input.obstacle_revision
       << " esdf_revision=" << result.esdf_revision
       << " memory_sequence=" << snapshot.memory_sequence
       << " pose_age_ms=" << snapshot.pose_age_ms
       << " observation_age_ms=" << snapshot.observation_age_ms
       << " esdf_content_age_ms=" << snapshot.esdf_age_ms
       << " local_world_generation=" << world.local_world_generation.generation
       << " control_feedback_age_ms=" << snapshot.control_feedback_age_ms
       << " state_position=(" << input.initial_state.x << ',' << input.initial_state.y
       << ',' << input.initial_state.z << ") state_velocity=(" << input.initial_state.vx
       << ',' << input.initial_state.vy << ',' << input.initial_state.vz << ')'
       << " planning_mode=" << (config_.world.use_static_map ? "static" : "no_static")
       << " planning_state=" << productionMppiPlanningStateName(planning_state)
       << detail::executionInfoFields(snapshot.execution) << " horizon_s="
       << static_cast<double>(config_.control.mppi.steps) *
              config_.control.mppi.dynamics.dt_s
       << " target_source=" << target_source << " target=(" << input.target.x << ','
       << input.target.y << ',' << input.target.z << ")"
       << " route_generation=" << route_candidate.candidate_generation
       << " route_objective_epoch=" << route_candidate.objective.mission_epoch
       << " route_objective_sample=" << route_candidate.objective.sample_sequence
       << " route_assignment_generation="
       << route_candidate.objective.assignment_generation
       << " route_target_detection_id=" << route_candidate.objective.target_detection_id
       << " route_target_track_id=" << route_candidate.objective.target_track_id
       << " route_reaches_mission_goal="
       << (route_candidate.reaches_mission_goal ? "true" : "false")
       << " route_intent_id=" << route_candidate.intent.id
       << " route_intent_planned_on=" << route_candidate.intent.planned_on_revision
       << " route_validated_through="
       << route_candidate.segment_evidence.validated_through_revision
       << " route_segment_evidence="
       << segmentEvidenceStatus3DName(route_candidate.segment_evidence.status)
       << " route_unknown_exposure="
       << (route_candidate.segment_evidence.unknown_exposure ? "true" : "false")
       << " route_known_clearance="
       << (route_candidate.segment_evidence.known_clearance_observed ? "true" : "false")
       << " goal_capture_latched=" << (snapshot.goal_capture.latched ? "true" : "false")
       << " goal_distance_m=" << snapshot.goal_capture.distance_m
       << " route_station_m=" << snapshot.route_station_m
       << " route_remaining_m=" << snapshot.route_remaining_m
       << " route_constraint_phase="
       << constrainedRoutePhaseName(route_constraint.phase)
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
       << route_constraint.cross_track_error_m
       << " route_constraint_vertical_window_ok="
       << (route_constraint.within_vertical_window ? "true" : "false")
       << " route_progress_action="
       << routeProgressAction3DName(snapshot.route_progress.action)
       << " route_local_reseed_generation="
       << snapshot.route_progress.local_reseed_generation << " planning_search_kind="
       << productionPlanningSearchKindName(route_candidate.provenance.kind)
       << " planning_search_base_route_instance_id="
       << route_candidate.provenance.base_route_instance_id.value
       << " planning_search_base_stitch_station_m="
       << route_candidate.provenance.base_stitch_station_m.value_or(-1.0)
       << " required_splice_base_route_instance_id="
       << route_candidate.provenance.required_splice_base_route_instance_id.value
       << " planning_search_start=(" << route_candidate.provenance.start.x << ','
       << route_candidate.provenance.start.y << ','
       << route_candidate.provenance.start.z << ')' << " planning_search_goal=("
       << route_candidate.provenance.goal.x << ',' << route_candidate.provenance.goal.y
       << ',' << route_candidate.provenance.goal.z << ") planning_candidate_endpoint=("
       << route_candidate.provenance.candidate_endpoint.x << ','
       << route_candidate.provenance.candidate_endpoint.y << ','
       << route_candidate.provenance.candidate_endpoint.z << ')'
       << " planning_search_direction=(" << route_candidate.provenance.direction.x
       << ',' << route_candidate.provenance.direction.y << ','
       << route_candidate.provenance.direction.z << ')'
       << " planning_candidate_points=" << route_candidate.provenance.candidate_points
       << " planning_candidate_samples=" << route_candidate.provenance.candidate_samples
       << persistentPlannerInfoFields(planner_telemetry) << " static_route_candidate="
       << staticRouteCandidateStatusName(admission.candidate_validation.status)
       << certifiedRouteReserveInfoFields(admission)
       << trackingErrorTubeInfoFields(execution_route) << " static_route_activation="
       << staticRouteActivationStatusName(admission.activation_status)
       << " route_successor_improvement="
       << routeSuccessorImprovementStatus3DName(admission.successor_improvement.status)
       << " route_successor_improvement_required="
       << (admission.successor_improvement_required ? "true" : "false")
       << " route_successor_compared_to_pending="
       << (admission.successor_compared_to_pending ? "true" : "false")
       << " route_successor_resident_remaining_s="
       << finiteOrNegative(admission.successor_improvement.resident_remaining_time_s)
       << " route_successor_candidate_remaining_s="
       << finiteOrNegative(admission.successor_improvement.candidate_remaining_time_s)
       << " route_successor_absolute_improvement_s="
       << admission.successor_improvement.absolute_improvement_s
       << " route_successor_relative_improvement="
       << admission.successor_improvement.relative_improvement
       << " static_route_publication_status="
       << routePublicationStatus3DName(admission.assessment.publication.status)
       << " static_route_world_compatible="
       << (admission.world_compatible ? "true" : "false")
       << " static_route_generation_matches=" << static_route_generation_matches
       << " route_selected_passage_traversals="
       << (route_candidate.selected_passage_traversal_ids
               ? route_candidate.selected_passage_traversal_ids->size()
               : 0U)
       << " pose_predicted=" << (snapshot.pose_predicted ? "true" : "false")
       << " target_lookahead_m=" << speed_policy.target_lookahead_m
       << " reference_speed_mps=" << input.reference_speed_mps
       << detail::trackingPursuitInfoFields(pursuit_diagnostics, speed_policy, result)
       << " curvature_speed_limit_mps="
       << finiteOrNegative(speed_policy.curvature_limit_mps)
       << " sensor_braking_speed_limit_mps="
       << finiteOrNegative(speed_policy.sensor_braking_limit_mps)
       << " clearance_speed_limit_mps="
       << finiteOrNegative(speed_policy.clearance_limit_mps)
       << " sensor_braking_assessed_speed_mps=" << sensor_braking.speed_mps
       << " sensor_braking_total_latency_s=" << sensor_braking.total_latency_s
       << " sensor_braking_latency_distance_m=" << sensor_braking.latency_distance_m
       << " sensor_braking_stopping_distance_m=" << sensor_braking.stopping_distance_m
       << " sensor_braking_physical_margin_m=" << sensor_braking.physical_margin_m
       << " sensor_braking_required_detection_range_m="
       << sensor_braking.required_detection_range_m
       << " sensor_braking_guaranteed_detection_range_m="
       << sensor_braking.guaranteed_detection_range_m
       << " sensor_braking_reserve_m=" << sensor_braking.reserve_m
       << " sensor_braking_accepted=" << (sensor_braking.accepted() ? "true" : "false")
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
       << " snapshot_ms=" << snapshot.phases.snapshot_ms
       << " capture_ms=" << snapshot.phases.capture_ms
       << " execution_input_ms=" << snapshot.phases.execution_input_ms
       << " cycle_prepare_ms=" << snapshot.phases.cycle_prepare_ms
       << " controller_ms=" << snapshot.phases.controller_ms
       << " publication_ms=" << snapshot.phases.publication_ms
       << " assembly_ms=" << snapshot.phases.assembly_ms
       << " commit_ms=" << snapshot.phases.commit_ms
       << " wire_ms=" << snapshot.phases.wire_ms
       << " tick_total_ms=" << snapshot.phases.total_ms
       << " stability_ms=" << snapshot.stability_ms << " rviz_ms=" << rviz_ms
       << " deadline_missed="
       << (result.timings.host_total_ms > config_.planning.deadline_ms ? "true"
                                                                       : "false")
       << " risk_tier=" << mppi::mppiRiskTierName(result.selected_tier)
       << " altitude_envelope_violation="
       << (result.altitude_envelope_violation ? "true" : "false")
       << " route_terminal_cross_track_violation="
       << (result.route_terminal_cross_track_violation ? "true" : "false")
       << " terminal_route_cross_track_m=" << result.terminal_route_cross_track_m
       << " route_terminal_arrival_shaping_attempts="
       << result.route_terminal_arrival_shaping_attempts
       << " route_terminal_nominal_prefix_controls="
       << result.route_terminal_nominal_prefix_control_count
       << " critical_exposure_m=" << result.critical_exposure_m
       << " planning_exposure_m=" << result.planning_exposure_m
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
       << " target_directed_candidate_device_feasible="
       << (result.target_directed_candidate_device_feasible ? "true" : "false")
       << " target_directed_candidate_best_feasible="
       << (result.target_directed_candidate_best_feasible ? "true" : "false")
       << " target_directed_candidate_weight="
       << result.target_directed_candidate_weight
       << " route_directed_candidate_injected="
       << (result.route_directed_candidate_injected ? "true" : "false")
       << " route_directed_candidate_device_feasible="
       << (result.route_directed_candidate_device_feasible ? "true" : "false")
       << " route_directed_candidate_best_feasible="
       << (result.route_directed_candidate_best_feasible ? "true" : "false")
       << " route_directed_candidate_weight=" << result.route_directed_candidate_weight
       << " route_directed_candidate_cost_excess="
       << result.route_directed_candidate_cost_excess
       << " effective_temperature=" << result.effective_temperature
       << " effective_sample_fraction=" << result.effective_sample_fraction
       << " collision_gate_lifted=" << (result.collision_gate_lifted ? "true" : "false")
       << " route_directed_candidate_generation="
       << result.route_directed_candidate_generation << " local_route_stop_is_terminal="
       << (snapshot.local_route_stop_is_terminal ? "true" : "false")
       << detail::rollingRouteInfoFields(snapshot.rolling_route)
       << " no_eligible_phase="
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
       << " esdf_build_ms=" << snapshot.world_build.build_ms
       << " esdf_x_pass_ms=" << snapshot.world_build.esdf_x_pass_ms
       << " esdf_y_pass_ms=" << snapshot.world_build.esdf_y_pass_ms
       << " esdf_z_pass_ms=" << snapshot.world_build.esdf_z_pass_ms
       << " esdf_finalize_ms=" << snapshot.world_build.esdf_finalize_ms
       << " route_search_ms=" << route_telemetry.route_search_ms
       << " continuation_validation_ms="
       << materialization_telemetry.continuation_validation_ms
       << " route_smoothing_ms=" << materialization_telemetry.route_smoothing_ms
       << " route_shortcuts_applied="
       << materialization_telemetry.route_shortcuts_applied
       << " route_corners_smoothed=" << materialization_telemetry.route_corners_smoothed
       << " candidate_validation_ms="
       << materialization_telemetry.candidate_validation_ms
       << " route_fingerprint=" << route_candidate.fingerprint
       << " esdf_upload_ms=" << snapshot.world_build.upload_ms
       << " dropped_diagnostics=" << diagnostics_sink_->droppedSnapshots();
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  if (now_ns - last_diagnostics_info_stamp_ns_ >= config_.diagnostics.info_period_ns) {
    RCLCPP_INFO(get_logger(), "%s", line.str().c_str());
    std_msgs::msg::String status;
    status.data = line.str();
    status_pub_->publish(status);
    last_diagnostics_info_stamp_ns_ = now_ns;
  }
  const bool diagnostics_error = result.altitude_envelope_violation;
  const NavigationDiagnosticsFileRecordDecision file_record =
      diagnostics_sink_->assessFileRecord(now_ns, diagnostics_error);
  if (file_record.required) {
    JsonOutputStream json;
    json
        << "{\"tick\":" << snapshot.tick_sequence
        << ",\"pose_revision\":" << input.pose_revision
        << ",\"raw_revision\":" << input.obstacle_revision
        << ",\"esdf_revision\":" << result.esdf_revision
        << ",\"pose_age_ms\":" << snapshot.pose_age_ms
        << ",\"observation_age_ms\":" << snapshot.observation_age_ms
        << ",\"esdf_content_age_ms\":" << snapshot.esdf_age_ms
        << ",\"local_world_generation\":" << world.local_world_generation.generation
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
        << (config_.world.use_static_map ? "static" : "no_static") << '"'
        << ",\"esdf_build_ms\":" << snapshot.world_build.build_ms
        << ",\"esdf_x_pass_ms\":" << snapshot.world_build.esdf_x_pass_ms
        << ",\"esdf_y_pass_ms\":" << snapshot.world_build.esdf_y_pass_ms
        << ",\"esdf_z_pass_ms\":" << snapshot.world_build.esdf_z_pass_ms
        << ",\"esdf_finalize_ms\":" << snapshot.world_build.esdf_finalize_ms
        << ",\"route_search_ms\":" << route_telemetry.route_search_ms
        << ",\"continuation_validation_ms\":"
        << materialization_telemetry.continuation_validation_ms
        << ",\"route_smoothing_ms\":" << materialization_telemetry.route_smoothing_ms
        << ",\"route_shortcuts_applied\":"
        << materialization_telemetry.route_shortcuts_applied
        << ",\"route_corners_smoothed\":"
        << materialization_telemetry.route_corners_smoothed
        << ",\"candidate_validation_ms\":"
        << materialization_telemetry.candidate_validation_ms
        << ",\"route_fingerprint\":" << route_candidate.fingerprint
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
        << static_cast<double>(config_.control.mppi.steps) *
               config_.control.mppi.dynamics.dt_s
        << ",\"horizontal_speed_cap_mps\":"
        << config_.control.mppi.dynamics.maximum_horizontal_speed_mps
        << ",\"translational_speed_cap_mps\":"
        << config_.control.mppi.dynamics.maximum_translational_speed_mps
        << ",\"acceleration_cap_mps2\":"
        << config_.control.mppi.dynamics.maximum_horizontal_acceleration_mps2
        << ",\"jerk_cap_mps3\":"
        << config_.control.mppi.dynamics.maximum_control_jerk_mps3
        << ",\"speed_tracking_weight\":"
        << config_.control.mppi.costs.speed_tracking_weight
        << ",\"route_generation\":" << route_candidate.candidate_generation
        << ",\"route_objective_epoch\":" << route_candidate.objective.mission_epoch
        << ",\"route_objective_sample\":" << route_candidate.objective.sample_sequence
        << ",\"route_assignment_generation\":"
        << route_candidate.objective.assignment_generation
        << ",\"route_target_detection_id\":"
        << route_candidate.objective.target_detection_id
        << ",\"route_target_track_id\":" << route_candidate.objective.target_track_id
        << ",\"route_reaches_mission_goal\":"
        << (route_candidate.reaches_mission_goal ? "true" : "false")
        << ",\"route_intent_id\":" << route_candidate.intent.id
        << ",\"route_intent_planned_on\":" << route_candidate.intent.planned_on_revision
        << ",\"route_validated_through\":"
        << route_candidate.segment_evidence.validated_through_revision
        << ",\"route_segment_evidence\":\""
        << segmentEvidenceStatus3DName(route_candidate.segment_evidence.status) << '"'
        << ",\"route_unknown_exposure\":"
        << (route_candidate.segment_evidence.unknown_exposure ? "true" : "false")
        << ",\"route_known_clearance\":"
        << (route_candidate.segment_evidence.known_clearance_observed ? "true"
                                                                      : "false")
        << ",\"goal_capture_latched\":"
        << (snapshot.goal_capture.latched ? "true" : "false")
        << ",\"goal_distance_m\":" << snapshot.goal_capture.distance_m
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
        << route_constraint.actual_vertical_speed_mps << ",\"route_progress_action\":\""
        << routeProgressAction3DName(snapshot.route_progress.action) << '"'
        << ",\"route_local_reseed_generation\":"
        << snapshot.route_progress.local_reseed_generation
        << ",\"planning_search_kind\":\""
        << productionPlanningSearchKindName(route_candidate.provenance.kind) << '"'
        << ",\"planning_search_base_route_instance_id\":"
        << route_candidate.provenance.base_route_instance_id.value
        << ",\"planning_search_base_stitch_station_m\":"
        << route_candidate.provenance.base_stitch_station_m.value_or(-1.0)
        << ",\"required_splice_base_route_instance_id\":"
        << route_candidate.provenance.required_splice_base_route_instance_id.value
        << ",\"planning_search_start_x\":" << route_candidate.provenance.start.x
        << ",\"planning_search_start_y\":" << route_candidate.provenance.start.y
        << ",\"planning_search_start_z\":" << route_candidate.provenance.start.z
        << ",\"planning_search_goal_x\":" << route_candidate.provenance.goal.x
        << ",\"planning_search_goal_y\":" << route_candidate.provenance.goal.y
        << ",\"planning_search_goal_z\":" << route_candidate.provenance.goal.z
        << ",\"planning_candidate_endpoint_x\":"
        << route_candidate.provenance.candidate_endpoint.x
        << ",\"planning_candidate_endpoint_y\":"
        << route_candidate.provenance.candidate_endpoint.y
        << ",\"planning_candidate_endpoint_z\":"
        << route_candidate.provenance.candidate_endpoint.z
        << ",\"planning_search_direction_x\":" << route_candidate.provenance.direction.x
        << ",\"planning_search_direction_y\":" << route_candidate.provenance.direction.y
        << ",\"planning_search_direction_z\":" << route_candidate.provenance.direction.z
        << ",\"planning_candidate_points\":"
        << route_candidate.provenance.candidate_points
        << ",\"planning_candidate_samples\":"
        << route_candidate.provenance.candidate_samples
        << persistentPlannerJsonFields(planner_telemetry)
        << ",\"static_route_candidate\":\""
        << staticRouteCandidateStatusName(admission.candidate_validation.status) << '"'
        << certifiedRouteReserveJsonFields(admission)
        << trackingErrorTubeJsonFields(execution_route)
        << ",\"static_route_activation\":\""
        << staticRouteActivationStatusName(admission.activation_status) << '"'
        << ",\"route_successor_improvement\":\""
        << routeSuccessorImprovementStatus3DName(admission.successor_improvement.status)
        << '"' << ",\"route_successor_improvement_required\":"
        << (admission.successor_improvement_required ? "true" : "false")
        << ",\"route_successor_compared_to_pending\":"
        << (admission.successor_compared_to_pending ? "true" : "false")
        << ",\"route_successor_resident_remaining_s\":"
        << finiteOrNegative(admission.successor_improvement.resident_remaining_time_s)
        << ",\"route_successor_candidate_remaining_s\":"
        << finiteOrNegative(admission.successor_improvement.candidate_remaining_time_s)
        << ",\"route_successor_absolute_improvement_s\":"
        << admission.successor_improvement.absolute_improvement_s
        << ",\"route_successor_relative_improvement\":"
        << admission.successor_improvement.relative_improvement
        << ",\"static_route_publication_status\":\""
        << routePublicationStatus3DName(admission.assessment.publication.status) << '"'
        << ",\"static_route_world_compatible\":"
        << (admission.world_compatible ? "true" : "false")
        << ",\"static_route_generation_matches\":";
    if (admission.generation_assessed) {
      json << (admission.generation_matches ? "true" : "false");
    } else {
      json << "null";
    }
    json << ",\"route_selected_passage_count\":"
         << (route_candidate.selected_passage_traversal_ids
                 ? route_candidate.selected_passage_traversal_ids->size()
                 : 0U)
         << ",\"pose_predicted\":" << (snapshot.pose_predicted ? "true" : "false")
         << ",\"target_lookahead_m\":" << speed_policy.target_lookahead_m
         << ",\"reference_speed_mps\":" << input.reference_speed_mps
         << detail::trackingPursuitJsonFields(pursuit_diagnostics, speed_policy, result)
         << ",\"curvature_speed_limit_mps\":"
         << finiteOrNegative(speed_policy.curvature_limit_mps)
         << ",\"sensor_braking_speed_limit_mps\":"
         << finiteOrNegative(speed_policy.sensor_braking_limit_mps)
         << ",\"clearance_speed_limit_mps\":"
         << finiteOrNegative(speed_policy.clearance_limit_mps)
         << ",\"sensor_braking_assessed_speed_mps\":" << sensor_braking.speed_mps
         << ",\"sensor_braking_total_latency_s\":" << sensor_braking.total_latency_s
         << ",\"sensor_braking_latency_distance_m\":"
         << sensor_braking.latency_distance_m
         << ",\"sensor_braking_stopping_distance_m\":"
         << sensor_braking.stopping_distance_m
         << ",\"sensor_braking_physical_margin_m\":" << sensor_braking.physical_margin_m
         << ",\"sensor_braking_required_detection_range_m\":"
         << sensor_braking.required_detection_range_m
         << ",\"sensor_braking_guaranteed_detection_range_m\":"
         << sensor_braking.guaranteed_detection_range_m
         << ",\"sensor_braking_reserve_m\":" << sensor_braking.reserve_m
         << ",\"sensor_braking_accepted\":"
         << (sensor_braking.accepted() ? "true" : "false")
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
         << ",\"snapshot_ms\":" << snapshot.phases.snapshot_ms
         << ",\"capture_ms\":" << snapshot.phases.capture_ms
         << ",\"execution_input_ms\":" << snapshot.phases.execution_input_ms
         << ",\"cycle_prepare_ms\":" << snapshot.phases.cycle_prepare_ms
         << ",\"controller_ms\":" << snapshot.phases.controller_ms
         << ",\"publication_ms\":" << snapshot.phases.publication_ms
         << ",\"assembly_ms\":" << snapshot.phases.assembly_ms
         << ",\"commit_ms\":" << snapshot.phases.commit_ms
         << ",\"wire_ms\":" << snapshot.phases.wire_ms
         << ",\"tick_total_ms\":" << snapshot.phases.total_ms
         << ",\"stability_ms\":" << snapshot.stability_ms << ",\"rviz_ms\":" << rviz_ms
         << ",\"altitude_envelope_violation\":"
         << (result.altitude_envelope_violation ? "true" : "false")
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
         << ",\"target_directed_candidate_device_feasible\":"
         << (result.target_directed_candidate_device_feasible ? "true" : "false")
         << ",\"target_directed_candidate_best_feasible\":"
         << (result.target_directed_candidate_best_feasible ? "true" : "false")
         << ",\"target_directed_candidate_weight\":"
         << result.target_directed_candidate_weight
         << ",\"route_directed_candidate_injected\":"
         << (result.route_directed_candidate_injected ? "true" : "false")
         << ",\"route_directed_candidate_device_feasible\":"
         << (result.route_directed_candidate_device_feasible ? "true" : "false")
         << ",\"route_directed_candidate_best_feasible\":"
         << (result.route_directed_candidate_best_feasible ? "true" : "false")
         << ",\"route_directed_candidate_weight\":"
         << result.route_directed_candidate_weight
         << ",\"route_directed_candidate_cost_excess\":"
         << result.route_directed_candidate_cost_excess
         << ",\"route_directed_candidate_critical_exposure_m\":"
         << result.route_directed_candidate_critical_exposure_m
         << ",\"route_directed_candidate_minimum_clearance_m\":"
         << result.route_directed_candidate_minimum_clearance_m
         << ",\"effective_temperature\":" << result.effective_temperature
         << ",\"effective_sample_fraction\":" << result.effective_sample_fraction
         << ",\"collision_gate_lifted\":"
         << (result.collision_gate_lifted ? "true" : "false")
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
         << ",\"dropped_diagnostics\":" << diagnostics_sink_->droppedSnapshots()
         << "}\n";
    diagnostics_sink_->appendFileRecord(file_record, snapshot.tick_sequence,
                                        json.str());
  }
  {
    // One compact line per tick. The full record above is far too large to
    // write at the tick rate and is throttled, and a throttled record cannot
    // answer how often the reference speed flips, how often the first control
    // opposes the velocity, or how long a stall lasted: those are properties
    // of the ticks it skips.
    JsonOutputStream track;
    track << "{\"tick\":" << snapshot.tick_sequence << ",\"stamp_ns\":" << now_ns
          << ",\"p\":[" << input.initial_state.x << ',' << input.initial_state.y << ','
          << input.initial_state.z << "],\"v\":[" << input.initial_state.vx << ','
          << input.initial_state.vy << ',' << input.initial_state.vz
          << "],\"first_control\":["
          << (result.controls.empty() ? 0.0F : result.controls.front().ax) << ','
          << (result.controls.empty() ? 0.0F : result.controls.front().ay) << ','
          << (result.controls.empty() ? 0.0F : result.controls.front().az)
          << "],\"control_selection\":\""
          << mppi::mppiControlSelectionName(result.control_selection)
          << "\",\"reference_speed_mps\":" << speed_policy.reference_speed_mps
          << ",\"unslewed_reference_speed_mps\":"
          << speed_policy.unslewed_reference_speed_mps << ",\"limiter\":\""
          << mppiSpeedLimiterName(speed_policy.active_limiter)
          << "\",\"minimum_esdf_m\":" << result.minimum_esdf_distance_m
          << ",\"risk_tier\":\"" << mppi::mppiRiskTierName(result.selected_tier)
          << "\",\"route_generation\":"
          << (execution_route != nullptr ? execution_route->identity.generation : 0U)
          << ",\"planning_state\":\"" << productionMppiPlanningStateName(planning_state)
          << "\",\"execution_reason\":\""
          << productionMppiExecutionReasonName(snapshot.execution.reason) << "\"}\n";
    diagnostics_sink_->appendTrackRecord(track.str());
  }
  diagnostics_sink_->flushFileIfDue();
  if (now_ns - last_summary_stamp_ns_ >= 5000000000LL) {
    publishSummary();
    last_summary_stamp_ns_ = now_ns;
  }
}

} // namespace drone_city_nav
