#include "drone_city_nav/mppi_debug_markers.hpp"
#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <utility>

#include "production_mppi_node.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] double percentile(std::vector<double> samples, const double ratio) {
  if (samples.empty()) {
    return 0.0;
  }
  std::sort(samples.begin(), samples.end());
  const std::size_t index = std::min(
      samples.size() - 1U,
      static_cast<std::size_t>(std::ceil(ratio * static_cast<double>(samples.size()))) -
          1U);
  return samples[index];
}

[[nodiscard]] double finiteOrNegative(const double value) noexcept {
  return std::isfinite(value) ? value : -1.0;
}

} // namespace

void ProductionMppiNode::diagnosticsWorker(const std::stop_token stop_token) {
  while (!stop_token.stop_requested()) {
    std::optional<ProductionMppiDiagnosticsSnapshot> snapshot =
        diagnostics_mailbox_.waitPop(stop_token);
    if (!snapshot.has_value()) {
      break;
    }
    processDiagnostics(*snapshot);
  }
  if (std::optional<ProductionMppiDiagnosticsSnapshot> pending =
          diagnostics_mailbox_.tryPop();
      pending.has_value()) {
    processDiagnostics(*pending);
  }
  if (diagnostics_stream_) {
    diagnostics_stream_.flush();
  }
}

void ProductionMppiNode::enqueueDiagnostics(
    ProductionMppiDiagnosticsSnapshot snapshot) {
  if (diagnostics_mailbox_.push(std::move(snapshot))) {
    dropped_diagnostics_snapshots_.fetch_add(1U, std::memory_order_relaxed);
  }
}

void ProductionMppiNode::recordTickStatistics(
    const mppi::MppiTickResult& result,
    const ProductionMppiPlanningState planning_state,
    const bool liveness_reseed_requested) {
  const std::scoped_lock lock{statistics_mutex_};
  ++completed_ticks_;
  runtime_samples_ms_.push_back(result.timings.host_total_ms);
  deadline_misses_ += result.timings.host_total_ms > deadline_ms_ ? 1U : 0U;
  raw_collision_horizons_ += result.raw_collision ? 1U : 0U;
  solid_collision_horizons_ += result.known_solid_collision ? 1U : 0U;
  post_update_contract_violations_ +=
      planning_state == ProductionMppiPlanningState::kPlanned &&
              !result.post_update_classification.contract_preserved
          ? 1U
          : 0U;
  no_progress_horizons_ += result.head_progress_m <= 0.0F ? 1U : 0U;
  liveness_reseeds_ += liveness_reseed_requested ? 1U : 0U;
  no_guide_braking_hold_ticks_ +=
      planning_state == ProductionMppiPlanningState::kNoGuideBrakingHold ? 1U : 0U;
  unavailable_world_braking_hold_ticks_ +=
      planning_state == ProductionMppiPlanningState::kUnavailableWorldBrakingHold ? 1U
                                                                                  : 0U;
  mission_goal_position_hold_ticks_ +=
      planning_state == ProductionMppiPlanningState::kMissionGoalPositionHold ? 1U : 0U;
}

void ProductionMppiNode::publishRviz(
    const ProductionMppiDiagnosticsSnapshot& snapshot) {
  if (!snapshot.rviz.has_value()) {
    return;
  }
  const ProductionMppiRvizSnapshot& rviz = *snapshot.rviz;
  const auto stamp = now();
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id_;
  path.header.stamp = stamp;
  path.poses.reserve(rviz.candidate_horizon.size());
  for (const mppi::State& state : rviz.candidate_horizon) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = state.x;
    pose.pose.position.y = state.y;
    pose.pose.position.z = gazeboAlignedRvizZ(state.z);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  path_pub_->publish(path);

  const std::span<const mppi::State> previous_horizon{rviz.previous_horizon};
  const std::span<const mppi::State> execution_horizon{rviz.execution_horizon};
  const std::span<const mppi::RouteSample3D> global_route =
      rviz.route ? std::span<const mppi::RouteSample3D>{*rviz.route}
                 : std::span<const mppi::RouteSample3D>{};
  const visualization_msgs::msg::MarkerArray markers =
      buildMppiDebugMarkers(MppiDebugMarkerInput{
          path.header, rviz.candidate_horizon, previous_horizon, execution_horizon,
          global_route, snapshot.input.initial_state, snapshot.input.target,
          mission_start_, mission_goal_, snapshot.result.selected_tier});
  markers_pub_->publish(markers);
}

void ProductionMppiNode::processDiagnostics(
    const ProductionMppiDiagnosticsSnapshot& snapshot) {
  const mppi::MppiTickInput& input = snapshot.input;
  const mppi::MppiTickResult& result = snapshot.result;
  const ProductionMppiPreparedEsdf& esdf = snapshot.esdf;
  const ProductionMppiStability& stability = snapshot.stability;
  const ProductionMppiPredictionError& prediction = snapshot.prediction;
  const MppiLivenessResult& liveness = snapshot.liveness;
  const MppiSpeedPolicyResult& speed_policy = snapshot.speed_policy;
  const std::span<const RouteSample3D> route =
      snapshot.route_projection_valid && esdf.route_3d
          ? std::span<const RouteSample3D>{*esdf.route_3d}
          : std::span<const RouteSample3D>{};
  const std::span<const ConstrainedRouteSpan> spans =
      esdf.constrained_spans
          ? std::span<const ConstrainedRouteSpan>{*esdf.constrained_spans}
          : std::span<const ConstrainedRouteSpan>{};
  const ConstrainedRouteObservation route_constraint = observeConstrainedRoute(
      route, spans, esdf.global_guide_generation, snapshot.route_station_m,
      Point3{input.initial_state.x, input.initial_state.y, input.initial_state.z},
      Vec3{input.initial_state.vx, input.initial_state.vy, input.initial_state.vz},
      route_envelope_config_, route_constraint_diagnostics_distance_m_);
  const ProductionMppiPlanningState planning_state = snapshot.planning_state;
  const std::string_view target_source = snapshot.target_source;
  const auto rviz_started = std::chrono::steady_clock::now();
  publishRviz(snapshot);
  const double rviz_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - rviz_started)
                             .count();

  if (snapshot.liveness_reseed_requested) {
    RCLCPP_WARN(get_logger(),
                "MPPI_LIVENESS_RESEED generation=%" PRIu64
                " observation_age_s=%.3f actual_displacement_m=%.3f speed_mps=%.3f "
                "predicted_head_progress_m=%.3f predicted_terminal_progress_m=%.3f",
                liveness.reseed_generation, liveness.observation_age_s,
                liveness.actual_displacement_m, liveness.actual_speed_mps,
                liveness.predicted_head_progress_m,
                liveness.predicted_terminal_progress_m);
  }
  if (snapshot.guide_progress.local_reseed_requested) {
    RCLCPP_WARN(
        get_logger(),
        "GLOBAL_GUIDE_LOCAL_RESEED guide_generation=%" PRIu64
        " reseed_generation=%" PRIu64
        " observation_age_s=%.3f along_guide_progress_m=%.3f "
        "predicted_head_progress_m=%.3f",
        esdf.global_guide_generation, snapshot.guide_progress.local_reseed_generation,
        snapshot.guide_progress.observation_age_s, snapshot.guide_progress.progress_m,
        snapshot.guide_progress.predicted_head_progress_m);
  }
  if (snapshot.guide_progress.stalled) {
    RCLCPP_WARN(get_logger(),
                "GLOBAL_GUIDE_STALL guide_generation=%" PRIu64 " reason=%s"
                " stall_generation=%" PRIu64
                " observation_age_s=%.3f along_guide_progress_m=%.3f "
                "predicted_head_progress_m=%.3f",
                esdf.global_guide_generation,
                globalGuideProgressActionName(snapshot.guide_progress.action),
                snapshot.guide_progress.stall_generation,
                snapshot.guide_progress.observation_age_s,
                snapshot.guide_progress.progress_m,
                snapshot.guide_progress.predicted_head_progress_m);
  }
  if (snapshot.no_eligible_recovery.guide_replan_requested) {
    RCLCPP_WARN(get_logger(),
                "MPPI_NO_ELIGIBLE_RECOVERY action=release_global_guide"
                " recovery_generation=%" PRIu64 " phase=%s guide_generation=%" PRIu64,
                snapshot.no_eligible_recovery.no_eligible_recovery_generation,
                mppiNoEligiblePhaseName(snapshot.no_eligible_recovery.phase),
                esdf.global_guide_generation);
  }
  if (planning_state == ProductionMppiPlanningState::kNoGuideBrakingHold) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "PRODUCTION_MPPI_NO_GUIDE mode=%s action=braking_hold "
                         "lattice_status=%s lattice_termination=%s speed_mps=%.2f",
                         use_static_map_ ? "static" : "no_static",
                         latticePlanStatusName(esdf.lattice_status),
                         latticeSearchTerminationName(esdf.lattice_termination),
                         std::hypot(input.initial_state.vx, input.initial_state.vy));
  }
  if (planning_state == ProductionMppiPlanningState::kUnavailableWorldBrakingHold) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "PRODUCTION_MPPI_UNAVAILABLE_WORLD action=braking_hold esdf_age_ms=%.1f "
        "maximum_esdf_age_ms=%.1f raw_revision=%" PRIu64,
        snapshot.esdf_age_ms, maximum_esdf_age_ms_, esdf.revision);
  }
  if (planning_state == ProductionMppiPlanningState::kPlanned &&
      snapshot.esdf_age_ms > maximum_esdf_age_ms_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "PRODUCTION_MPPI_STALE_WORLD action=continue_resident_esdf "
                         "esdf_age_ms=%.1f warning_age_ms=%.1f revision=%" PRIu64,
                         snapshot.esdf_age_ms, maximum_esdf_age_ms_, esdf.revision);
  }
  if (snapshot.goal_capture.newly_latched) {
    RCLCPP_INFO(get_logger(),
                "MISSION_GOAL_CAPTURE state=latched goal=(%.2f, %.2f, %.2f) "
                "distance_m=%.3f action=position_hold",
                mission_goal_.x, mission_goal_.y, mission_goal_.z,
                snapshot.goal_capture.horizontal_distance_m);
  }

  const bool constraint_transition =
      !last_route_constraint_observation_.has_value() ||
      last_route_constraint_observation_->route_generation !=
          route_constraint.route_generation ||
      last_route_constraint_observation_->phase != route_constraint.phase ||
      last_route_constraint_observation_->span_available !=
          route_constraint.span_available ||
      (route_constraint.span_available &&
       last_route_constraint_observation_->span_index != route_constraint.span_index);
  const bool constraint_event_relevant =
      route_constraint.span_available ||
      (last_route_constraint_observation_.has_value() &&
       last_route_constraint_observation_->span_available);
  if (constraint_transition && constraint_event_relevant) {
    RCLCPP_INFO(
        get_logger(),
        "ROUTE_CONSTRAINT_EVENT route_generation=%" PRIu64
        " phase=%s span_index=%zd span_count=%zu station_m=%.2f "
        "span_station_m=(%.2f,%.2f) distance_m=(entry:%.2f,exit:%.2f) "
        "entry=(%.2f,%.2f,%.2f) exit=(%.2f,%.2f,%.2f) "
        "z=(actual:%.2f,reference:%.2f,min:%.2f,max:%.2f,error:%.2f,ok:%s) "
        "free_space=(left:%.2f,right:%.2f,lateral_width:%.2f,vertical_height:%.2f) "
        "constraint=(lateral:%s,vertical:%s) "
        "cross_track_m=%.2f speed_mps=%.2f vz_mps=%.2f reference_speed_mps=%.2f "
        "execution_mode=%s execution_reason=%s",
        route_constraint.route_generation,
        constrainedRoutePhaseName(route_constraint.phase).data(),
        route_constraint.span_available
            ? static_cast<std::ptrdiff_t>(route_constraint.span_index)
            : static_cast<std::ptrdiff_t>(-1),
        route_constraint.span_count, route_constraint.station_m,
        route_constraint.begin_station_m, route_constraint.end_station_m,
        route_constraint.distance_to_entry_m, route_constraint.distance_to_exit_m,
        route_constraint.entry_position.x, route_constraint.entry_position.y,
        route_constraint.entry_position.z, route_constraint.exit_position.x,
        route_constraint.exit_position.y, route_constraint.exit_position.z,
        input.initial_state.z, route_constraint.reference_z_m, route_constraint.min_z_m,
        route_constraint.max_z_m, route_constraint.vertical_error_m,
        route_constraint.within_vertical_window ? "true" : "false",
        route_constraint.lateral_free_left_m, route_constraint.lateral_free_right_m,
        route_constraint.lateral_width_m, route_constraint.vertical_height_m,
        route_constraint.lateral_constrained ? "true" : "false",
        route_constraint.vertical_constrained ? "true" : "false",
        route_constraint.cross_track_error_m,
        route_constraint.actual_horizontal_speed_mps,
        route_constraint.actual_vertical_speed_mps,
        route_constraint.reference_speed_mps,
        productionMppiExecutionModeName(snapshot.execution.mode),
        productionMppiExecutionReasonName(snapshot.execution.reason));
  }
  last_route_constraint_observation_ = route_constraint;

  std::ostringstream line;
  line << std::fixed << std::setprecision(3)
       << "PRODUCTION_MPPI_TICK tick=" << snapshot.tick_sequence
       << " pose_revision=" << input.pose_revision
       << " raw_revision=" << input.obstacle_revision
       << " esdf_revision=" << result.esdf_revision
       << " memory_sequence=" << snapshot.memory_sequence
       << " pose_age_ms=" << snapshot.pose_age_ms
       << " esdf_age_ms=" << snapshot.esdf_age_ms
       << " control_feedback_age_ms=" << snapshot.control_feedback_age_ms
       << " planning_mode=" << (use_static_map_ ? "static" : "no_static")
       << " planning_state=" << productionMppiPlanningStateName(planning_state)
       << " execution_mode=" << productionMppiExecutionModeName(snapshot.execution.mode)
       << " execution_reason="
       << productionMppiExecutionReasonName(snapshot.execution.reason)
       << " execution_published=" << (snapshot.execution.published ? "true" : "false")
       << " horizon_s="
       << static_cast<double>(mppi_config_.steps) * mppi_config_.dynamics.dt_s
       << " target_source=" << target_source << " target=(" << input.target.x << ','
       << input.target.y << ',' << input.target.z << ")"
       << " guide_generation=" << esdf.global_guide_generation
       << " guide_reused=" << (esdf.global_guide_reused ? "true" : "false")
       << " guide_reaches_mission_goal="
       << (esdf.global_guide_reaches_mission_goal ? "true" : "false")
       << " goal_capture_latched=" << (snapshot.goal_capture.latched ? "true" : "false")
       << " goal_distance_m=" << snapshot.goal_capture.horizontal_distance_m
       << " guide_release="
       << globalGuideReleaseReasonName(esdf.global_guide_release_reason)
       << " guide_heading_source="
       << globalGuideHeadingSourceName(esdf.global_guide_heading_source)
       << " guide_risk=" << globalGuideRiskTierName(esdf.global_guide_risk)
       << " guide_acceptance="
       << globalGuideAcceptanceReasonName(esdf.global_guide_acceptance_reason)
       << " guide_station_m=" << snapshot.route_station_m
       << " guide_remaining_m=" << snapshot.route_remaining_m
       << " route_constraint_phase="
       << constrainedRoutePhaseName(route_constraint.phase)
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
       << " guide_progress_action="
       << globalGuideProgressActionName(snapshot.guide_progress.action)
       << " guide_local_reseed_generation="
       << snapshot.guide_progress.local_reseed_generation
       << " lattice_search_performed="
       << (esdf.lattice_search_performed ? "true" : "false")
       << " lattice_status=" << latticePlanStatusName(esdf.lattice_status)
       << " lattice_termination="
       << latticeSearchTerminationName(esdf.lattice_termination)
       << " lattice_continuation_attempt=" << esdf.lattice_continuation_attempt
       << " lattice_search_session_resumed="
       << (esdf.lattice_search_session_resumed ? "true" : "false")
       << " lattice_risk_stage=" << latticeRiskStageName(esdf.lattice_risk_stage)
       << " lattice_stale_pops=" << esdf.lattice_stale_queue_pops
       << " lattice_open_peak=" << esdf.lattice_open_peak
       << " lattice_records_peak=" << esdf.lattice_records_peak
       << " lattice_two_step_states=" << esdf.lattice_two_step_reachable_states
       << " lattice_reachable_depth_m=" << esdf.lattice_reachable_depth_m
       << " pose_predicted=" << (snapshot.pose_predicted ? "true" : "false")
       << " maximum_eligible_risk_tier="
       << mppi::mppiRiskTierName(snapshot.maximum_eligible_risk_tier)
       << " target_lookahead_m=" << speed_policy.target_lookahead_m
       << " reference_speed_mps=" << input.reference_speed_mps
       << " curvature_speed_limit_mps="
       << finiteOrNegative(speed_policy.curvature_limit_mps)
       << " observation_speed_limit_mps="
       << finiteOrNegative(speed_policy.observation_limit_mps)
       << " goal_speed_limit_mps=" << finiteOrNegative(speed_policy.goal_limit_mps)
       << " gpu_ms=" << result.timings.gpu_total_ms
       << " total_ms=" << result.timings.host_total_ms
       << " snapshot_ms=" << snapshot.snapshot_ms
       << " stability_ms=" << snapshot.stability_ms << " rviz_ms=" << rviz_ms
       << " deadline_missed="
       << (result.timings.host_total_ms > deadline_ms_ ? "true" : "false")
       << " risk_tier=" << mppi::mppiRiskTierName(result.selected_tier)
       << " raw_collision=" << (result.raw_collision ? "true" : "false")
       << " known_solid_collision=" << (result.known_solid_collision ? "true" : "false")
       << " critical_exposure_m=" << result.critical_exposure_m
       << " planning_exposure_m=" << result.planning_exposure_m
       << " eligible_available="
       << (result.eligible_risk_contract.available ? "true" : "false")
       << " eligible_risk_tier="
       << mppi::mppiRiskTierName(result.eligible_risk_contract.tier)
       << " eligible_best_critical_exposure_m="
       << finiteOrNegative(result.eligible_risk_contract.best_critical_exposure_m)
       << " eligible_best_planning_exposure_m="
       << finiteOrNegative(result.eligible_risk_contract.best_planning_exposure_m)
       << " eligible_weight_sum="
       << finiteOrNegative(result.eligible_risk_contract.weight_sum)
       << " eligible_critical_limit_m="
       << finiteOrNegative(result.post_update_classification.critical_exposure_limit_m)
       << " eligible_planning_limit_m="
       << finiteOrNegative(result.post_update_classification.planning_exposure_limit_m)
       << " post_update_classification="
       << mppi::mppiPostUpdateClassificationName(
              result.post_update_classification.classification)
       << " post_update_contract_preserved="
       << (result.post_update_classification.contract_preserved ? "true" : "false")
       << " post_update_repair="
       << mppi::mppiPostUpdateRepairName(result.post_update_repair)
       << " post_update_backtrack_ratio=" << result.post_update_backtrack_ratio
       << " minimum_esdf_m=" << result.minimum_esdf_distance_m
       << " head_progress_m=" << result.head_progress_m
       << " terminal_progress_m=" << result.terminal_progress_m
       << " warm_start_shift_ms=" << result.warm_start_shift_s * 1000.0
       << " previous_control_source="
       << (input.previous_applied_control.has_value() ? "offboard_feedback"
                                                      : "engine_fallback")
       << " nominal_reseeded=" << (result.nominal_reseeded ? "true" : "false")
       << " no_eligible_phase="
       << mppiNoEligiblePhaseName(snapshot.no_eligible_recovery.phase)
       << " no_eligible_recovery_generation="
       << snapshot.no_eligible_recovery.no_eligible_recovery_generation
       << " no_eligible_guide_replan="
       << (snapshot.no_eligible_recovery.guide_replan_requested ? "true" : "false")
       << " liveness_state=" << mppiLivenessStateName(liveness.state)
       << " liveness_window_s=" << liveness.observation_age_s
       << " liveness_actual_displacement_m=" << liveness.actual_displacement_m
       << " liveness_reseed_generation=" << liveness.reseed_generation
       << " maximum_acceleration_mps2=" << result.maximum_acceleration_mps2
       << " maximum_jerk_mps3=" << result.maximum_jerk_mps3
       << " first_control_delta=" << result.first_control_delta
       << " horizon_stability_rms="
       << (stability.valid ? stability.position_rms_m : -1.0)
       << " shifted_horizon_first_control_delta="
       << (stability.valid ? stability.first_control_delta : -1.0)
       << " prediction_position_error_m="
       << (prediction.valid ? prediction.position_m : -1.0)
       << " esdf_build_ms=" << esdf.build_ms << " esdf_upload_ms=" << esdf.upload_ms
       << " dropped_diagnostics="
       << dropped_diagnostics_snapshots_.load(std::memory_order_relaxed);
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  if (now_ns - last_diagnostics_info_stamp_ns_ >= diagnostics_info_period_ns_) {
    RCLCPP_INFO(get_logger(), "%s", line.str().c_str());
    std_msgs::msg::String status;
    status.data = line.str();
    status_pub_->publish(status);
    last_diagnostics_info_stamp_ns_ = now_ns;
  }
  if (diagnostics_stream_) {
    diagnostics_stream_
        << "{\"tick\":" << snapshot.tick_sequence
        << ",\"pose_revision\":" << input.pose_revision
        << ",\"raw_revision\":" << input.obstacle_revision
        << ",\"esdf_revision\":" << result.esdf_revision
        << ",\"pose_age_ms\":" << snapshot.pose_age_ms
        << ",\"esdf_age_ms\":" << snapshot.esdf_age_ms
        << ",\"control_feedback_age_ms\":" << snapshot.control_feedback_age_ms
        << ",\"planning_mode\":\"" << (use_static_map_ ? "static" : "no_static") << '"'
        << ",\"planning_state\":\"" << productionMppiPlanningStateName(planning_state)
        << '"' << ",\"execution_mode\":\""
        << productionMppiExecutionModeName(snapshot.execution.mode) << '"'
        << ",\"execution_reason\":\""
        << productionMppiExecutionReasonName(snapshot.execution.reason) << '"'
        << ",\"execution_published\":"
        << (snapshot.execution.published ? "true" : "false") << ",\"target_source\":\""
        << target_source << '"' << ",\"horizon_s\":"
        << static_cast<double>(mppi_config_.steps) * mppi_config_.dynamics.dt_s
        << ",\"speed_cap_mps\":" << mppi_config_.dynamics.maximum_horizontal_speed_mps
        << ",\"acceleration_cap_mps2\":"
        << mppi_config_.dynamics.maximum_horizontal_acceleration_mps2
        << ",\"jerk_cap_mps3\":" << mppi_config_.dynamics.maximum_control_jerk_mps3
        << ",\"speed_tracking_weight\":" << mppi_config_.costs.speed_tracking_weight
        << ",\"guide_generation\":" << esdf.global_guide_generation
        << ",\"guide_reused\":" << (esdf.global_guide_reused ? "true" : "false")
        << ",\"guide_reaches_mission_goal\":"
        << (esdf.global_guide_reaches_mission_goal ? "true" : "false")
        << ",\"goal_capture_latched\":"
        << (snapshot.goal_capture.latched ? "true" : "false")
        << ",\"goal_distance_m\":" << snapshot.goal_capture.horizontal_distance_m
        << ",\"guide_release\":\""
        << globalGuideReleaseReasonName(esdf.global_guide_release_reason) << '"'
        << ",\"guide_heading_source\":\""
        << globalGuideHeadingSourceName(esdf.global_guide_heading_source) << '"'
        << ",\"guide_risk\":\"" << globalGuideRiskTierName(esdf.global_guide_risk)
        << '"' << ",\"guide_acceptance\":\""
        << globalGuideAcceptanceReasonName(esdf.global_guide_acceptance_reason) << '"'
        << ",\"guide_station_m\":" << snapshot.route_station_m
        << ",\"guide_remaining_m\":" << snapshot.route_remaining_m
        << ",\"route_constraint_phase\":\""
        << constrainedRoutePhaseName(route_constraint.phase) << '"'
        << ",\"route_constraint_span_available\":"
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
        << route_constraint.actual_vertical_speed_mps << ",\"guide_progress_action\":\""
        << globalGuideProgressActionName(snapshot.guide_progress.action) << '"'
        << ",\"guide_local_reseed_generation\":"
        << snapshot.guide_progress.local_reseed_generation
        << ",\"lattice_search_performed\":"
        << (esdf.lattice_search_performed ? "true" : "false")
        << ",\"lattice_executable\":" << (esdf.lattice_executable ? "true" : "false")
        << ",\"lattice_status\":\"" << latticePlanStatusName(esdf.lattice_status) << '"'
        << ",\"lattice_termination\":\""
        << latticeSearchTerminationName(esdf.lattice_termination) << '"'
        << ",\"lattice_continuation_attempt\":" << esdf.lattice_continuation_attempt
        << ",\"lattice_search_session_resumed\":"
        << (esdf.lattice_search_session_resumed ? "true" : "false")
        << ",\"lattice_risk_stage\":\"" << latticeRiskStageName(esdf.lattice_risk_stage)
        << '"' << ",\"lattice_stale_queue_pops\":" << esdf.lattice_stale_queue_pops
        << ",\"lattice_open_peak\":" << esdf.lattice_open_peak
        << ",\"lattice_records_peak\":" << esdf.lattice_records_peak
        << ",\"lattice_two_step_reachable_states\":"
        << esdf.lattice_two_step_reachable_states
        << ",\"lattice_reachable_depth_m\":" << esdf.lattice_reachable_depth_m
        << ",\"lattice_frontier_candidates_considered\":"
        << esdf.lattice_frontier_candidates_considered
        << ",\"lattice_successors_generated\":"
        << esdf.lattice_successor_diagnostics.generated
        << ",\"lattice_successors_accepted\":"
        << esdf.lattice_successor_diagnostics.accepted
        << ",\"lattice_successors_rejected_outside_roi\":"
        << esdf.lattice_successor_diagnostics.rejected_outside_roi
        << ",\"lattice_successors_rejected_outside_grid\":"
        << esdf.lattice_successor_diagnostics.rejected_outside_grid
        << ",\"lattice_successors_rejected_invalid_clearance\":"
        << esdf.lattice_successor_diagnostics.rejected_invalid_clearance
        << ",\"lattice_successors_rejected_raw_collision\":"
        << esdf.lattice_successor_diagnostics.rejected_raw_collision
        << ",\"lattice_successors_rejected_risk_stage\":"
        << esdf.lattice_successor_diagnostics.rejected_risk_stage
        << ",\"lattice_successors_rejected_blacklisted_failure\":"
        << esdf.lattice_successor_diagnostics.rejected_blacklisted_failure
        << ",\"lattice_successors_rejected_no_cost_improvement\":"
        << esdf.lattice_successor_diagnostics.rejected_no_cost_improvement
        << ",\"pose_predicted\":" << (snapshot.pose_predicted ? "true" : "false")
        << ",\"maximum_eligible_risk_tier\":\""
        << mppi::mppiRiskTierName(snapshot.maximum_eligible_risk_tier) << '"'
        << ",\"lattice_planning_goal_reached\":"
        << (esdf.lattice_planning_goal_reached ? "true" : "false")
        << ",\"lattice_achieved_progress_m\":" << esdf.lattice_achieved_progress_m
        << ",\"lattice_guide_length_m\":" << esdf.lattice_guide_length_m
        << ",\"lattice_remaining_goal_distance_m\":"
        << esdf.lattice_remaining_goal_distance_m
        << ",\"lattice_terminal_successors\":" << esdf.lattice_terminal_successor_count
        << ",\"target_lookahead_m\":" << speed_policy.target_lookahead_m
        << ",\"reference_speed_mps\":" << input.reference_speed_mps
        << ",\"curvature_speed_limit_mps\":"
        << finiteOrNegative(speed_policy.curvature_limit_mps)
        << ",\"observation_speed_limit_mps\":"
        << finiteOrNegative(speed_policy.observation_limit_mps)
        << ",\"goal_speed_limit_mps\":" << finiteOrNegative(speed_policy.goal_limit_mps)
        << ",\"gpu_ms\":" << result.timings.gpu_total_ms
        << ",\"total_ms\":" << result.timings.host_total_ms
        << ",\"snapshot_ms\":" << snapshot.snapshot_ms
        << ",\"stability_ms\":" << snapshot.stability_ms << ",\"rviz_ms\":" << rviz_ms
        << ",\"raw_collision\":" << (result.raw_collision ? "true" : "false")
        << ",\"known_solid_collision\":"
        << (result.known_solid_collision ? "true" : "false") << ",\"risk_tier\":\""
        << mppi::mppiRiskTierName(result.selected_tier) << '"'
        << ",\"eligible_available\":"
        << (result.eligible_risk_contract.available ? "true" : "false")
        << ",\"eligible_risk_tier\":\""
        << mppi::mppiRiskTierName(result.eligible_risk_contract.tier) << '"'
        << ",\"eligible_best_critical_exposure_m\":"
        << finiteOrNegative(result.eligible_risk_contract.best_critical_exposure_m)
        << ",\"eligible_best_planning_exposure_m\":"
        << finiteOrNegative(result.eligible_risk_contract.best_planning_exposure_m)
        << ",\"eligible_weight_sum\":"
        << finiteOrNegative(result.eligible_risk_contract.weight_sum)
        << ",\"eligible_critical_limit_m\":"
        << finiteOrNegative(result.post_update_classification.critical_exposure_limit_m)
        << ",\"eligible_planning_limit_m\":"
        << finiteOrNegative(result.post_update_classification.planning_exposure_limit_m)
        << ",\"post_update_classification\":\""
        << mppi::mppiPostUpdateClassificationName(
               result.post_update_classification.classification)
        << '"' << ",\"post_update_contract_preserved\":"
        << (result.post_update_classification.contract_preserved ? "true" : "false")
        << ",\"post_update_repair\":\""
        << mppi::mppiPostUpdateRepairName(result.post_update_repair) << '"'
        << ",\"post_update_backtrack_ratio\":" << result.post_update_backtrack_ratio
        << ",\"critical_exposure_m\":" << result.critical_exposure_m
        << ",\"planning_exposure_m\":" << result.planning_exposure_m
        << ",\"head_progress_m\":" << result.head_progress_m
        << ",\"terminal_progress_m\":" << result.terminal_progress_m
        << ",\"warm_start_shift_ms\":" << result.warm_start_shift_s * 1000.0
        << ",\"nominal_reseeded\":" << (result.nominal_reseeded ? "true" : "false")
        << ",\"no_eligible_phase\":\""
        << mppiNoEligiblePhaseName(snapshot.no_eligible_recovery.phase) << '"'
        << ",\"no_eligible_recovery_generation\":"
        << snapshot.no_eligible_recovery.no_eligible_recovery_generation
        << ",\"no_eligible_guide_replan\":"
        << (snapshot.no_eligible_recovery.guide_replan_requested ? "true" : "false")
        << ",\"liveness_state\":\"" << mppiLivenessStateName(liveness.state) << '"'
        << ",\"liveness_actual_displacement_m\":" << liveness.actual_displacement_m
        << ",\"liveness_reseed_generation\":" << liveness.reseed_generation
        << ",\"maximum_acceleration_mps2\":" << result.maximum_acceleration_mps2
        << ",\"maximum_jerk_mps3\":" << result.maximum_jerk_mps3
        << ",\"first_control_delta\":" << result.first_control_delta
        << ",\"stability_rms_m\":"
        << (stability.valid ? stability.position_rms_m : -1.0)
        << ",\"dropped_diagnostics\":"
        << dropped_diagnostics_snapshots_.load(std::memory_order_relaxed) << "}\n";
    diagnostics_stream_.flush();
  }
  if (now_ns - last_summary_stamp_ns_ >= 5000000000LL) {
    publishSummary();
    last_summary_stamp_ns_ = now_ns;
  }
}

void ProductionMppiNode::publishSummary() {
  std::vector<double> runtime_samples_ms;
  std::uint64_t completed_ticks{0U};
  std::uint64_t deadline_misses{0U};
  std::uint64_t raw_collision_horizons{0U};
  std::uint64_t solid_collision_horizons{0U};
  std::uint64_t post_update_contract_violations{0U};
  std::uint64_t no_progress_horizons{0U};
  std::uint64_t liveness_reseeds{0U};
  std::uint64_t no_guide_braking_hold_ticks{0U};
  std::uint64_t unavailable_world_braking_hold_ticks{0U};
  std::uint64_t mission_goal_position_hold_ticks{0U};
  {
    const std::scoped_lock lock{statistics_mutex_};
    runtime_samples_ms = runtime_samples_ms_;
    completed_ticks = completed_ticks_;
    deadline_misses = deadline_misses_;
    raw_collision_horizons = raw_collision_horizons_;
    solid_collision_horizons = solid_collision_horizons_;
    post_update_contract_violations = post_update_contract_violations_;
    no_progress_horizons = no_progress_horizons_;
    liveness_reseeds = liveness_reseeds_;
    no_guide_braking_hold_ticks = no_guide_braking_hold_ticks_;
    unavailable_world_braking_hold_ticks = unavailable_world_braking_hold_ticks_;
    mission_goal_position_hold_ticks = mission_goal_position_hold_ticks_;
  }
  if (runtime_samples_ms.empty()) {
    return;
  }
  std::uint64_t dropped_esdf_updates{0U};
  {
    const std::scoped_lock lock{raw_queue_mutex_};
    dropped_esdf_updates = dropped_raw_snapshots_;
  }
  const double maximum =
      *std::max_element(runtime_samples_ms.begin(), runtime_samples_ms.end());
  RCLCPP_INFO(
      get_logger(),
      "PRODUCTION_MPPI_SUMMARY ticks=%" PRIu64
      " runtime_p50=%.3f runtime_p95=%.3f runtime_p99=%.3f runtime_max=%.3f "
      "deadline_misses=%" PRIu64 " raw_collision_horizons=%" PRIu64
      " solid_collision_horizons=%" PRIu64 " post_update_contract_violations=%" PRIu64
      " no_progress_horizons=%" PRIu64 " liveness_reseeds=%" PRIu64
      " no_guide_braking_hold_ticks=%" PRIu64
      " unavailable_world_braking_hold_ticks=%" PRIu64
      " mission_goal_position_hold_ticks=%" PRIu64 " dropped_esdf_updates=%" PRIu64
      " dropped_diagnostics=%" PRIu64,
      completed_ticks, percentile(runtime_samples_ms, 0.50),
      percentile(runtime_samples_ms, 0.95), percentile(runtime_samples_ms, 0.99),
      maximum, deadline_misses, raw_collision_horizons, solid_collision_horizons,
      post_update_contract_violations, no_progress_horizons, liveness_reseeds,
      no_guide_braking_hold_ticks, unavailable_world_braking_hold_ticks,
      mission_goal_position_hold_ticks, dropped_esdf_updates,
      dropped_diagnostics_snapshots_.load(std::memory_order_relaxed));
}

} // namespace drone_city_nav
