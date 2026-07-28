#include "drone_city_nav/mppi_debug_markers.hpp"
#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <sstream>

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

void ProductionMppiNode::publishRviz(const mppi::MppiTickInput& input,
                                     const mppi::MppiTickResult& result,
                                     const ProductionMppiPreparedEsdf& esdf) {
  const auto stamp = now();
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id_;
  path.header.stamp = stamp;
  path.poses.reserve(result.horizon.size());
  for (const mppi::State& state : result.horizon) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = state.x;
    pose.pose.position.y = state.y;
    pose.pose.position.z = gazeboAlignedRvizZ(state.z);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  path_pub_->publish(path);

  const std::span<const mppi::State> previous_horizon =
      previous_result_.has_value()
          ? std::span<const mppi::State>{previous_result_->horizon}
          : std::span<const mppi::State>{};
  const std::span<const Point2> global_guide =
      esdf.global_guide ? std::span<const Point2>{*esdf.global_guide}
                        : std::span<const Point2>{};
  const visualization_msgs::msg::MarkerArray markers = buildMppiDebugMarkers(
      MppiDebugMarkerInput{path.header, result.horizon, previous_horizon, global_guide,
                           input.initial_state, input.target, mission_start_,
                           mission_goal_, input.passage, result.selected_tier});
  markers_pub_->publish(markers);
}

void ProductionMppiNode::publishDiagnostics(
    const mppi::MppiTickInput& input, const mppi::MppiTickResult& result,
    const ProductionMppiPreparedEsdf& esdf, const ProductionMppiStability& stability,
    const ProductionMppiPredictionError& prediction, const MppiLivenessResult& liveness,
    const MppiSpeedPolicyResult& speed_policy,
    const PassageCoordinatorResult& passage_coordinator,
    const ProductionMppiPlanningState planning_state,
    const std::string_view target_source, const double pose_age_ms,
    const double esdf_age_ms, const double control_feedback_age_ms,
    const double snapshot_ms, const double stability_ms, const double rviz_ms) {
  ++tick_sequence_;
  ++completed_ticks_;
  runtime_samples_ms_.push_back(result.timings.host_total_ms);
  if (result.timings.host_total_ms > deadline_ms_) {
    ++deadline_misses_;
  }
  raw_collision_horizons_ += result.raw_collision ? 1U : 0U;
  solid_collision_horizons_ += result.known_solid_collision ? 1U : 0U;
  post_update_contract_violations_ +=
      planning_state == ProductionMppiPlanningState::kPlanned &&
              !result.post_update_classification.contract_preserved
          ? 1U
          : 0U;
  no_progress_horizons_ += result.head_progress_m <= 0.0F ? 1U : 0U;
  passage_vertical_alignment_ticks_ += passage_coordinator.hold_xy ? 1U : 0U;
  passage_traversal_ticks_ +=
      passage_coordinator.phase == PassageCoordinatorPhase::kTraversal ||
              passage_coordinator.phase == PassageCoordinatorPhase::kPartialFromInside
          ? 1U
          : 0U;
  std::ostringstream line;
  line << std::fixed << std::setprecision(3)
       << "PRODUCTION_MPPI_TICK tick=" << tick_sequence_
       << " pose_revision=" << input.pose_revision
       << " raw_revision=" << input.obstacle_revision
       << " esdf_revision=" << result.esdf_revision
       << " memory_sequence=" << memory_sequence_ << " pose_age_ms=" << pose_age_ms
       << " esdf_age_ms=" << esdf_age_ms
       << " control_feedback_age_ms=" << control_feedback_age_ms << " planning_mode="
       << (passage_speed_policy_.use_static_map ? "static" : "no_static")
       << " planning_state=" << productionMppiPlanningStateName(planning_state)
       << " horizon_s="
       << static_cast<double>(mppi_config_.steps) * mppi_config_.dynamics.dt_s
       << " target_source=" << target_source << " target=(" << input.target.x << ','
       << input.target.y << ',' << input.target.z << ")"
       << " guide_generation=" << esdf.global_guide_generation
       << " guide_reused=" << (esdf.global_guide_reused ? "true" : "false")
       << " guide_mission_goal_hold="
       << (esdf.global_guide_mission_goal_hold ? "true" : "false") << " guide_release="
       << globalGuideReleaseReasonName(esdf.global_guide_release_reason)
       << " guide_heading_source="
       << globalGuideHeadingSourceName(esdf.global_guide_heading_source)
       << " guide_risk=" << globalGuideRiskTierName(esdf.global_guide_risk)
       << " guide_acceptance="
       << globalGuideAcceptanceReasonName(esdf.global_guide_acceptance_reason)
       << " guide_station_m=" << esdf.global_guide_projection.station_m
       << " guide_remaining_m=" << esdf.global_guide_projection.remaining_m
       << " lattice_search_performed="
       << (esdf.lattice_search_performed ? "true" : "false")
       << " lattice_status=" << latticePlanStatusName(esdf.lattice_status)
       << " lattice_termination="
       << latticeSearchTerminationName(esdf.lattice_termination)
       << " target_lookahead_m=" << speed_policy.target_lookahead_m
       << " reference_speed_mps=" << input.reference_speed_mps
       << " curvature_speed_limit_mps="
       << finiteOrNegative(speed_policy.curvature_limit_mps)
       << " observation_speed_limit_mps="
       << finiteOrNegative(speed_policy.observation_limit_mps)
       << " goal_speed_limit_mps=" << finiteOrNegative(speed_policy.goal_limit_mps)
       << " passage_speed_limit_mps="
       << finiteOrNegative(speed_policy.passage_limit_mps)
       << " passage_phase=" << passageCoordinatorPhaseName(passage_coordinator.phase)
       << " passage_opening="
       << (passage_coordinator.opening_id.empty() ? "none"
                                                  : passage_coordinator.opening_id)
       << " passage_xy_hold=" << (passage_coordinator.hold_xy ? "true" : "false")
       << " passage_vertical_ready="
       << (passage_coordinator.vertical_ready ? "true" : "false")
       << " passage_target_z_m=" << passage_coordinator.preferred_z_m
       << " passage_vertical_error_m=" << passage_coordinator.vertical_error_m
       << " passage_distance_to_entry_m=" << passage_coordinator.distance_to_entry_m
       << " passage_required_alignment_time_s="
       << passage_coordinator.required_alignment_time_s
       << " passage_required_stopping_distance_m="
       << passage_coordinator.required_stopping_distance_m
       << " passage_required_alignment_distance_m="
       << passage_coordinator.required_alignment_distance_m
       << " gpu_ms=" << result.timings.gpu_total_ms
       << " total_ms=" << result.timings.host_total_ms << " snapshot_ms=" << snapshot_ms
       << " stability_ms=" << stability_ms << " rviz_ms=" << rviz_ms
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
       << " minimum_esdf_m=" << result.minimum_esdf_distance_m
       << " head_progress_m=" << result.head_progress_m
       << " terminal_progress_m=" << result.terminal_progress_m
       << " warm_start_shift_ms=" << result.warm_start_shift_s * 1000.0
       << " previous_control_source="
       << (input.previous_applied_control.has_value() ? "offboard_feedback"
                                                      : "engine_fallback")
       << " nominal_reseeded=" << (result.nominal_reseeded ? "true" : "false")
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
       << " esdf_build_ms=" << esdf.build_ms << " esdf_upload_ms=" << esdf.upload_ms;
  RCLCPP_INFO(get_logger(), "%s", line.str().c_str());
  std_msgs::msg::String status;
  status.data = line.str();
  status_pub_->publish(status);
  if (diagnostics_stream_) {
    diagnostics_stream_
        << "{\"tick\":" << tick_sequence_
        << ",\"pose_revision\":" << input.pose_revision
        << ",\"raw_revision\":" << input.obstacle_revision
        << ",\"esdf_revision\":" << result.esdf_revision << ",\"planning_mode\":\""
        << (passage_speed_policy_.use_static_map ? "static" : "no_static") << '"'
        << ",\"planning_state\":\"" << productionMppiPlanningStateName(planning_state)
        << '"' << ",\"target_source\":\"" << target_source << '"' << ",\"horizon_s\":"
        << static_cast<double>(mppi_config_.steps) * mppi_config_.dynamics.dt_s
        << ",\"speed_cap_mps\":" << mppi_config_.dynamics.maximum_horizontal_speed_mps
        << ",\"acceleration_cap_mps2\":"
        << mppi_config_.dynamics.maximum_horizontal_acceleration_mps2
        << ",\"jerk_cap_mps3\":" << mppi_config_.dynamics.maximum_control_jerk_mps3
        << ",\"speed_tracking_weight\":" << mppi_config_.costs.speed_tracking_weight
        << ",\"guide_generation\":" << esdf.global_guide_generation
        << ",\"guide_reused\":" << (esdf.global_guide_reused ? "true" : "false")
        << ",\"guide_mission_goal_hold\":"
        << (esdf.global_guide_mission_goal_hold ? "true" : "false")
        << ",\"guide_release\":\""
        << globalGuideReleaseReasonName(esdf.global_guide_release_reason) << '"'
        << ",\"guide_heading_source\":\""
        << globalGuideHeadingSourceName(esdf.global_guide_heading_source) << '"'
        << ",\"guide_risk\":\"" << globalGuideRiskTierName(esdf.global_guide_risk)
        << '"' << ",\"guide_acceptance\":\""
        << globalGuideAcceptanceReasonName(esdf.global_guide_acceptance_reason) << '"'
        << ",\"guide_station_m\":" << esdf.global_guide_projection.station_m
        << ",\"guide_remaining_m\":" << esdf.global_guide_projection.remaining_m
        << ",\"lattice_search_performed\":"
        << (esdf.lattice_search_performed ? "true" : "false")
        << ",\"lattice_legacy_valid\":"
        << (esdf.lattice_legacy_valid ? "true" : "false") << ",\"lattice_status\":\""
        << latticePlanStatusName(esdf.lattice_status) << '"'
        << ",\"lattice_termination\":\""
        << latticeSearchTerminationName(esdf.lattice_termination) << '"'
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
        << ",\"passage_speed_limit_mps\":"
        << finiteOrNegative(speed_policy.passage_limit_mps) << ",\"passage_phase\":\""
        << passageCoordinatorPhaseName(passage_coordinator.phase) << '"'
        << ",\"passage_opening\":\"" << passage_coordinator.opening_id << '"'
        << ",\"passage_xy_hold\":" << (passage_coordinator.hold_xy ? "true" : "false")
        << ",\"passage_vertical_ready\":"
        << (passage_coordinator.vertical_ready ? "true" : "false")
        << ",\"passage_target_z_m\":" << passage_coordinator.preferred_z_m
        << ",\"passage_vertical_error_m\":" << passage_coordinator.vertical_error_m
        << ",\"passage_distance_to_entry_m\":"
        << passage_coordinator.distance_to_entry_m
        << ",\"passage_required_alignment_time_s\":"
        << passage_coordinator.required_alignment_time_s
        << ",\"passage_required_stopping_distance_m\":"
        << passage_coordinator.required_stopping_distance_m
        << ",\"passage_required_alignment_distance_m\":"
        << passage_coordinator.required_alignment_distance_m
        << ",\"gpu_ms\":" << result.timings.gpu_total_ms
        << ",\"total_ms\":" << result.timings.host_total_ms
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
        << ",\"critical_exposure_m\":" << result.critical_exposure_m
        << ",\"planning_exposure_m\":" << result.planning_exposure_m
        << ",\"head_progress_m\":" << result.head_progress_m
        << ",\"terminal_progress_m\":" << result.terminal_progress_m
        << ",\"warm_start_shift_ms\":" << result.warm_start_shift_s * 1000.0
        << ",\"nominal_reseeded\":" << (result.nominal_reseeded ? "true" : "false")
        << ",\"liveness_state\":\"" << mppiLivenessStateName(liveness.state) << '"'
        << ",\"liveness_actual_displacement_m\":" << liveness.actual_displacement_m
        << ",\"liveness_reseed_generation\":" << liveness.reseed_generation
        << ",\"maximum_acceleration_mps2\":" << result.maximum_acceleration_mps2
        << ",\"maximum_jerk_mps3\":" << result.maximum_jerk_mps3
        << ",\"first_control_delta\":" << result.first_control_delta
        << ",\"stability_rms_m\":"
        << (stability.valid ? stability.position_rms_m : -1.0) << "}\n";
    diagnostics_stream_.flush();
  }
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  if (now_ns - last_summary_stamp_ns_ >= 5000000000LL) {
    publishSummary();
    last_summary_stamp_ns_ = now_ns;
  }
}

void ProductionMppiNode::publishSummary() {
  if (runtime_samples_ms_.empty()) {
    return;
  }
  const double maximum =
      *std::max_element(runtime_samples_ms_.begin(), runtime_samples_ms_.end());
  RCLCPP_INFO(
      get_logger(),
      "PRODUCTION_MPPI_SUMMARY ticks=%" PRIu64
      " runtime_p50=%.3f runtime_p95=%.3f runtime_p99=%.3f runtime_max=%.3f "
      "deadline_misses=%" PRIu64 " raw_collision_horizons=%" PRIu64
      " solid_collision_horizons=%" PRIu64 " post_update_contract_violations=%" PRIu64
      " no_progress_horizons=%" PRIu64 " liveness_reseeds=%" PRIu64
      " no_guide_braking_hold_ticks=%" PRIu64
      " passage_vertical_alignment_ticks=%" PRIu64 " passage_traversal_ticks=%" PRIu64
      " dropped_esdf_updates=%" PRIu64,
      completed_ticks_, percentile(runtime_samples_ms_, 0.50),
      percentile(runtime_samples_ms_, 0.95), percentile(runtime_samples_ms_, 0.99),
      maximum, deadline_misses_, raw_collision_horizons_, solid_collision_horizons_,
      post_update_contract_violations_, no_progress_horizons_, liveness_reseeds_,
      no_guide_braking_hold_ticks_, passage_vertical_alignment_ticks_,
      passage_traversal_ticks_, dropped_raw_snapshots_);
}

} // namespace drone_city_nav
