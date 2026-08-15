#include "drone_city_nav/mppi_debug_markers.hpp"
#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <memory>
#include <optional>
#include <span>
#include <stop_token>
#include <string>
#include <utility>

#include "production_mppi_node.hpp"
#include "tracking_objective_diagnostics.hpp"

namespace drone_city_nav {

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
  if (diagnostics_error_stream_) {
    diagnostics_error_stream_.flush();
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
    const ProductionMppiExecutionPublication& execution,
    const bool liveness_reseed_requested) {
  const std::scoped_lock lock{statistics_mutex_};
  ++completed_ticks_;
  runtime_samples_ms_.push_back(result.timings.host_total_ms);
  deadline_misses_ += result.timings.host_total_ms > deadline_ms_ ? 1U : 0U;
  altitude_envelope_violation_horizons_ += result.altitude_envelope_violation ? 1U : 0U;
  raw_collision_horizons_ += result.raw_collision ? 1U : 0U;
  solid_collision_horizons_ += result.known_solid_collision ? 1U : 0U;
  post_update_contract_violations_ +=
      planning_state == ProductionMppiPlanningState::kPlanned &&
              !result.post_update_classification.executable
          ? 1U
          : 0U;
  no_progress_horizons_ += result.head_progress_m <= 0.0F ? 1U : 0U;
  liveness_reseeds_ += liveness_reseed_requested ? 1U : 0U;
  mission_goal_position_hold_ticks_ +=
      planning_state == ProductionMppiPlanningState::kMissionGoalPositionHold ? 1U : 0U;
  no_executable_route_hold_ticks_ +=
      planning_state == ProductionMppiPlanningState::kNoExecutableRouteHold ? 1U : 0U;
  no_executable_horizon_hold_ticks_ +=
      execution.reason == ProductionMppiExecutionReason::kNoExecutableHorizon ? 1U : 0U;
  terminal_rest_horizon_ticks_ += execution.terminal_rest_state ? 1U : 0U;
  finite_path_validation_backoff_ticks_ +=
      execution.finite_path_validation_backoff ? 1U : 0U;
  latest_lidar_path_validation_backoff_ticks_ +=
      execution.latest_lidar_path_validation_backoff ? 1U : 0U;
  retained_previous_finite_path_ticks_ +=
      execution.retained_previous_finite_path ? 1U : 0U;
  arrival_control_total_ += execution.arrival_control_count;
  arrival_shaping_attempt_total_ += execution.arrival_shaping_attempts;
  if (result.active_rollouts > 0U) {
    active_rollout_total_ += result.active_rollouts;
    full_rollout_ticks_ += result.active_rollouts == mppi_config_.rollouts ? 1U : 0U;
    reduced_rollout_ticks_ += result.active_rollouts < mppi_config_.rollouts ? 1U : 0U;
  }
}

void ProductionMppiNode::publishRviz(
    const ProductionMppiDiagnosticsSnapshot& snapshot) {
  if (!snapshot.rviz.has_value()) {
    return;
  }
  const std::shared_ptr<const ProductionNavigationObjective>& objective =
      snapshot.objective;
  const Point3 mission_goal = objective ? objective->goal : mission_goal_;
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
  const std::span<const ConstrainedFreeSpaceEdge> channel_edges =
      rviz.channel_edges
          ? std::span<const ConstrainedFreeSpaceEdge>{*rviz.channel_edges}
          : std::span<const ConstrainedFreeSpaceEdge>{};
  const std::span<const std::string> selected_channel_ids =
      rviz.selected_channel_ids
          ? std::span<const std::string>{*rviz.selected_channel_ids}
          : std::span<const std::string>{};
  MppiDebugMarkerInput marker_input{
      .header = path.header,
      .horizon = rviz.candidate_horizon,
      .previous_horizon = previous_horizon,
      .execution_horizon = execution_horizon,
      .global_route = global_route,
      .channel_edges = channel_edges,
      .selected_channel_ids = selected_channel_ids,
      .initial_state = snapshot.input.initial_state,
      .target = snapshot.input.target,
      .mission_start = mission_start_,
      .mission_goal = mission_goal,
      .selected_tier = snapshot.result.selected_tier,
  };
  detail::populateTrackingObjectiveMarkers(objective.get(), marker_input);
  const visualization_msgs::msg::MarkerArray markers =
      buildMppiDebugMarkers(marker_input);
  markers_pub_->publish(markers);
}

} // namespace drone_city_nav
