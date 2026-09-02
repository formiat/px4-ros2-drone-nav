#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "navigation_diagnostics_sink.hpp"
#include "production_mppi_node.hpp"
#include "world_pipeline_3d.hpp"

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

} // namespace

void ProductionMppiNode::publishSummary() {
  const NavigationDiagnosticsStatistics diagnostics = diagnostics_sink_->statistics();
  const std::vector<double>& runtime_samples_ms = diagnostics.runtime_samples_ms;
  const std::uint64_t completed_ticks = diagnostics.completed_ticks;
  const std::uint64_t deadline_misses = diagnostics.deadline_misses;
  const std::uint64_t altitude_envelope_violation_horizons =
      diagnostics.altitude_envelope_violation_horizons;
  const std::uint64_t post_update_contract_violations =
      diagnostics.post_update_contract_violations;
  const std::uint64_t no_progress_horizons = diagnostics.no_progress_horizons;
  const std::uint64_t liveness_reseeds = diagnostics.liveness_reseeds;
  const std::uint64_t mission_goal_position_hold_ticks =
      diagnostics.mission_goal_position_hold_ticks;
  const std::uint64_t no_executable_route_hold_ticks =
      diagnostics.no_executable_route_hold_ticks;
  const std::uint64_t no_executable_horizon_hold_ticks =
      diagnostics.no_executable_horizon_hold_ticks;
  const std::uint64_t terminal_rest_horizon_ticks =
      diagnostics.terminal_rest_horizon_ticks;
  const std::uint64_t finite_path_validation_backoff_ticks =
      diagnostics.finite_path_validation_backoff_ticks;
  const std::uint64_t latest_lidar_path_validation_backoff_ticks =
      diagnostics.latest_lidar_path_validation_backoff_ticks;
  const std::uint64_t retained_previous_finite_path_ticks =
      diagnostics.retained_previous_finite_path_ticks;
  const std::uint64_t arrival_control_total = diagnostics.arrival_control_total;
  const std::uint64_t arrival_shaping_attempt_total =
      diagnostics.arrival_shaping_attempt_total;
  const std::uint64_t full_rollout_ticks = diagnostics.full_rollout_ticks;
  const std::uint64_t reduced_rollout_ticks = diagnostics.reduced_rollout_ticks;
  const std::uint64_t active_rollout_total = diagnostics.active_rollout_total;
  const RollingRouteTelemetrySnapshot3D& rolling_route = diagnostics.rolling_route;
  if (runtime_samples_ms.empty()) {
    return;
  }
  const WorldPipelineStatistics3D world_statistics = world_pipeline_->statistics();
  const RoutePlanningCoordinatorStatistics3D route_planning_statistics =
      route_lifecycle_coordinator_->statistics();
  const double maximum =
      *std::max_element(runtime_samples_ms.begin(), runtime_samples_ms.end());
  const std::uint64_t rollout_ticks = full_rollout_ticks + reduced_rollout_ticks;
  const double average_active_rollouts =
      rollout_ticks > 0U ? static_cast<double>(active_rollout_total) /
                               static_cast<double>(rollout_ticks)
                         : 0.0;
  const double average_arrival_controls =
      terminal_rest_horizon_ticks > 0U
          ? static_cast<double>(arrival_control_total) /
                static_cast<double>(terminal_rest_horizon_ticks)
          : 0.0;
  const double average_arrival_shaping_attempts =
      terminal_rest_horizon_ticks > 0U
          ? static_cast<double>(arrival_shaping_attempt_total) /
                static_cast<double>(terminal_rest_horizon_ticks)
          : 0.0;
  const BoundedWorkerPoolSnapshot workers = planning_worker_pool_
                                                ? planning_worker_pool_->snapshot()
                                                : BoundedWorkerPoolSnapshot{};
  const StaticRoutePlanningLatencyStats planning_latency =
      route_lifecycle_coordinator_->planningLatencyStatistics();
  RCLCPP_INFO(
      get_logger(),
      "PRODUCTION_MPPI_SUMMARY ticks=%" PRIu64
      " runtime_p50=%.3f runtime_p95=%.3f runtime_p99=%.3f runtime_max=%.3f "
      "deadline_misses=%" PRIu64 " altitude_envelope_violation_horizons=%" PRIu64
      " post_update_contract_violations=%" PRIu64 " no_progress_horizons=%" PRIu64
      " liveness_reseeds=%" PRIu64 " mission_goal_position_hold_ticks=%" PRIu64
      " no_executable_route_hold_ticks=%" PRIu64
      " no_executable_horizon_hold_ticks=%" PRIu64
      " terminal_rest_horizon_ticks=%" PRIu64
      " finite_path_validation_backoff_ticks=%" PRIu64
      " latest_lidar_path_validation_backoff_ticks=%" PRIu64
      " retained_previous_finite_path_ticks=%" PRIu64
      " average_arrival_controls=%.1f average_arrival_shaping_attempts=%.2f"
      " dropped_esdf_updates=%" PRIu64 " no_static_raw_updates=%" PRIu64
      " no_static_esdf_builds=%" PRIu64 " no_static_esdf_throttled=%" PRIu64
      " static_esdf_builds=%" PRIu64 " static_esdf_cpu_reuses=%" PRIu64
      " static_esdf_gpu_reuses=%" PRIu64 " static_esdf_refreshes=%" PRIu64
      " dropped_diagnostics=%" PRIu64 " full_rollout_ticks=%" PRIu64
      " reduced_rollout_ticks=%" PRIu64 " average_active_rollouts=%.1f"
      " rolling_route_observations=%" PRIu64 " continuation_boundary_ticks=%" PRIu64
      " continuation_minimum_speed_mps=%.3f"
      " continuation_zero_speed_ticks=%" PRIu64
      " continuation_endpoint_limited_ticks=%" PRIu64
      " continuity_transition_ticks=%" PRIu64
      " continuity_transition_minimum_speed_mps=%.3f"
      " continuity_transition_zero_speed_ticks=%" PRIu64
      " maximum_continuity_transition_speed_drop_mps=%.3f"
      " ownership_gap_ticks=%" PRIu64 " ownership_gap_episodes=%" PRIu64
      " maximum_ownership_gap_ticks=%" PRIu64 " moving_raw_invalidation_ticks=%" PRIu64
      " moving_raw_invalidation_without_braking_tail_ticks=%" PRIu64
      " finite_braking_tail_activations=%" PRIu64 " nominal_reseed_ticks=%" PRIu64
      " continuity_preserving_reseed_ticks=%" PRIu64
      " route_generation_changes=%" PRIu64
      " continuity_preserving_generation_changes=%" PRIu64
      " geometry_revision_changes=%" PRIu64
      " post_bootstrap_route_observations=%" PRIu64
      " post_bootstrap_route_available_ticks=%" PRIu64
      " post_bootstrap_route_availability_ratio=%.6f"
      " post_bootstrap_no_executable_route_hold_ticks=%" PRIu64
      " planner_latency_samples=%zu planner_p95_ms=%.3f planner_p99_ms=%.3f"
      " planner_build_and_planning_p99_ms=%.3f"
      " worker_route_pending=%zu worker_world_pending=%zu "
      "worker_background_pending=%zu worker_route_capacity_waits=%" PRIu64
      " worker_world_capacity_waits=%" PRIu64
      " worker_background_capacity_waits=%" PRIu64
      " world_generation_superseded_ticks=%" PRIu64
      " world_generation_rejected_publications=%" PRIu64
      " world_pipeline_processing_failures=%" PRIu64
      " world_pipeline_failure_handler_failures=%" PRIu64
      " route_planning_queued=%" PRIu64 " route_planning_processed=%" PRIu64
      " route_planning_displaced=%" PRIu64 " route_planning_busy_rejections=%" PRIu64
      " route_planning_invalid_rejections=%" PRIu64
      " route_planning_stopped_rejections=%" PRIu64
      " route_planning_processing_failures=%" PRIu64
      " route_planning_handler_failures=%" PRIu64
      " tick_snapshot_p50_ms=%.3f tick_snapshot_p95_ms=%.3f tick_snapshot_max_ms=%.3f"
      " tick_controller_p50_ms=%.3f tick_controller_p95_ms=%.3f"
      " tick_publication_p50_ms=%.3f tick_publication_p95_ms=%.3f"
      " tick_publication_max_ms=%.3f tick_total_p50_ms=%.3f tick_total_p95_ms=%.3f"
      " tick_total_max_ms=%.3f horizon_publications=%" PRIu64
      " horizon_commit_rejections=%" PRIu64 " horizon_supersession_deferrals=%" PRIu64
      " horizon_supersession_grace_replacements=%" PRIu64
      " resident_owner_continuation_ticks=%" PRIu64,
      completed_ticks, percentile(runtime_samples_ms, 0.50),
      percentile(runtime_samples_ms, 0.95), percentile(runtime_samples_ms, 0.99),
      maximum, deadline_misses, altitude_envelope_violation_horizons,
      post_update_contract_violations, no_progress_horizons, liveness_reseeds,
      mission_goal_position_hold_ticks, no_executable_route_hold_ticks,
      no_executable_horizon_hold_ticks, terminal_rest_horizon_ticks,
      finite_path_validation_backoff_ticks, latest_lidar_path_validation_backoff_ticks,
      retained_previous_finite_path_ticks, average_arrival_controls,
      average_arrival_shaping_attempts, world_statistics.dropped_raw_worlds,
      world_statistics.raw_updates, world_statistics.observedBuilds(),
      world_statistics.throttled_observed_builds, world_statistics.static_builds,
      world_statistics.static_cpu_reuses, world_statistics.static_gpu_reuses,
      world_statistics.static_refreshes, diagnostics_sink_->droppedSnapshots(),
      full_rollout_ticks, reduced_rollout_ticks, average_active_rollouts,
      rolling_route.observations, rolling_route.continuation_boundary_ticks,
      std::isfinite(rolling_route.minimum_continuation_boundary_speed_mps)
          ? rolling_route.minimum_continuation_boundary_speed_mps
          : -1.0,
      rolling_route.continuation_zero_speed_ticks,
      rolling_route.continuation_endpoint_limited_ticks,
      rolling_route.continuity_transition_ticks,
      std::isfinite(rolling_route.minimum_continuity_transition_speed_mps)
          ? rolling_route.minimum_continuity_transition_speed_mps
          : -1.0,
      rolling_route.continuity_transition_zero_speed_ticks,
      rolling_route.maximum_continuity_transition_speed_drop_mps,
      rolling_route.ownership_gap_ticks, rolling_route.ownership_gap_episodes,
      rolling_route.maximum_consecutive_ownership_gap_ticks,
      rolling_route.moving_raw_invalidation_ticks,
      rolling_route.moving_raw_invalidation_without_braking_tail_ticks,
      rolling_route.finite_braking_tail_activations, rolling_route.nominal_reseed_ticks,
      rolling_route.continuity_preserving_reseed_ticks,
      rolling_route.route_generation_changes,
      rolling_route.continuity_preserving_generation_changes,
      rolling_route.geometry_revision_changes,
      rolling_route.post_bootstrap_observations,
      rolling_route.post_bootstrap_route_available_ticks,
      rolling_route.postBootstrapRouteAvailabilityRatio(),
      rolling_route.post_bootstrap_no_executable_route_hold_ticks,
      planning_latency.sample_count, planning_latency.planning_p95_ms,
      planning_latency.planning_p99_ms, planning_latency.build_and_planning_p99_ms,
      workers.lanes[0U].pending, workers.lanes[1U].pending, workers.lanes[2U].pending,
      workers.lanes[0U].capacity_waits, workers.lanes[1U].capacity_waits,
      workers.lanes[2U].capacity_waits,
      world_statistics.superseded_planning_generations,
      world_statistics.rejected_world_publications,
      world_statistics.processing_failures, world_statistics.failure_handler_failures,
      route_planning_statistics.queued, route_planning_statistics.processed,
      route_planning_statistics.displaced, route_planning_statistics.busy_rejections,
      route_planning_statistics.invalid_rejections,
      route_planning_statistics.stopped_rejections,
      route_planning_statistics.processing_failures,
      route_planning_statistics.handler_failures,
      percentile(diagnostics.snapshot_phase_samples_ms, 0.50),
      percentile(diagnostics.snapshot_phase_samples_ms, 0.95),
      percentile(diagnostics.snapshot_phase_samples_ms, 1.0),
      percentile(diagnostics.controller_phase_samples_ms, 0.50),
      percentile(diagnostics.controller_phase_samples_ms, 0.95),
      percentile(diagnostics.publication_phase_samples_ms, 0.50),
      percentile(diagnostics.publication_phase_samples_ms, 0.95),
      percentile(diagnostics.publication_phase_samples_ms, 1.0),
      percentile(diagnostics.tick_total_samples_ms, 0.50),
      percentile(diagnostics.tick_total_samples_ms, 0.95),
      percentile(diagnostics.tick_total_samples_ms, 1.0),
      horizon_publications_.load(std::memory_order_relaxed),
      horizon_commit_rejections_.load(std::memory_order_relaxed),
      horizon_supersession_deferrals_.load(std::memory_order_relaxed),
      horizon_supersession_grace_replacements_.load(std::memory_order_relaxed),
      diagnostics.resident_owner_continuation_ticks);
}

} // namespace drone_city_nav
