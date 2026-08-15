#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

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

} // namespace

void ProductionMppiNode::publishSummary() {
  std::vector<double> runtime_samples_ms;
  std::uint64_t completed_ticks{0U};
  std::uint64_t deadline_misses{0U};
  std::uint64_t altitude_envelope_violation_horizons{0U};
  std::uint64_t raw_collision_horizons{0U};
  std::uint64_t solid_collision_horizons{0U};
  std::uint64_t post_update_contract_violations{0U};
  std::uint64_t no_progress_horizons{0U};
  std::uint64_t liveness_reseeds{0U};
  std::uint64_t mission_goal_position_hold_ticks{0U};
  std::uint64_t no_executable_route_hold_ticks{0U};
  std::uint64_t no_executable_horizon_hold_ticks{0U};
  std::uint64_t terminal_rest_horizon_ticks{0U};
  std::uint64_t finite_path_validation_backoff_ticks{0U};
  std::uint64_t latest_lidar_path_validation_backoff_ticks{0U};
  std::uint64_t retained_previous_finite_path_ticks{0U};
  std::uint64_t arrival_control_total{0U};
  std::uint64_t arrival_shaping_attempt_total{0U};
  std::uint64_t full_rollout_ticks{0U};
  std::uint64_t reduced_rollout_ticks{0U};
  std::uint64_t active_rollout_total{0U};
  {
    const std::scoped_lock lock{statistics_mutex_};
    runtime_samples_ms = runtime_samples_ms_;
    completed_ticks = completed_ticks_;
    deadline_misses = deadline_misses_;
    altitude_envelope_violation_horizons = altitude_envelope_violation_horizons_;
    raw_collision_horizons = raw_collision_horizons_;
    solid_collision_horizons = solid_collision_horizons_;
    post_update_contract_violations = post_update_contract_violations_;
    no_progress_horizons = no_progress_horizons_;
    liveness_reseeds = liveness_reseeds_;
    mission_goal_position_hold_ticks = mission_goal_position_hold_ticks_;
    no_executable_route_hold_ticks = no_executable_route_hold_ticks_;
    no_executable_horizon_hold_ticks = no_executable_horizon_hold_ticks_;
    terminal_rest_horizon_ticks = terminal_rest_horizon_ticks_;
    finite_path_validation_backoff_ticks = finite_path_validation_backoff_ticks_;
    latest_lidar_path_validation_backoff_ticks =
        latest_lidar_path_validation_backoff_ticks_;
    retained_previous_finite_path_ticks = retained_previous_finite_path_ticks_;
    arrival_control_total = arrival_control_total_;
    arrival_shaping_attempt_total = arrival_shaping_attempt_total_;
    full_rollout_ticks = full_rollout_ticks_;
    reduced_rollout_ticks = reduced_rollout_ticks_;
    active_rollout_total = active_rollout_total_;
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
  RCLCPP_INFO(
      get_logger(),
      "PRODUCTION_MPPI_SUMMARY ticks=%" PRIu64
      " runtime_p50=%.3f runtime_p95=%.3f runtime_p99=%.3f runtime_max=%.3f "
      "deadline_misses=%" PRIu64 " altitude_envelope_violation_horizons=%" PRIu64
      " raw_collision_horizons=%" PRIu64 " solid_collision_horizons=%" PRIu64
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
      " dropped_diagnostics=%" PRIu64 " full_rollout_ticks=%" PRIu64
      " reduced_rollout_ticks=%" PRIu64 " average_active_rollouts=%.1f",
      completed_ticks, percentile(runtime_samples_ms, 0.50),
      percentile(runtime_samples_ms, 0.95), percentile(runtime_samples_ms, 0.99),
      maximum, deadline_misses, altitude_envelope_violation_horizons,
      raw_collision_horizons, solid_collision_horizons, post_update_contract_violations,
      no_progress_horizons, liveness_reseeds, mission_goal_position_hold_ticks,
      no_executable_route_hold_ticks, no_executable_horizon_hold_ticks,
      terminal_rest_horizon_ticks, finite_path_validation_backoff_ticks,
      latest_lidar_path_validation_backoff_ticks, retained_previous_finite_path_ticks,
      average_arrival_controls, average_arrival_shaping_attempts, dropped_esdf_updates,
      no_static_raw_updates_.load(std::memory_order_relaxed),
      no_static_esdf_builds_.load(std::memory_order_relaxed),
      no_static_esdf_throttled_updates_.load(std::memory_order_relaxed),
      dropped_diagnostics_snapshots_.load(std::memory_order_relaxed),
      full_rollout_ticks, reduced_rollout_ticks, average_active_rollouts);
}

} // namespace drone_city_nav
