#include "drone_city_nav/offboard_trajectory_state.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace drone_city_nav {

[[nodiscard]] std::uint64_t
messageStampNanoseconds(const builtin_interfaces::msg::Time& stamp) {
  constexpr std::uint64_t kNanosecondsPerSecond = 1'000'000'000U;
  return static_cast<std::uint64_t>(stamp.sec) * kNanosecondsPerSecond +
         static_cast<std::uint64_t>(stamp.nanosec);
}

[[nodiscard]] std::vector<Point2>
pathPointsFromMessage(const nav_msgs::msg::Path& path) {
  std::vector<Point2> points;
  points.reserve(path.poses.size());
  for (const auto& pose : path.poses) {
    points.push_back(Point2{pose.pose.position.x, pose.pose.position.y});
  }
  return points;
}

[[nodiscard]] bool trajectoryDiagnosticsMatchesPath(
    const TrajectoryPlannerDiagnosticsEnvelope& diagnostics,
    const std::uint64_t expected_path_stamp_ns, const bool planner_path_id_seen,
    const std::uint64_t expected_planner_path_id) {
  if (diagnostics.path_stamp_ns != expected_path_stamp_ns) {
    return false;
  }
  return !planner_path_id_seen ||
         diagnostics.planner_path_id == expected_planner_path_id;
}

void mergePlannerDiagnosticsIntoTrajectoryStats(
    TrajectoryPlannerStats& output_stats,
    const TrajectoryPlannerDiagnosticsEnvelope& diagnostics) {
  output_stats.corridor = diagnostics.stats.corridor;
  output_stats.trajectory_optimizer = diagnostics.stats.trajectory_optimizer;
  output_stats.turn_smoothing = diagnostics.stats.turn_smoothing;
  output_stats.total_duration_ms = diagnostics.stats.total_duration_ms;
  output_stats.corridor_duration_ms = diagnostics.stats.corridor_duration_ms;
  output_stats.trajectory_optimizer_duration_ms =
      diagnostics.stats.trajectory_optimizer_duration_ms;
  output_stats.turn_smoothing_duration_ms =
      diagnostics.stats.turn_smoothing_duration_ms;
  output_stats.speed_profile_duration_ms = diagnostics.stats.speed_profile_duration_ms;
  output_stats.isolated_curvature_spike_candidates =
      diagnostics.stats.isolated_curvature_spike_candidates;
  output_stats.isolated_curvature_spikes_smoothed_geometry =
      diagnostics.stats.isolated_curvature_spikes_smoothed_geometry;
  output_stats.isolated_curvature_spikes_smoothed_speed_profile =
      diagnostics.stats.isolated_curvature_spikes_smoothed_speed_profile;
  output_stats.isolated_curvature_spike_max_before_1pm =
      diagnostics.stats.isolated_curvature_spike_max_before_1pm;
  output_stats.isolated_curvature_spike_max_after_1pm =
      diagnostics.stats.isolated_curvature_spike_max_after_1pm;
  output_stats.top_speed_constraints = diagnostics.stats.top_speed_constraints;
}

[[nodiscard]] TrajectoryPlannerStats buildReceivedTrajectoryPlannerStats(
    const std::span<const Point2> route_points,
    const std::span<const TrajectoryPointSample> samples,
    const std::span<const TrajectorySegment> trajectory,
    const TrajectoryMetrics& metrics, const TrajectorySpeedProfile& speed_profile,
    const bool trajectory_valid) {
  TrajectoryPlannerStats stats{};
  stats.status = trajectory_valid ? TrajectoryPlannerStatus::kOk
                                  : TrajectoryPlannerStatus::kInvalidTrajectory;
  stats.input_points = route_points.size();
  stats.samples = samples.size();
  stats.compact_segments = trajectory.size();
  stats.line_segments = metrics.line_segments;
  stats.arc_segments = metrics.arc_segments;
  stats.length_m = metrics.length_m;

  double curvature_abs_sum = 0.0;
  for (std::size_t i = 0U; i < samples.size(); ++i) {
    const double curvature = samples[i].curvature_1pm;
    if (i == 0U) {
      stats.curvature_min_1pm = curvature;
      stats.curvature_max_1pm = curvature;
    } else {
      stats.curvature_min_1pm = std::min(stats.curvature_min_1pm, curvature);
      stats.curvature_max_1pm = std::max(stats.curvature_max_1pm, curvature);
    }
    curvature_abs_sum += std::abs(curvature);
  }
  if (!samples.empty()) {
    stats.curvature_mean_abs_1pm =
        curvature_abs_sum / static_cast<double>(samples.size());
  }

  double speed_sum = 0.0;
  for (std::size_t i = 0U; i < speed_profile.samples.size(); ++i) {
    const TrajectorySpeedSample& sample = speed_profile.samples[i];
    const double speed = sample.profiled_limit_mps;
    if (i == 0U) {
      stats.speed_profile_min_mps = speed;
      stats.speed_profile_max_mps = speed;
    } else {
      stats.speed_profile_min_mps = std::min(stats.speed_profile_min_mps, speed);
      stats.speed_profile_max_mps = std::max(stats.speed_profile_max_mps, speed);
    }
    speed_sum += speed;
    if (sample.reason == SpeedConstraintType::kArc) {
      ++stats.speed_profile_curvature_limited_samples;
    }
  }
  if (!speed_profile.samples.empty()) {
    stats.speed_profile_mean_mps =
        speed_sum / static_cast<double>(speed_profile.samples.size());
  }
  stats.top_speed_constraints = topSpeedProfileConstraints(speed_profile, 5U);
  return stats;
}

[[nodiscard]] OffboardTrajectoryState
buildOffboardTrajectoryState(const std::span<const Point2> path_points,
                             const VelocityFollowerConfig& velocity_config) {
  OffboardTrajectoryState state;
  state.samples = trajectoryPointSamplesFromPoints(path_points);
  state.trajectory = lineTrajectoryFromSamples(state.samples);
  state.speed_profile = buildTrajectorySpeedProfile(state.samples, velocity_config);
  state.valid = trajectorySamplesAreUsable(state.samples) && state.speed_profile.valid;
  state.metrics = trajectoryMetrics(state.trajectory);
  state.shape = computeTrajectoryShapeDiagnostics(state.samples);
  state.stats = buildReceivedTrajectoryPlannerStats(path_points, state.samples,
                                                    state.trajectory, state.metrics,
                                                    state.speed_profile, state.valid);
  return state;
}

} // namespace drone_city_nav
