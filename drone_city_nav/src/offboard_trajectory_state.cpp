#include "drone_city_nav/offboard_trajectory_state.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <span>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;

[[nodiscard]] std::vector<double>
retainedPathAltitudes(const std::span<const Point2> points,
                      const std::span<const double> altitudes_m) {
  std::vector<double> retained_altitudes_m;
  if (points.size() != altitudes_m.size()) {
    return retained_altitudes_m;
  }
  retained_altitudes_m.reserve(altitudes_m.size());
  for (std::size_t i = 0U; i < points.size(); ++i) {
    if (i > 0U) {
      const double ds = distance(points[i - 1U], points[i]);
      if (!(ds > kTinyDistanceM) || !std::isfinite(ds)) {
        continue;
      }
    }
    retained_altitudes_m.push_back(altitudes_m[i]);
  }
  return retained_altitudes_m;
}

} // namespace

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

[[nodiscard]] std::vector<TrajectoryPointSample>
pathSamplesFromMessage(const nav_msgs::msg::Path& path) {
  std::vector<Point2> points;
  std::vector<double> altitudes_m;
  points.reserve(path.poses.size());
  altitudes_m.reserve(path.poses.size());
  for (const auto& pose : path.poses) {
    points.push_back(Point2{pose.pose.position.x, pose.pose.position.y});
    altitudes_m.push_back(pose.pose.position.z);
  }

  std::vector<TrajectoryPointSample> samples = trajectoryPointSamplesFromPoints(points);
  const std::vector<double> retained_altitudes_m = retainedPathAltitudes(
      std::span<const Point2>{points.data(), points.size()},
      std::span<const double>{altitudes_m.data(), altitudes_m.size()});
  if (samples.size() != retained_altitudes_m.size()) {
    return {};
  }
  for (std::size_t i = 0U; i < samples.size(); ++i) {
    samples[i].z_m = retained_altitudes_m[i];
  }
  return samples;
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

[[nodiscard]] double
elapsedMilliseconds(const std::chrono::steady_clock::time_point start) {
  return static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                 std::chrono::steady_clock::now() - start)
                                 .count()) /
         1000.0;
}

void clearVerticalProfileMetadata(TrajectoryPointSample& sample) {
  sample.vertical_profile_passage_id.clear();
  sample.vertical_hard_window_active = false;
  sample.vertical_safe_min_z_m = std::numeric_limits<double>::quiet_NaN();
  sample.vertical_safe_max_z_m = std::numeric_limits<double>::quiet_NaN();
  sample.vertical_gate_z_m = std::numeric_limits<double>::quiet_NaN();
}

[[nodiscard]] double
diagnosticSafeMinZ(const VerticalProfilePassageDiagnostic& diagnostic) noexcept {
  return std::isfinite(diagnostic.safe_min_z_m) ? diagnostic.safe_min_z_m
                                                : diagnostic.min_z_m;
}

[[nodiscard]] double
diagnosticSafeMaxZ(const VerticalProfilePassageDiagnostic& diagnostic) noexcept {
  return std::isfinite(diagnostic.safe_max_z_m) ? diagnostic.safe_max_z_m
                                                : diagnostic.max_z_m;
}

[[nodiscard]] bool stationWithinInclusive(const double s_m, const double start_s_m,
                                          const double end_s_m) noexcept {
  return std::isfinite(s_m) && std::isfinite(start_s_m) && std::isfinite(end_s_m) &&
         s_m + kTinyDistanceM >= start_s_m && s_m <= end_s_m + kTinyDistanceM;
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
  output_stats.passage_insertion_duration_ms =
      diagnostics.stats.passage_insertion_duration_ms;
  output_stats.isolated_curvature_spike_candidates =
      diagnostics.stats.isolated_curvature_spike_candidates;
  output_stats.isolated_curvature_spikes_smoothed_geometry =
      diagnostics.stats.isolated_curvature_spikes_smoothed_geometry;
  output_stats.isolated_curvature_spike_max_before_1pm =
      diagnostics.stats.isolated_curvature_spike_max_before_1pm;
  output_stats.isolated_curvature_spike_max_after_1pm =
      diagnostics.stats.isolated_curvature_spike_max_after_1pm;
  output_stats.known_passage_validation = diagnostics.stats.known_passage_validation;
  output_stats.vertical_profile = diagnostics.stats.vertical_profile;
  output_stats.passage_insertion = diagnostics.stats.passage_insertion;
}

bool applyPlannerVerticalProfileMetadata(const std::span<TrajectoryPointSample> samples,
                                         const VerticalProfileStats& vertical_profile) {
  if (samples.empty() || vertical_profile.diagnostics.empty()) {
    return false;
  }

  for (TrajectoryPointSample& sample : samples) {
    clearVerticalProfileMetadata(sample);
  }

  bool applied = false;
  for (const VerticalProfilePassageDiagnostic& diagnostic :
       vertical_profile.diagnostics) {
    if (!diagnostic.valid || diagnostic.opening_id.empty()) {
      continue;
    }
    const double safe_min_z_m = diagnosticSafeMinZ(diagnostic);
    const double safe_max_z_m = diagnosticSafeMaxZ(diagnostic);
    const bool safe_interval_valid = std::isfinite(safe_min_z_m) &&
                                     std::isfinite(safe_max_z_m) &&
                                     safe_max_z_m >= safe_min_z_m;
    for (TrajectoryPointSample& sample : samples) {
      if (stationWithinInclusive(sample.s_m, diagnostic.approach_start_s_m,
                                 diagnostic.exit_s_m)) {
        sample.vertical_profile_passage_id = diagnostic.opening_id;
        applied = true;
      }
      if (safe_interval_valid &&
          stationWithinInclusive(sample.s_m, diagnostic.gate_hold_start_s_m,
                                 diagnostic.exit_s_m)) {
        sample.vertical_profile_passage_id = diagnostic.opening_id;
        sample.vertical_hard_window_active = true;
        sample.vertical_safe_min_z_m = safe_min_z_m;
        sample.vertical_safe_max_z_m = safe_max_z_m;
        sample.vertical_gate_z_m = diagnostic.gate_z_m;
        applied = true;
      }
    }
  }
  return applied;
}

void shiftVerticalProfileStations(VerticalProfileStats& vertical_profile,
                                  const double station_offset_m) {
  if (!std::isfinite(station_offset_m) ||
      std::abs(station_offset_m) <= kTinyDistanceM) {
    return;
  }
  for (VerticalProfilePassageDiagnostic& diagnostic : vertical_profile.diagnostics) {
    diagnostic.entry_s_m += station_offset_m;
    diagnostic.exit_s_m += station_offset_m;
    diagnostic.approach_start_s_m += station_offset_m;
    diagnostic.gate_hold_start_s_m += station_offset_m;
    diagnostic.exit_end_s_m += station_offset_m;
  }
}

[[nodiscard]] TrajectoryPlannerStats buildReceivedTrajectoryPlannerStats(
    const std::span<const Point2> route_points,
    const std::span<const TrajectoryPointSample> samples,
    const std::span<const TrajectorySegment> trajectory,
    const TrajectoryMetrics& metrics, const TrajectorySpeedProfile& speed_profile,
    const VelocityFollowerConfig& velocity_config, const bool trajectory_valid) {
  TrajectoryPlannerStats stats{};
  stats.status = trajectory_valid ? TrajectoryPlannerStatus::kOk
                                  : TrajectoryPlannerStatus::kInvalidTrajectory;
  stats.input_points = route_points.size();
  stats.samples = samples.size();
  stats.compact_segments = trajectory.size();
  stats.line_segments = metrics.line_segments;
  stats.arc_segments = metrics.arc_segments;
  stats.length_m = metrics.length_m;
  stats.speed_profile_construction_config_fingerprint =
      speedProfileConstructionConfigFingerprint(velocity_config);
  stats.runtime_speed_policy_config_fingerprint =
      runtimeSpeedPolicyConfigFingerprint(velocity_config);
  stats.runtime_velocity_control_config_fingerprint =
      runtimeVelocityControlConfigFingerprint(velocity_config);

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
  std::vector<TrajectoryPointSample> samples =
      trajectoryPointSamplesFromPoints(path_points);
  return buildOffboardTrajectoryState(
      std::span<const TrajectoryPointSample>{samples.data(), samples.size()},
      velocity_config);
}

[[nodiscard]] OffboardTrajectoryState
buildOffboardTrajectoryState(const std::span<const TrajectoryPointSample> path_samples,
                             const VelocityFollowerConfig& velocity_config) {
  return buildOffboardTrajectoryState(path_samples, velocity_config, nullptr);
}

[[nodiscard]] OffboardTrajectoryState
buildOffboardTrajectoryState(const std::span<const TrajectoryPointSample> path_samples,
                             const VelocityFollowerConfig& velocity_config,
                             const TrajectoryPlannerStats* const planner_stats) {
  OffboardTrajectoryState state;
  state.samples.assign(path_samples.begin(), path_samples.end());
  if (planner_stats != nullptr) {
    applyPlannerVerticalProfileMetadata(state.samples, planner_stats->vertical_profile);
  }
  populateTrajectoryVerticalSpeedConstraints(state.samples, velocity_config);
  state.trajectory = lineTrajectoryFromSamples(state.samples);
  const auto speed_profile_started_at = std::chrono::steady_clock::now();
  state.speed_profile = buildTrajectorySpeedProfile(state.samples, velocity_config);
  const double speed_profile_duration_ms =
      elapsedMilliseconds(speed_profile_started_at);
  state.valid = trajectorySamplesAreUsable(state.samples) && state.speed_profile.valid;
  state.metrics = trajectoryMetrics(state.trajectory);
  state.shape = computeTrajectoryShapeDiagnostics(state.samples);
  std::vector<Point2> path_points;
  path_points.reserve(state.samples.size());
  for (const TrajectoryPointSample& sample : state.samples) {
    path_points.push_back(sample.point);
  }
  state.stats = buildReceivedTrajectoryPlannerStats(
      path_points, state.samples, state.trajectory, state.metrics, state.speed_profile,
      velocity_config, state.valid);
  state.stats.speed_profile_duration_ms = speed_profile_duration_ms;
  return state;
}

TrajectoryContinuityResult evaluateOffboardTrajectoryUpdateContinuity(
    const std::span<const TrajectoryPointSample> current_samples,
    const TrajectorySpeedProfile& current_speed_profile,
    const OffboardTrajectoryState& candidate_state, const Point2 current_position,
    const Point2 previous_velocity_setpoint,
    const bool previous_velocity_setpoint_valid, const bool local_position_fresh,
    const double current_altitude_m, const bool altitude_valid,
    const TrajectoryContinuityThresholds& thresholds) {
  if (!candidate_state.valid || !trajectorySamplesAreUsable(candidate_state.samples) ||
      !candidate_state.speed_profile.valid) {
    TrajectoryContinuityResult result{};
    result.decision = TrajectoryContinuityDecision::kRejectTrajectory;
    result.reason = "new_trajectory_invalid";
    return result;
  }
  if (!local_position_fresh) {
    TrajectoryContinuityResult result{};
    result.decision = TrajectoryContinuityDecision::kResetSmoother;
    result.reason = "pose_stale";
    return result;
  }
  return evaluateTrajectoryContinuity(
      current_samples, current_speed_profile, candidate_state.samples,
      candidate_state.speed_profile, current_position, previous_velocity_setpoint,
      previous_velocity_setpoint_valid, thresholds,
      TrajectoryVerticalContinuityState{.current_altitude_m = current_altitude_m,
                                        .altitude_valid = altitude_valid});
}

} // namespace drone_city_nav
