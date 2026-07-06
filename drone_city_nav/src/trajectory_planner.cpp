#include "drone_city_nav/trajectory_planner.hpp"

#include "drone_city_nav/trajectory_diagnostics.hpp"
#include "drone_city_nav/trajectory_shape_cleanup.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;
constexpr std::size_t kTopSpeedConstraintCount = 5U;

void computeCurvatureStats(const std::span<const TrajectoryPointSample> samples,
                           TrajectoryPlannerStats& stats) {
  if (samples.empty()) {
    return;
  }
  double abs_sum = 0.0;
  for (std::size_t i = 0U; i < samples.size(); ++i) {
    const double curvature = samples[i].curvature_1pm;
    if (i == 0U) {
      stats.curvature_min_1pm = curvature;
      stats.curvature_max_1pm = curvature;
    } else {
      stats.curvature_min_1pm = std::min(stats.curvature_min_1pm, curvature);
      stats.curvature_max_1pm = std::max(stats.curvature_max_1pm, curvature);
    }
    abs_sum += std::abs(curvature);
  }
  stats.curvature_mean_abs_1pm = abs_sum / static_cast<double>(samples.size());
}

void computeSpeedProfileStats(const TrajectorySpeedProfile& profile,
                              TrajectoryPlannerStats& stats) {
  if (!profile.valid || profile.samples.empty()) {
    return;
  }
  double sum = 0.0;
  for (std::size_t i = 0U; i < profile.samples.size(); ++i) {
    const double speed = profile.samples[i].profiled_limit_mps;
    if (i == 0U) {
      stats.speed_profile_min_mps = speed;
      stats.speed_profile_max_mps = speed;
    } else {
      stats.speed_profile_min_mps = std::min(stats.speed_profile_min_mps, speed);
      stats.speed_profile_max_mps = std::max(stats.speed_profile_max_mps, speed);
    }
    sum += speed;
    if (profile.samples[i].reason == SpeedConstraintType::kArc) {
      ++stats.speed_profile_curvature_limited_samples;
    }
  }
  stats.speed_profile_mean_mps = sum / static_cast<double>(profile.samples.size());
  stats.top_speed_constraints =
      topSpeedProfileConstraints(profile, kTopSpeedConstraintCount);
}

[[nodiscard]] double
elapsedMilliseconds(const std::chrono::steady_clock::time_point start) {
  return static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                 std::chrono::steady_clock::now() - start)
                                 .count()) /
         1000.0;
}

void finalizeResult(TrajectoryPlannerResult& result,
                    const TrajectoryPlannerConfig& config) {
  assignTrajectorySampleAltitude(result.samples, config.default_altitude_m);
  const TrajectoryMetrics metrics = trajectoryMetrics(result.compact_segments);
  result.stats.compact_segments = result.compact_segments.size();
  result.stats.line_segments = metrics.line_segments;
  result.stats.arc_segments = metrics.arc_segments;
  result.stats.length_m = metrics.length_m;
  result.stats.samples = result.samples.size();
  result.stats.speed_profile_construction_config_fingerprint =
      speedProfileConstructionConfigFingerprint(config.speed_profile);
  result.stats.runtime_speed_policy_config_fingerprint =
      runtimeSpeedPolicyConfigFingerprint(config.speed_profile);
  result.stats.runtime_velocity_control_config_fingerprint =
      runtimeVelocityControlConfigFingerprint(config.speed_profile);
  computeCurvatureStats(result.samples, result.stats);
  computeSpeedProfileStats(result.speed_profile, result.stats);
  result.valid = trajectoryIsUsable(result.compact_segments) &&
                 trajectorySamplesAreUsable(result.samples) &&
                 result.speed_profile.valid;
  if (!result.valid && result.stats.status == TrajectoryPlannerStatus::kOk) {
    result.stats.status = TrajectoryPlannerStatus::kInvalidTrajectory;
  }
}

void populateCorridorReuseStats(const std::span<const CorridorSample> samples,
                                CorridorStats& stats) {
  stats.samples = samples.size();
  stats.samples_reused = true;
  stats.reused_samples = samples.size();
  stats.parallel_workers_used = 0U;
  stats.sample_build_duration_ms = 0.0;
  stats.raycast_duration_ms = 0.0;
  stats.lateral_limit_duration_ms = 0.0;
  stats.clearance_field_build_duration_ms = 0.0;
  if (samples.empty()) {
    return;
  }

  double width_sum = 0.0;
  double clearance_sum = 0.0;
  for (std::size_t i = 0U; i < samples.size(); ++i) {
    const CorridorSample& sample = samples[i];
    const double width = sample.left_bound_m + sample.right_bound_m;
    if (i == 0U) {
      stats.min_width_m = width;
      stats.max_width_m = width;
      stats.min_clearance_m = sample.clearance_m;
      stats.max_clearance_m = sample.clearance_m;
    } else {
      stats.min_width_m = std::min(stats.min_width_m, width);
      stats.max_width_m = std::max(stats.max_width_m, width);
      stats.min_clearance_m = std::min(stats.min_clearance_m, sample.clearance_m);
      stats.max_clearance_m = std::max(stats.max_clearance_m, sample.clearance_m);
    }
    width_sum += width;
    clearance_sum += sample.clearance_m;
    stats.max_center_recovery_m =
        std::max(stats.max_center_recovery_m, sample.center_recovery_m);
  }
  stats.mean_width_m = width_sum / static_cast<double>(samples.size());
  stats.mean_clearance_m = clearance_sum / static_cast<double>(samples.size());
}

[[nodiscard]] CorridorResult
corridorFromPrecomputedSamples(const std::span<const CorridorSample> samples,
                               const CorridorStats* source_stats,
                               const std::size_t input_points) {
  CorridorResult result{};
  result.samples.assign(samples.begin(), samples.end());
  if (source_stats != nullptr) {
    result.stats = *source_stats;
  }
  result.stats.input_points = input_points;
  populateCorridorReuseStats(result.samples, result.stats);
  result.valid = result.samples.size() >= 2U &&
                 result.stats.route_prohibited_samples == 0U &&
                 result.stats.center_unrecoverable_samples == 0U;
  return result;
}

[[nodiscard]] bool precomputedCorridorMatchesRoute(
    const std::span<const CorridorSample> samples, const CorridorStats* source_stats,
    const std::span<const Point2> route_points, const OccupancyGrid2D& prohibited_grid,
    const CorridorConfig& config) {
  if (samples.size() < 2U || route_points.size() < 2U || source_stats == nullptr ||
      source_stats->samples != samples.size()) {
    return false;
  }
  if (source_stats->route_fingerprint != corridorRouteFingerprint(route_points) ||
      source_stats->config_fingerprint != corridorConfigFingerprint(config) ||
      !occupancyGridFingerprintsEqual(source_stats->prohibited_grid_fingerprint,
                                      prohibited_grid.prohibitedFingerprint())) {
    return false;
  }
  constexpr double kEndpointToleranceM = 1.0e-6;
  return distance(samples.front().route_center, route_points.front()) <=
             kEndpointToleranceM &&
         distance(samples.back().route_center, route_points.back()) <=
             kEndpointToleranceM;
}

[[nodiscard]] Point2 operator-(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x - rhs.x, lhs.y - rhs.y};
}

[[nodiscard]] Point2 normalized(const Point2 point) noexcept {
  const double length = std::hypot(point.x, point.y);
  if (!(length > kTinyDistanceM)) {
    return Point2{1.0, 0.0};
  }
  return Point2{point.x / length, point.y / length};
}

[[nodiscard]] double cross(const Point2 lhs, const Point2 rhs) noexcept {
  return lhs.x * rhs.y - lhs.y * rhs.x;
}

[[nodiscard]] double signedCurvatureFromTriplet(const Point2 previous,
                                                const Point2 current,
                                                const Point2 next) noexcept {
  const Point2 a = current - previous;
  const Point2 b = next - current;
  const double ab = distance(previous, current);
  const double bc = distance(current, next);
  const double ac = distance(previous, next);
  const double denominator = ab * bc * ac;
  if (!(denominator > kTinyDistanceM)) {
    return 0.0;
  }
  return 2.0 * cross(a, b) / denominator;
}

void populateSampleGeometry(std::vector<TrajectoryPointSample>& samples) {
  double s_m = 0.0;
  for (std::size_t i = 0U; i < samples.size(); ++i) {
    if (i > 0U) {
      s_m += distance(samples[i - 1U].point, samples[i].point);
    }
    samples[i].s_m = s_m;
    samples[i].curvature_1pm = 0.0;
    if (samples.size() == 1U) {
      samples[i].tangent = Point2{1.0, 0.0};
    } else if (i == 0U) {
      samples[i].tangent = normalized(samples[i + 1U].point - samples[i].point);
    } else if (i + 1U == samples.size()) {
      samples[i].tangent = normalized(samples[i].point - samples[i - 1U].point);
    } else {
      samples[i].tangent = normalized(samples[i + 1U].point - samples[i - 1U].point);
      samples[i].curvature_1pm = signedCurvatureFromTriplet(
          samples[i - 1U].point, samples[i].point, samples[i + 1U].point);
    }
  }
}

[[nodiscard]] std::vector<TrajectoryPointSample>
baselineSamplesFromCorridor(const std::span<const CorridorSample> corridor_samples) {
  std::vector<TrajectoryPointSample> samples;
  samples.reserve(corridor_samples.size());
  for (const CorridorSample& corridor_sample : corridor_samples) {
    TrajectoryPointSample sample{};
    sample.point = corridor_sample.center;
    sample.left_bound_m = corridor_sample.left_bound_m;
    sample.right_bound_m = corridor_sample.right_bound_m;
    sample.lateral_offset_m = 0.0;
    samples.push_back(sample);
  }
  populateSampleGeometry(samples);
  return samples;
}

[[nodiscard]] bool segmentTraversable(const OccupancyGrid2D& grid, const Point2 start,
                                      const Point2 end) {
  const std::optional<GridIndex> start_cell = grid.worldToCell(start);
  const std::optional<GridIndex> end_cell = grid.worldToCell(end);
  if (!start_cell.has_value() || !end_cell.has_value()) {
    return false;
  }
  return std::ranges::all_of(
      grid.cellsOnLine(*start_cell, *end_cell),
      [&grid](const GridIndex cell) { return !grid.isProhibited(cell); });
}

[[nodiscard]] bool pathTraversable(const OccupancyGrid2D& grid,
                                   const std::span<const Point2> points) {
  if (points.size() < 2U) {
    return false;
  }
  for (std::size_t i = 1U; i < points.size(); ++i) {
    if (!segmentTraversable(grid, points[i - 1U], points[i])) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] bool
trajectoryStageInvariantsHold(const std::span<const TrajectoryPointSample> samples,
                              const OccupancyGrid2D& grid, const Point2 expected_start,
                              const Point2 expected_goal) {
  constexpr double kEndpointToleranceM = 1.0e-4;
  if (!trajectorySamplesAreUsable(samples) ||
      distance(samples.front().point, expected_start) > kEndpointToleranceM ||
      distance(samples.back().point, expected_goal) > kEndpointToleranceM) {
    return false;
  }
  std::vector<Point2> points;
  points.reserve(samples.size());
  for (const TrajectoryPointSample& sample : samples) {
    points.push_back(sample.point);
  }
  if (!pathTraversable(grid, points)) {
    return false;
  }
  const TrajectoryShapeDiagnostics shape = computeTrajectoryShapeDiagnostics(samples);
  return std::isfinite(shape.max_heading_delta_rad) &&
         std::isfinite(shape.max_curvature_jump_1pm) &&
         std::isfinite(shape.max_segment_length_m);
}

} // namespace

std::string_view
trajectoryPlannerStatusName(const TrajectoryPlannerStatus status) noexcept {
  switch (status) {
    case TrajectoryPlannerStatus::kOk:
      return "none";
    case TrajectoryPlannerStatus::kInvalidRoute:
      return "invalid_route";
    case TrajectoryPlannerStatus::kMissingGrid:
      return "missing_grid";
    case TrajectoryPlannerStatus::kCorridorInvalid:
      return "corridor_invalid";
    case TrajectoryPlannerStatus::kTrajectoryOptimizerInvalid:
      return "trajectory_optimizer_invalid";
    case TrajectoryPlannerStatus::kInvalidTrajectory:
      return "invalid_trajectory";
  }
  return "unknown";
}

std::string_view trajectoryQualityName(const TrajectoryQuality quality) noexcept {
  switch (quality) {
    case TrajectoryQuality::kUnknown:
      return "unknown";
    case TrajectoryQuality::kBaseline:
      return "baseline";
    case TrajectoryQuality::kRefined:
      return "refined";
  }
  return "unknown";
}

std::string_view
refinementDecisionReasonName(const TrajectoryRefinementDecisionReason reason) noexcept {
  switch (reason) {
    case TrajectoryRefinementDecisionReason::kAccepted:
      return "accepted";
    case TrajectoryRefinementDecisionReason::kStaleGeneration:
      return "stale_generation";
    case TrajectoryRefinementDecisionReason::kInvalidRefined:
      return "invalid_refined";
    case TrajectoryRefinementDecisionReason::kEndpointMismatch:
      return "endpoint_mismatch";
    case TrajectoryRefinementDecisionReason::kNonTraversable:
      return "non_traversable";
  }
  return "unknown";
}

TrajectoryPlannerResult planBaselineTrajectory(const TrajectoryPlannerInput& input,
                                               const TrajectoryPlannerConfig& config) {
  const auto total_started_at = std::chrono::steady_clock::now();
  TrajectoryPlannerResult result{};
  result.stats.input_points = input.route_points.size();
  result.stats.quality = TrajectoryQuality::kBaseline;
  if (input.route_points.size() < 2U) {
    result.stats.status = TrajectoryPlannerStatus::kInvalidRoute;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }
  if (input.prohibited_grid == nullptr) {
    result.stats.status = TrajectoryPlannerStatus::kMissingGrid;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }

  const auto corridor_started_at = std::chrono::steady_clock::now();
  const CorridorResult corridor =
      buildCorridor(CorridorInput{input.route_points, input.prohibited_grid,
                                  input.prohibited_clearance_field,
                                  input.prohibited_clearance_field_cache_hit},
                    config.corridor);
  result.stats.corridor_duration_ms = elapsedMilliseconds(corridor_started_at);
  result.corridor_samples = corridor.samples;
  result.stats.corridor = corridor.stats;
  if (!corridor.valid) {
    result.stats.status = TrajectoryPlannerStatus::kCorridorInvalid;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }

  result.stats.status = TrajectoryPlannerStatus::kOk;
  result.samples = baselineSamplesFromCorridor(corridor.samples);
  if (!trajectoryStageInvariantsHold(result.samples, *input.prohibited_grid,
                                     input.route_points.front(),
                                     input.route_points.back())) {
    result.stats.status = TrajectoryPlannerStatus::kInvalidTrajectory;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }
  result.compact_segments = lineTrajectoryFromSamples(result.samples);
  const auto speed_profile_started_at = std::chrono::steady_clock::now();
  result.speed_profile =
      buildTrajectorySpeedProfile(result.samples, config.speed_profile);
  result.stats.speed_profile_duration_ms =
      elapsedMilliseconds(speed_profile_started_at);

  const TraversalTimeEstimate traversal_estimate =
      estimateTraversalTime(result.samples, config.speed_profile, true);
  result.stats.trajectory_optimizer.input_samples = corridor.samples.size();
  result.stats.trajectory_optimizer.optimizer_samples = corridor.samples.size();
  result.stats.trajectory_optimizer.output_samples = result.samples.size();
  result.stats.trajectory_optimizer.centerline_length_m =
      trajectoryLengthM(result.compact_segments);
  result.stats.trajectory_optimizer.final_length_m =
      result.stats.trajectory_optimizer.centerline_length_m;
  result.stats.trajectory_optimizer.final_length_ratio = 1.0;
  result.stats.trajectory_optimizer.estimated_time_s =
      traversal_estimate.estimated_time_s;
  result.stats.trajectory_optimizer.min_speed_limit_mps =
      traversal_estimate.min_speed_limit_mps;
  result.stats.trajectory_optimizer.max_speed_limit_mps =
      traversal_estimate.max_speed_limit_mps;
  result.stats.trajectory_optimizer.curvature_limited_samples =
      traversal_estimate.curvature_limited_samples;

  finalizeResult(result, config);
  result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
  return result;
}

TrajectoryPlannerResult planOptimizedTrajectory(const TrajectoryPlannerInput& input,
                                                const TrajectoryPlannerConfig& config) {
  const auto total_started_at = std::chrono::steady_clock::now();
  TrajectoryPlannerResult result{};
  result.stats.input_points = input.route_points.size();
  result.stats.quality = TrajectoryQuality::kRefined;
  if (input.route_points.size() < 2U) {
    result.stats.status = TrajectoryPlannerStatus::kInvalidRoute;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }
  if (input.prohibited_grid == nullptr) {
    result.stats.status = TrajectoryPlannerStatus::kMissingGrid;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }

  const auto corridor_started_at = std::chrono::steady_clock::now();
  const CorridorResult corridor =
      precomputedCorridorMatchesRoute(
          input.precomputed_corridor_samples, input.precomputed_corridor_stats,
          input.route_points, *input.prohibited_grid, config.corridor)
          ? corridorFromPrecomputedSamples(input.precomputed_corridor_samples,
                                           input.precomputed_corridor_stats,
                                           input.route_points.size())
          : buildCorridor(CorridorInput{input.route_points, input.prohibited_grid,
                                        input.prohibited_clearance_field,
                                        input.prohibited_clearance_field_cache_hit},
                          config.corridor);
  result.stats.corridor_duration_ms = elapsedMilliseconds(corridor_started_at);
  result.corridor_samples = corridor.samples;
  if (!corridor.valid) {
    result.stats.status = TrajectoryPlannerStatus::kCorridorInvalid;
    result.stats.corridor = corridor.stats;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }

  const auto trajectory_optimizer_started_at = std::chrono::steady_clock::now();
  const TrajectoryOptimizerResult optimized_trajectory =
      optimizeTrajectory(corridor.samples, *input.prohibited_grid,
                         config.trajectory_optimizer, config.speed_profile);
  result.stats.trajectory_optimizer_duration_ms =
      elapsedMilliseconds(trajectory_optimizer_started_at);
  if (!optimized_trajectory.valid) {
    result.stats.status = TrajectoryPlannerStatus::kTrajectoryOptimizerInvalid;
    result.stats.corridor = corridor.stats;
    result.stats.trajectory_optimizer = optimized_trajectory.stats;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }

  result.stats.input_points = input.route_points.size();
  result.stats.status = TrajectoryPlannerStatus::kOk;
  result.stats.corridor = corridor.stats;
  result.stats.trajectory_optimizer = optimized_trajectory.stats;
  result.trajectory_optimizer_windows = optimized_trajectory.active_windows;
  const auto turn_smoothing_started_at = std::chrono::steady_clock::now();
  const TurnSmoothingResult turn_smoothing = smoothTrajectoryTurns(
      optimized_trajectory.samples, corridor.samples, *input.prohibited_grid,
      config.turn_smoothing, config.speed_profile);
  result.stats.turn_smoothing_duration_ms =
      elapsedMilliseconds(turn_smoothing_started_at);
  result.stats.turn_smoothing = turn_smoothing.stats;
  if (!turn_smoothing.valid) {
    result.stats.status = TrajectoryPlannerStatus::kInvalidTrajectory;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }
  result.samples = turn_smoothing.samples;
  if (!trajectoryStageInvariantsHold(result.samples, *input.prohibited_grid,
                                     input.route_points.front(),
                                     input.route_points.back())) {
    result.stats.status = TrajectoryPlannerStatus::kInvalidTrajectory;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }
  result.stats.isolated_curvature_spike_candidates =
      countIsolatedCurvatureSpikes(result.samples);
  result.stats.isolated_curvature_spike_max_before_1pm =
      maxIsolatedCurvatureSpike(result.samples);
  result.stats.isolated_curvature_spikes_smoothed_geometry =
      smoothIsolatedCurvatureSpikeGeometry(result.samples, *input.prohibited_grid);
  result.stats.isolated_curvature_spike_max_after_1pm =
      maxIsolatedCurvatureSpike(result.samples);
  if (!trajectoryStageInvariantsHold(result.samples, *input.prohibited_grid,
                                     input.route_points.front(),
                                     input.route_points.back())) {
    result.stats.status = TrajectoryPlannerStatus::kInvalidTrajectory;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }
  result.compact_segments = lineTrajectoryFromSamples(result.samples);
  const auto speed_profile_started_at = std::chrono::steady_clock::now();
  result.speed_profile =
      buildTrajectorySpeedProfile(result.samples, config.speed_profile);
  result.stats.speed_profile_duration_ms =
      elapsedMilliseconds(speed_profile_started_at);
  finalizeResult(result, config);
  result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
  return result;
}

TrajectoryPlannerResult planOptimizedTrajectoryFromSnapshots(
    const std::span<const Point2> route_points, const OccupancyGrid2D& prohibited_grid,
    const ClearanceField2D* prohibited_clearance_field,
    const bool prohibited_clearance_field_cache_hit,
    const std::span<const CorridorSample> precomputed_corridor_samples,
    const CorridorStats* precomputed_corridor_stats,
    const TrajectoryPlannerConfig& config) {
  return planOptimizedTrajectory(
      TrajectoryPlannerInput{
          route_points,
          &prohibited_grid,
          prohibited_clearance_field,
          prohibited_clearance_field_cache_hit,
          precomputed_corridor_samples,
          precomputed_corridor_stats,
      },
      config);
}

TrajectoryPlannerResult planTrajectory(const TrajectoryPlannerInput& input,
                                       const TrajectoryPlannerConfig& config) {
  return planOptimizedTrajectory(input, config);
}

TrajectoryRefinementDecision
evaluateTrajectoryRefinement(const TrajectoryRefinementDecisionInput& input) {
  if (input.current_generation != input.snapshot_generation) {
    return TrajectoryRefinementDecision{
        .accepted = false,
        .reason = TrajectoryRefinementDecisionReason::kStaleGeneration,
    };
  }
  if (input.refined == nullptr || !input.refined->valid ||
      input.refined_points.size() < 2U) {
    return TrajectoryRefinementDecision{
        .accepted = false,
        .reason = TrajectoryRefinementDecisionReason::kInvalidRefined,
    };
  }

  const double endpoint_tolerance_m = std::max(0.0, input.endpoint_tolerance_m);
  if (distance(input.refined_points.front(), input.expected_start) >
          endpoint_tolerance_m ||
      distance(input.refined_points.back(), input.expected_goal) >
          endpoint_tolerance_m) {
    return TrajectoryRefinementDecision{
        .accepted = false,
        .reason = TrajectoryRefinementDecisionReason::kEndpointMismatch,
    };
  }

  if (input.validation_grid != nullptr &&
      !pathTraversable(*input.validation_grid, input.refined_points)) {
    return TrajectoryRefinementDecision{
        .accepted = false,
        .reason = TrajectoryRefinementDecisionReason::kNonTraversable,
    };
  }

  return TrajectoryRefinementDecision{
      .accepted = true,
      .reason = TrajectoryRefinementDecisionReason::kAccepted,
  };
}

} // namespace drone_city_nav
