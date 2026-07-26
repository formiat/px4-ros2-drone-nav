#include "drone_city_nav/trajectory_planner.hpp"

#include "drone_city_nav/trajectory_diagnostics.hpp"
#include "drone_city_nav/trajectory_shape_cleanup.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <ranges>
#include <string>

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
  if (!result.stats.vertical_profile.applied) {
    assignTrajectorySampleAltitude(result.samples, config.initial_altitude_m);
  }
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
                 result.speed_profile.valid && result.stats.vertical_profile.valid &&
                 result.stats.known_passage_solid_validation.valid &&
                 result.stats.final_risk.hardValid();
  if (result.valid && (!result.stats.known_passage_validation.valid ||
                       result.stats.passage_insertion.quality ==
                           PassageInsertionQuality::kDegradedJoin)) {
    result.stats.quality = TrajectoryQuality::kDegradedPassage;
  }
  if (!result.valid && result.stats.status == TrajectoryPlannerStatus::kOk) {
    result.stats.status = TrajectoryPlannerStatus::kInvalidTrajectory;
  }
}

bool applyVerticalProfileStage(TrajectoryPlannerResult& result,
                               const TrajectoryPlannerInput& input,
                               const TrajectoryPlannerConfig& config) {
  const VerticalProfileResult vertical_profile = applyVerticalProfile(
      result.samples, input.known_passage_map, config.known_passage_validation,
      config.vertical_profile, config.initial_altitude_m,
      input.passage_insertion_start_mode ==
              PassageInsertionStartMode::kTerminalHoldRestart
          ? VerticalProfileStartMode::kStationaryHoldRestart
          : VerticalProfileStartMode::kMoving);
  result.stats.vertical_profile = vertical_profile.stats;
  result.stats.known_passage_validation = validateKnownPassageTraversal(
      result.samples, input.known_passage_map, config.known_passage_validation);
  result.stats.known_passage_solid_validation =
      validateTrajectoryAgainstKnownPassageSolids(result.samples,
                                                  input.known_passage_map);
  if (!vertical_profile.valid) {
    result.stats.status = TrajectoryPlannerStatus::kInvalidTrajectory;
    return false;
  }
  return true;
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
      !occupancyGridFingerprintsEqual(source_stats->raw_occupancy_fingerprint,
                                      prohibited_grid.rawFingerprint())) {
    return false;
  }
  constexpr double kEndpointToleranceM = 1.0e-6;
  return distance(samples.front().route_center, route_points.front()) <=
             kEndpointToleranceM &&
         distance(samples.back().route_center, route_points.back()) <=
             kEndpointToleranceM;
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
  populateTrajectorySampleGeometry(samples);
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
      [&grid](const GridIndex cell) { return !grid.isOccupied(cell); });
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

[[nodiscard]] const TrajectoryRiskContext*
primaryRiskContext(const std::span<const TrajectoryRiskContext> contexts) noexcept {
  const auto found = std::ranges::find_if(
      contexts, [](const TrajectoryRiskContext& context) { return context.valid(); });
  return found != contexts.end() ? &*found : nullptr;
}

[[nodiscard]] PathRiskScore
evaluateTrajectoryRisk(const TrajectoryRiskContext& context,
                       const std::span<const TrajectoryPointSample> samples) {
  return context.risk_field->evaluate(*context.raw_occupancy, samples);
}

[[nodiscard]] bool applyRiskGate(const char* const stage,
                                 const TrajectoryRiskContext& context,
                                 const std::span<const TrajectoryPointSample> before,
                                 const std::span<const TrajectoryPointSample> after,
                                 const bool allow_degraded,
                                 TrajectoryPlannerStats& stats) {
  const PathRiskScore before_risk = evaluateTrajectoryRisk(context, before);
  const PathRiskScore after_risk = evaluateTrajectoryRisk(context, after);
  const bool worsened = pathRiskLess(before_risk, after_risk);
  const bool accepted = after_risk.hardValid() && (!worsened || allow_degraded);
  stats.stage_risk.push_back(TrajectoryStageRiskDiagnostic{
      .stage = stage,
      .before = before_risk,
      .after = after_risk,
      .changed = !pathRiskEqual(before_risk, after_risk),
      .accepted = accepted,
      .degraded = accepted && worsened,
  });
  return accepted;
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
    case TrajectoryPlannerStatus::kCanceled:
      return "canceled";
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
    case TrajectoryQuality::kDegradedPassage:
      return "degraded_passage";
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
  const TrajectoryRiskContext* const risk_context =
      primaryRiskContext(input.risk_contexts);
  if (risk_context == nullptr) {
    result.stats.status = TrajectoryPlannerStatus::kMissingGrid;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }

  const auto corridor_started_at = std::chrono::steady_clock::now();
  result.stats.grid_stages.corridor_attempts = 1U;
  CorridorResult corridor = buildCorridor(
      CorridorInput{input.route_points, risk_context->raw_occupancy,
                    risk_context->raw_clearance, risk_context->raw_clearance_cache_hit},
      config.corridor);
  result.stats.grid_stages.corridor =
      corridor.valid ? std::string{risk_context->name} : "none";
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
  result.stats.grid_stages.trajectory_validation_attempts = 1U;
  const PathRiskScore baseline_risk =
      evaluateTrajectoryRisk(*risk_context, result.samples);
  if (!baseline_risk.hardValid() ||
      !trajectoryStageInvariantsHold(result.samples, *risk_context->raw_occupancy,
                                     input.route_points.front(),
                                     input.route_points.back())) {
    result.stats.status = TrajectoryPlannerStatus::kInvalidTrajectory;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }
  result.stats.grid_stages.trajectory_validation = std::string{risk_context->name};
  result.stats.final_risk = baseline_risk;
  result.compact_segments = lineTrajectoryFromSamples(result.samples);
  if (!applyVerticalProfileStage(result, input, config)) {
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }
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
  const TrajectoryRiskContext* const risk_context =
      primaryRiskContext(input.risk_contexts);
  if (risk_context == nullptr) {
    result.stats.status = TrajectoryPlannerStatus::kMissingGrid;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }
  if (input.stop_token.stop_requested()) {
    result.stats.status = TrajectoryPlannerStatus::kCanceled;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }

  const auto corridor_started_at = std::chrono::steady_clock::now();
  result.stats.grid_stages.corridor_attempts = 1U;
  CorridorResult corridor =
      precomputedCorridorMatchesRoute(
          input.precomputed_corridor_samples, input.precomputed_corridor_stats,
          input.route_points, *risk_context->raw_occupancy, config.corridor)
          ? corridorFromPrecomputedSamples(input.precomputed_corridor_samples,
                                           input.precomputed_corridor_stats,
                                           input.route_points.size())
          : buildCorridor(CorridorInput{input.route_points, risk_context->raw_occupancy,
                                        risk_context->raw_clearance,
                                        risk_context->raw_clearance_cache_hit},
                          config.corridor);
  result.stats.grid_stages.corridor =
      corridor.valid ? std::string{risk_context->name} : "none";
  result.stats.corridor_duration_ms = elapsedMilliseconds(corridor_started_at);
  result.corridor_samples = corridor.samples;
  if (input.stop_token.stop_requested()) {
    result.stats.status = TrajectoryPlannerStatus::kCanceled;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }
  if (!corridor.valid) {
    result.stats.status = TrajectoryPlannerStatus::kCorridorInvalid;
    result.stats.corridor = corridor.stats;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }

  const auto trajectory_optimizer_started_at = std::chrono::steady_clock::now();
  result.stats.grid_stages.optimizer_attempts = 1U;
  TrajectoryOptimizerResult optimized_trajectory = optimizeTrajectory(
      corridor.samples, *risk_context->raw_occupancy, *risk_context->risk_field,
      config.trajectory_optimizer, config.speed_profile, input.stop_token);
  result.stats.trajectory_optimizer = optimized_trajectory.stats;
  result.stats.grid_stages.optimizer =
      optimized_trajectory.valid ? std::string{risk_context->name} : "none";
  result.stats.trajectory_optimizer_duration_ms =
      elapsedMilliseconds(trajectory_optimizer_started_at);
  if (input.stop_token.stop_requested() || optimized_trajectory.stats.canceled) {
    result.stats.status = TrajectoryPlannerStatus::kCanceled;
    result.stats.trajectory_optimizer = optimized_trajectory.stats;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }
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
  result.stats.grid_stages.turn_smoothing_attempts = 1U;
  TurnSmoothingResult turn_smoothing = smoothTrajectoryTurns(
      optimized_trajectory.samples, corridor.samples, *risk_context->raw_occupancy,
      config.turn_smoothing, config.speed_profile);
  result.stats.turn_smoothing = turn_smoothing.stats;
  result.stats.grid_stages.turn_smoothing =
      turn_smoothing.valid ? std::string{risk_context->name} : "none";
  if (turn_smoothing.valid &&
      !applyRiskGate("turn_smoothing", *risk_context, optimized_trajectory.samples,
                     turn_smoothing.samples, false, result.stats)) {
    turn_smoothing.samples = optimized_trajectory.samples;
    turn_smoothing.valid = trajectorySamplesAreUsable(turn_smoothing.samples);
    turn_smoothing.changed = false;
  }
  result.stats.turn_smoothing_duration_ms =
      elapsedMilliseconds(turn_smoothing_started_at);
  if (input.stop_token.stop_requested()) {
    result.stats.status = TrajectoryPlannerStatus::kCanceled;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }
  result.stats.turn_smoothing = turn_smoothing.stats;
  if (!turn_smoothing.valid) {
    result.stats.status = TrajectoryPlannerStatus::kInvalidTrajectory;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }
  result.samples = turn_smoothing.samples;
  result.stats.grid_stages.trajectory_validation_attempts = 1U;
  const PathRiskScore initial_risk =
      evaluateTrajectoryRisk(*risk_context, result.samples);
  if (!initial_risk.hardValid() ||
      !trajectoryStageInvariantsHold(result.samples, *risk_context->raw_occupancy,
                                     input.route_points.front(),
                                     input.route_points.back())) {
    result.stats.status = TrajectoryPlannerStatus::kInvalidTrajectory;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }
  result.stats.grid_stages.trajectory_validation = std::string{risk_context->name};
  result.stats.isolated_curvature_spike_candidates =
      countIsolatedCurvatureSpikes(result.samples);
  result.stats.isolated_curvature_spike_max_before_1pm =
      maxIsolatedCurvatureSpike(result.samples);
  result.stats.grid_stages.shape_cleanup_attempts = 1U;
  std::vector<TrajectoryPointSample> shape_cleaned_samples = result.samples;
  const std::size_t smoothed = smoothIsolatedCurvatureSpikeGeometry(
      shape_cleaned_samples, *risk_context->raw_occupancy);
  if (!trajectoryStageInvariantsHold(
          shape_cleaned_samples, *risk_context->raw_occupancy,
          input.route_points.front(), input.route_points.back())) {
    result.stats.status = TrajectoryPlannerStatus::kInvalidTrajectory;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }
  if (!applyRiskGate("shape_cleanup", *risk_context, result.samples,
                     shape_cleaned_samples, false, result.stats)) {
    shape_cleaned_samples = result.samples;
    result.stats.isolated_curvature_spikes_smoothed_geometry = 0U;
  } else {
    result.stats.isolated_curvature_spikes_smoothed_geometry = smoothed;
  }
  result.stats.grid_stages.shape_cleanup = std::string{risk_context->name};
  result.samples = std::move(shape_cleaned_samples);
  result.stats.isolated_curvature_spike_max_after_1pm =
      maxIsolatedCurvatureSpike(result.samples);
  const auto passage_insertion_started_at = std::chrono::steady_clock::now();
  PassageInsertionResult passage_insertion{};
  std::vector<TrajectoryPointSample> passage_samples;
  result.stats.grid_stages.passage_insertion_attempts = 1U;
  PassageInsertionResult attempt = insertLocalPassageSegments(
      result.samples, *risk_context->raw_occupancy, input.known_passage_map,
      config.known_passage_validation, config.passage_insertion,
      config.initial_altitude_m, input.passage_insertion_start_mode);
  const std::string candidate_name{risk_context->name};
  for (PassageInsertionDiagnostic& diagnostic : attempt.stats.diagnostics) {
    diagnostic.grid_name = candidate_name;
  }
  result.stats.passage_insertion = attempt.stats;
  const std::span<const TrajectoryPointSample> attempt_samples =
      attempt.applied ? std::span<const TrajectoryPointSample>{attempt.samples}
                      : std::span<const TrajectoryPointSample>{result.samples};
  const bool trajectory_invariants_hold =
      attempt.valid && trajectoryStageInvariantsHold(
                           attempt_samples, *risk_context->raw_occupancy,
                           input.route_points.front(), input.route_points.back());
  const bool risk_accepted =
      trajectory_invariants_hold &&
      applyRiskGate("passage_insertion", *risk_context, result.samples, attempt_samples,
                    true, result.stats);
  // Passage insertion is best-effort. An unrepaired passage is diagnostic data,
  // not a reason to suppress an otherwise executable route.
  const bool accepted = attempt.valid && trajectory_invariants_hold && risk_accepted;
  result.stats.passage_insertion_risk_attempts.push_back(PassageInsertionGridAttempt{
      .grid_name = candidate_name,
      .reason = attempt.stats.final_reason,
      .valid = attempt.valid,
      .repair_required = attempt.repair_required,
      .repair_satisfied = attempt.repair_satisfied,
      .applied = attempt.applied,
      .trajectory_invariants_hold = trajectory_invariants_hold,
      .accepted = accepted});
  if (accepted) {
    passage_samples.assign(attempt_samples.begin(), attempt_samples.end());
    result.stats.grid_stages.passage_insertion = candidate_name;
  }
  passage_insertion = std::move(attempt);
  result.stats.passage_insertion_duration_ms =
      elapsedMilliseconds(passage_insertion_started_at);
  if (input.stop_token.stop_requested()) {
    result.stats.status = TrajectoryPlannerStatus::kCanceled;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }
  result.stats.passage_insertion = passage_insertion.stats;
  result.stats.passage_insertion.hold_restart_recommended =
      passage_insertion.hold_restart_recommended;
  if (!passage_insertion.valid || passage_samples.empty()) {
    result.stats.status = TrajectoryPlannerStatus::kInvalidTrajectory;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }
  result.samples = std::move(passage_samples);
  result.compact_segments = lineTrajectoryFromSamples(result.samples);
  if (!applyVerticalProfileStage(result, input, config)) {
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }
  const auto speed_profile_started_at = std::chrono::steady_clock::now();
  result.speed_profile =
      buildTrajectorySpeedProfile(result.samples, config.speed_profile);
  result.stats.speed_profile_duration_ms =
      elapsedMilliseconds(speed_profile_started_at);
  result.stats.final_risk = evaluateTrajectoryRisk(*risk_context, result.samples);
  if (!result.stats.final_risk.hardValid()) {
    result.stats.status = TrajectoryPlannerStatus::kInvalidTrajectory;
  }
  finalizeResult(result, config);
  result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
  return result;
}

TrajectoryPlannerResult
finalizeStitchedTrajectory(const StitchedTrajectoryFinalizationInput& input,
                           const TrajectoryPlannerConfig& config) {
  const auto total_started_at = std::chrono::steady_clock::now();
  TrajectoryPlannerResult result{};
  result.stats.quality = TrajectoryQuality::kRefined;
  result.stats.input_points = input.geometry_samples.size();
  const TrajectoryRiskContext* const risk_context =
      primaryRiskContext(input.risk_contexts);
  if (!trajectorySamplesAreUsable(input.geometry_samples) || risk_context == nullptr) {
    result.stats.status = TrajectoryPlannerStatus::kInvalidRoute;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }

  result.samples.assign(input.geometry_samples.begin(), input.geometry_samples.end());
  populateTrajectorySampleGeometry(result.samples);
  result.stats.grid_stages.trajectory_validation_attempts = 1U;
  result.stats.final_risk = evaluateTrajectoryRisk(*risk_context, result.samples);
  if (!result.stats.final_risk.hardValid() ||
      !trajectoryStageInvariantsHold(result.samples, *risk_context->raw_occupancy,
                                     result.samples.front().point,
                                     result.samples.back().point)) {
    result.stats.status = TrajectoryPlannerStatus::kInvalidTrajectory;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }
  result.stats.grid_stages.trajectory_validation = std::string{risk_context->name};

  result.compact_segments = lineTrajectoryFromSamples(result.samples);
  const TrajectoryPlannerInput vertical_input{
      .route_points = {},
      .precomputed_corridor_samples = {},
      .known_passage_map = input.known_passage_map,
      .risk_contexts = input.risk_contexts,
      .passage_insertion_start_mode = input.start_mode,
  };
  if (!applyVerticalProfileStage(result, vertical_input, config)) {
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }
  const auto speed_started_at = std::chrono::steady_clock::now();
  result.speed_profile =
      buildTrajectorySpeedProfile(result.samples, config.speed_profile);
  result.stats.speed_profile_duration_ms = elapsedMilliseconds(speed_started_at);
  result.stats.status = TrajectoryPlannerStatus::kOk;
  finalizeResult(result, config);
  result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
  return result;
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
