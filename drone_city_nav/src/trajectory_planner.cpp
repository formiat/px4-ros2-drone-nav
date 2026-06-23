#include "drone_city_nav/trajectory_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::vector<TrajectoryPointSample>
fallbackSamplesFromSegments(const std::span<const TrajectorySegment> segments,
                            const double sample_step_m) {
  return sampleTrajectoryDetailed(segments, sample_step_m);
}

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
}

[[nodiscard]] bool materiallyDifferent(const double lhs, const double rhs,
                                       const double threshold) noexcept {
  if (!std::isfinite(lhs) || !std::isfinite(rhs)) {
    return std::isfinite(lhs) != std::isfinite(rhs);
  }
  return std::abs(lhs - rhs) > threshold;
}

[[nodiscard]] bool corridorStatsChangedMaterially(const CorridorStats& previous,
                                                  const CorridorStats& current,
                                                  const double threshold) noexcept {
  if (previous.samples == 0U) {
    return true;
  }
  if (current.samples != previous.samples ||
      current.route_prohibited_samples != previous.route_prohibited_samples) {
    return true;
  }
  return materiallyDifferent(current.min_width_m, previous.min_width_m, threshold) ||
         materiallyDifferent(current.mean_width_m, previous.mean_width_m, threshold) ||
         materiallyDifferent(current.max_width_m, previous.max_width_m, threshold) ||
         materiallyDifferent(current.min_clearance_m, previous.min_clearance_m,
                             threshold);
}

void finalizeResult(TrajectoryPlannerResult& result,
                    const TrajectoryPlannerConfig& config) {
  const TrajectoryMetrics metrics = trajectoryMetrics(result.compact_segments);
  result.stats.compact_segments = result.compact_segments.size();
  result.stats.line_segments = metrics.line_segments;
  result.stats.arc_segments = metrics.arc_segments;
  result.stats.length_m = metrics.length_m;
  result.stats.samples = result.samples.size();
  computeCurvatureStats(result.samples, result.stats);
  computeSpeedProfileStats(result.speed_profile, result.stats);
  result.valid = trajectoryIsUsable(result.compact_segments) &&
                 trajectorySamplesAreUsable(result.samples) &&
                 result.speed_profile.valid;
  if (!result.valid &&
      result.stats.fallback_reason == TrajectoryPlannerFallbackReason::kNone) {
    result.stats.fallback_reason = TrajectoryPlannerFallbackReason::kBaselineInvalid;
  }
  (void)config;
}

[[nodiscard]] TrajectoryPlannerResult
baselineTrajectory(const std::span<const Point2> route_points,
                   const OccupancyGrid2D* prohibited_grid,
                   const TrajectoryPlannerConfig& config,
                   const TrajectoryPlannerFallbackReason reason) {
  TrajectoryPlannerResult result{};
  result.stats.input_points = route_points.size();
  result.stats.fallback_reason = reason;
  if (prohibited_grid == nullptr) {
    result.compact_segments = lineTrajectoryFromPoints(route_points);
  } else {
    const CornerRoundingResult rounded =
        roundCorners(route_points, config.baseline_rounding, prohibited_grid);
    result.compact_segments = rounded.segments;
    result.stats.baseline_rounding = rounded.stats;
    if (!trajectoryIsUsable(result.compact_segments)) {
      result.compact_segments = lineTrajectoryFromPoints(route_points);
      result.stats.fallback_reason = TrajectoryPlannerFallbackReason::kBaselineInvalid;
    }
  }
  if (!trajectoryIsUsable(result.compact_segments)) {
    result.stats.fallback_reason = TrajectoryPlannerFallbackReason::kBaselineInvalid;
  }
  result.samples =
      fallbackSamplesFromSegments(result.compact_segments, config.debug_sample_step_m);
  result.speed_profile =
      buildTrajectorySpeedProfile(result.samples, config.speed_profile);
  finalizeResult(result, config);
  return result;
}

} // namespace

std::string_view trajectoryPlannerFallbackReasonName(
    const TrajectoryPlannerFallbackReason reason) noexcept {
  switch (reason) {
    case TrajectoryPlannerFallbackReason::kNone:
      return "none";
    case TrajectoryPlannerFallbackReason::kInvalidRoute:
      return "invalid_route";
    case TrajectoryPlannerFallbackReason::kMissingGrid:
      return "missing_grid";
    case TrajectoryPlannerFallbackReason::kRacingDisabled:
      return "racing_disabled";
    case TrajectoryPlannerFallbackReason::kCorridorInvalid:
      return "corridor_invalid";
    case TrajectoryPlannerFallbackReason::kRacingLineInvalid:
      return "racing_line_invalid";
    case TrajectoryPlannerFallbackReason::kBaselineInvalid:
      return "baseline_invalid";
  }
  return "unknown";
}

std::string_view
trajectoryGridRebuildReasonName(const TrajectoryGridRebuildReason reason) noexcept {
  switch (reason) {
    case TrajectoryGridRebuildReason::kNone:
      return "none";
    case TrajectoryGridRebuildReason::kInvalidTrajectory:
      return "invalid_trajectory";
    case TrajectoryGridRebuildReason::kMissingGridFallback:
      return "missing_grid_fallback";
    case TrajectoryGridRebuildReason::kProhibitedIntersection:
      return "prohibited_intersection";
    case TrajectoryGridRebuildReason::kCorridorBoundsChanged:
      return "corridor_bounds_changed";
  }
  return "unknown";
}

TrajectoryGridRebuildReason
trajectoryGridRebuildReason(const TrajectoryGridRebuildDecisionInput& input) noexcept {
  if (!input.trajectory_valid) {
    return TrajectoryGridRebuildReason::kInvalidTrajectory;
  }
  if (input.fallback_reason == TrajectoryPlannerFallbackReason::kMissingGrid) {
    return TrajectoryGridRebuildReason::kMissingGridFallback;
  }
  if (input.final_trajectory_intersects_prohibited) {
    return TrajectoryGridRebuildReason::kProhibitedIntersection;
  }
  if (input.racing_trajectory_enabled) {
    if (!input.current_corridor_valid ||
        corridorStatsChangedMaterially(input.previous_corridor, input.current_corridor,
                                       input.corridor_width_threshold_m)) {
      return TrajectoryGridRebuildReason::kCorridorBoundsChanged;
    }
  }
  return TrajectoryGridRebuildReason::kNone;
}

TrajectoryPlannerResult planTrajectory(const TrajectoryPlannerInput& input,
                                       const TrajectoryPlannerConfig& config) {
  TrajectoryPlannerResult result{};
  result.stats.input_points = input.route_points.size();
  if (input.route_points.size() < 2U) {
    result.stats.fallback_reason = TrajectoryPlannerFallbackReason::kInvalidRoute;
    return result;
  }
  if (input.prohibited_grid == nullptr) {
    return baselineTrajectory(input.route_points, nullptr, config,
                              TrajectoryPlannerFallbackReason::kMissingGrid);
  }
  if (!config.racing_trajectory_enabled) {
    return baselineTrajectory(input.route_points, input.prohibited_grid, config,
                              TrajectoryPlannerFallbackReason::kRacingDisabled);
  }

  const CorridorResult corridor =
      buildCorridor(input.route_points, *input.prohibited_grid, config.corridor);
  if (!corridor.valid) {
    TrajectoryPlannerResult fallback =
        baselineTrajectory(input.route_points, input.prohibited_grid, config,
                           TrajectoryPlannerFallbackReason::kCorridorInvalid);
    fallback.stats.corridor = corridor.stats;
    return fallback;
  }

  const RacingLineResult racing =
      optimizeRacingLine(corridor.samples, *input.prohibited_grid, config.racing_line);
  if (!racing.valid) {
    TrajectoryPlannerResult fallback =
        baselineTrajectory(input.route_points, input.prohibited_grid, config,
                           TrajectoryPlannerFallbackReason::kRacingLineInvalid);
    fallback.stats.corridor = corridor.stats;
    fallback.stats.racing_line = racing.stats;
    return fallback;
  }

  result.stats.input_points = input.route_points.size();
  result.stats.fallback_reason = TrajectoryPlannerFallbackReason::kNone;
  result.stats.corridor = corridor.stats;
  result.stats.racing_line = racing.stats;
  result.samples = racing.samples;
  result.compact_segments = lineTrajectoryFromSamples(result.samples);
  result.speed_profile =
      buildTrajectorySpeedProfile(result.samples, config.speed_profile);
  finalizeResult(result, config);
  return result;
}

} // namespace drone_city_nav
