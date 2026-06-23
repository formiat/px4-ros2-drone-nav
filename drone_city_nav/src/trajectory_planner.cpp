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
  const CornerRoundingResult rounded =
      roundCorners(route_points, config.baseline_rounding, prohibited_grid);
  result.compact_segments = rounded.segments;
  result.stats.baseline_rounding = rounded.stats;
  if (!trajectoryIsUsable(result.compact_segments)) {
    result.compact_segments = lineTrajectoryFromPoints(route_points);
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
