#include "drone_city_nav/trajectory_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace drone_city_nav {
namespace {

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
  if (!result.valid && result.stats.status == TrajectoryPlannerStatus::kOk) {
    result.stats.status = TrajectoryPlannerStatus::kInvalidTrajectory;
  }
  (void)config;
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
    case TrajectoryPlannerStatus::kRacingLineInvalid:
      return "racing_line_invalid";
    case TrajectoryPlannerStatus::kInvalidTrajectory:
      return "invalid_trajectory";
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
  if (input.final_trajectory_intersects_prohibited) {
    return TrajectoryGridRebuildReason::kProhibitedIntersection;
  }
  if (!input.current_corridor_valid ||
      corridorStatsChangedMaterially(input.previous_corridor, input.current_corridor,
                                     input.corridor_width_threshold_m)) {
    return TrajectoryGridRebuildReason::kCorridorBoundsChanged;
  }
  return TrajectoryGridRebuildReason::kNone;
}

TrajectoryPlannerResult planTrajectory(const TrajectoryPlannerInput& input,
                                       const TrajectoryPlannerConfig& config) {
  TrajectoryPlannerResult result{};
  result.stats.input_points = input.route_points.size();
  if (input.route_points.size() < 2U) {
    result.stats.status = TrajectoryPlannerStatus::kInvalidRoute;
    return result;
  }
  if (input.prohibited_grid == nullptr) {
    result.stats.status = TrajectoryPlannerStatus::kMissingGrid;
    return result;
  }

  const CorridorResult corridor =
      buildCorridor(input.route_points, *input.prohibited_grid, config.corridor);
  result.corridor_samples = corridor.samples;
  if (!corridor.valid) {
    result.stats.status = TrajectoryPlannerStatus::kCorridorInvalid;
    result.stats.corridor = corridor.stats;
    return result;
  }

  const RacingLineResult racing =
      optimizeRacingLine(corridor.samples, *input.prohibited_grid, config.racing_line);
  if (!racing.valid) {
    result.stats.status = TrajectoryPlannerStatus::kRacingLineInvalid;
    result.stats.corridor = corridor.stats;
    result.stats.racing_line = racing.stats;
    return result;
  }

  result.stats.input_points = input.route_points.size();
  result.stats.status = TrajectoryPlannerStatus::kOk;
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
