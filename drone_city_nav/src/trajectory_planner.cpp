#include "drone_city_nav/trajectory_planner.hpp"

#include <algorithm>
#include <chrono>
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

[[nodiscard]] double
elapsedMilliseconds(const std::chrono::steady_clock::time_point start) {
  return static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                 std::chrono::steady_clock::now() - start)
                                 .count()) /
         1000.0;
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

TrajectoryPlannerResult planRacingTrajectory(const TrajectoryPlannerInput& input,
                                             const TrajectoryPlannerConfig& config) {
  const auto total_started_at = std::chrono::steady_clock::now();
  TrajectoryPlannerResult result{};
  result.stats.input_points = input.route_points.size();
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
      buildCorridor(input.route_points, *input.prohibited_grid, config.corridor);
  result.stats.corridor_duration_ms = elapsedMilliseconds(corridor_started_at);
  result.corridor_samples = corridor.samples;
  if (!corridor.valid) {
    result.stats.status = TrajectoryPlannerStatus::kCorridorInvalid;
    result.stats.corridor = corridor.stats;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }

  const auto racing_line_started_at = std::chrono::steady_clock::now();
  const RacingLineResult racing =
      optimizeRacingLine(corridor.samples, *input.prohibited_grid, config.racing_line,
                         config.speed_profile);
  result.stats.racing_line_duration_ms = elapsedMilliseconds(racing_line_started_at);
  if (!racing.valid) {
    result.stats.status = TrajectoryPlannerStatus::kRacingLineInvalid;
    result.stats.corridor = corridor.stats;
    result.stats.racing_line = racing.stats;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }

  result.stats.input_points = input.route_points.size();
  result.stats.status = TrajectoryPlannerStatus::kOk;
  result.stats.corridor = corridor.stats;
  result.stats.racing_line = racing.stats;
  const auto straightening_started_at = std::chrono::steady_clock::now();
  const TrajectoryStraighteningResult straightening = straightenTrajectory(
      racing.samples, corridor.samples, *input.prohibited_grid, config.straightening);
  result.stats.straightening_duration_ms =
      elapsedMilliseconds(straightening_started_at);
  result.stats.straightening = straightening.stats;
  if (!straightening.valid) {
    result.stats.status = TrajectoryPlannerStatus::kInvalidTrajectory;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }
  const auto turn_smoothing_started_at = std::chrono::steady_clock::now();
  const TurnSmoothingResult turn_smoothing =
      smoothTrajectoryTurns(straightening.samples, corridor.samples,
                            *input.prohibited_grid, config.turn_smoothing);
  result.stats.turn_smoothing_duration_ms =
      elapsedMilliseconds(turn_smoothing_started_at);
  result.stats.turn_smoothing = turn_smoothing.stats;
  if (!turn_smoothing.valid) {
    result.stats.status = TrajectoryPlannerStatus::kInvalidTrajectory;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }
  result.samples = turn_smoothing.samples;
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

TrajectoryPlannerResult planTrajectory(const TrajectoryPlannerInput& input,
                                       const TrajectoryPlannerConfig& config) {
  return planRacingTrajectory(input, config);
}

} // namespace drone_city_nav
