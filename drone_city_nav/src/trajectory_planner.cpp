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

[[nodiscard]] std::vector<TrajectoryPointSample>
centerlineSamplesFromCorridor(const std::span<const CorridorSample> corridor_samples) {
  std::vector<TrajectoryPointSample> samples;
  samples.reserve(corridor_samples.size());
  for (const CorridorSample& corridor_sample : corridor_samples) {
    TrajectoryPointSample sample{};
    sample.s_m = corridor_sample.s_m;
    sample.point = corridor_sample.center;
    sample.tangent = corridor_sample.tangent;
    sample.left_bound_m = corridor_sample.left_bound_m;
    sample.right_bound_m = corridor_sample.right_bound_m;
    sample.racing_offset_m = 0.0;
    samples.push_back(sample);
  }
  for (std::size_t i = 1U; i + 1U < samples.size(); ++i) {
    const Point2 previous = samples[i - 1U].point;
    const Point2 current = samples[i].point;
    const Point2 next = samples[i + 1U].point;
    const double a = distance(previous, current);
    const double b = distance(current, next);
    const double c = distance(previous, next);
    if (a > 1.0e-6 && b > 1.0e-6 && c > 1.0e-6) {
      const double signed_double_area =
          (current.x - previous.x) * (next.y - previous.y) -
          (current.y - previous.y) * (next.x - previous.x);
      samples[i].curvature_1pm = 2.0 * signed_double_area / (a * b * c);
    }
  }
  return samples;
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

TrajectoryPlannerResult planBaselineTrajectory(const TrajectoryPlannerInput& input,
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

  result.stats.input_points = input.route_points.size();
  result.stats.status = TrajectoryPlannerStatus::kOk;
  result.stats.corridor = corridor.stats;
  result.corridor_samples = corridor.samples;
  result.samples = centerlineSamplesFromCorridor(corridor.samples);
  result.compact_segments = lineTrajectoryFromSamples(result.samples);
  result.stats.racing_line.input_samples = corridor.samples.size();
  result.stats.racing_line.optimizer_samples = corridor.samples.size();
  result.stats.racing_line.output_samples = result.samples.size();
  result.stats.racing_line.centerline_length_m =
      trajectoryLengthM(result.compact_segments);
  result.stats.racing_line.final_length_m =
      result.stats.racing_line.centerline_length_m;
  result.speed_profile =
      buildTrajectorySpeedProfile(result.samples, config.speed_profile);
  const TraversalTimeEstimate estimate =
      estimateTraversalTime(result.samples, config.speed_profile, true);
  result.stats.racing_line.estimated_time_s = estimate.estimated_time_s;
  result.stats.racing_line.min_speed_limit_mps = estimate.min_speed_limit_mps;
  result.stats.racing_line.max_speed_limit_mps = estimate.max_speed_limit_mps;
  result.stats.racing_line.curvature_limited_samples =
      estimate.curvature_limited_samples;
  result.stats.racing_line.centerline_estimated_time_s = estimate.estimated_time_s;
  result.stats.racing_line.centerline_min_speed_limit_mps =
      estimate.min_speed_limit_mps;
  result.stats.racing_line.centerline_max_speed_limit_mps =
      estimate.max_speed_limit_mps;
  result.stats.racing_line.centerline_curvature_limited_samples =
      estimate.curvature_limited_samples;
  result.stats.racing_line.time_gain_s = 0.0;
  finalizeResult(result, config);
  return result;
}

TrajectoryPlannerResult planRacingTrajectory(const TrajectoryPlannerInput& input,
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
      optimizeRacingLine(corridor.samples, *input.prohibited_grid, config.racing_line,
                         config.speed_profile);
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

TrajectoryPlannerResult planTrajectory(const TrajectoryPlannerInput& input,
                                       const TrajectoryPlannerConfig& config) {
  return planRacingTrajectory(input, config);
}

} // namespace drone_city_nav
