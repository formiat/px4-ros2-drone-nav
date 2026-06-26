#include "drone_city_nav/trajectory_planner.hpp"

#include "drone_city_nav/trajectory_diagnostics.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;

struct CurveRefinementResult {
  std::vector<TrajectoryPointSample> samples;
  CurveRefinementStats stats{};
  bool valid{false};
};

[[nodiscard]] bool finite2D(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

[[nodiscard]] Point2 operator+(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x + rhs.x, lhs.y + rhs.y};
}

[[nodiscard]] Point2 operator-(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x - rhs.x, lhs.y - rhs.y};
}

[[nodiscard]] Point2 operator*(const Point2 point, const double scale) noexcept {
  return Point2{point.x * scale, point.y * scale};
}

[[nodiscard]] double dot(const Point2 lhs, const Point2 rhs) noexcept {
  return lhs.x * rhs.x + lhs.y * rhs.y;
}

[[nodiscard]] double norm(const Point2 point) noexcept {
  return std::hypot(point.x, point.y);
}

[[nodiscard]] Point2 normalized(const Point2 point) noexcept {
  const double length = norm(point);
  if (!(length > kTinyDistanceM)) {
    return Point2{};
  }
  return Point2{point.x / length, point.y / length};
}

[[nodiscard]] double sanitizedPositive(const double value, const double fallback,
                                       const double min_value,
                                       const double max_value) noexcept {
  if (!std::isfinite(value)) {
    return fallback;
  }
  return std::clamp(value, min_value, max_value);
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

[[nodiscard]] double
elapsedMilliseconds(const std::chrono::steady_clock::time_point start) {
  return static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                 std::chrono::steady_clock::now() - start)
                                 .count()) /
         1000.0;
}

[[nodiscard]] bool segmentIsTraversable(const OccupancyGrid2D& grid, const Point2 start,
                                        const Point2 end) {
  const std::optional<GridIndex> start_cell = grid.worldToCell(start);
  const std::optional<GridIndex> end_cell = grid.worldToCell(end);
  if (!start_cell.has_value() || !end_cell.has_value()) {
    return false;
  }
  const std::vector<GridIndex> cells = grid.cellsOnLine(*start_cell, *end_cell);
  return std::ranges::none_of(
      cells, [&grid](const GridIndex cell) { return grid.isProhibited(cell); });
}

[[nodiscard]] const CorridorSample*
nearestCorridorSample(const std::span<const CorridorSample> corridor_samples,
                      const Point2 point) {
  if (corridor_samples.empty()) {
    return nullptr;
  }
  const CorridorSample* best_sample = &corridor_samples.front();
  double best_distance_sq = std::numeric_limits<double>::infinity();
  for (const CorridorSample& sample : corridor_samples) {
    const Point2 delta = point - sample.center;
    const double distance_sq = dot(delta, delta);
    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      best_sample = &sample;
    }
  }
  return best_sample;
}

[[nodiscard]] bool
assignCorridorMetadata(TrajectoryPointSample& trajectory_sample,
                       const std::span<const CorridorSample> corridor_samples,
                       const double min_corridor_margin_m) {
  const CorridorSample* corridor_sample =
      nearestCorridorSample(corridor_samples, trajectory_sample.point);
  if (corridor_sample == nullptr) {
    return false;
  }

  const Point2 offset = trajectory_sample.point - corridor_sample->center;
  const double lateral_offset_m = dot(offset, corridor_sample->normal);
  trajectory_sample.left_bound_m = corridor_sample->left_bound_m;
  trajectory_sample.right_bound_m = corridor_sample->right_bound_m;
  trajectory_sample.racing_offset_m = lateral_offset_m;

  const double width_m =
      std::max(0.0, corridor_sample->left_bound_m + corridor_sample->right_bound_m);
  const double effective_margin_m =
      std::min(std::max(0.0, min_corridor_margin_m), 0.45 * width_m);
  return lateral_offset_m <= corridor_sample->left_bound_m - effective_margin_m &&
         -lateral_offset_m <= corridor_sample->right_bound_m - effective_margin_m;
}

[[nodiscard]] Point2 cubicHermite(const Point2 p0, const Point2 m0, const Point2 p1,
                                  const Point2 m1, const double t) noexcept {
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
  const double h10 = t3 - 2.0 * t2 + t;
  const double h01 = -2.0 * t3 + 3.0 * t2;
  const double h11 = t3 - t2;
  return p0 * h00 + m0 * h10 + p1 * h01 + m1 * h11;
}

[[nodiscard]] std::vector<Point2>
refinedCurvePoints(const std::span<const TrajectoryPointSample> samples,
                   const double sample_step_m, const double tangent_scale) {
  std::vector<Point2> points;
  if (samples.size() < 2U) {
    return points;
  }
  points.reserve(samples.size() * 2U);
  points.push_back(samples.front().point);

  for (std::size_t i = 0U; i + 1U < samples.size(); ++i) {
    const Point2 p0 = samples[i].point;
    const Point2 p1 = samples[i + 1U].point;
    const double chord_length_m = distance(p0, p1);
    if (!(chord_length_m > kTinyDistanceM) || !finite2D(p0) || !finite2D(p1)) {
      continue;
    }

    Point2 t0 = normalized(samples[i].tangent);
    Point2 t1 = normalized(samples[i + 1U].tangent);
    const Point2 segment_direction = normalized(p1 - p0);
    if (!(norm(t0) > kTinyDistanceM)) {
      t0 = segment_direction;
    }
    if (!(norm(t1) > kTinyDistanceM)) {
      t1 = segment_direction;
    }

    const Point2 m0 = t0 * (chord_length_m * tangent_scale);
    const Point2 m1 = t1 * (chord_length_m * tangent_scale);
    const std::size_t steps = std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::ceil(chord_length_m / sample_step_m)));
    for (std::size_t step = 1U; step <= steps; ++step) {
      const double t = static_cast<double>(step) / static_cast<double>(steps);
      points.push_back(cubicHermite(p0, m0, p1, m1, t));
    }
  }

  points.front() = samples.front().point;
  points.back() = samples.back().point;
  return points;
}

[[nodiscard]] CurveRefinementResult
refineTrajectoryCurve(const std::span<const TrajectoryPointSample> samples,
                      const std::span<const CorridorSample> corridor_samples,
                      const OccupancyGrid2D& prohibited_grid,
                      const TrajectoryPlannerConfig& config) {
  CurveRefinementResult result{};
  result.stats.input_samples = samples.size();
  result.stats.sample_step_m =
      sanitizedPositive(config.curve_refinement_sample_step_m, 1.0, 0.1, 100.0);
  result.stats.tangent_scale =
      sanitizedPositive(config.curve_refinement_tangent_scale, 0.45, 0.0, 2.0);
  result.stats.min_corridor_margin_m = sanitizedPositive(
      config.curve_refinement_min_corridor_margin_m, 0.5, 0.0, 1000.0);
  if (samples.size() < 2U || corridor_samples.empty()) {
    return result;
  }

  const std::vector<Point2> points = refinedCurvePoints(
      samples, result.stats.sample_step_m, result.stats.tangent_scale);
  if (points.size() < 2U) {
    return result;
  }

  for (std::size_t i = 1U; i < points.size(); ++i) {
    if (!segmentIsTraversable(prohibited_grid, points[i - 1U], points[i])) {
      ++result.stats.rejected_prohibited_segments;
    }
  }
  if (result.stats.rejected_prohibited_segments > 0U) {
    return result;
  }

  result.samples = trajectoryPointSamplesFromPoints(points);
  for (TrajectoryPointSample& sample : result.samples) {
    if (!assignCorridorMetadata(sample, corridor_samples,
                                result.stats.min_corridor_margin_m)) {
      ++result.stats.rejected_corridor_samples;
    }
  }
  if (result.stats.rejected_corridor_samples > 0U) {
    result.samples.clear();
    return result;
  }

  const TrajectoryShapeDiagnostics shape =
      computeTrajectoryShapeDiagnostics(result.samples);
  result.stats.output_samples = result.samples.size();
  result.stats.max_segment_length_m = shape.max_segment_length_m;
  result.stats.max_heading_delta_rad = shape.max_heading_delta_rad;
  result.stats.valid = trajectorySamplesAreUsable(result.samples);
  result.valid = result.stats.valid;
  return result;
}

[[nodiscard]] bool
validateFinalTrajectoryInvariants(const std::span<const TrajectoryPointSample> samples,
                                  const std::span<const Point2> route_points,
                                  const TrajectoryPlannerConfig& config,
                                  TrajectoryPlannerStats& stats) {
  if (samples.size() < 2U || route_points.size() < 2U) {
    return false;
  }
  stats.final_start_route_distance_m =
      distance(samples.front().point, route_points.front());
  stats.final_endpoint_route_distance_m =
      distance(samples.back().point, route_points.back());
  const TrajectoryShapeDiagnostics shape = computeTrajectoryShapeDiagnostics(samples);
  const double endpoint_tolerance_m =
      sanitizedPositive(config.final_endpoint_tolerance_m, 0.75, 0.0, 1000.0);
  const double max_segment_length_m =
      sanitizedPositive(config.final_max_segment_length_m, 2.0, 0.1, 10000.0);
  stats.final_invariants_ok =
      stats.final_start_route_distance_m <= endpoint_tolerance_m &&
      stats.final_endpoint_route_distance_m <= endpoint_tolerance_m &&
      shape.max_segment_length_m <= max_segment_length_m + kTinyDistanceM;
  return stats.final_invariants_ok;
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
                 result.speed_profile.valid && result.stats.final_invariants_ok;
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
  const auto curve_refinement_started_at = std::chrono::steady_clock::now();
  const CurveRefinementResult curve_refinement = refineTrajectoryCurve(
      turn_smoothing.samples, corridor.samples, *input.prohibited_grid, config);
  result.stats.curve_refinement_duration_ms =
      elapsedMilliseconds(curve_refinement_started_at);
  result.stats.curve_refinement = curve_refinement.stats;
  if (!curve_refinement.valid) {
    result.stats.status = TrajectoryPlannerStatus::kInvalidTrajectory;
    result.stats.total_duration_ms = elapsedMilliseconds(total_started_at);
    return result;
  }
  result.samples = curve_refinement.samples;
  if (!validateFinalTrajectoryInvariants(result.samples, input.route_points, config,
                                         result.stats)) {
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

TrajectoryPlannerResult planTrajectory(const TrajectoryPlannerInput& input,
                                       const TrajectoryPlannerConfig& config) {
  return planRacingTrajectory(input, config);
}

} // namespace drone_city_nav
