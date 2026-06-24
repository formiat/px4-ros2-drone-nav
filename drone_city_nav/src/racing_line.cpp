#include "drone_city_nav/racing_line.hpp"

#include "drone_city_nav/trajectory_diagnostics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;
constexpr double kCollisionPenalty = 1.0e9;
constexpr double kOutsideGridPenalty = 1.0e9;
constexpr double kLengthOverrunPenalty = 1.0e6;

struct PathEvaluation {
  double length_m{0.0};
  std::size_t prohibited_cells{0U};
  std::size_t outside_grid_segments{0U};

  [[nodiscard]] bool traversable() const noexcept {
    return prohibited_cells == 0U && outside_grid_segments == 0U;
  }
};

struct CandidateScore {
  double score{std::numeric_limits<double>::infinity()};
  TraversalTimeEstimate traversal_time{};
};

[[nodiscard]] Point2 operator+(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x + rhs.x, lhs.y + rhs.y};
}

[[nodiscard]] Point2 operator-(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x - rhs.x, lhs.y - rhs.y};
}

[[nodiscard]] Point2 operator*(const Point2 point, const double scale) noexcept {
  return Point2{point.x * scale, point.y * scale};
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

[[nodiscard]] double cross(const Point2 lhs, const Point2 rhs) noexcept {
  return lhs.x * rhs.y - lhs.y * rhs.x;
}

[[nodiscard]] double sanitizedPositive(const double value, const double fallback,
                                       const double min_value,
                                       const double max_value) noexcept {
  if (!std::isfinite(value)) {
    return fallback;
  }
  return std::clamp(value, min_value, max_value);
}

[[nodiscard]] std::vector<Point2>
pointsFromOffsets(const std::span<const CorridorSample> corridor_samples,
                  const std::span<const double> offsets) {
  std::vector<Point2> points;
  points.reserve(corridor_samples.size());
  for (std::size_t i = 0U; i < corridor_samples.size(); ++i) {
    points.push_back(corridor_samples[i].center +
                     corridor_samples[i].normal * offsets[i]);
  }
  return points;
}

[[nodiscard]] std::vector<TrajectoryPointSample>
samplesFromPointsAndOffsets(const std::span<const CorridorSample> corridor_samples,
                            const std::span<const Point2> points,
                            const std::span<const double> offsets) {
  std::vector<TrajectoryPointSample> samples;
  if (corridor_samples.size() != points.size() || points.size() != offsets.size()) {
    return samples;
  }
  samples.reserve(points.size());
  for (std::size_t i = 0U; i < points.size(); ++i) {
    TrajectoryPointSample sample{};
    sample.point = points[i];
    sample.left_bound_m = corridor_samples[i].left_bound_m;
    sample.right_bound_m = corridor_samples[i].right_bound_m;
    sample.racing_offset_m = offsets[i];
    samples.push_back(sample);
  }
  return samples;
}

void applyOffsetDelta(std::vector<double>& offsets,
                      const std::span<const CorridorSample> corridor_samples,
                      const std::size_t center_index, const double delta_m) {
  constexpr std::array<std::pair<int, double>, 7U> kSmoothingKernel{
      {{-3, 0.125}, {-2, 0.25}, {-1, 0.5}, {0, 1.0}, {1, 0.5}, {2, 0.25}, {3, 0.125}}};
  if (offsets.size() != corridor_samples.size() || offsets.size() <= 2U) {
    return;
  }

  for (const auto& [relative_index, weight] : kSmoothingKernel) {
    if (relative_index < 0 &&
        center_index < static_cast<std::size_t>(-relative_index)) {
      continue;
    }
    const std::size_t index =
        relative_index < 0 ? center_index - static_cast<std::size_t>(-relative_index)
                           : center_index + static_cast<std::size_t>(relative_index);
    if (index == 0U || index + 1U >= offsets.size()) {
      continue;
    }
    offsets[index] = std::clamp(offsets[index] + delta_m * weight,
                                -corridor_samples[index].right_bound_m,
                                corridor_samples[index].left_bound_m);
  }
}

enum class InitialOffsetSeed : std::uint8_t {
  kCenterline,
  kCorridorMidline,
  kLeftBiased,
  kRightBiased,
};

[[nodiscard]] double offsetForSeed(const CorridorSample& sample,
                                   const InitialOffsetSeed seed) noexcept {
  switch (seed) {
    case InitialOffsetSeed::kCenterline:
      return 0.0;
    case InitialOffsetSeed::kCorridorMidline:
      return std::clamp(0.5 * (sample.left_bound_m - sample.right_bound_m),
                        -sample.right_bound_m, sample.left_bound_m);
    case InitialOffsetSeed::kLeftBiased:
      return 0.75 * sample.left_bound_m;
    case InitialOffsetSeed::kRightBiased:
      return -0.75 * sample.right_bound_m;
  }
  return 0.0;
}

[[nodiscard]] std::vector<double>
offsetsFromSeed(const std::span<const CorridorSample> corridor_samples,
                const InitialOffsetSeed seed) {
  std::vector<double> offsets(corridor_samples.size(), 0.0);
  if (corridor_samples.size() <= 2U) {
    return offsets;
  }
  for (std::size_t i = 1U; i + 1U < corridor_samples.size(); ++i) {
    offsets[i] = offsetForSeed(corridor_samples[i], seed);
  }
  return offsets;
}

[[nodiscard]] std::vector<CorridorSample>
optimizerCorridorSamples(const std::span<const CorridorSample> corridor_samples,
                         const RacingLineConfig& config) {
  const double sample_step_m =
      sanitizedPositive(config.optimizer_sample_step_m, 0.0, 0.0, 5000.0);
  if (!(sample_step_m > kTinyDistanceM) || corridor_samples.size() <= 2U) {
    return std::vector<CorridorSample>{corridor_samples.begin(),
                                       corridor_samples.end()};
  }

  std::vector<CorridorSample> samples;
  samples.reserve(corridor_samples.size());
  samples.push_back(corridor_samples.front());
  double last_s_m = corridor_samples.front().s_m;
  for (std::size_t i = 1U; i + 1U < corridor_samples.size(); ++i) {
    if (corridor_samples[i].s_m - last_s_m + kTinyDistanceM < sample_step_m) {
      continue;
    }
    samples.push_back(corridor_samples[i]);
    last_s_m = corridor_samples[i].s_m;
  }
  if (distance(samples.back().center, corridor_samples.back().center) >
      kTinyDistanceM) {
    samples.push_back(corridor_samples.back());
  }
  return samples;
}

[[nodiscard]] double pathLength(const std::span<const Point2> points) {
  double length = 0.0;
  for (std::size_t i = 1U; i < points.size(); ++i) {
    length += distance(points[i - 1U], points[i]);
  }
  return length;
}

[[nodiscard]] double discreteCurvature(const Point2 previous, const Point2 current,
                                       const Point2 next) {
  const double a = distance(previous, current);
  const double b = distance(current, next);
  const double c = distance(previous, next);
  if (!(a > kTinyDistanceM) || !(b > kTinyDistanceM) || !(c > kTinyDistanceM)) {
    return 0.0;
  }
  const double signed_double_area = cross(current - previous, next - previous);
  return 2.0 * signed_double_area / (a * b * c);
}

void populateSampleGeometry(std::vector<TrajectoryPointSample>& samples);

[[nodiscard]] PathEvaluation evaluatePath(const OccupancyGrid2D& grid,
                                          const std::span<const Point2> points) {
  PathEvaluation evaluation{};
  if (points.size() < 2U) {
    ++evaluation.outside_grid_segments;
    return evaluation;
  }

  for (std::size_t i = 1U; i < points.size(); ++i) {
    evaluation.length_m += distance(points[i - 1U], points[i]);
    const Point2 start = points[i - 1U];
    const Point2 end = points[i];
    const std::optional<GridIndex> start_cell = grid.worldToCell(start);
    const std::optional<GridIndex> end_cell = grid.worldToCell(end);
    if (!start_cell.has_value() || !end_cell.has_value()) {
      ++evaluation.outside_grid_segments;
      continue;
    }
    const std::vector<GridIndex> cells = grid.cellsOnLine(*start_cell, *end_cell);
    for (const GridIndex cell : cells) {
      if (grid.isProhibited(cell)) {
        ++evaluation.prohibited_cells;
      }
    }
  }
  return evaluation;
}

[[nodiscard]] double costForPoints(const std::span<const Point2> points,
                                   const std::span<const double> offsets,
                                   const RacingLineConfig& config) {
  if (points.size() < 2U) {
    return std::numeric_limits<double>::infinity();
  }

  const double weight_length = sanitizedPositive(config.weight_length, 1.0, 0.0, 1.0e6);
  const double weight_curvature =
      sanitizedPositive(config.weight_curvature, 25.0, 0.0, 1.0e9);
  const double weight_curvature_change =
      sanitizedPositive(config.weight_curvature_change, 10.0, 0.0, 1.0e9);
  const double weight_offset_change =
      sanitizedPositive(config.weight_offset_change, 1.0, 0.0, 1.0e9);
  const double weight_offset_second_change =
      sanitizedPositive(config.weight_offset_second_change, 10.0, 0.0, 1.0e9);
  const double weight_center_bias =
      sanitizedPositive(config.weight_center_bias, 0.02, 0.0, 1.0e6);

  double curvature_cost = 0.0;
  double curvature_change_cost = 0.0;
  double offset_change_cost = 0.0;
  double offset_second_change_cost = 0.0;
  double center_bias_cost = 0.0;
  double previous_curvature = 0.0;
  bool previous_curvature_valid = false;
  for (const double offset : offsets) {
    center_bias_cost += offset * offset;
  }
  for (std::size_t i = 1U; i < offsets.size(); ++i) {
    const double change = offsets[i] - offsets[i - 1U];
    offset_change_cost += change * change;
  }
  for (std::size_t i = 1U; i + 1U < offsets.size(); ++i) {
    const double second_change = offsets[i + 1U] - 2.0 * offsets[i] + offsets[i - 1U];
    offset_second_change_cost += second_change * second_change;
  }
  for (std::size_t i = 1U; i + 1U < points.size(); ++i) {
    const double curvature =
        discreteCurvature(points[i - 1U], points[i], points[i + 1U]);
    curvature_cost += curvature * curvature;
    if (previous_curvature_valid) {
      const double change = curvature - previous_curvature;
      curvature_change_cost += change * change;
    }
    previous_curvature = curvature;
    previous_curvature_valid = true;
  }

  return weight_length * pathLength(points) + weight_curvature * curvature_cost +
         weight_curvature_change * curvature_change_cost +
         weight_offset_change * offset_change_cost +
         weight_offset_second_change * offset_second_change_cost +
         weight_center_bias * center_bias_cost;
}

[[nodiscard]] CandidateScore scoreForCandidate(
    const std::span<const CorridorSample> corridor_samples,
    const std::span<const Point2> points, const std::span<const double> offsets,
    const PathEvaluation& evaluation, const RacingLineConfig& config,
    const VelocityFollowerConfig& speed_config, const double max_length_m) {
  CandidateScore result{};
  double score = costForPoints(points, offsets, config);
  score += static_cast<double>(evaluation.prohibited_cells) * kCollisionPenalty;
  score += static_cast<double>(evaluation.outside_grid_segments) * kOutsideGridPenalty;
  if (std::isfinite(max_length_m) && evaluation.length_m > max_length_m) {
    const double overrun_m = evaluation.length_m - max_length_m;
    score += overrun_m * overrun_m * kLengthOverrunPenalty;
  }
  const double weight_time = sanitizedPositive(config.weight_time, 0.0, 0.0, 1.0e9);
  if (evaluation.traversable()) {
    std::vector<TrajectoryPointSample> samples =
        samplesFromPointsAndOffsets(corridor_samples, points, offsets);
    populateSampleGeometry(samples);
    result.traversal_time = estimateTraversalTime(samples, speed_config, true);
    if (weight_time > 0.0 && result.traversal_time.valid &&
        std::isfinite(result.traversal_time.estimated_time_s)) {
      score += weight_time * result.traversal_time.estimated_time_s;
    }
  }
  result.score = score;
  return result;
}

void populateSampleGeometry(std::vector<TrajectoryPointSample>& samples) {
  double s_m = 0.0;
  for (std::size_t i = 0U; i < samples.size(); ++i) {
    if (i > 0U) {
      s_m += distance(samples[i - 1U].point, samples[i].point);
    }
    samples[i].s_m = s_m;
    if (samples.size() == 1U) {
      samples[i].tangent = Point2{1.0, 0.0};
    } else if (i == 0U) {
      samples[i].tangent = normalized(samples[i + 1U].point - samples[i].point);
    } else if (i + 1U == samples.size()) {
      samples[i].tangent = normalized(samples[i].point - samples[i - 1U].point);
    } else {
      samples[i].tangent = normalized(samples[i + 1U].point - samples[i - 1U].point);
      samples[i].curvature_1pm = discreteCurvature(
          samples[i - 1U].point, samples[i].point, samples[i + 1U].point);
    }
  }
}

void updateCurvatureStats(const std::span<const TrajectoryPointSample> samples,
                          RacingLineStats& stats) {
  double curvature_sum = 0.0;
  std::size_t curvature_count = 0U;
  for (const TrajectoryPointSample& sample : samples) {
    const double abs_curvature = std::abs(sample.curvature_1pm);
    stats.max_abs_curvature_1pm = std::max(stats.max_abs_curvature_1pm, abs_curvature);
    curvature_sum += abs_curvature;
    ++curvature_count;
  }
  if (curvature_count > 0U) {
    stats.mean_abs_curvature_1pm = curvature_sum / static_cast<double>(curvature_count);
  }
}

void copyTraversalEstimateToFinalStats(const TraversalTimeEstimate& estimate,
                                       RacingLineStats& stats) {
  stats.estimated_time_s = estimate.estimated_time_s;
  stats.min_speed_limit_mps = estimate.min_speed_limit_mps;
  stats.max_speed_limit_mps = estimate.max_speed_limit_mps;
  stats.curvature_limited_samples = estimate.curvature_limited_samples;
}

void copyTraversalEstimateToCenterlineStats(const TraversalTimeEstimate& estimate,
                                            RacingLineStats& stats) {
  stats.centerline_estimated_time_s = estimate.estimated_time_s;
  stats.centerline_min_speed_limit_mps = estimate.min_speed_limit_mps;
  stats.centerline_max_speed_limit_mps = estimate.max_speed_limit_mps;
  stats.centerline_curvature_limited_samples = estimate.curvature_limited_samples;
}

void copyTraversalEstimateToBestCandidateStats(const TraversalTimeEstimate& estimate,
                                               const double score,
                                               RacingLineStats& stats) {
  stats.best_candidate_estimated_time_s = estimate.estimated_time_s;
  stats.best_candidate_score = score;
  stats.best_candidate_min_speed_limit_mps = estimate.min_speed_limit_mps;
  stats.best_candidate_max_speed_limit_mps = estimate.max_speed_limit_mps;
  stats.best_candidate_curvature_limited_samples = estimate.curvature_limited_samples;
}

[[nodiscard]] bool updateBestCandidate(
    const std::span<const CorridorSample> corridor_samples,
    const std::span<const double> candidate_offsets,
    const std::span<const Point2> candidate_points,
    const OccupancyGrid2D& prohibited_grid, const RacingLineConfig& config,
    const VelocityFollowerConfig& speed_config, const double max_length_m,
    double& best_cost, std::vector<double>& offsets, std::vector<Point2>& best_points,
    RacingLineStats& stats) {
  ++stats.candidate_evaluations;
  const PathEvaluation evaluation = evaluatePath(prohibited_grid, candidate_points);
  if (!evaluation.traversable()) {
    ++stats.collision_rejections;
  }
  const CandidateScore candidate_score =
      scoreForCandidate(corridor_samples, candidate_points, candidate_offsets,
                        evaluation, config, speed_config, max_length_m);
  if (candidate_score.score + 1.0e-9 < best_cost) {
    best_cost = candidate_score.score;
    offsets.assign(candidate_offsets.begin(), candidate_offsets.end());
    best_points.assign(candidate_points.begin(), candidate_points.end());
    copyTraversalEstimateToBestCandidateStats(candidate_score.traversal_time,
                                              candidate_score.score, stats);
    return true;
  }
  return false;
}

[[nodiscard]] std::vector<double>
smoothedOffsets(const std::span<const double> offsets,
                const std::span<const CorridorSample> corridor_samples) {
  std::vector<double> smoothed{offsets.begin(), offsets.end()};
  if (offsets.size() <= 2U || offsets.size() != corridor_samples.size()) {
    return smoothed;
  }
  for (std::size_t i = 1U; i + 1U < offsets.size(); ++i) {
    const double value =
        0.25 * offsets[i - 1U] + 0.5 * offsets[i] + 0.25 * offsets[i + 1U];
    smoothed[i] = std::clamp(value, -corridor_samples[i].right_bound_m,
                             corridor_samples[i].left_bound_m);
  }
  return smoothed;
}

} // namespace

RacingLineResult
optimizeRacingLine(const std::span<const CorridorSample> corridor_samples,
                   const OccupancyGrid2D& prohibited_grid,
                   const RacingLineConfig& config,
                   const VelocityFollowerConfig& speed_config) {
  RacingLineResult result{};
  result.stats.input_samples = corridor_samples.size();
  if (!config.enabled || corridor_samples.size() < 2U) {
    return result;
  }

  const std::vector<CorridorSample> optimizer_samples =
      optimizerCorridorSamples(corridor_samples, config);
  const std::size_t sample_count = optimizer_samples.size();
  result.stats.optimizer_samples = sample_count;
  const std::vector<double> zero_offsets(sample_count, 0.0);
  const std::vector<Point2> centerline =
      pointsFromOffsets(optimizer_samples, zero_offsets);
  result.stats.centerline_length_m = pathLength(centerline);
  std::vector<TrajectoryPointSample> centerline_samples =
      samplesFromPointsAndOffsets(optimizer_samples, centerline, zero_offsets);
  populateSampleGeometry(centerline_samples);
  copyTraversalEstimateToCenterlineStats(
      estimateTraversalTime(centerline_samples, speed_config, true), result.stats);

  const double min_step =
      sanitizedPositive(config.min_offset_step_m, 0.1, 0.001, 100.0);
  const double cooling = sanitizedPositive(config.cooling_ratio, 0.5, 0.05, 0.95);
  double step = std::max(
      min_step, sanitizedPositive(config.initial_offset_step_m, 2.0, 0.001, 500.0));
  const std::size_t max_iterations = std::clamp<std::size_t>(
      config.max_iterations, 1U, static_cast<std::size_t>(10000U));
  const double max_length_ratio =
      sanitizedPositive(config.max_length_ratio, 1.25, 1.0, 100.0);
  const double max_length_m = result.stats.centerline_length_m * max_length_ratio;

  std::vector<double> offsets;
  std::vector<Point2> best_points;
  double best_cost = std::numeric_limits<double>::infinity();
  constexpr std::array kInitialSeeds{
      InitialOffsetSeed::kCenterline, InitialOffsetSeed::kCorridorMidline,
      InitialOffsetSeed::kLeftBiased, InitialOffsetSeed::kRightBiased};
  for (const InitialOffsetSeed seed : kInitialSeeds) {
    std::vector<double> candidate_offsets = offsetsFromSeed(optimizer_samples, seed);
    std::vector<Point2> candidate_points =
        pointsFromOffsets(optimizer_samples, candidate_offsets);
    (void)updateBestCandidate(optimizer_samples, candidate_offsets, candidate_points,
                              prohibited_grid, config, speed_config, max_length_m,
                              best_cost, offsets, best_points, result.stats);
  }
  if (offsets.empty()) {
    return result;
  }
  result.stats.initial_cost = best_cost;

  for (std::size_t iteration = 0U; iteration < max_iterations && step >= min_step;
       ++iteration) {
    bool changed = false;
    for (std::size_t i = 1U; i + 1U < sample_count; ++i) {
      std::vector<double> best_offsets = offsets;
      for (const double delta : {-step, 0.0, step}) {
        std::vector<double> candidate_offsets = offsets;
        applyOffsetDelta(candidate_offsets, optimizer_samples, i, delta);
        std::vector<Point2> candidate_points =
            pointsFromOffsets(optimizer_samples, candidate_offsets);
        std::vector<double> accepted_offsets;
        std::vector<Point2> accepted_points;
        const bool accepted = updateBestCandidate(
            optimizer_samples, candidate_offsets, candidate_points, prohibited_grid,
            config, speed_config, max_length_m, best_cost, accepted_offsets,
            accepted_points, result.stats);
        if (accepted) {
          best_offsets = std::move(accepted_offsets);
          best_points = std::move(accepted_points);
          changed = true;
        }
      }
      offsets = std::move(best_offsets);
    }
    ++result.stats.iterations;
    if (!changed) {
      step *= cooling;
    }
  }

  std::vector<Point2> final_points = std::move(best_points);
  if (final_points.empty()) {
    final_points = pointsFromOffsets(optimizer_samples, offsets);
  }
  std::vector<TrajectoryPointSample> pre_regularization_samples =
      samplesFromPointsAndOffsets(optimizer_samples, final_points, offsets);
  populateSampleGeometry(pre_regularization_samples);
  const TrajectoryShapeDiagnostics pre_diagnostics =
      computeTrajectoryShapeDiagnostics(pre_regularization_samples);
  result.stats.pre_regularization_max_curvature_jump_1pm =
      pre_diagnostics.max_curvature_jump_1pm;
  const TraversalTimeEstimate best_time =
      estimateTraversalTime(pre_regularization_samples, speed_config, true);

  std::vector<double> final_offsets = offsets;
  const std::size_t regularization_iterations = std::clamp<std::size_t>(
      config.regularization_iterations, 0U, static_cast<std::size_t>(100U));
  for (std::size_t iteration = 0U; iteration < regularization_iterations; ++iteration) {
    std::vector<double> candidate_offsets =
        smoothedOffsets(final_offsets, optimizer_samples);
    std::vector<Point2> candidate_points =
        pointsFromOffsets(optimizer_samples, candidate_offsets);
    const PathEvaluation candidate_evaluation =
        evaluatePath(prohibited_grid, candidate_points);
    if (!candidate_evaluation.traversable()) {
      ++result.stats.collision_rejections;
      break;
    }
    std::vector<TrajectoryPointSample> candidate_samples = samplesFromPointsAndOffsets(
        optimizer_samples, candidate_points, candidate_offsets);
    populateSampleGeometry(candidate_samples);
    const TrajectoryShapeDiagnostics candidate_diagnostics =
        computeTrajectoryShapeDiagnostics(candidate_samples);
    const TraversalTimeEstimate candidate_time =
        estimateTraversalTime(candidate_samples, speed_config, true);
    const double max_regression = sanitizedPositive(
        config.regularization_max_time_regression_s, 0.5, 0.0, 3600.0);
    const bool time_acceptable =
        !best_time.valid || !candidate_time.valid ||
        candidate_time.estimated_time_s <= best_time.estimated_time_s + max_regression;
    if (candidate_diagnostics.max_curvature_jump_1pm + 1.0e-9 >=
            pre_diagnostics.max_curvature_jump_1pm ||
        !time_acceptable) {
      break;
    }
    final_offsets = std::move(candidate_offsets);
    final_points = std::move(candidate_points);
    result.stats.regularization_applied = true;
    ++result.stats.regularization_iterations;
  }

  const PathEvaluation final_evaluation = evaluatePath(prohibited_grid, final_points);
  if (!final_evaluation.traversable()) {
    ++result.stats.collision_rejections;
    return result;
  }

  result.samples.reserve(sample_count);
  for (std::size_t i = 0U; i < sample_count; ++i) {
    TrajectoryPointSample sample{};
    sample.point = final_points[i];
    sample.left_bound_m = optimizer_samples[i].left_bound_m;
    sample.right_bound_m = optimizer_samples[i].right_bound_m;
    sample.racing_offset_m = final_offsets[i];
    result.stats.max_abs_offset_m =
        std::max(result.stats.max_abs_offset_m, std::abs(final_offsets[i]));
    result.samples.push_back(sample);
  }
  populateSampleGeometry(result.samples);
  result.stats.output_samples = result.samples.size();
  result.stats.final_length_m = pathLength(final_points);
  result.stats.final_cost = best_cost;
  const TrajectoryShapeDiagnostics post_diagnostics =
      computeTrajectoryShapeDiagnostics(result.samples);
  result.stats.post_regularization_max_curvature_jump_1pm =
      post_diagnostics.max_curvature_jump_1pm;
  const TraversalTimeEstimate final_time =
      estimateTraversalTime(result.samples, speed_config, true);
  copyTraversalEstimateToFinalStats(final_time, result.stats);
  if (std::isfinite(result.stats.centerline_estimated_time_s) &&
      std::isfinite(result.stats.estimated_time_s)) {
    result.stats.time_gain_s =
        result.stats.centerline_estimated_time_s - result.stats.estimated_time_s;
  }
  if (std::isfinite(result.stats.best_candidate_estimated_time_s) &&
      std::isfinite(result.stats.estimated_time_s)) {
    result.stats.regularization_time_delta_s =
        result.stats.estimated_time_s - result.stats.best_candidate_estimated_time_s;
  }
  updateCurvatureStats(result.samples, result.stats);
  result.valid = trajectorySamplesAreUsable(result.samples);
  return result;
}

} // namespace drone_city_nav
