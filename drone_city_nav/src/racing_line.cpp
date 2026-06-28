#include "drone_city_nav/racing_line.hpp"

#include "drone_city_nav/trajectory_diagnostics.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
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

struct CostBreakdown {
  double length_cost{0.0};
  double time_cost{0.0};
  double curvature_cost{0.0};
  double curvature_change_cost{0.0};
  double offset_change_cost{0.0};
  double offset_second_change_cost{0.0};
  double center_bias_cost{0.0};
  double edge_margin_cost{0.0};
  double collision_cost{0.0};
  double outside_grid_cost{0.0};
  double length_overrun_cost{0.0};

  [[nodiscard]] double total() const noexcept {
    return length_cost + time_cost + curvature_cost + curvature_change_cost +
           offset_change_cost + offset_second_change_cost + center_bias_cost +
           edge_margin_cost + collision_cost + outside_grid_cost + length_overrun_cost;
  }
};

struct CandidateScore {
  double score{std::numeric_limits<double>::infinity()};
  TraversalTimeEstimate traversal_time{};
  CostBreakdown breakdown{};
};

struct EvaluatedCandidate {
  bool noop{false};
  std::vector<double> offsets;
  std::vector<Point2> points;
  PathEvaluation path{};
  CandidateScore score{};
  double point_build_duration_ms{0.0};
  double path_evaluation_duration_ms{0.0};
  double score_duration_ms{0.0};
  double sample_build_duration_ms{0.0};
};

struct RacingLineScratch {
  std::vector<double> candidate_offsets;
  std::vector<double> accepted_offsets;
  std::vector<double> iteration_best_offsets;
  std::vector<double> smoothed_offsets;
  std::vector<Point2> candidate_points;
  std::vector<Point2> accepted_points;
  std::vector<TrajectoryPointSample> candidate_samples;
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

[[nodiscard]] double
elapsedMilliseconds(const std::chrono::steady_clock::time_point start) {
  return static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                 std::chrono::steady_clock::now() - start)
                                 .count()) /
         1000.0;
}

void pointsFromOffsets(const std::span<const CorridorSample> corridor_samples,
                       const std::span<const double> offsets,
                       std::vector<Point2>& points) {
  points.clear();
  points.reserve(corridor_samples.size());
  if (corridor_samples.size() != offsets.size()) {
    return;
  }
  for (std::size_t i = 0U; i < corridor_samples.size(); ++i) {
    points.push_back(corridor_samples[i].center +
                     corridor_samples[i].normal * offsets[i]);
  }
}

[[nodiscard]] std::vector<Point2>
pointsFromOffsets(const std::span<const CorridorSample> corridor_samples,
                  const std::span<const double> offsets) {
  std::vector<Point2> points;
  pointsFromOffsets(corridor_samples, offsets, points);
  return points;
}

[[nodiscard]] bool offsetsNearlyEqual(const std::span<const double> lhs,
                                      const std::span<const double> rhs) noexcept {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (std::size_t i = 0U; i < lhs.size(); ++i) {
    if (std::abs(lhs[i] - rhs[i]) > 1.0e-9) {
      return false;
    }
  }
  return true;
}

void samplesFromPointsAndOffsets(const std::span<const CorridorSample> corridor_samples,
                                 const std::span<const Point2> points,
                                 const std::span<const double> offsets,
                                 std::vector<TrajectoryPointSample>& samples) {
  samples.clear();
  if (corridor_samples.size() != points.size() || points.size() != offsets.size()) {
    return;
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
}

[[nodiscard]] std::vector<TrajectoryPointSample>
samplesFromPointsAndOffsets(const std::span<const CorridorSample> corridor_samples,
                            const std::span<const Point2> points,
                            const std::span<const double> offsets) {
  std::vector<TrajectoryPointSample> samples;
  samplesFromPointsAndOffsets(corridor_samples, points, offsets, samples);
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

void offsetsFromSeed(const std::span<const CorridorSample> corridor_samples,
                     const InitialOffsetSeed seed, std::vector<double>& offsets) {
  offsets.assign(corridor_samples.size(), 0.0);
  if (corridor_samples.size() <= 2U) {
    return;
  }
  for (std::size_t i = 1U; i + 1U < corridor_samples.size(); ++i) {
    offsets[i] = offsetForSeed(corridor_samples[i], seed);
  }
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

[[nodiscard]] double edgeMarginM(const CorridorSample& sample,
                                 const double offset_m) noexcept {
  return std::min(sample.left_bound_m - offset_m, sample.right_bound_m + offset_m);
}

[[nodiscard]] double turnSeverity(const double curvature_1pm) noexcept {
  constexpr double kCurvatureScale = 0.05;
  const double abs_curvature = std::abs(curvature_1pm);
  if (!(abs_curvature > 0.0) || !std::isfinite(abs_curvature)) {
    return 0.0;
  }
  return abs_curvature / (abs_curvature + kCurvatureScale);
}

[[nodiscard]] double effectiveDesiredEdgeMarginM(const CorridorSample& sample,
                                                 const RacingLineConfig& config) {
  const double desired_margin =
      sanitizedPositive(config.desired_edge_margin_m, 0.0, 0.0, 1000.0);
  const double corridor_width = sample.left_bound_m + sample.right_bound_m;
  if (!(desired_margin > 0.0) || !(corridor_width > 0.0)) {
    return 0.0;
  }
  return std::min(desired_margin, 0.45 * corridor_width);
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

[[nodiscard]] CostBreakdown
costBreakdownForPoints(const std::span<const CorridorSample> corridor_samples,
                       const std::span<const Point2> points,
                       const std::span<const double> offsets,
                       const RacingLineConfig& config) {
  CostBreakdown breakdown{};
  if (points.size() < 2U) {
    breakdown.outside_grid_cost = kOutsideGridPenalty;
    return breakdown;
  }

  const double weight_length =
      sanitizedPositive(config.weight_length, 0.02, 0.0, 1.0e6);
  const double weight_curvature =
      sanitizedPositive(config.weight_curvature, 250.0, 0.0, 1.0e9);
  const double weight_curvature_change =
      sanitizedPositive(config.weight_curvature_change, 100.0, 0.0, 1.0e9);
  const double weight_offset_change =
      sanitizedPositive(config.weight_offset_change, 0.5, 0.0, 1.0e9);
  const double weight_offset_second_change =
      sanitizedPositive(config.weight_offset_second_change, 5.0, 0.0, 1.0e9);
  const double weight_center_bias =
      sanitizedPositive(config.weight_center_bias, 0.0, 0.0, 1.0e6);
  const double weight_edge_margin =
      sanitizedPositive(config.weight_edge_margin, 80.0, 0.0, 1.0e9);

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
    if (i < corridor_samples.size() && i < offsets.size()) {
      const double desired_margin =
          effectiveDesiredEdgeMarginM(corridor_samples[i], config);
      const double margin = edgeMarginM(corridor_samples[i], offsets[i]);
      const double deficit = desired_margin - margin;
      if (deficit > 0.0) {
        breakdown.edge_margin_cost +=
            turnSeverity(curvature) * deficit * deficit * weight_edge_margin;
      }
    }
    if (previous_curvature_valid) {
      const double change = curvature - previous_curvature;
      curvature_change_cost += change * change;
    }
    previous_curvature = curvature;
    previous_curvature_valid = true;
  }

  breakdown.length_cost = weight_length * pathLength(points);
  breakdown.curvature_cost = weight_curvature * curvature_cost;
  breakdown.curvature_change_cost = weight_curvature_change * curvature_change_cost;
  breakdown.offset_change_cost = weight_offset_change * offset_change_cost;
  breakdown.offset_second_change_cost =
      weight_offset_second_change * offset_second_change_cost;
  breakdown.center_bias_cost = weight_center_bias * center_bias_cost;
  return breakdown;
}

[[nodiscard]] CandidateScore scoreForCandidate(
    const std::span<const CorridorSample> corridor_samples,
    const std::span<const Point2> points, const std::span<const double> offsets,
    const PathEvaluation& evaluation, const RacingLineConfig& config,
    const VelocityFollowerConfig& speed_config, const double max_length_m,
    std::vector<TrajectoryPointSample>& scratch_samples, RacingLineStats& stats) {
  CandidateScore result{};
  result.breakdown = costBreakdownForPoints(corridor_samples, points, offsets, config);
  result.breakdown.collision_cost =
      static_cast<double>(evaluation.prohibited_cells) * kCollisionPenalty;
  result.breakdown.outside_grid_cost =
      static_cast<double>(evaluation.outside_grid_segments) * kOutsideGridPenalty;
  if (std::isfinite(max_length_m) && evaluation.length_m > max_length_m) {
    const double overrun_m = evaluation.length_m - max_length_m;
    result.breakdown.length_overrun_cost =
        overrun_m * overrun_m * kLengthOverrunPenalty;
  }
  const double weight_time = sanitizedPositive(config.weight_time, 50.0, 0.0, 1.0e9);
  if (evaluation.traversable()) {
    const auto sample_started_at = std::chrono::steady_clock::now();
    samplesFromPointsAndOffsets(corridor_samples, points, offsets, scratch_samples);
    populateSampleGeometry(scratch_samples);
    stats.candidate_sample_build_duration_ms += elapsedMilliseconds(sample_started_at);
    result.traversal_time = estimateTraversalTime(scratch_samples, speed_config, true);
    if (weight_time > 0.0 && result.traversal_time.valid &&
        std::isfinite(result.traversal_time.estimated_time_s)) {
      result.breakdown.time_cost = weight_time * result.traversal_time.estimated_time_s;
    }
  }
  result.score = result.breakdown.total();
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

void copyCostBreakdownToStats(const CostBreakdown& breakdown, RacingLineStats& stats) {
  stats.cost_length = breakdown.length_cost;
  stats.cost_time = breakdown.time_cost;
  stats.cost_curvature = breakdown.curvature_cost;
  stats.cost_curvature_change = breakdown.curvature_change_cost;
  stats.cost_offset_change = breakdown.offset_change_cost;
  stats.cost_offset_second_change = breakdown.offset_second_change_cost;
  stats.cost_center_bias = breakdown.center_bias_cost;
  stats.cost_edge_margin = breakdown.edge_margin_cost;
  stats.cost_collision = breakdown.collision_cost;
  stats.cost_outside_grid = breakdown.outside_grid_cost;
  stats.cost_length_overrun = breakdown.length_overrun_cost;
}

void updateEdgeMarginStats(const std::span<const TrajectoryPointSample> samples,
                           const RacingLineConfig& config, RacingLineStats& stats) {
  double margin_sum = 0.0;
  std::size_t margin_count = 0U;
  for (const TrajectoryPointSample& sample : samples) {
    CorridorSample bounds{};
    bounds.left_bound_m = sample.left_bound_m;
    bounds.right_bound_m = sample.right_bound_m;
    const double margin = edgeMarginM(bounds, sample.racing_offset_m);
    if (!std::isfinite(margin)) {
      continue;
    }
    if (!std::isfinite(stats.min_edge_margin_m)) {
      stats.min_edge_margin_m = margin;
    } else {
      stats.min_edge_margin_m = std::min(stats.min_edge_margin_m, margin);
    }
    margin_sum += margin;
    ++margin_count;

    const double desired_margin = effectiveDesiredEdgeMarginM(bounds, config);
    if (turnSeverity(sample.curvature_1pm) > 1.0e-3 && desired_margin - margin > 0.0) {
      ++stats.edge_margin_limited_samples;
    }
  }
  if (margin_count > 0U) {
    stats.mean_edge_margin_m = margin_sum / static_cast<double>(margin_count);
  }
}

[[nodiscard]] bool updateBestCandidate(
    const std::span<const CorridorSample> corridor_samples,
    const std::span<const double> candidate_offsets,
    const std::span<const Point2> candidate_points,
    const OccupancyGrid2D& prohibited_grid, const RacingLineConfig& config,
    const VelocityFollowerConfig& speed_config, const double max_length_m,
    double& best_cost, std::vector<double>& offsets, std::vector<Point2>& best_points,
    std::vector<TrajectoryPointSample>& scratch_samples, RacingLineStats& stats) {
  ++stats.candidate_evaluations;
  ++stats.scratch_reused_candidates;
  const auto evaluation_started_at = std::chrono::steady_clock::now();
  const PathEvaluation evaluation = evaluatePath(prohibited_grid, candidate_points);
  stats.candidate_path_evaluation_duration_ms +=
      elapsedMilliseconds(evaluation_started_at);
  if (!evaluation.traversable()) {
    ++stats.collision_rejections;
  }
  const auto score_started_at = std::chrono::steady_clock::now();
  const CandidateScore candidate_score = scoreForCandidate(
      corridor_samples, candidate_points, candidate_offsets, evaluation, config,
      speed_config, max_length_m, scratch_samples, stats);
  stats.candidate_score_duration_ms += elapsedMilliseconds(score_started_at);
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

[[nodiscard]] EvaluatedCandidate evaluateCandidateSnapshot(
    const std::span<const CorridorSample> corridor_samples,
    const std::span<const double> base_offsets, const std::size_t center_index,
    const double delta_m, const OccupancyGrid2D& prohibited_grid,
    const RacingLineConfig& config, const VelocityFollowerConfig& speed_config,
    const double max_length_m) {
  EvaluatedCandidate result{};
  result.offsets.assign(base_offsets.begin(), base_offsets.end());
  applyOffsetDelta(result.offsets, corridor_samples, center_index, delta_m);
  if (offsetsNearlyEqual(result.offsets, base_offsets)) {
    result.noop = true;
    return result;
  }

  const auto points_started_at = std::chrono::steady_clock::now();
  pointsFromOffsets(corridor_samples, result.offsets, result.points);
  result.point_build_duration_ms = elapsedMilliseconds(points_started_at);

  const auto evaluation_started_at = std::chrono::steady_clock::now();
  result.path = evaluatePath(prohibited_grid, result.points);
  result.path_evaluation_duration_ms = elapsedMilliseconds(evaluation_started_at);

  std::vector<TrajectoryPointSample> scratch_samples;
  scratch_samples.reserve(corridor_samples.size());
  RacingLineStats local_stats{};
  const auto score_started_at = std::chrono::steady_clock::now();
  result.score = scoreForCandidate(corridor_samples, result.points, result.offsets,
                                   result.path, config, speed_config, max_length_m,
                                   scratch_samples, local_stats);
  result.score_duration_ms = elapsedMilliseconds(score_started_at);
  result.sample_build_duration_ms = local_stats.candidate_sample_build_duration_ms;
  return result;
}

void smoothedOffsets(const std::span<const double> offsets,
                     const std::span<const CorridorSample> corridor_samples,
                     std::vector<double>& smoothed) {
  smoothed.assign(offsets.begin(), offsets.end());
  if (offsets.size() <= 2U || offsets.size() != corridor_samples.size()) {
    return;
  }
  for (std::size_t i = 1U; i + 1U < offsets.size(); ++i) {
    const double value =
        0.25 * offsets[i - 1U] + 0.5 * offsets[i] + 0.25 * offsets[i + 1U];
    smoothed[i] = std::clamp(value, -corridor_samples[i].right_bound_m,
                             corridor_samples[i].left_bound_m);
  }
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

  RacingLineScratch scratch{};
  scratch.candidate_offsets.reserve(sample_count);
  scratch.accepted_offsets.reserve(sample_count);
  scratch.iteration_best_offsets.reserve(sample_count);
  scratch.smoothed_offsets.reserve(sample_count);
  scratch.candidate_points.reserve(sample_count);
  scratch.accepted_points.reserve(sample_count);
  scratch.candidate_samples.reserve(sample_count);
  const bool use_parallel_candidates = config.parallel_candidate_evaluation &&
                                       config.parallel_workers != 1U &&
                                       sample_count > 2U;
  result.stats.parallel_candidate_evaluation_used = use_parallel_candidates;

  std::vector<double> offsets;
  offsets.reserve(sample_count);
  std::vector<Point2> best_points;
  best_points.reserve(sample_count);
  double best_cost = std::numeric_limits<double>::infinity();
  constexpr std::array kInitialSeeds{
      InitialOffsetSeed::kCenterline, InitialOffsetSeed::kCorridorMidline,
      InitialOffsetSeed::kLeftBiased, InitialOffsetSeed::kRightBiased};
  for (const InitialOffsetSeed seed : kInitialSeeds) {
    offsetsFromSeed(optimizer_samples, seed, scratch.candidate_offsets);
    const auto points_started_at = std::chrono::steady_clock::now();
    pointsFromOffsets(optimizer_samples, scratch.candidate_offsets,
                      scratch.candidate_points);
    result.stats.candidate_point_build_duration_ms +=
        elapsedMilliseconds(points_started_at);
    (void)updateBestCandidate(optimizer_samples, scratch.candidate_offsets,
                              scratch.candidate_points, prohibited_grid, config,
                              speed_config, max_length_m, best_cost, offsets,
                              best_points, scratch.candidate_samples, result.stats);
  }
  if (offsets.empty()) {
    return result;
  }
  result.stats.initial_cost = best_cost;

  for (std::size_t iteration = 0U; iteration < max_iterations && step >= min_step;
       ++iteration) {
    bool changed = false;
    for (std::size_t i = 1U; i + 1U < sample_count; ++i) {
      scratch.iteration_best_offsets = offsets;
      if (use_parallel_candidates) {
        std::array<std::future<EvaluatedCandidate>, 2U> futures{
            std::async(std::launch::async,
                       [&, i, step] {
                         return evaluateCandidateSnapshot(
                             optimizer_samples, offsets, i, -step, prohibited_grid,
                             config, speed_config, max_length_m);
                       }),
            std::async(std::launch::async, [&, i, step] {
              return evaluateCandidateSnapshot(optimizer_samples, offsets, i, step,
                                               prohibited_grid, config, speed_config,
                                               max_length_m);
            })};
        std::array<EvaluatedCandidate, 2U> candidates{futures[0].get(),
                                                      futures[1].get()};
        for (const EvaluatedCandidate& candidate : candidates) {
          if (candidate.noop) {
            ++result.stats.skipped_noop_candidates;
            continue;
          }
          ++result.stats.candidate_evaluations;
          result.stats.candidate_point_build_duration_ms +=
              candidate.point_build_duration_ms;
          result.stats.candidate_path_evaluation_duration_ms +=
              candidate.path_evaluation_duration_ms;
          result.stats.candidate_score_duration_ms += candidate.score_duration_ms;
          result.stats.candidate_sample_build_duration_ms +=
              candidate.sample_build_duration_ms;
          if (!candidate.path.traversable()) {
            ++result.stats.collision_rejections;
          }
          if (candidate.score.score + 1.0e-9 < best_cost) {
            best_cost = candidate.score.score;
            scratch.iteration_best_offsets = candidate.offsets;
            best_points = candidate.points;
            copyTraversalEstimateToBestCandidateStats(
                candidate.score.traversal_time, candidate.score.score, result.stats);
            changed = true;
          }
        }
      } else {
        for (const double delta : {-step, step}) {
          scratch.candidate_offsets = offsets;
          applyOffsetDelta(scratch.candidate_offsets, optimizer_samples, i, delta);
          if (offsetsNearlyEqual(scratch.candidate_offsets, offsets)) {
            ++result.stats.skipped_noop_candidates;
            continue;
          }
          const auto points_started_at = std::chrono::steady_clock::now();
          pointsFromOffsets(optimizer_samples, scratch.candidate_offsets,
                            scratch.candidate_points);
          result.stats.candidate_point_build_duration_ms +=
              elapsedMilliseconds(points_started_at);
          const bool accepted = updateBestCandidate(
              optimizer_samples, scratch.candidate_offsets, scratch.candidate_points,
              prohibited_grid, config, speed_config, max_length_m, best_cost,
              scratch.accepted_offsets, scratch.accepted_points,
              scratch.candidate_samples, result.stats);
          if (accepted) {
            scratch.iteration_best_offsets = scratch.accepted_offsets;
            best_points = scratch.accepted_points;
            changed = true;
          }
        }
      }
      offsets = scratch.iteration_best_offsets;
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
  const auto regularization_started_at = std::chrono::steady_clock::now();
  for (std::size_t iteration = 0U; iteration < regularization_iterations; ++iteration) {
    smoothedOffsets(final_offsets, optimizer_samples, scratch.smoothed_offsets);
    const auto points_started_at = std::chrono::steady_clock::now();
    pointsFromOffsets(optimizer_samples, scratch.smoothed_offsets,
                      scratch.candidate_points);
    result.stats.candidate_point_build_duration_ms +=
        elapsedMilliseconds(points_started_at);
    const PathEvaluation candidate_evaluation =
        evaluatePath(prohibited_grid, scratch.candidate_points);
    if (!candidate_evaluation.traversable()) {
      ++result.stats.collision_rejections;
      break;
    }
    const auto sample_started_at = std::chrono::steady_clock::now();
    samplesFromPointsAndOffsets(optimizer_samples, scratch.candidate_points,
                                scratch.smoothed_offsets, scratch.candidate_samples);
    populateSampleGeometry(scratch.candidate_samples);
    result.stats.candidate_sample_build_duration_ms +=
        elapsedMilliseconds(sample_started_at);
    const TrajectoryShapeDiagnostics candidate_diagnostics =
        computeTrajectoryShapeDiagnostics(scratch.candidate_samples);
    const TraversalTimeEstimate candidate_time =
        estimateTraversalTime(scratch.candidate_samples, speed_config, true);
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
    final_offsets = scratch.smoothed_offsets;
    final_points = scratch.candidate_points;
    result.stats.regularization_applied = true;
    ++result.stats.regularization_iterations;
  }
  result.stats.regularization_duration_ms =
      elapsedMilliseconds(regularization_started_at);
  const PathEvaluation final_evaluation = evaluatePath(prohibited_grid, final_points);
  if (!final_evaluation.traversable()) {
    ++result.stats.collision_rejections;
    return result;
  }
  const CandidateScore final_score = scoreForCandidate(
      optimizer_samples, final_points, final_offsets, final_evaluation, config,
      speed_config, max_length_m, scratch.candidate_samples, result.stats);

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
  if (result.stats.centerline_length_m > kTinyDistanceM) {
    result.stats.final_length_ratio =
        result.stats.final_length_m / result.stats.centerline_length_m;
  }
  result.stats.final_cost = final_score.score;
  copyCostBreakdownToStats(final_score.breakdown, result.stats);
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
  updateEdgeMarginStats(result.samples, config, result.stats);
  result.valid = trajectorySamplesAreUsable(result.samples);
  return result;
}

} // namespace drone_city_nav
