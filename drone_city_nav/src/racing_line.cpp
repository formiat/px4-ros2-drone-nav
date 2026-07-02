#include "drone_city_nav/racing_line.hpp"

#include "drone_city_nav/trajectory_diagnostics.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <thread>
#include <unordered_map>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;
constexpr double kCollisionPenalty = 1.0e9;
constexpr double kOutsideGridPenalty = 1.0e9;
constexpr double kLengthOverrunPenalty = 1.0e6;
constexpr double kHeadingJumpPenalty = 1.0e6;
constexpr double kHeadingJumpHardPenalty = 1.0e9;
constexpr double kHeadingJumpSoftLimitRad = std::numbers::pi / 2.0;
constexpr double kHeadingJumpHardLimitRad = 2.0 * std::numbers::pi / 3.0;

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
  double heading_jump_cost{0.0};
  double offset_change_cost{0.0};
  double offset_second_change_cost{0.0};
  double offset_slope_cost{0.0};
  double collision_cost{0.0};
  double outside_grid_cost{0.0};
  double length_overrun_cost{0.0};

  [[nodiscard]] double total() const noexcept {
    return length_cost + time_cost + curvature_cost + curvature_change_cost +
           heading_jump_cost + offset_change_cost + offset_second_change_cost +
           offset_slope_cost + collision_cost + outside_grid_cost + length_overrun_cost;
  }
};

struct CandidateScore {
  double score{std::numeric_limits<double>::infinity()};
  TraversalTimeEstimate traversal_time{};
  CostBreakdown breakdown{};
};

enum class LocalFullScoreReason : std::uint8_t {
  kNone,
  kInvalidInput,
  kBoundaryWindow,
  kUnsafeBase,
  kWindowInvalid,
};

struct EvaluatedCandidate {
  bool noop{false};
  std::vector<double> offsets;
  PathEvaluation path{};
  CandidateScore score{};
  double point_build_duration_ms{0.0};
  double path_evaluation_duration_ms{0.0};
  double score_duration_ms{0.0};
  double sample_build_duration_ms{0.0};
  double cost_breakdown_duration_ms{0.0};
  double shape_diagnostics_duration_ms{0.0};
  double speed_profile_duration_ms{0.0};
  double local_point_build_duration_ms{0.0};
  double local_path_evaluation_duration_ms{0.0};
  double local_score_duration_ms{0.0};
  double local_traversal_estimate_duration_ms{0.0};
  double full_score_duration_ms{0.0};
  std::size_t local_segment_cache_hits{0U};
  std::size_t local_segment_cache_misses{0U};
  std::size_t full_path_segment_cache_hits{0U};
  std::size_t full_path_segment_cache_misses{0U};
  bool local_evaluated{false};
  bool requires_full_score{false};
  bool full_score_used{false};
  LocalFullScoreReason local_full_score_reason{LocalFullScoreReason::kNone};
  bool scratch_reused{false};
  bool snapshot_allocation_avoided{false};
  bool shadow_lower_bound_valid{false};
  bool shadow_lower_bound_would_prune{false};
  double shadow_lower_bound_score{std::numeric_limits<double>::quiet_NaN()};
  double shadow_lower_bound_incumbent_score{std::numeric_limits<double>::quiet_NaN()};
};

struct CandidateTask {
  std::size_t order{0U};
  std::size_t center_index{0U};
  double delta_m{0.0};
};

struct CandidateBatchResult {
  std::size_t order{0U};
  EvaluatedCandidate candidate{};
};

struct SegmentCellKey {
  GridIndex start{};
  GridIndex end{};

  [[nodiscard]] bool operator==(const SegmentCellKey& other) const noexcept {
    return start.x == other.start.x && start.y == other.start.y &&
           end.x == other.end.x && end.y == other.end.y;
  }
};

struct SegmentCellKeyHash {
  [[nodiscard]] std::size_t operator()(const SegmentCellKey& key) const noexcept {
    const auto mix = [](const std::size_t lhs, const std::size_t rhs) {
      return lhs ^ (rhs + 0x9e3779b97f4a7c15ULL + (lhs << 6U) + (lhs >> 2U));
    };
    std::size_t hash = std::hash<int>{}(key.start.x);
    hash = mix(hash, std::hash<int>{}(key.start.y));
    hash = mix(hash, std::hash<int>{}(key.end.x));
    hash = mix(hash, std::hash<int>{}(key.end.y));
    return hash;
  }
};

struct SegmentTraversabilityCache {
  std::unordered_map<SegmentCellKey, bool, SegmentCellKeyHash> values;
};

struct SegmentProhibitedCountCache {
  std::unordered_map<SegmentCellKey, std::size_t, SegmentCellKeyHash> values;
};

struct CandidateWorkBuffer {
  std::vector<double> offsets;
  std::vector<Point2> points;
  std::vector<Point2> local_base_points;
  std::vector<Point2> local_candidate_points;
  std::vector<CorridorSample> local_corridor_samples;
  std::vector<double> local_base_offsets;
  std::vector<double> local_candidate_offsets;
  std::vector<TrajectoryPointSample> local_base_samples;
  std::vector<TrajectoryPointSample> local_candidate_samples;
  std::vector<TrajectoryPointSample> samples;
  SegmentTraversabilityCache candidate_segment_cache;
  SegmentProhibitedCountCache full_path_segment_cache;
};

struct RacingLineScratch {
  std::vector<double> candidate_offsets;
  std::vector<double> accepted_offsets;
  std::vector<double> iteration_best_offsets;
  std::vector<double> smoothed_offsets;
  std::vector<Point2> candidate_points;
  std::vector<Point2> accepted_points;
  std::vector<TrajectoryPointSample> candidate_samples;
  SegmentProhibitedCountCache full_path_segment_cache;
};

struct ActiveWindow {
  std::size_t begin_index{0U};
  std::size_t end_index{0U};
};

struct LocalCandidateScore {
  bool valid{false};
  bool requires_full_score{false};
  LocalFullScoreReason full_score_reason{LocalFullScoreReason::kNone};
  PathEvaluation path{};
  TraversalTimeEstimate estimated_traversal_time{};
  double point_build_duration_ms{0.0};
  double path_evaluation_duration_ms{0.0};
  double score_duration_ms{0.0};
  std::size_t segment_cache_hits{0U};
  std::size_t segment_cache_misses{0U};
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

[[nodiscard]] std::size_t desiredWorkerCount(const std::size_t requested_workers,
                                             const std::size_t work_items) noexcept {
  if (work_items < 2U) {
    return 1U;
  }
  std::size_t worker_count = requested_workers;
  if (worker_count == 0U) {
    worker_count = static_cast<std::size_t>(std::thread::hardware_concurrency());
  }
  if (worker_count == 0U) {
    worker_count = 2U;
  }
  return std::clamp<std::size_t>(worker_count, 1U,
                                 std::min<std::size_t>(work_items, 16U));
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
                      const std::size_t center_index, const double delta_m,
                      const std::span<const std::uint8_t> mutable_indices = {}) {
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
    if (!mutable_indices.empty() && mutable_indices[index] == 0U) {
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

[[nodiscard]] double headingDeltaRad(const Point2 lhs, const Point2 rhs) noexcept {
  const double lhs_norm = norm(lhs);
  const double rhs_norm = norm(rhs);
  if (!(lhs_norm > kTinyDistanceM) || !(rhs_norm > kTinyDistanceM)) {
    return 0.0;
  }
  const double cosine =
      std::clamp((lhs.x * rhs.x + lhs.y * rhs.y) / (lhs_norm * rhs_norm), -1.0, 1.0);
  return std::acos(cosine);
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

void populateSampleGeometry(std::vector<TrajectoryPointSample>& samples);

[[nodiscard]] CostBreakdown costBreakdownForPoints(std::span<const Point2> points,
                                                   std::span<const double> offsets,
                                                   const RacingLineConfig& config);

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

[[nodiscard]] SegmentCellKey orderedSegmentKey(GridIndex start,
                                               GridIndex end) noexcept {
  if (end.x < start.x || (end.x == start.x && end.y < start.y)) {
    std::swap(start, end);
  }
  return SegmentCellKey{.start = start, .end = end};
}

[[nodiscard]] bool cachedSegmentTraversable(const OccupancyGrid2D& grid,
                                            const Point2 start, const Point2 end,
                                            SegmentTraversabilityCache& cache,
                                            std::size_t& hits, std::size_t& misses) {
  const std::optional<GridIndex> start_cell = grid.worldToCell(start);
  const std::optional<GridIndex> end_cell = grid.worldToCell(end);
  if (!start_cell.has_value() || !end_cell.has_value()) {
    ++misses;
    return false;
  }
  const SegmentCellKey key = orderedSegmentKey(*start_cell, *end_cell);
  if (const auto iter = cache.values.find(key); iter != cache.values.end()) {
    ++hits;
    return iter->second;
  }
  ++misses;
  const bool traversable = std::ranges::all_of(
      grid.cellsOnLine(*start_cell, *end_cell),
      [&grid](const GridIndex cell) { return !grid.isProhibited(cell); });
  cache.values.emplace(key, traversable);
  return traversable;
}

[[nodiscard]] std::pair<std::size_t, std::size_t>
localScoreWindowForCenter(const std::size_t center_index,
                          const std::size_t sample_count,
                          const std::size_t radius_samples) noexcept {
  if (sample_count == 0U) {
    return {0U, 0U};
  }
  const std::size_t begin =
      center_index > radius_samples ? center_index - radius_samples : 0U;
  const std::size_t end = std::min(sample_count - 1U, center_index + radius_samples);
  return {begin, end};
}

void pointsFromOffsetsRange(const std::span<const CorridorSample> corridor_samples,
                            const std::span<const double> offsets,
                            const std::size_t begin_index, const std::size_t end_index,
                            std::vector<Point2>& points) {
  points.clear();
  if (corridor_samples.size() != offsets.size() || begin_index > end_index ||
      end_index >= corridor_samples.size()) {
    return;
  }
  points.reserve(end_index - begin_index + 1U);
  for (std::size_t i = begin_index; i <= end_index; ++i) {
    points.push_back(corridor_samples[i].center +
                     corridor_samples[i].normal * offsets[i]);
  }
}

void copyRange(const std::span<const Point2> source, const std::size_t begin_index,
               const std::size_t end_index, std::vector<Point2>& destination) {
  destination.clear();
  if (begin_index > end_index || end_index >= source.size()) {
    return;
  }
  destination.reserve(end_index - begin_index + 1U);
  for (std::size_t i = begin_index; i <= end_index; ++i) {
    destination.push_back(source[i]);
  }
}

void copyRange(const std::span<const double> source, const std::size_t begin_index,
               const std::size_t end_index, std::vector<double>& destination) {
  destination.clear();
  if (begin_index > end_index || end_index >= source.size()) {
    return;
  }
  destination.reserve(end_index - begin_index + 1U);
  for (std::size_t i = begin_index; i <= end_index; ++i) {
    destination.push_back(source[i]);
  }
}

void copyRange(const std::span<const CorridorSample> source,
               const std::size_t begin_index, const std::size_t end_index,
               std::vector<CorridorSample>& destination) {
  destination.clear();
  if (begin_index > end_index || end_index >= source.size()) {
    return;
  }
  destination.reserve(end_index - begin_index + 1U);
  for (std::size_t i = begin_index; i <= end_index; ++i) {
    destination.push_back(source[i]);
  }
}

[[nodiscard]] PathEvaluation
evaluateLocalPathWindowCached(const OccupancyGrid2D& grid,
                              const std::span<const Point2> local_points,
                              SegmentTraversabilityCache& cache,
                              std::size_t& cache_hits, std::size_t& cache_misses) {
  PathEvaluation evaluation{};
  if (local_points.size() < 2U) {
    ++evaluation.outside_grid_segments;
    return evaluation;
  }
  for (std::size_t i = 1U; i < local_points.size(); ++i) {
    const Point2 start = local_points[i - 1U];
    const Point2 end = local_points[i];
    evaluation.length_m += distance(start, end);
    if (!cachedSegmentTraversable(grid, start, end, cache, cache_hits, cache_misses)) {
      ++evaluation.prohibited_cells;
    }
  }
  return evaluation;
}

[[nodiscard]] PathEvaluation evaluatePathCached(const OccupancyGrid2D& grid,
                                                const std::span<const Point2> points,
                                                SegmentProhibitedCountCache& cache,
                                                std::size_t& cache_hits,
                                                std::size_t& cache_misses) {
  PathEvaluation evaluation{};
  if (points.size() < 2U) {
    ++evaluation.outside_grid_segments;
    return evaluation;
  }
  for (std::size_t i = 1U; i < points.size(); ++i) {
    const Point2 start = points[i - 1U];
    const Point2 end = points[i];
    evaluation.length_m += distance(start, end);
    const std::optional<GridIndex> start_cell = grid.worldToCell(start);
    const std::optional<GridIndex> end_cell = grid.worldToCell(end);
    if (!start_cell.has_value() || !end_cell.has_value()) {
      ++cache_misses;
      ++evaluation.outside_grid_segments;
      continue;
    }
    const SegmentCellKey key = orderedSegmentKey(*start_cell, *end_cell);
    const auto cached = cache.values.find(key);
    const std::size_t prohibited_cells =
        cached != cache.values.end() ? (++cache_hits, cached->second) : [&] {
          ++cache_misses;
          std::size_t computed = 0U;
          for (const GridIndex cell : grid.cellsOnLine(*start_cell, *end_cell)) {
            if (grid.isProhibited(cell)) {
              ++computed;
            }
          }
          cache.values.emplace(key, computed);
          return computed;
        }();
    evaluation.prohibited_cells += prohibited_cells;
  }
  return evaluation;
}

[[nodiscard]] LocalCandidateScore evaluateLocalOffsetPath(
    const std::span<const CorridorSample> corridor_samples,
    const std::span<const Point2> base_points,
    const std::span<const double> base_offsets,
    const std::span<const double> candidate_offsets,
    const OccupancyGrid2D& prohibited_grid, const CandidateScore& base_score,
    const double base_length_m, const std::size_t center_index,
    const VelocityFollowerConfig& speed_config, CandidateWorkBuffer& buffer) {
  LocalCandidateScore result{};
  result.valid = false;
  constexpr std::size_t kLocalScoreRadiusSamples = 6U;
  if (corridor_samples.size() != base_points.size() ||
      base_points.size() != base_offsets.size() ||
      base_offsets.size() != candidate_offsets.size() || base_points.size() < 3U ||
      center_index == 0U || center_index + 1U >= base_points.size()) {
    result.requires_full_score = true;
    result.full_score_reason = LocalFullScoreReason::kInvalidInput;
    return result;
  }

  const auto [begin_index, end_index] = localScoreWindowForCenter(
      center_index, base_points.size(), kLocalScoreRadiusSamples);
  if (begin_index == 0U || end_index + 1U >= base_points.size()) {
    result.requires_full_score = true;
    result.full_score_reason = LocalFullScoreReason::kBoundaryWindow;
    return result;
  }
  if (base_score.breakdown.collision_cost > 0.0 ||
      base_score.breakdown.outside_grid_cost > 0.0) {
    result.requires_full_score = true;
    result.full_score_reason = LocalFullScoreReason::kUnsafeBase;
    return result;
  }

  const auto points_started_at = std::chrono::steady_clock::now();
  copyRange(base_points, begin_index, end_index, buffer.local_base_points);
  copyRange(corridor_samples, begin_index, end_index, buffer.local_corridor_samples);
  copyRange(base_offsets, begin_index, end_index, buffer.local_base_offsets);
  copyRange(candidate_offsets, begin_index, end_index, buffer.local_candidate_offsets);
  pointsFromOffsetsRange(corridor_samples, candidate_offsets, begin_index, end_index,
                         buffer.local_candidate_points);
  result.point_build_duration_ms = elapsedMilliseconds(points_started_at);
  if (buffer.local_base_points.size() != buffer.local_candidate_points.size() ||
      buffer.local_base_points.size() < 2U) {
    result.requires_full_score = true;
    result.full_score_reason = LocalFullScoreReason::kWindowInvalid;
    return result;
  }

  const double base_local_length_m = pathLength(buffer.local_base_points);
  const auto evaluation_started_at = std::chrono::steady_clock::now();
  result.path = evaluateLocalPathWindowCached(
      prohibited_grid, buffer.local_candidate_points, buffer.candidate_segment_cache,
      result.segment_cache_hits, result.segment_cache_misses);
  result.path_evaluation_duration_ms = elapsedMilliseconds(evaluation_started_at);

  const double candidate_length_m =
      base_length_m - base_local_length_m + result.path.length_m;
  result.path.length_m = candidate_length_m;
  const auto score_started_at = std::chrono::steady_clock::now();
  samplesFromPointsAndOffsets(buffer.local_corridor_samples, buffer.local_base_points,
                              buffer.local_base_offsets, buffer.local_base_samples);
  samplesFromPointsAndOffsets(
      buffer.local_corridor_samples, buffer.local_candidate_points,
      buffer.local_candidate_offsets, buffer.local_candidate_samples);
  populateSampleGeometry(buffer.local_base_samples);
  populateSampleGeometry(buffer.local_candidate_samples);
  const TraversalTimeEstimate base_local_time =
      estimateTraversalTime(buffer.local_base_samples, speed_config, false);
  const TraversalTimeEstimate candidate_local_time =
      estimateTraversalTime(buffer.local_candidate_samples, speed_config, false);
  if (base_score.traversal_time.valid && base_local_time.valid &&
      candidate_local_time.valid &&
      std::isfinite(base_score.traversal_time.estimated_time_s) &&
      std::isfinite(base_local_time.estimated_time_s) &&
      std::isfinite(candidate_local_time.estimated_time_s)) {
    result.estimated_traversal_time = candidate_local_time;
    result.estimated_traversal_time.estimated_time_s =
        std::max(0.0, base_score.traversal_time.estimated_time_s -
                          base_local_time.estimated_time_s +
                          candidate_local_time.estimated_time_s);
  }
  result.score_duration_ms = elapsedMilliseconds(score_started_at);
  result.valid = true;
  return result;
}

void addActiveWindow(std::vector<ActiveWindow>& windows,
                     const std::span<const CorridorSample> samples,
                     const std::size_t center_index, const double pre_margin_m,
                     const double post_margin_m) {
  if (samples.empty()) {
    return;
  }
  const double begin_s = samples[center_index].s_m - pre_margin_m;
  const double end_s = samples[center_index].s_m + post_margin_m;
  std::size_t begin = center_index;
  while (begin > 0U && samples[begin].s_m > begin_s) {
    --begin;
  }
  std::size_t end = center_index;
  while (end + 1U < samples.size() && samples[end].s_m < end_s) {
    ++end;
  }
  if (begin >= end) {
    return;
  }
  if (!windows.empty() && begin <= windows.back().end_index + 1U) {
    windows.back().end_index = std::max(windows.back().end_index, end);
    return;
  }
  windows.push_back(ActiveWindow{begin, end});
}

[[nodiscard]] double headingSpanAround(const std::span<const CorridorSample> samples,
                                       const std::size_t center_index,
                                       const double pre_margin_m,
                                       const double post_margin_m) {
  if (samples.empty() || center_index >= samples.size()) {
    return 0.0;
  }
  const double begin_s = samples[center_index].s_m - pre_margin_m;
  const double end_s = samples[center_index].s_m + post_margin_m;
  std::size_t begin = center_index;
  while (begin > 0U && samples[begin].s_m > begin_s) {
    --begin;
  }
  std::size_t end = center_index;
  while (end + 1U < samples.size() && samples[end].s_m < end_s) {
    ++end;
  }
  if (begin >= end) {
    return 0.0;
  }

  const double reference =
      std::atan2(samples[begin].tangent.y, samples[begin].tangent.x);
  double min_heading = 0.0;
  double max_heading = 0.0;
  for (std::size_t i = begin; i <= end; ++i) {
    double heading = std::atan2(samples[i].tangent.y, samples[i].tangent.x);
    while (heading - reference > std::numbers::pi) {
      heading -= 2.0 * std::numbers::pi;
    }
    while (heading - reference < -std::numbers::pi) {
      heading += 2.0 * std::numbers::pi;
    }
    const double relative_heading = heading - reference;
    min_heading = std::min(min_heading, relative_heading);
    max_heading = std::max(max_heading, relative_heading);
  }
  return max_heading - min_heading;
}

[[nodiscard]] std::vector<ActiveWindow>
detectActiveWindows(const std::span<const CorridorSample> samples,
                    const std::span<const Point2> centerline,
                    const OccupancyGrid2D& prohibited_grid,
                    const RacingLineConfig& config, RacingLineStats& stats) {
  const auto started_at = std::chrono::steady_clock::now();
  std::vector<ActiveWindow> windows;
  if (samples.size() < 3U) {
    stats.window_detection_duration_ms = elapsedMilliseconds(started_at);
    return windows;
  }

  const PathEvaluation centerline_evaluation =
      evaluatePath(prohibited_grid, centerline);
  const double pre_margin_m =
      sanitizedPositive(config.window_pre_margin_m, 25.0, 0.0, 5000.0);
  const double post_margin_m =
      sanitizedPositive(config.window_post_margin_m, 25.0, 0.0, 5000.0);
  if (!centerline_evaluation.traversable()) {
    windows.push_back(ActiveWindow{0U, samples.size() - 1U});
    stats.window_count = 1U;
    stats.active_window_count = 1U;
    stats.active_window_samples = samples.size() > 2U ? samples.size() - 2U : 0U;
    stats.window_detection_duration_ms = elapsedMilliseconds(started_at);
    return windows;
  }

  const double heading_threshold_rad =
      sanitizedPositive(config.window_heading_threshold_rad,
                        10.0 * std::numbers::pi / 180.0, 0.0, std::numbers::pi);
  const double heading_span_threshold_rad =
      sanitizedPositive(config.window_min_heading_span_rad,
                        10.0 * std::numbers::pi / 180.0, 0.0, std::numbers::pi);
  const double curvature_threshold =
      sanitizedPositive(config.window_min_curvature_1pm, 0.01, 0.0, 1000.0);
  const double width_threshold_m =
      sanitizedPositive(config.window_width_change_threshold_m, 2.0, 0.0, 5000.0);
  const double width_asymmetry_threshold_m =
      sanitizedPositive(config.window_min_width_asymmetry_m, 1.0, 0.0, 5000.0);
  for (std::size_t i = 1U; i + 1U < samples.size(); ++i) {
    const double heading_change =
        headingDeltaRad(samples[i - 1U].tangent, samples[i + 1U].tangent);
    const double heading_span =
        headingSpanAround(samples, i, pre_margin_m, post_margin_m);
    const double curvature = std::abs(
        discreteCurvature(centerline[i - 1U], centerline[i], centerline[i + 1U]));
    const double previous_width =
        samples[i - 1U].left_bound_m + samples[i - 1U].right_bound_m;
    const double next_width =
        samples[i + 1U].left_bound_m + samples[i + 1U].right_bound_m;
    const double width_asymmetry =
        std::abs(samples[i].left_bound_m - samples[i].right_bound_m);
    const bool turn_zone = heading_change >= heading_threshold_rad ||
                           heading_span >= heading_span_threshold_rad ||
                           curvature >= curvature_threshold;
    const bool width_zone =
        std::abs(next_width - previous_width) >= width_threshold_m ||
        width_asymmetry >= width_asymmetry_threshold_m;
    if (turn_zone || width_zone) {
      addActiveWindow(windows, samples, i, pre_margin_m, post_margin_m);
    }
  }

  stats.window_count = windows.size();
  stats.active_window_count = windows.size();
  for (const ActiveWindow& window : windows) {
    if (window.end_index > window.begin_index + 1U) {
      stats.active_window_samples += window.end_index - window.begin_index - 1U;
    }
  }
  stats.window_detection_duration_ms = elapsedMilliseconds(started_at);
  return windows;
}

[[nodiscard]] std::vector<std::size_t>
activeControlIndices(const std::span<const ActiveWindow> windows,
                     const std::size_t sample_count,
                     std::vector<std::uint8_t>& mutable_indices) {
  mutable_indices.assign(sample_count, 0U);
  std::vector<std::size_t> indices;
  for (const ActiveWindow& window : windows) {
    const std::size_t begin = std::min(window.begin_index + 1U, sample_count);
    const std::size_t end = std::min(window.end_index, sample_count);
    for (std::size_t i = begin; i < end; ++i) {
      if (i == 0U || i + 1U >= sample_count || mutable_indices[i] != 0U) {
        continue;
      }
      mutable_indices[i] = 1U;
      indices.push_back(i);
    }
  }
  return indices;
}

[[nodiscard]] std::vector<RacingLineWindowMetadata>
windowMetadata(const std::span<const ActiveWindow> windows,
               const std::span<const CorridorSample> samples) {
  std::vector<RacingLineWindowMetadata> metadata;
  metadata.reserve(windows.size());
  for (std::size_t i = 0U; i < windows.size(); ++i) {
    const ActiveWindow& window = windows[i];
    if (window.begin_index >= samples.size() || window.end_index >= samples.size()) {
      continue;
    }
    metadata.push_back(RacingLineWindowMetadata{
        .id = i + 1U,
        .begin_s_m = samples[window.begin_index].s_m,
        .end_s_m = samples[window.end_index].s_m,
    });
  }
  return metadata;
}

[[nodiscard]] CostBreakdown
costBreakdownForPoints(const std::span<const Point2> points,
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
      sanitizedPositive(config.weight_curvature, 300.0, 0.0, 1.0e9);
  const double weight_curvature_change =
      sanitizedPositive(config.weight_curvature_change, 130.0, 0.0, 1.0e9);
  const double weight_offset_change =
      sanitizedPositive(config.weight_offset_change, 0.5, 0.0, 1.0e9);
  const double weight_offset_second_change =
      sanitizedPositive(config.weight_offset_second_change, 6.5, 0.0, 1.0e9);
  const double weight_offset_slope =
      sanitizedPositive(config.weight_offset_slope, 100.0, 0.0, 1.0e9);
  const double max_offset_slope =
      sanitizedPositive(config.max_offset_slope_per_m, 0.32, 0.0, 100.0);

  double curvature_cost = 0.0;
  double curvature_change_cost = 0.0;
  double offset_change_cost = 0.0;
  double offset_second_change_cost = 0.0;
  double offset_slope_cost = 0.0;
  double previous_curvature = 0.0;
  bool previous_curvature_valid = false;
  for (std::size_t i = 1U; i < offsets.size(); ++i) {
    const double change = offsets[i] - offsets[i - 1U];
    offset_change_cost += change * change;
    const double ds = i < points.size() ? distance(points[i - 1U], points[i]) : 0.0;
    if (ds > kTinyDistanceM) {
      const double slope_violation =
          std::max(0.0, std::abs(change) / ds - max_offset_slope);
      offset_slope_cost += slope_violation * slope_violation * ds;
    }
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

  breakdown.length_cost = weight_length * pathLength(points);
  breakdown.curvature_cost = weight_curvature * curvature_cost;
  breakdown.curvature_change_cost = weight_curvature_change * curvature_change_cost;
  breakdown.offset_change_cost = weight_offset_change * offset_change_cost;
  breakdown.offset_second_change_cost =
      weight_offset_second_change * offset_second_change_cost;
  breakdown.offset_slope_cost = weight_offset_slope * offset_slope_cost;
  return breakdown;
}

[[nodiscard]] CandidateScore scoreForCandidate(
    const std::span<const CorridorSample> corridor_samples,
    const std::span<const Point2> points, const std::span<const double> offsets,
    const PathEvaluation& evaluation, const RacingLineConfig& config,
    const VelocityFollowerConfig& speed_config, const double max_length_m,
    std::vector<TrajectoryPointSample>& scratch_samples, RacingLineStats& stats) {
  CandidateScore result{};
  const auto cost_started_at = std::chrono::steady_clock::now();
  result.breakdown = costBreakdownForPoints(points, offsets, config);
  result.breakdown.collision_cost =
      static_cast<double>(evaluation.prohibited_cells) * kCollisionPenalty;
  result.breakdown.outside_grid_cost =
      static_cast<double>(evaluation.outside_grid_segments) * kOutsideGridPenalty;
  if (std::isfinite(max_length_m) && evaluation.length_m > max_length_m) {
    const double overrun_m = evaluation.length_m - max_length_m;
    result.breakdown.length_overrun_cost =
        overrun_m * overrun_m * kLengthOverrunPenalty;
  }
  stats.candidate_cost_breakdown_duration_ms += elapsedMilliseconds(cost_started_at);
  const double weight_time = sanitizedPositive(config.weight_time, 40.0, 0.0, 1.0e9);
  if (evaluation.traversable()) {
    const auto sample_started_at = std::chrono::steady_clock::now();
    samplesFromPointsAndOffsets(corridor_samples, points, offsets, scratch_samples);
    populateSampleGeometry(scratch_samples);
    stats.candidate_sample_build_duration_ms += elapsedMilliseconds(sample_started_at);
    const auto shape_started_at = std::chrono::steady_clock::now();
    const TrajectoryShapeDiagnostics shape =
        computeTrajectoryShapeDiagnostics(scratch_samples);
    stats.candidate_shape_diagnostics_duration_ms +=
        elapsedMilliseconds(shape_started_at);
    const double heading_jump_overrun =
        std::max(0.0, shape.max_heading_delta_rad - kHeadingJumpSoftLimitRad);
    result.breakdown.heading_jump_cost =
        heading_jump_overrun * heading_jump_overrun * kHeadingJumpPenalty;
    if (shape.max_heading_delta_rad > kHeadingJumpHardLimitRad) {
      result.breakdown.heading_jump_cost += kHeadingJumpHardPenalty;
    }
    const auto speed_started_at = std::chrono::steady_clock::now();
    result.traversal_time = estimateTraversalTime(scratch_samples, speed_config, true);
    stats.candidate_speed_profile_duration_ms += elapsedMilliseconds(speed_started_at);
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
  stats.cost_heading_jump = breakdown.heading_jump_cost;
  stats.cost_offset_change = breakdown.offset_change_cost;
  stats.cost_offset_second_change = breakdown.offset_second_change_cost;
  stats.cost_offset_slope = breakdown.offset_slope_cost;
  stats.cost_collision = breakdown.collision_cost;
  stats.cost_outside_grid = breakdown.outside_grid_cost;
  stats.cost_length_overrun = breakdown.length_overrun_cost;
}

void updateEdgeMarginStats(const std::span<const TrajectoryPointSample> samples,
                           RacingLineStats& stats) {
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
    CandidateScore& best_score, double& best_length_m,
    std::vector<TrajectoryPointSample>& scratch_samples,
    SegmentProhibitedCountCache& segment_cache, RacingLineStats& stats) {
  ++stats.candidate_evaluations;
  ++stats.scratch_reused_candidates;
  const auto evaluation_started_at = std::chrono::steady_clock::now();
  std::size_t cache_hits = 0U;
  std::size_t cache_misses = 0U;
  const PathEvaluation evaluation = evaluatePathCached(
      prohibited_grid, candidate_points, segment_cache, cache_hits, cache_misses);
  stats.full_path_segment_cache_hits += cache_hits;
  stats.full_path_segment_cache_misses += cache_misses;
  stats.candidate_path_evaluation_duration_ms +=
      elapsedMilliseconds(evaluation_started_at);
  if (!evaluation.traversable()) {
    ++stats.collision_rejections;
  }
  const auto score_started_at = std::chrono::steady_clock::now();
  const CandidateScore candidate_score = scoreForCandidate(
      corridor_samples, candidate_points, candidate_offsets, evaluation, config,
      speed_config, max_length_m, scratch_samples, stats);
  const double score_duration_ms = elapsedMilliseconds(score_started_at);
  stats.candidate_score_duration_ms += score_duration_ms;
  stats.full_candidate_score_duration_ms += score_duration_ms;
  if (candidate_score.score + 1.0e-9 < best_cost) {
    best_cost = candidate_score.score;
    offsets.assign(candidate_offsets.begin(), candidate_offsets.end());
    best_points.assign(candidate_points.begin(), candidate_points.end());
    best_score = candidate_score;
    best_length_m = evaluation.length_m;
    copyTraversalEstimateToBestCandidateStats(candidate_score.traversal_time,
                                              candidate_score.score, stats);
    return true;
  }
  return false;
}

[[nodiscard]] EvaluatedCandidate evaluateCandidateSnapshot(
    const std::span<const CorridorSample> corridor_samples,
    const std::span<const double> base_offsets,
    const std::span<const Point2> base_points, const CandidateScore& base_score,
    const double base_length_m, const std::size_t center_index, const double delta_m,
    const OccupancyGrid2D& prohibited_grid, const RacingLineConfig& config,
    const VelocityFollowerConfig& speed_config, const double max_length_m,
    const std::span<const std::uint8_t> mutable_indices, const double incumbent_score,
    CandidateWorkBuffer& buffer) {
  EvaluatedCandidate result{};
  result.scratch_reused = true;
  result.snapshot_allocation_avoided =
      buffer.offsets.capacity() >= base_offsets.size() &&
      buffer.points.capacity() >= corridor_samples.size() &&
      buffer.samples.capacity() >= corridor_samples.size();
  buffer.offsets.assign(base_offsets.begin(), base_offsets.end());
  applyOffsetDelta(buffer.offsets, corridor_samples, center_index, delta_m,
                   mutable_indices);
  if (offsetsNearlyEqual(buffer.offsets, base_offsets)) {
    result.noop = true;
    return result;
  }

  result.local_evaluated = true;
  LocalCandidateScore local_score = evaluateLocalOffsetPath(
      corridor_samples, base_points, base_offsets, buffer.offsets, prohibited_grid,
      base_score, base_length_m, center_index, speed_config, buffer);
  result.point_build_duration_ms = local_score.point_build_duration_ms;
  result.path_evaluation_duration_ms = local_score.path_evaluation_duration_ms;
  result.score_duration_ms = local_score.score_duration_ms;
  result.local_point_build_duration_ms = local_score.point_build_duration_ms;
  result.local_path_evaluation_duration_ms = local_score.path_evaluation_duration_ms;
  result.local_score_duration_ms =
      local_score.path_evaluation_duration_ms + local_score.score_duration_ms;
  result.local_traversal_estimate_duration_ms = local_score.score_duration_ms;
  result.local_segment_cache_hits = local_score.segment_cache_hits;
  result.local_segment_cache_misses = local_score.segment_cache_misses;
  result.local_full_score_reason = local_score.full_score_reason;
  if (local_score.valid && !local_score.requires_full_score) {
    result.path = local_score.path;
    result.offsets.assign(buffer.offsets.begin(), buffer.offsets.end());
    if (!result.path.traversable()) {
      return result;
    }
    RacingLineStats local_stats{};
    const auto points_started_at = std::chrono::steady_clock::now();
    pointsFromOffsets(corridor_samples, buffer.offsets, buffer.points);
    result.point_build_duration_ms += elapsedMilliseconds(points_started_at);
    const auto prefilter_started_at = std::chrono::steady_clock::now();
    result.score.breakdown =
        costBreakdownForPoints(buffer.points, buffer.offsets, config);
    result.score.breakdown.collision_cost =
        static_cast<double>(result.path.prohibited_cells) * kCollisionPenalty;
    result.score.breakdown.outside_grid_cost =
        static_cast<double>(result.path.outside_grid_segments) * kOutsideGridPenalty;
    if (std::isfinite(max_length_m) && result.path.length_m > max_length_m) {
      const double overrun_m = result.path.length_m - max_length_m;
      result.score.breakdown.length_overrun_cost =
          overrun_m * overrun_m * kLengthOverrunPenalty;
    }
    const double weight_time = sanitizedPositive(config.weight_time, 40.0, 0.0, 1.0e9);
    if (weight_time > 0.0 && local_score.estimated_traversal_time.valid &&
        std::isfinite(local_score.estimated_traversal_time.estimated_time_s)) {
      result.score.traversal_time = local_score.estimated_traversal_time;
      result.score.breakdown.time_cost =
          weight_time * local_score.estimated_traversal_time.estimated_time_s;
    }
    result.score.score = result.score.breakdown.total();
    result.shadow_lower_bound_valid = std::isfinite(result.score.score);
    result.shadow_lower_bound_score = result.score.score;
    result.shadow_lower_bound_incumbent_score = incumbent_score;
    result.shadow_lower_bound_would_prune =
        std::isfinite(result.shadow_lower_bound_score) &&
        std::isfinite(incumbent_score) &&
        result.shadow_lower_bound_score + 1.0e-9 >= incumbent_score;
    result.score_duration_ms += elapsedMilliseconds(prefilter_started_at);
    const auto score_started_at = std::chrono::steady_clock::now();
    result.score = scoreForCandidate(corridor_samples, buffer.points, buffer.offsets,
                                     result.path, config, speed_config, max_length_m,
                                     buffer.samples, local_stats);
    const double full_score_duration_ms = elapsedMilliseconds(score_started_at);
    result.score_duration_ms += full_score_duration_ms;
    result.full_score_duration_ms += full_score_duration_ms;
    result.sample_build_duration_ms = local_stats.candidate_sample_build_duration_ms;
    result.cost_breakdown_duration_ms =
        local_stats.candidate_cost_breakdown_duration_ms;
    result.shape_diagnostics_duration_ms =
        local_stats.candidate_shape_diagnostics_duration_ms;
    result.speed_profile_duration_ms = local_stats.candidate_speed_profile_duration_ms;
    result.full_score_used = true;
    return result;
  }

  result.requires_full_score = true;
  result.offsets.assign(buffer.offsets.begin(), buffer.offsets.end());

  RacingLineStats local_stats{};
  const auto points_started_at = std::chrono::steady_clock::now();
  pointsFromOffsets(corridor_samples, buffer.offsets, buffer.points);
  result.point_build_duration_ms += elapsedMilliseconds(points_started_at);
  const auto full_evaluation_started_at = std::chrono::steady_clock::now();
  result.path = evaluatePathCached(
      prohibited_grid, buffer.points, buffer.full_path_segment_cache,
      result.full_path_segment_cache_hits, result.full_path_segment_cache_misses);
  result.path_evaluation_duration_ms += elapsedMilliseconds(full_evaluation_started_at);
  const auto score_started_at = std::chrono::steady_clock::now();
  result.score = scoreForCandidate(corridor_samples, buffer.points, buffer.offsets,
                                   result.path, config, speed_config, max_length_m,
                                   buffer.samples, local_stats);
  const double full_score_duration_ms = elapsedMilliseconds(score_started_at);
  result.score_duration_ms += full_score_duration_ms;
  result.full_score_duration_ms += full_score_duration_ms;
  result.sample_build_duration_ms = local_stats.candidate_sample_build_duration_ms;
  result.cost_breakdown_duration_ms = local_stats.candidate_cost_breakdown_duration_ms;
  result.shape_diagnostics_duration_ms =
      local_stats.candidate_shape_diagnostics_duration_ms;
  result.speed_profile_duration_ms = local_stats.candidate_speed_profile_duration_ms;
  result.full_score_used = true;
  return result;
}

[[nodiscard]] std::vector<double> offsetCandidatesForSample(
    const CorridorSample& sample, const double offset_step_m,
    const std::optional<double> center_offset = std::nullopt,
    const double radius_m = std::numeric_limits<double>::infinity()) {
  const double step = sanitizedPositive(offset_step_m, 1.0, 0.05, 100.0);
  std::vector<double> candidates;
  const double lower_bound =
      center_offset.has_value() && std::isfinite(radius_m)
          ? std::max(-sample.right_bound_m, *center_offset - radius_m)
          : -sample.right_bound_m;
  const double upper_bound =
      center_offset.has_value() && std::isfinite(radius_m)
          ? std::min(sample.left_bound_m, *center_offset + radius_m)
          : sample.left_bound_m;
  const auto push_if_allowed = [&](const double offset) {
    if (offset + 1.0e-9 >= lower_bound && offset <= upper_bound + 1.0e-9) {
      candidates.push_back(std::clamp(offset, lower_bound, upper_bound));
    }
  };
  push_if_allowed(0.0);
  if (center_offset.has_value()) {
    push_if_allowed(*center_offset);
  }
  if (std::isfinite(sample.left_bound_m) && sample.left_bound_m > 0.0) {
    const auto left_steps =
        static_cast<std::size_t>(std::floor(sample.left_bound_m / step));
    for (std::size_t step_index = 1U; step_index <= left_steps; ++step_index) {
      const double offset = static_cast<double>(step_index) * step;
      push_if_allowed(std::min(offset, sample.left_bound_m));
    }
    push_if_allowed(sample.left_bound_m);
  }
  if (std::isfinite(sample.right_bound_m) && sample.right_bound_m > 0.0) {
    const auto right_steps =
        static_cast<std::size_t>(std::floor(sample.right_bound_m / step));
    for (std::size_t step_index = 1U; step_index <= right_steps; ++step_index) {
      const double offset = -static_cast<double>(step_index) * step;
      push_if_allowed(std::max(offset, -sample.right_bound_m));
    }
    push_if_allowed(-sample.right_bound_m);
  }
  if (candidates.empty() && lower_bound <= upper_bound) {
    candidates.push_back(
        std::clamp(center_offset.value_or(0.0), lower_bound, upper_bound));
  }
  std::sort(candidates.begin(), candidates.end());
  candidates.erase(std::unique(candidates.begin(), candidates.end(),
                               [](const double lhs, const double rhs) {
                                 return std::abs(lhs - rhs) <= 1.0e-9;
                               }),
                   candidates.end());
  return candidates;
}

[[nodiscard]] bool buildDpSeedForWindow(
    const std::span<const CorridorSample> corridor_samples, const ActiveWindow& window,
    const OccupancyGrid2D& prohibited_grid, const RacingLineConfig& config,
    const double requested_step_m, const std::span<const double> base_offsets,
    const std::span<const double> guide_offsets, const double guide_radius_m,
    std::vector<double>& output_offsets, RacingLineStats& stats) {
  if (window.end_index <= window.begin_index + 1U ||
      base_offsets.size() != corridor_samples.size()) {
    return false;
  }

  const auto started_at = std::chrono::steady_clock::now();
  const double offset_step_m = sanitizedPositive(requested_step_m, 1.0, 0.05, 100.0);
  std::vector<std::size_t> indices;
  indices.reserve(window.end_index - window.begin_index - 1U);
  for (std::size_t i = window.begin_index + 1U; i < window.end_index; ++i) {
    indices.push_back(i);
  }
  if (indices.empty()) {
    stats.dp_duration_ms += elapsedMilliseconds(started_at);
    return false;
  }

  std::vector<std::vector<double>> offset_candidates;
  offset_candidates.reserve(indices.size());
  for (const std::size_t sample_index : indices) {
    const std::optional<double> guide =
        guide_offsets.size() == corridor_samples.size()
            ? std::optional<double>{guide_offsets[sample_index]}
            : std::nullopt;
    offset_candidates.push_back(offsetCandidatesForSample(
        corridor_samples[sample_index], offset_step_m, guide, guide_radius_m));
    stats.dp_states += offset_candidates.back().size();
  }

  constexpr double kDpInfinity = std::numeric_limits<double>::infinity();
  std::vector<std::vector<double>> cost(offset_candidates.size());
  std::vector<std::vector<std::size_t>> parent(offset_candidates.size());
  for (std::size_t row = 0U; row < offset_candidates.size(); ++row) {
    cost[row].assign(offset_candidates[row].size(), kDpInfinity);
    parent[row].assign(offset_candidates[row].size(), 0U);
  }

  const auto point_for = [&](const std::size_t sample_index, const double offset) {
    return corridor_samples[sample_index].center +
           corridor_samples[sample_index].normal * offset;
  };
  SegmentTraversabilityCache segment_cache{};
  const Point2 window_start =
      point_for(window.begin_index, base_offsets[window.begin_index]);
  const Point2 window_end = point_for(window.end_index, base_offsets[window.end_index]);
  const double weight_offset_change =
      sanitizedPositive(config.weight_offset_change, 0.5, 0.0, 1.0e9);

  for (std::size_t candidate_index = 0U;
       candidate_index < offset_candidates.front().size(); ++candidate_index) {
    const double offset = offset_candidates.front()[candidate_index];
    const Point2 point = point_for(indices.front(), offset);
    if (!cachedSegmentTraversable(prohibited_grid, window_start, point, segment_cache,
                                  stats.dp_segment_cache_hits,
                                  stats.dp_segment_cache_misses)) {
      continue;
    }
    const double offset_delta = offset - base_offsets[window.begin_index];
    cost.front()[candidate_index] = distance(window_start, point) +
                                    weight_offset_change * offset_delta * offset_delta;
  }

  for (std::size_t row = 1U; row < offset_candidates.size(); ++row) {
    const std::size_t previous_sample_index = indices[row - 1U];
    const std::size_t sample_index = indices[row];
    for (std::size_t candidate_index = 0U;
         candidate_index < offset_candidates[row].size(); ++candidate_index) {
      const double offset = offset_candidates[row][candidate_index];
      const Point2 point = point_for(sample_index, offset);
      for (std::size_t previous_candidate_index = 0U;
           previous_candidate_index < offset_candidates[row - 1U].size();
           ++previous_candidate_index) {
        ++stats.dp_transitions;
        if (!std::isfinite(cost[row - 1U][previous_candidate_index])) {
          continue;
        }
        const double previous_offset =
            offset_candidates[row - 1U][previous_candidate_index];
        const Point2 previous_point = point_for(previous_sample_index, previous_offset);
        if (!cachedSegmentTraversable(prohibited_grid, previous_point, point,
                                      segment_cache, stats.dp_segment_cache_hits,
                                      stats.dp_segment_cache_misses)) {
          continue;
        }
        const double offset_delta = offset - previous_offset;
        const double candidate_cost =
            cost[row - 1U][previous_candidate_index] + distance(previous_point, point) +
            weight_offset_change * offset_delta * offset_delta;
        if (candidate_cost + 1.0e-9 < cost[row][candidate_index]) {
          cost[row][candidate_index] = candidate_cost;
          parent[row][candidate_index] = previous_candidate_index;
        }
      }
    }
  }

  std::size_t best_index = 0U;
  double best_cost = kDpInfinity;
  const std::size_t last_row = offset_candidates.size() - 1U;
  const std::size_t last_sample_index = indices.back();
  for (std::size_t candidate_index = 0U;
       candidate_index < offset_candidates[last_row].size(); ++candidate_index) {
    if (!std::isfinite(cost[last_row][candidate_index])) {
      continue;
    }
    const double offset = offset_candidates[last_row][candidate_index];
    const Point2 point = point_for(last_sample_index, offset);
    if (!cachedSegmentTraversable(prohibited_grid, point, window_end, segment_cache,
                                  stats.dp_segment_cache_hits,
                                  stats.dp_segment_cache_misses)) {
      continue;
    }
    const double offset_delta = base_offsets[window.end_index] - offset;
    const double candidate_cost = cost[last_row][candidate_index] +
                                  distance(point, window_end) +
                                  weight_offset_change * offset_delta * offset_delta;
    if (candidate_cost + 1.0e-9 < best_cost) {
      best_cost = candidate_cost;
      best_index = candidate_index;
    }
  }

  if (!std::isfinite(best_cost)) {
    stats.dp_duration_ms += elapsedMilliseconds(started_at);
    return false;
  }

  output_offsets.assign(base_offsets.begin(), base_offsets.end());
  std::size_t current_index = best_index;
  for (std::size_t reverse_row = offset_candidates.size(); reverse_row > 0U;
       --reverse_row) {
    const std::size_t row = reverse_row - 1U;
    output_offsets[indices[row]] = offset_candidates[row][current_index];
    if (row > 0U) {
      current_index = parent[row][current_index];
    }
  }
  stats.dp_duration_ms += elapsedMilliseconds(started_at);
  return true;
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

[[nodiscard]] std::vector<CandidateTask>
candidateTasksForStep(const std::span<const std::size_t> control_indices,
                      const double step) {
  std::vector<CandidateTask> tasks;
  tasks.reserve(control_indices.size() * 2U);
  for (const std::size_t index : control_indices) {
    tasks.push_back(CandidateTask{
        .order = tasks.size(),
        .center_index = index,
        .delta_m = -step,
    });
    tasks.push_back(CandidateTask{
        .order = tasks.size(),
        .center_index = index,
        .delta_m = step,
    });
  }
  return tasks;
}

[[nodiscard]] std::vector<CandidateBatchResult> evaluateCandidateBatch(
    const std::span<const CandidateTask> tasks,
    const std::span<const CorridorSample> corridor_samples,
    const std::span<const double> base_offsets,
    const std::span<const Point2> base_points, const CandidateScore& base_score,
    const double base_length_m, const OccupancyGrid2D& prohibited_grid,
    const RacingLineConfig& config, const VelocityFollowerConfig& speed_config,
    const double max_length_m, const std::span<const std::uint8_t> mutable_indices,
    const double incumbent_score, const std::size_t worker_count) {
  std::vector<CandidateBatchResult> results(tasks.size());
  if (tasks.empty()) {
    return results;
  }

  const std::size_t resolved_workers = std::clamp<std::size_t>(
      worker_count, 1U, std::max<std::size_t>(1U, tasks.size()));
  std::vector<CandidateWorkBuffer> worker_buffers(resolved_workers);
  for (CandidateWorkBuffer& buffer : worker_buffers) {
    buffer.offsets.reserve(base_offsets.size());
    buffer.points.reserve(corridor_samples.size());
    buffer.local_base_points.reserve(
        std::min<std::size_t>(corridor_samples.size(), 16U));
    buffer.local_candidate_points.reserve(
        std::min<std::size_t>(corridor_samples.size(), 16U));
    buffer.local_corridor_samples.reserve(
        std::min<std::size_t>(corridor_samples.size(), 16U));
    buffer.local_base_offsets.reserve(
        std::min<std::size_t>(corridor_samples.size(), 16U));
    buffer.local_candidate_offsets.reserve(
        std::min<std::size_t>(corridor_samples.size(), 16U));
    buffer.local_base_samples.reserve(
        std::min<std::size_t>(corridor_samples.size(), 16U));
    buffer.local_candidate_samples.reserve(
        std::min<std::size_t>(corridor_samples.size(), 16U));
    buffer.samples.reserve(corridor_samples.size());
  }

  const auto evaluate_one = [&](const std::size_t task_index,
                                CandidateWorkBuffer& buffer) {
    const CandidateTask& task = tasks[task_index];
    results[task_index].order = task.order;
    results[task_index].candidate = evaluateCandidateSnapshot(
        corridor_samples, base_offsets, base_points, base_score, base_length_m,
        task.center_index, task.delta_m, prohibited_grid, config, speed_config,
        max_length_m, mutable_indices, incumbent_score, buffer);
  };

  if (resolved_workers == 1U) {
    for (std::size_t task_index = 0U; task_index < tasks.size(); ++task_index) {
      evaluate_one(task_index, worker_buffers.front());
    }
    return results;
  }

  std::vector<std::thread> workers;
  workers.reserve(resolved_workers);
  for (std::size_t worker_index = 0U; worker_index < resolved_workers; ++worker_index) {
    workers.emplace_back([&, worker_index] {
      CandidateWorkBuffer& buffer = worker_buffers[worker_index];
      const std::size_t begin = tasks.size() * worker_index / resolved_workers;
      const std::size_t end = tasks.size() * (worker_index + 1U) / resolved_workers;
      for (std::size_t task_index = begin; task_index < end; ++task_index) {
        evaluate_one(task_index, buffer);
      }
    });
  }
  for (std::thread& worker : workers) {
    worker.join();
  }
  return results;
}

void incrementLocalFullScoreReason(const LocalFullScoreReason reason,
                                   RacingLineStats& stats) {
  switch (reason) {
    case LocalFullScoreReason::kNone:
      return;
    case LocalFullScoreReason::kInvalidInput:
      ++stats.local_candidate_full_score_required_invalid_input;
      return;
    case LocalFullScoreReason::kBoundaryWindow:
      ++stats.local_candidate_full_score_required_boundary;
      return;
    case LocalFullScoreReason::kUnsafeBase:
      ++stats.local_candidate_full_score_required_unsafe_base;
      return;
    case LocalFullScoreReason::kWindowInvalid:
      ++stats.local_candidate_full_score_required_window_invalid;
      return;
  }
}

void mergeCandidateStats(const EvaluatedCandidate& candidate, RacingLineStats& stats) {
  if (candidate.scratch_reused) {
    ++stats.worker_scratch_reuses;
  }
  if (candidate.snapshot_allocation_avoided) {
    ++stats.candidate_snapshot_allocations_avoided;
  }
  if (candidate.noop) {
    ++stats.skipped_noop_candidates;
    return;
  }
  if (candidate.local_evaluated) {
    ++stats.local_candidate_evaluations;
    stats.local_candidate_point_build_duration_ms +=
        candidate.local_point_build_duration_ms;
    stats.local_candidate_path_evaluation_duration_ms +=
        candidate.local_path_evaluation_duration_ms;
    stats.local_candidate_score_duration_ms += candidate.local_score_duration_ms;
    stats.local_candidate_traversal_estimate_duration_ms +=
        candidate.local_traversal_estimate_duration_ms;
    stats.candidate_segment_cache_hits += candidate.local_segment_cache_hits;
    stats.candidate_segment_cache_misses += candidate.local_segment_cache_misses;
    if (candidate.requires_full_score) {
      ++stats.local_candidate_full_score_required;
      incrementLocalFullScoreReason(candidate.local_full_score_reason, stats);
    }
  }
  stats.full_path_segment_cache_hits += candidate.full_path_segment_cache_hits;
  stats.full_path_segment_cache_misses += candidate.full_path_segment_cache_misses;
  if (candidate.full_score_used) {
    ++stats.local_candidate_full_score_fallbacks;
    stats.full_candidate_score_duration_ms += candidate.full_score_duration_ms;
  }
  if (candidate.shadow_lower_bound_valid) {
    ++stats.shadow_lower_bound_evaluations;
    if (candidate.full_score_used) {
      ++stats.shadow_lower_bound_validation_full_scores;
      stats.shadow_lower_bound_validation_full_score_duration_ms +=
          candidate.full_score_duration_ms;
    }
    if (candidate.full_score_used && std::isfinite(candidate.score.score)) {
      const double delta = candidate.shadow_lower_bound_score - candidate.score.score;
      if (std::isfinite(delta)) {
        stats.shadow_lower_bound_max_overestimate_score =
            std::max(stats.shadow_lower_bound_max_overestimate_score, delta);
        stats.shadow_lower_bound_max_underestimate_score =
            std::max(stats.shadow_lower_bound_max_underestimate_score, -delta);
      }
    }
    if (candidate.shadow_lower_bound_would_prune) {
      ++stats.shadow_lower_bound_prunable;
      if (candidate.full_score_used) {
        stats.shadow_lower_bound_prunable_full_score_duration_ms +=
            candidate.full_score_duration_ms;
      }
      if (candidate.full_score_used && std::isfinite(candidate.score.score) &&
          std::isfinite(candidate.shadow_lower_bound_incumbent_score) &&
          candidate.score.score + 1.0e-9 <
              candidate.shadow_lower_bound_incumbent_score) {
        ++stats.shadow_lower_bound_false_prunes;
        stats.shadow_lower_bound_max_false_prune_improvement_score = std::max(
            stats.shadow_lower_bound_max_false_prune_improvement_score,
            candidate.shadow_lower_bound_incumbent_score - candidate.score.score);
      }
    }
  } else if (candidate.local_evaluated) {
    ++stats.shadow_lower_bound_unavailable;
  }
  ++stats.candidate_evaluations;
  stats.candidate_point_build_duration_ms += candidate.point_build_duration_ms;
  stats.candidate_path_evaluation_duration_ms += candidate.path_evaluation_duration_ms;
  stats.candidate_score_duration_ms += candidate.score_duration_ms;
  stats.candidate_sample_build_duration_ms += candidate.sample_build_duration_ms;
  stats.candidate_cost_breakdown_duration_ms += candidate.cost_breakdown_duration_ms;
  stats.candidate_shape_diagnostics_duration_ms +=
      candidate.shape_diagnostics_duration_ms;
  stats.candidate_speed_profile_duration_ms += candidate.speed_profile_duration_ms;
  if (!candidate.path.traversable()) {
    ++stats.collision_rejections;
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
  if (corridor_samples.size() < 2U) {
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
  const std::vector<ActiveWindow> active_windows = detectActiveWindows(
      optimizer_samples, centerline, prohibited_grid, config, result.stats);
  result.active_windows = windowMetadata(active_windows, optimizer_samples);
  std::vector<std::uint8_t> mutable_indices;
  const std::vector<std::size_t> control_indices =
      activeControlIndices(active_windows, sample_count, mutable_indices);
  result.stats.parallel_workers_used = 1U;

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
  std::size_t candidate_worker_count = 1U;

  std::vector<double> offsets;
  offsets.reserve(sample_count);
  std::vector<Point2> best_points;
  best_points.reserve(sample_count);
  double best_cost = std::numeric_limits<double>::infinity();
  CandidateScore best_score{};
  double best_length_m = 0.0;
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
    (void)updateBestCandidate(
        optimizer_samples, scratch.candidate_offsets, scratch.candidate_points,
        prohibited_grid, config, speed_config, max_length_m, best_cost, offsets,
        best_points, best_score, best_length_m, scratch.candidate_samples,
        scratch.full_path_segment_cache, result.stats);
  }
  if (offsets.empty()) {
    return result;
  }
  result.stats.initial_cost = best_cost;

  const auto window_eval_started_at = std::chrono::steady_clock::now();
  for (const ActiveWindow& window : active_windows) {
    const std::size_t states_before = result.stats.dp_states;
    const std::size_t transitions_before = result.stats.dp_transitions;
    const double coarse_step =
        sanitizedPositive(config.dp_coarse_offset_step_m, 2.0, 0.05, 100.0);
    const double fine_step =
        sanitizedPositive(config.dp_fine_offset_step_m, 0.75, 0.05, 100.0);
    const double fine_radius =
        sanitizedPositive(config.dp_fine_radius_m, 1.5, 0.05, 5000.0);
    const bool coarse_ok = buildDpSeedForWindow(
        optimizer_samples, window, prohibited_grid, config, coarse_step, offsets, {},
        std::numeric_limits<double>::infinity(), scratch.accepted_offsets,
        result.stats);
    result.stats.dp_coarse_states += result.stats.dp_states - states_before;
    result.stats.dp_coarse_transitions +=
        result.stats.dp_transitions - transitions_before;
    bool dp_ok = false;
    if (coarse_ok) {
      const std::size_t fine_states_before = result.stats.dp_states;
      const std::size_t fine_transitions_before = result.stats.dp_transitions;
      dp_ok =
          buildDpSeedForWindow(optimizer_samples, window, prohibited_grid, config,
                               fine_step, offsets, scratch.accepted_offsets,
                               fine_radius, scratch.candidate_offsets, result.stats);
      result.stats.dp_fine_states += result.stats.dp_states - fine_states_before;
      result.stats.dp_fine_transitions +=
          result.stats.dp_transitions - fine_transitions_before;
      result.stats.dp_coarse_to_fine_used =
          result.stats.dp_coarse_to_fine_used || dp_ok;
    }
    if (!dp_ok && !buildDpSeedForWindow(optimizer_samples, window, prohibited_grid,
                                        config, config.dp_offset_step_m, offsets, {},
                                        std::numeric_limits<double>::infinity(),
                                        scratch.candidate_offsets, result.stats)) {
      continue;
    }
    const auto points_started_at = std::chrono::steady_clock::now();
    pointsFromOffsets(optimizer_samples, scratch.candidate_offsets,
                      scratch.candidate_points);
    result.stats.candidate_point_build_duration_ms +=
        elapsedMilliseconds(points_started_at);
    const bool accepted = updateBestCandidate(
        optimizer_samples, scratch.candidate_offsets, scratch.candidate_points,
        prohibited_grid, config, speed_config, max_length_m, best_cost, offsets,
        best_points, best_score, best_length_m, scratch.candidate_samples,
        scratch.full_path_segment_cache, result.stats);
    if (accepted) {
      result.stats.initial_cost = best_cost;
    }
  }
  result.stats.window_eval_duration_ms += elapsedMilliseconds(window_eval_started_at);

  for (std::size_t iteration = 0U; iteration < max_iterations && step >= min_step;
       ++iteration) {
    if (control_indices.empty()) {
      break;
    }
    const std::vector<CandidateTask> tasks =
        candidateTasksForStep(control_indices, step);
    candidate_worker_count = desiredWorkerCount(config.parallel_workers, tasks.size());
    result.stats.parallel_candidate_evaluation_used =
        result.stats.parallel_candidate_evaluation_used || candidate_worker_count > 1U;
    result.stats.parallel_workers_used =
        std::max(result.stats.parallel_workers_used, candidate_worker_count);
    ++result.stats.candidate_chunks;
    std::vector<CandidateBatchResult> candidates = evaluateCandidateBatch(
        tasks, optimizer_samples, offsets, best_points, best_score, best_length_m,
        prohibited_grid, config, speed_config, max_length_m, mutable_indices, best_cost,
        candidate_worker_count);

    bool changed = false;
    const EvaluatedCandidate* iteration_winner = nullptr;
    double best_estimated_score = best_cost;
    for (const CandidateBatchResult& batch_result : candidates) {
      const EvaluatedCandidate& candidate = batch_result.candidate;
      mergeCandidateStats(candidate, result.stats);
      if (candidate.noop || candidate.offsets.empty()) {
        continue;
      }
      if (!candidate.full_score_used) {
        continue;
      }
      if (candidate.score.score + 1.0e-9 < best_cost) {
        if (candidate.score.score + 1.0e-9 < best_estimated_score) {
          best_estimated_score = candidate.score.score;
          iteration_winner = &candidate;
        }
      }
    }
    if (iteration_winner != nullptr) {
      if (iteration_winner->shadow_lower_bound_would_prune) {
        ++result.stats.shadow_lower_bound_winner_prunes;
      }
      best_cost = iteration_winner->score.score;
      offsets = iteration_winner->offsets;
      const auto accepted_points_started_at = std::chrono::steady_clock::now();
      pointsFromOffsets(optimizer_samples, offsets, scratch.accepted_points);
      result.stats.candidate_point_build_duration_ms +=
          elapsedMilliseconds(accepted_points_started_at);
      best_points = scratch.accepted_points;
      best_score = iteration_winner->score;
      best_length_m = iteration_winner->path.length_m;
      ++result.stats.local_candidate_acceptance_full_scores;
      copyTraversalEstimateToBestCandidateStats(iteration_winner->score.traversal_time,
                                                iteration_winner->score.score,
                                                result.stats);
      changed = true;
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
    std::size_t cache_hits = 0U;
    std::size_t cache_misses = 0U;
    const PathEvaluation candidate_evaluation =
        evaluatePathCached(prohibited_grid, scratch.candidate_points,
                           scratch.full_path_segment_cache, cache_hits, cache_misses);
    result.stats.full_path_segment_cache_hits += cache_hits;
    result.stats.full_path_segment_cache_misses += cache_misses;
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
  std::size_t final_cache_hits = 0U;
  std::size_t final_cache_misses = 0U;
  const PathEvaluation final_evaluation =
      evaluatePathCached(prohibited_grid, final_points, scratch.full_path_segment_cache,
                         final_cache_hits, final_cache_misses);
  result.stats.full_path_segment_cache_hits += final_cache_hits;
  result.stats.full_path_segment_cache_misses += final_cache_misses;
  if (!final_evaluation.traversable()) {
    ++result.stats.collision_rejections;
    return result;
  }
  const auto final_score_started_at = std::chrono::steady_clock::now();
  const CandidateScore final_score = scoreForCandidate(
      optimizer_samples, final_points, final_offsets, final_evaluation, config,
      speed_config, max_length_m, scratch.candidate_samples, result.stats);
  result.stats.full_final_score_duration_ms =
      elapsedMilliseconds(final_score_started_at);

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
  updateEdgeMarginStats(result.samples, result.stats);
  result.valid = trajectorySamplesAreUsable(result.samples);
  return result;
}

} // namespace drone_city_nav
