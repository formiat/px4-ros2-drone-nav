#include "drone_city_nav/turn_smoothing.hpp"

#include "drone_city_nav/trajectory_speed_planner.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;

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

[[nodiscard]] double cross(const Point2 lhs, const Point2 rhs) noexcept {
  return lhs.x * rhs.y - lhs.y * rhs.x;
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

[[nodiscard]] Point2 leftNormal(const Point2 tangent) noexcept {
  return Point2{-tangent.y, tangent.x};
}

[[nodiscard]] double normalizeAngle(const double angle_rad) noexcept {
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

[[nodiscard]] double headingOf(const Point2 vector) noexcept {
  return std::atan2(vector.y, vector.x);
}

[[nodiscard]] double signedHeadingDelta(const Point2 previous,
                                        const Point2 next) noexcept {
  return normalizeAngle(headingOf(next) - headingOf(previous));
}

[[nodiscard]] double radiansToDegrees(const double radians) noexcept {
  return radians * 180.0 / std::numbers::pi;
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
curvatureSpeedLimitMps(const double radius_m,
                       const VelocityFollowerConfig& config) noexcept {
  if (!(radius_m > kTinyDistanceM) || !std::isfinite(radius_m)) {
    return sanitizedPositive(config.cruise_speed_mps, 12.0, 0.0, 100.0);
  }
  const double cruise_speed =
      sanitizedPositive(config.cruise_speed_mps, 12.0, 0.0, 100.0);
  const double min_turn_speed = std::min(
      sanitizedPositive(config.min_turn_speed_mps, 2.0, 0.0, 100.0), cruise_speed);
  const double max_lateral_accel =
      sanitizedPositive(config.max_lateral_accel_mps2, 3.0, 1.0e-6, 100.0);
  return std::clamp(std::sqrt(max_lateral_accel * radius_m), min_turn_speed,
                    cruise_speed);
}

[[nodiscard]] double
elapsedMilliseconds(const std::chrono::steady_clock::time_point start) {
  return static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                 std::chrono::steady_clock::now() - start)
                                 .count()) /
         1000.0;
}

[[nodiscard]] std::int64_t quantizedCacheValue(const double value) noexcept {
  if (!std::isfinite(value)) {
    return 0;
  }
  return static_cast<std::int64_t>(std::llround(value * 1.0e6));
}

[[nodiscard]] std::vector<double>
distanceFallbackCandidates(const double max_distance_m) {
  std::vector<double> candidates;
  if (!(max_distance_m > kTinyDistanceM)) {
    return candidates;
  }
  constexpr double kFallbackStepM = 5.0;
  constexpr double kMinFallbackDistanceM = 5.0;
  constexpr double kMaxFallbackDistanceM = 60.0;
  double candidate = kMaxFallbackDistanceM;
  while (candidate >= kMinFallbackDistanceM - kTinyDistanceM) {
    candidates.push_back(candidate);
    candidate -= kFallbackStepM;
  }
  return candidates;
}

[[nodiscard]] std::array<double, 6U> relaxedTangentAngleCandidatesRad() noexcept {
  return std::array<double, 6U>{
      5.0 * std::numbers::pi / 180.0,  10.0 * std::numbers::pi / 180.0,
      15.0 * std::numbers::pi / 180.0, 20.0 * std::numbers::pi / 180.0,
      25.0 * std::numbers::pi / 180.0, 30.0 * std::numbers::pi / 180.0};
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
      samples[i].tangent = normalized(samples[1U].point - samples[0U].point);
    } else if (i + 1U == samples.size()) {
      samples[i].tangent = normalized(samples[i].point - samples[i - 1U].point);
    } else {
      samples[i].tangent = normalized(samples[i + 1U].point - samples[i - 1U].point);
      samples[i].curvature_1pm = discreteCurvature(
          samples[i - 1U].point, samples[i].point, samples[i + 1U].point);
    }
  }
}

[[nodiscard]] double pathLength(const std::span<const TrajectoryPointSample> samples,
                                const std::size_t start_index,
                                const std::size_t end_index) {
  if (samples.empty() || start_index >= samples.size() || end_index >= samples.size() ||
      start_index >= end_index) {
    return 0.0;
  }
  double length = 0.0;
  for (std::size_t i = start_index + 1U; i <= end_index; ++i) {
    length += distance(samples[i - 1U].point, samples[i].point);
  }
  return length;
}

[[nodiscard]] double pathLength(const std::span<const TrajectoryPointSample> samples) {
  if (samples.size() < 2U) {
    return 0.0;
  }
  return pathLength(samples, 0U, samples.size() - 1U);
}

[[nodiscard]] std::optional<CorridorSample>
corridorSampleAtS(const std::span<const CorridorSample> corridor_samples,
                  const double s_m) {
  if (corridor_samples.empty()) {
    return std::nullopt;
  }
  if (s_m <= corridor_samples.front().s_m) {
    return corridor_samples.front();
  }
  for (std::size_t i = 1U; i < corridor_samples.size(); ++i) {
    if (s_m > corridor_samples[i].s_m && i + 1U < corridor_samples.size()) {
      continue;
    }
    const CorridorSample& previous = corridor_samples[i - 1U];
    const CorridorSample& next = corridor_samples[i];
    const double span = next.s_m - previous.s_m;
    const double t =
        span > kTinyDistanceM ? std::clamp((s_m - previous.s_m) / span, 0.0, 1.0) : 0.0;
    CorridorSample sample{};
    sample.s_m = previous.s_m + (next.s_m - previous.s_m) * t;
    sample.route_center =
        previous.route_center + (next.route_center - previous.route_center) * t;
    sample.center = previous.center + (next.center - previous.center) * t;
    sample.tangent =
        normalized(previous.tangent + (next.tangent - previous.tangent) * t);
    if (!(norm(sample.tangent) > kTinyDistanceM)) {
      sample.tangent = previous.tangent;
    }
    sample.normal = leftNormal(sample.tangent);
    sample.left_bound_m =
        previous.left_bound_m + (next.left_bound_m - previous.left_bound_m) * t;
    sample.right_bound_m =
        previous.right_bound_m + (next.right_bound_m - previous.right_bound_m) * t;
    sample.clearance_m =
        previous.clearance_m + (next.clearance_m - previous.clearance_m) * t;
    sample.center_recovery_m =
        previous.center_recovery_m +
        (next.center_recovery_m - previous.center_recovery_m) * t;
    return sample;
  }
  return corridor_samples.back();
}

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

[[nodiscard]] SegmentCellKey orderedSegmentKey(GridIndex start,
                                               GridIndex end) noexcept {
  if (end.x < start.x || (end.x == start.x && end.y < start.y)) {
    std::swap(start, end);
  }
  return SegmentCellKey{.start = start, .end = end};
}

[[nodiscard]] bool cachedSegmentIsTraversable(const OccupancyGrid2D& grid,
                                              const Point2 start, const Point2 end,
                                              SegmentTraversabilityCache& cache,
                                              TurnSmoothingStats& stats) {
  const std::optional<GridIndex> start_cell = grid.worldToCell(start);
  const std::optional<GridIndex> end_cell = grid.worldToCell(end);
  if (!start_cell.has_value() || !end_cell.has_value()) {
    ++stats.traversability_cache_misses;
    return false;
  }
  const SegmentCellKey key = orderedSegmentKey(*start_cell, *end_cell);
  if (const auto iter = cache.values.find(key); iter != cache.values.end()) {
    ++stats.traversability_cache_hits;
    return iter->second;
  }
  ++stats.traversability_cache_misses;
  const bool traversable = segmentIsTraversable(grid, start, end);
  cache.values.emplace(key, traversable);
  return traversable;
}

[[nodiscard]] bool
pathIsTraversable(const OccupancyGrid2D& grid,
                  const std::span<const TrajectoryPointSample> samples) {
  if (samples.size() < 2U) {
    return false;
  }
  for (std::size_t i = 1U; i < samples.size(); ++i) {
    if (!segmentIsTraversable(grid, samples[i - 1U].point, samples[i].point)) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] bool
pathIsTraversableCached(const OccupancyGrid2D& grid,
                        const std::span<const TrajectoryPointSample> samples,
                        SegmentTraversabilityCache& cache, TurnSmoothingStats& stats) {
  if (samples.size() < 2U) {
    return false;
  }
  for (std::size_t i = 1U; i < samples.size(); ++i) {
    if (!cachedSegmentIsTraversable(grid, samples[i - 1U].point, samples[i].point,
                                    cache, stats)) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] double innerMarginM(const TrajectoryPointSample& sample,
                                  const double turn_sign) noexcept {
  if (turn_sign > 0.0) {
    return sample.left_bound_m - sample.lateral_offset_m;
  }
  return sample.right_bound_m + sample.lateral_offset_m;
}

void updateMinInnerMargin(TurnSmoothingStats& stats,
                          const TrajectoryPointSample& sample,
                          const double turn_sign) noexcept {
  const double margin = innerMarginM(sample, turn_sign);
  if (!std::isfinite(margin)) {
    return;
  }
  if (!std::isfinite(stats.min_inner_margin_m)) {
    stats.min_inner_margin_m = margin;
    return;
  }
  stats.min_inner_margin_m = std::min(stats.min_inner_margin_m, margin);
}

struct CornerCandidate {
  bool valid{false};
  std::size_t index{0U};
  double signed_heading_delta_rad{0.0};
  double abs_heading_delta_rad{0.0};
  double radius_m{std::numeric_limits<double>::infinity()};
  double turn_sign{1.0};
};

enum class SmoothingRejectReason : std::uint8_t {
  kNone,
  kProhibited,
  kNotImproved,
  kCurvatureRegression,
  kRadiusRegression,
  kSpeedRegression,
};

struct LocalTrajectoryMetrics {
  bool valid{false};
  double length_m{0.0};
  double max_abs_curvature_1pm{0.0};
  double min_radius_m{std::numeric_limits<double>::infinity()};
  double min_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  double estimated_time_s{std::numeric_limits<double>::quiet_NaN()};
  TrajectoryShapeDiagnostics shape{};
};

struct SmoothingAttempt {
  std::vector<TrajectoryPointSample> samples;
  SmoothingRejectReason reject_reason{SmoothingRejectReason::kNone};
  const char* reject_detail{"none"};
  LocalTrajectoryMetrics before_metrics{};
  LocalTrajectoryMetrics after_metrics{};
  double applied_shift_m{0.0};
  double entry_distance_m{0.0};
  double exit_distance_m{0.0};
  double shift_scale{0.0};
  double relaxed_angle_rad{0.0};
  double score{std::numeric_limits<double>::infinity()};
  bool accepted{false};
};

struct BezierCacheKey {
  std::size_t corner_index{0U};
  std::size_t entry_index{0U};
  std::size_t exit_index{0U};
  std::int64_t shift_scale{0};
  std::int64_t relaxed_angle_rad{0};
  std::int64_t sample_step_m{0};

  [[nodiscard]] bool operator==(const BezierCacheKey& other) const noexcept {
    return corner_index == other.corner_index && entry_index == other.entry_index &&
           exit_index == other.exit_index && shift_scale == other.shift_scale &&
           relaxed_angle_rad == other.relaxed_angle_rad &&
           sample_step_m == other.sample_step_m;
  }
};

struct BezierCacheKeyHash {
  [[nodiscard]] std::size_t operator()(const BezierCacheKey& key) const noexcept {
    const auto mix = [](const std::size_t lhs, const std::size_t rhs) {
      return lhs ^ (rhs + 0x9e3779b97f4a7c15ULL + (lhs << 6U) + (lhs >> 2U));
    };
    std::size_t hash = std::hash<std::size_t>{}(key.corner_index);
    hash = mix(hash, std::hash<std::size_t>{}(key.entry_index));
    hash = mix(hash, std::hash<std::size_t>{}(key.exit_index));
    hash = mix(hash, std::hash<std::int64_t>{}(key.shift_scale));
    hash = mix(hash, std::hash<std::int64_t>{}(key.relaxed_angle_rad));
    hash = mix(hash, std::hash<std::int64_t>{}(key.sample_step_m));
    return hash;
  }
};

struct IndexRangeKey {
  std::size_t begin_index{0U};
  std::size_t end_index{0U};

  [[nodiscard]] bool operator==(const IndexRangeKey& other) const noexcept {
    return begin_index == other.begin_index && end_index == other.end_index;
  }
};

struct IndexRangeKeyHash {
  [[nodiscard]] std::size_t operator()(const IndexRangeKey& key) const noexcept {
    return std::hash<std::size_t>{}(key.begin_index) ^
           (std::hash<std::size_t>{}(key.end_index) + 0x9e3779b97f4a7c15ULL);
  }
};

struct TurnSmoothingWorkBuffer {
  std::vector<TrajectoryPointSample> before_local;
  std::vector<TrajectoryPointSample> replacement;
  std::vector<TrajectoryPointSample> candidate;
  std::vector<TrajectoryPointSample> after_local;
  SegmentTraversabilityCache traversability_cache;
  std::unordered_map<BezierCacheKey, std::vector<TrajectoryPointSample>,
                     BezierCacheKeyHash>
      bezier_cache;
  std::unordered_map<IndexRangeKey, LocalTrajectoryMetrics, IndexRangeKeyHash>
      before_metrics_cache;
};

[[nodiscard]] CornerCandidate
cornerCandidateAt(const std::span<const TrajectoryPointSample> samples,
                  const std::size_t index, const TurnSmoothingConfig& config,
                  const VelocityFollowerConfig& speed_config) {
  CornerCandidate candidate{};
  if (index == 0U || index + 1U >= samples.size()) {
    return candidate;
  }

  const Point2 previous_direction =
      normalized(samples[index].point - samples[index - 1U].point);
  const Point2 next_direction =
      normalized(samples[index + 1U].point - samples[index].point);
  if (!(norm(previous_direction) > kTinyDistanceM) ||
      !(norm(next_direction) > kTinyDistanceM)) {
    return candidate;
  }

  const double signed_delta = signedHeadingDelta(previous_direction, next_direction);
  const double abs_delta = std::abs(signed_delta);
  const double curvature = std::abs(discreteCurvature(
      samples[index - 1U].point, samples[index].point, samples[index + 1U].point));
  const double radius = curvature > kTinyDistanceM
                            ? 1.0 / curvature
                            : std::numeric_limits<double>::infinity();
  const double trigger_heading_delta =
      sanitizedPositive(config.trigger_heading_delta_rad, 0.65, 0.0, std::numbers::pi);
  const double trigger_min_radius =
      sanitizedPositive(config.trigger_min_radius_m, 16.0, 0.0, 10000.0);
  const double trigger_speed_limit =
      sanitizedPositive(config.trigger_speed_limit_mps, 12.0, 0.0, 1000.0);
  const double geometric_speed_limit = curvatureSpeedLimitMps(radius, speed_config);
  const bool triggered_by_heading = abs_delta >= trigger_heading_delta;
  const bool triggered_by_radius =
      radius < trigger_min_radius && abs_delta > trigger_heading_delta * 0.35;
  const bool triggered_by_speed = trigger_speed_limit > kTinyDistanceM &&
                                  geometric_speed_limit <= trigger_speed_limit &&
                                  abs_delta > trigger_heading_delta * 0.30;
  if (!triggered_by_heading && !triggered_by_radius && !triggered_by_speed) {
    return candidate;
  }

  candidate.valid = true;
  candidate.index = index;
  candidate.signed_heading_delta_rad = signed_delta;
  candidate.abs_heading_delta_rad = abs_delta;
  candidate.radius_m = radius;
  candidate.turn_sign = signed_delta >= 0.0 ? 1.0 : -1.0;
  return candidate;
}

[[nodiscard]] std::size_t
findEntryIndex(const std::span<const TrajectoryPointSample> samples,
               const std::size_t corner, const double entry_distance_m) {
  double accumulated = 0.0;
  std::size_t index = corner;
  while (index > 0U && accumulated < entry_distance_m) {
    accumulated += distance(samples[index].point, samples[index - 1U].point);
    --index;
  }
  return index;
}

[[nodiscard]] std::size_t
findExitIndex(const std::span<const TrajectoryPointSample> samples,
              const std::size_t corner, const double exit_distance_m) {
  double accumulated = 0.0;
  std::size_t index = corner;
  while (index + 1U < samples.size() && accumulated < exit_distance_m) {
    accumulated += distance(samples[index].point, samples[index + 1U].point);
    ++index;
  }
  return index;
}

[[nodiscard]] double outwardShiftFor(const TrajectoryPointSample& sample,
                                     const Point2 outward,
                                     const TurnSmoothingConfig& config) {
  const Point2 normal = leftNormal(sample.tangent);
  const double side = dot(outward, normal);
  const double bound = side >= 0.0 ? sample.left_bound_m : sample.right_bound_m;
  if (!std::isfinite(bound) || !(bound > 0.0)) {
    return 0.0;
  }
  const double ratio = sanitizedPositive(config.outer_bias_ratio, 0.45, 0.0, 1.0);
  const double max_shift =
      sanitizedPositive(config.max_outer_shift_m, 12.0, 0.0, 1000.0);
  const double min_shift =
      sanitizedPositive(config.min_outer_shift_m, 2.0, 0.0, max_shift);
  const double available_shift = bound;
  return std::min(max_shift, std::min(available_shift,
                                      std::max(min_shift, available_shift * ratio)));
}

[[nodiscard]] Point2 cubicBezier(const Point2 p0, const Point2 p1, const Point2 p2,
                                 const Point2 p3, const double t) noexcept {
  const double u = 1.0 - t;
  return p0 * (u * u * u) + p1 * (3.0 * u * u * t) + p2 * (3.0 * u * t * t) +
         p3 * (t * t * t);
}

[[nodiscard]] Point2 tangentRelaxedOutward(const Point2 tangent, const Point2 outward,
                                           const double angle_rad) noexcept {
  const double angle = std::clamp(angle_rad, 0.0, std::numbers::pi / 2.0);
  const Point2 relaxed = tangent * std::cos(angle) + outward * std::sin(angle);
  const Point2 result = normalized(relaxed);
  if (!(norm(result) > kTinyDistanceM)) {
    return tangent;
  }
  return result;
}

[[nodiscard]] TrajectoryPointSample
sampleForPoint(const Point2 point, const double station_hint_m,
               const std::span<const CorridorSample> corridor_samples) {
  TrajectoryPointSample sample{};
  sample.point = point;
  const std::optional<CorridorSample> corridor =
      corridorSampleAtS(corridor_samples, station_hint_m);
  if (corridor.has_value()) {
    sample.tangent = corridor->tangent;
    sample.left_bound_m = corridor->left_bound_m;
    sample.right_bound_m = corridor->right_bound_m;
    sample.lateral_offset_m = dot(point - corridor->center, corridor->normal);
  }
  return sample;
}

[[nodiscard]] std::vector<TrajectoryPointSample>
buildBezierSamples(const std::span<const TrajectoryPointSample> samples,
                   const std::span<const CorridorSample> corridor_samples,
                   const std::size_t entry_index, const std::size_t corner_index,
                   const std::size_t exit_index, const CornerCandidate& corner,
                   const TurnSmoothingConfig& config, const double outward_shift_scale,
                   const double relaxed_angle_rad, double& applied_shift_m) {
  const Point2 p0 = samples[entry_index].point;
  const Point2 p3 = samples[exit_index].point;
  const Point2 incoming =
      normalized(samples[corner_index].point - samples[corner_index - 1U].point);
  const Point2 outgoing =
      normalized(samples[corner_index + 1U].point - samples[corner_index].point);
  if (!(norm(incoming) > kTinyDistanceM) || !(norm(outgoing) > kTinyDistanceM)) {
    return {};
  }

  const Point2 entry_outward = leftNormal(incoming) * -corner.turn_sign;
  const Point2 exit_outward = leftNormal(outgoing) * -corner.turn_sign;
  const double shift_scale = std::clamp(outward_shift_scale, 0.0, 1.0);
  const double entry_shift =
      outwardShiftFor(samples[entry_index], entry_outward, config) * shift_scale;
  const double exit_shift =
      outwardShiftFor(samples[exit_index], exit_outward, config) * shift_scale;
  applied_shift_m = std::max(entry_shift, exit_shift);

  const double local_length = pathLength(samples, entry_index, exit_index);
  const double control_distance =
      std::clamp(local_length * 0.35, 2.0, std::max(2.0, local_length * 0.6));
  const Point2 entry_tangent =
      tangentRelaxedOutward(incoming, entry_outward, relaxed_angle_rad);
  const Point2 exit_tangent =
      tangentRelaxedOutward(outgoing, exit_outward, relaxed_angle_rad);
  const Point2 p1 = p0 + entry_tangent * control_distance + entry_outward * entry_shift;
  const Point2 p2 = p3 - exit_tangent * control_distance + exit_outward * exit_shift;
  const double sample_step =
      sanitizedPositive(config.sample_step_m, 1.0, 0.1, std::max(0.1, local_length));
  const std::size_t steps = std::max<std::size_t>(
      3U, static_cast<std::size_t>(std::ceil(local_length / sample_step)));

  std::vector<TrajectoryPointSample> bezier_samples;
  bezier_samples.reserve(steps + 1U);
  const double s0 = samples[entry_index].s_m;
  const double s1 = samples[exit_index].s_m;
  for (std::size_t i = 0U; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const Point2 point = cubicBezier(p0, p1, p2, p3, t);
    if (!finite2D(point)) {
      return {};
    }
    bezier_samples.push_back(
        sampleForPoint(point, s0 + (s1 - s0) * t, corridor_samples));
  }
  populateSampleGeometry(bezier_samples);
  return bezier_samples;
}

void sampleRangeInto(const std::span<const TrajectoryPointSample> samples,
                     const std::size_t start_index, const std::size_t end_index,
                     std::vector<TrajectoryPointSample>& result) {
  result.clear();
  if (samples.empty() || start_index >= samples.size() || end_index >= samples.size() ||
      start_index >= end_index) {
    return;
  }
  result.reserve(end_index - start_index + 1U);
  result.insert(result.end(),
                samples.begin() + static_cast<std::ptrdiff_t>(start_index),
                samples.begin() + static_cast<std::ptrdiff_t>(end_index + 1U));
  populateSampleGeometry(result);
}

void replaceRangeInto(const std::span<const TrajectoryPointSample> samples,
                      const std::size_t entry_index, const std::size_t exit_index,
                      const std::span<const TrajectoryPointSample> replacement,
                      std::vector<TrajectoryPointSample>& result) {
  result.clear();
  result.reserve(samples.size() + replacement.size());
  result.insert(result.end(), samples.begin(),
                samples.begin() + static_cast<std::ptrdiff_t>(entry_index));
  result.insert(result.end(), replacement.begin(), replacement.end());
  result.insert(result.end(),
                samples.begin() + static_cast<std::ptrdiff_t>(exit_index + 1U),
                samples.end());
  populateSampleGeometry(result);
}

[[nodiscard]] LocalTrajectoryMetrics
localTrajectoryMetrics(const std::span<const TrajectoryPointSample> samples,
                       const VelocityFollowerConfig& speed_config,
                       TurnSmoothingStats* stats = nullptr) {
  const auto metrics_started_at = std::chrono::steady_clock::now();
  LocalTrajectoryMetrics metrics{};
  if (samples.size() < 2U) {
    if (stats != nullptr) {
      stats->metrics_duration_ms += elapsedMilliseconds(metrics_started_at);
    }
    return metrics;
  }

  metrics.valid = trajectorySamplesAreUsable(samples);
  metrics.length_m = pathLength(samples);
  const auto shape_started_at = std::chrono::steady_clock::now();
  metrics.shape = computeTrajectoryShapeDiagnostics(samples);
  if (stats != nullptr) {
    stats->shape_diagnostics_duration_ms += elapsedMilliseconds(shape_started_at);
  }
  for (const TrajectoryPointSample& sample : samples) {
    const double abs_curvature = std::abs(sample.curvature_1pm);
    metrics.max_abs_curvature_1pm =
        std::max(metrics.max_abs_curvature_1pm, abs_curvature);
    if (abs_curvature > kTinyDistanceM) {
      metrics.min_radius_m = std::min(metrics.min_radius_m, 1.0 / abs_curvature);
    }
  }
  if (!std::isfinite(metrics.min_radius_m)) {
    metrics.min_radius_m = std::numeric_limits<double>::infinity();
  }

  const auto speed_started_at = std::chrono::steady_clock::now();
  const TraversalTimeEstimate time =
      estimateTraversalTime(samples, speed_config, false);
  if (stats != nullptr) {
    stats->speed_profile_duration_ms += elapsedMilliseconds(speed_started_at);
  }
  metrics.min_speed_limit_mps = time.min_speed_limit_mps;
  metrics.estimated_time_s = time.estimated_time_s;
  if (stats != nullptr) {
    stats->metrics_duration_ms += elapsedMilliseconds(metrics_started_at);
  }
  return metrics;
}

[[nodiscard]] LocalTrajectoryMetrics
cachedBeforeMetrics(const std::span<const TrajectoryPointSample> samples,
                    const std::size_t entry_index, const std::size_t exit_index,
                    const VelocityFollowerConfig& speed_config,
                    TurnSmoothingWorkBuffer& buffer, TurnSmoothingStats& stats) {
  const IndexRangeKey key{.begin_index = entry_index, .end_index = exit_index};
  if (const auto iter = buffer.before_metrics_cache.find(key);
      iter != buffer.before_metrics_cache.end()) {
    ++stats.before_metrics_cache_hits;
    return iter->second;
  }
  ++stats.before_metrics_cache_misses;
  sampleRangeInto(samples, entry_index, exit_index, buffer.before_local);
  const LocalTrajectoryMetrics metrics =
      localTrajectoryMetrics(buffer.before_local, speed_config, &stats);
  buffer.before_metrics_cache.emplace(key, metrics);
  return metrics;
}

void cachedBezierSamples(const std::span<const TrajectoryPointSample> samples,
                         const std::span<const CorridorSample> corridor_samples,
                         const std::size_t entry_index, const std::size_t corner_index,
                         const std::size_t exit_index, const CornerCandidate& corner,
                         const TurnSmoothingConfig& config,
                         const double outward_shift_scale,
                         const double relaxed_angle_rad, double& applied_shift_m,
                         TurnSmoothingWorkBuffer& buffer, TurnSmoothingStats& stats) {
  const BezierCacheKey key{
      .corner_index = corner_index,
      .entry_index = entry_index,
      .exit_index = exit_index,
      .shift_scale = quantizedCacheValue(outward_shift_scale),
      .relaxed_angle_rad = quantizedCacheValue(relaxed_angle_rad),
      .sample_step_m = quantizedCacheValue(
          sanitizedPositive(config.sample_step_m, 1.0, 0.1, 10000.0)),
  };
  if (const auto iter = buffer.bezier_cache.find(key);
      iter != buffer.bezier_cache.end()) {
    ++stats.bezier_cache_hits;
    buffer.replacement = iter->second;
    applied_shift_m = 0.0;
    if (!buffer.replacement.empty()) {
      const Point2 entry_outward =
          leftNormal(normalized(samples[corner_index].point -
                                samples[corner_index - 1U].point)) *
          -corner.turn_sign;
      const Point2 exit_outward =
          leftNormal(normalized(samples[corner_index + 1U].point -
                                samples[corner_index].point)) *
          -corner.turn_sign;
      const double shift_scale = std::clamp(outward_shift_scale, 0.0, 1.0);
      applied_shift_m =
          std::max(outwardShiftFor(samples[entry_index], entry_outward, config),
                   outwardShiftFor(samples[exit_index], exit_outward, config)) *
          shift_scale;
    }
    return;
  }

  ++stats.bezier_cache_misses;
  const auto build_started_at = std::chrono::steady_clock::now();
  buffer.replacement = buildBezierSamples(
      samples, corridor_samples, entry_index, corner_index, exit_index, corner, config,
      outward_shift_scale, relaxed_angle_rad, applied_shift_m);
  stats.candidate_build_duration_ms += elapsedMilliseconds(build_started_at);
  buffer.bezier_cache.emplace(key, buffer.replacement);
}

[[nodiscard]] double smoothingAttemptScore(const LocalTrajectoryMetrics& metrics) {
  if (!metrics.valid || !std::isfinite(metrics.estimated_time_s)) {
    return std::numeric_limits<double>::infinity();
  }
  constexpr double kCurvatureWeight = 5.0;
  constexpr double kCurvatureJumpWeight = 2.0;
  return metrics.estimated_time_s + kCurvatureWeight * metrics.max_abs_curvature_1pm +
         kCurvatureJumpWeight * metrics.shape.max_curvature_jump_1pm;
}

[[nodiscard]] std::optional<CornerCandidate>
worstCorner(const std::span<const TrajectoryPointSample> samples,
            const TurnSmoothingConfig& config,
            const VelocityFollowerConfig& speed_config, TurnSmoothingStats& stats) {
  std::optional<CornerCandidate> best;
  for (std::size_t i = 1U; i + 1U < samples.size(); ++i) {
    const CornerCandidate candidate =
        cornerCandidateAt(samples, i, config, speed_config);
    if (!candidate.valid) {
      continue;
    }
    ++stats.detected_corners;
    updateMinInnerMargin(stats, samples[i], candidate.turn_sign);
    if (!best.has_value() ||
        candidate.abs_heading_delta_rad > best->abs_heading_delta_rad) {
      best = candidate;
    }
  }
  return best;
}

[[nodiscard]] const char*
shapeImprovementRejectDetail(const TrajectoryShapeDiagnostics& before,
                             const TrajectoryShapeDiagnostics& after,
                             const TurnSmoothingConfig& config) {
  const double min_improvement = sanitizedPositive(config.min_heading_improvement_rad,
                                                   0.05, 0.0, std::numbers::pi);
  constexpr double kMaxAcceptedHeadingDeltaRad = std::numbers::pi / 3.0;
  constexpr double kCurvatureJumpRegressionTolerance = 0.05;
  constexpr double kCurvatureJumpRegressionFactor = 1.5;
  const double max_allowed_curvature_jump =
      std::max(before.max_curvature_jump_1pm + kCurvatureJumpRegressionTolerance,
               before.max_curvature_jump_1pm * kCurvatureJumpRegressionFactor);
  if (after.max_heading_delta_rad > kMaxAcceptedHeadingDeltaRad) {
    return "heading_delta_too_high";
  }
  if (after.max_curvature_jump_1pm > max_allowed_curvature_jump) {
    return "curvature_jump_too_high";
  }
  if (after.max_heading_delta_rad + min_improvement < before.max_heading_delta_rad) {
    return "none";
  }
  if (after.max_heading_delta_rad <= before.max_heading_delta_rad + 1.0e-9 &&
      after.max_curvature_jump_1pm + 1.0e-9 < before.max_curvature_jump_1pm) {
    return "none";
  }
  return "shape_not_improved";
}

[[nodiscard]] SmoothingRejectReason
candidateRegressionReason(const LocalTrajectoryMetrics& before,
                          const LocalTrajectoryMetrics& after) noexcept {
  constexpr double kCurvatureRegressionTolerance = 0.05;
  constexpr double kCurvatureRegressionFactor = 1.5;
  constexpr double kRadiusToleranceM = 0.25;
  constexpr double kSpeedToleranceMps = 0.1;
  if (!before.valid || !after.valid) {
    return SmoothingRejectReason::kNotImproved;
  }

  const double max_allowed_curvature_jump =
      std::max(before.shape.max_curvature_jump_1pm + kCurvatureRegressionTolerance,
               before.shape.max_curvature_jump_1pm * kCurvatureRegressionFactor);
  if (after.shape.max_curvature_jump_1pm > max_allowed_curvature_jump) {
    return SmoothingRejectReason::kCurvatureRegression;
  }
  if (std::isfinite(before.min_radius_m) && std::isfinite(after.min_radius_m) &&
      after.min_radius_m + kRadiusToleranceM < before.min_radius_m) {
    return SmoothingRejectReason::kRadiusRegression;
  }
  if (std::isfinite(before.min_speed_limit_mps) &&
      std::isfinite(after.min_speed_limit_mps) &&
      after.min_speed_limit_mps + kSpeedToleranceMps < before.min_speed_limit_mps) {
    return SmoothingRejectReason::kSpeedRegression;
  }
  return SmoothingRejectReason::kNone;
}

[[nodiscard]] const char*
regressionRejectDetail(const SmoothingRejectReason reason) noexcept {
  switch (reason) {
    case SmoothingRejectReason::kCurvatureRegression:
      return "curvature_jump_regression";
    case SmoothingRejectReason::kRadiusRegression:
      return "radius_regression";
    case SmoothingRejectReason::kSpeedRegression:
      return "speed_limit_regression";
    case SmoothingRejectReason::kNotImproved:
      return "local_metrics_invalid";
    case SmoothingRejectReason::kNone:
    case SmoothingRejectReason::kProhibited:
      return "none";
  }
  return "unknown";
}

[[nodiscard]] SmoothingAttempt
trySmoothCorner(const std::span<const TrajectoryPointSample> samples,
                const std::span<const CorridorSample> corridor_samples,
                const OccupancyGrid2D& prohibited_grid, const CornerCandidate& corner,
                const TurnSmoothingConfig& config, const double entry_distance_m,
                const double exit_distance_m, const double outward_shift_scale,
                const double relaxed_angle_rad,
                const VelocityFollowerConfig& speed_config,
                TurnSmoothingWorkBuffer& buffer, TurnSmoothingStats& stats) {
  SmoothingAttempt attempt{};
  attempt.entry_distance_m = entry_distance_m;
  attempt.exit_distance_m = exit_distance_m;
  attempt.shift_scale = outward_shift_scale;
  attempt.relaxed_angle_rad = relaxed_angle_rad;
  const std::size_t entry_index =
      findEntryIndex(samples, corner.index, entry_distance_m);
  const std::size_t exit_index = findExitIndex(samples, corner.index, exit_distance_m);
  if (entry_index >= corner.index || exit_index <= corner.index) {
    attempt.reject_reason = SmoothingRejectReason::kNotImproved;
    attempt.reject_detail = "invalid_window";
    return attempt;
  }
  attempt.before_metrics = cachedBeforeMetrics(samples, entry_index, exit_index,
                                               speed_config, buffer, stats);

  cachedBezierSamples(samples, corridor_samples, entry_index, corner.index, exit_index,
                      corner, config, outward_shift_scale, relaxed_angle_rad,
                      attempt.applied_shift_m, buffer, stats);
  if (buffer.replacement.size() < 3U) {
    attempt.reject_reason = SmoothingRejectReason::kNotImproved;
    attempt.reject_detail = "replacement_too_short";
    return attempt;
  }

  const auto replace_started_at = std::chrono::steady_clock::now();
  replaceRangeInto(samples, entry_index, exit_index, buffer.replacement,
                   buffer.candidate);
  stats.candidate_replace_duration_ms += elapsedMilliseconds(replace_started_at);
  sampleRangeInto(buffer.candidate, entry_index,
                  entry_index + buffer.replacement.size() - 1U, buffer.after_local);
  attempt.after_metrics =
      localTrajectoryMetrics(buffer.after_local, speed_config, &stats);
  attempt.score = smoothingAttemptScore(attempt.after_metrics);
  const auto collision_started_at = std::chrono::steady_clock::now();
  const bool traversable = pathIsTraversableCached(prohibited_grid, buffer.candidate,
                                                   buffer.traversability_cache, stats);
  stats.collision_check_duration_ms += elapsedMilliseconds(collision_started_at);
  if (!traversable) {
    attempt.reject_reason = SmoothingRejectReason::kProhibited;
    attempt.reject_detail = "prohibited_intersection";
    return attempt;
  }

  const auto shape_started_at = std::chrono::steady_clock::now();
  const TrajectoryShapeDiagnostics before_shape =
      computeTrajectoryShapeDiagnostics(samples);
  const TrajectoryShapeDiagnostics after_shape =
      computeTrajectoryShapeDiagnostics(buffer.candidate);
  stats.shape_diagnostics_duration_ms += elapsedMilliseconds(shape_started_at);
  const char* const shape_reject_detail =
      shapeImprovementRejectDetail(before_shape, after_shape, config);
  if (std::string_view{shape_reject_detail} != "none") {
    attempt.reject_reason = SmoothingRejectReason::kNotImproved;
    attempt.reject_detail = shape_reject_detail;
    return attempt;
  }
  const SmoothingRejectReason regression_reason =
      candidateRegressionReason(attempt.before_metrics, attempt.after_metrics);
  if (regression_reason != SmoothingRejectReason::kNone) {
    attempt.reject_reason = regression_reason;
    attempt.reject_detail = regressionRejectDetail(regression_reason);
    return attempt;
  }

  attempt.samples = buffer.candidate;
  attempt.reject_reason = SmoothingRejectReason::kNone;
  attempt.reject_detail = "none";
  attempt.accepted = true;
  return attempt;
}

[[nodiscard]] bool rejectReasonDominates(const SmoothingRejectReason candidate,
                                         const SmoothingRejectReason current) noexcept {
  const auto priority = [](const SmoothingRejectReason reason) {
    switch (reason) {
      case SmoothingRejectReason::kProhibited:
        return 7;
      case SmoothingRejectReason::kCurvatureRegression:
        return 5;
      case SmoothingRejectReason::kRadiusRegression:
        return 4;
      case SmoothingRejectReason::kSpeedRegression:
        return 3;
      case SmoothingRejectReason::kNotImproved:
        return 1;
      case SmoothingRejectReason::kNone:
        return 0;
    }
    return 0;
  };
  return priority(candidate) > priority(current);
}

[[nodiscard]] const char*
rejectReasonName(const SmoothingRejectReason reason) noexcept {
  switch (reason) {
    case SmoothingRejectReason::kNone:
      return "none";
    case SmoothingRejectReason::kProhibited:
      return "prohibited";
    case SmoothingRejectReason::kNotImproved:
      return "not_improved";
    case SmoothingRejectReason::kCurvatureRegression:
      return "curvature_regression";
    case SmoothingRejectReason::kRadiusRegression:
      return "radius_regression";
    case SmoothingRejectReason::kSpeedRegression:
      return "speed_regression";
  }
  return "unknown";
}

[[nodiscard]] TurnSmoothingCornerDiagnostic
cornerDiagnosticFromAttempt(const SmoothingAttempt& attempt,
                            const double corner_s_m) noexcept {
  TurnSmoothingCornerDiagnostic diagnostic{};
  diagnostic.accepted = attempt.accepted;
  diagnostic.reject_reason = rejectReasonName(attempt.reject_reason);
  diagnostic.reject_detail = attempt.reject_detail;
  diagnostic.corner_s_m = corner_s_m;
  diagnostic.entry_distance_m = attempt.entry_distance_m;
  diagnostic.exit_distance_m = attempt.exit_distance_m;
  diagnostic.shift_scale = attempt.shift_scale;
  diagnostic.relaxed_angle_deg = radiansToDegrees(attempt.relaxed_angle_rad);
  diagnostic.score = attempt.score;
  diagnostic.min_radius_before_m = attempt.before_metrics.min_radius_m;
  diagnostic.min_radius_after_m = attempt.after_metrics.min_radius_m;
  diagnostic.min_speed_before_mps = attempt.before_metrics.min_speed_limit_mps;
  diagnostic.min_speed_after_mps = attempt.after_metrics.min_speed_limit_mps;
  diagnostic.local_time_before_s = attempt.before_metrics.estimated_time_s;
  diagnostic.local_time_after_s = attempt.after_metrics.estimated_time_s;
  diagnostic.curvature_jump_before_1pm =
      attempt.before_metrics.shape.max_curvature_jump_1pm;
  diagnostic.curvature_jump_after_1pm =
      attempt.after_metrics.shape.max_curvature_jump_1pm;
  diagnostic.heading_delta_before_rad =
      attempt.before_metrics.shape.max_heading_delta_rad;
  diagnostic.heading_delta_after_rad =
      attempt.after_metrics.shape.max_heading_delta_rad;
  return diagnostic;
}

[[nodiscard]] TurnSmoothingCandidateDiagnostic candidateDiagnosticFromAttempt(
    const SmoothingAttempt& attempt, const CornerCandidate& corner,
    const std::size_t pass, const std::size_t attempt_index, const double corner_s_m) {
  TurnSmoothingCandidateDiagnostic diagnostic{};
  diagnostic.decision = attempt.accepted ? "valid_not_best" : "rejected";
  diagnostic.reject_reason = rejectReasonName(attempt.reject_reason);
  diagnostic.reject_detail = attempt.reject_detail;
  diagnostic.pass = pass;
  diagnostic.attempt_index = attempt_index;
  diagnostic.corner_index = corner.index;
  diagnostic.corner_s_m = corner_s_m;
  diagnostic.entry_distance_m = attempt.entry_distance_m;
  diagnostic.exit_distance_m = attempt.exit_distance_m;
  diagnostic.shift_scale = attempt.shift_scale;
  diagnostic.applied_shift_m = attempt.applied_shift_m;
  diagnostic.relaxed_angle_deg = radiansToDegrees(attempt.relaxed_angle_rad);
  diagnostic.score = attempt.score;
  diagnostic.min_radius_before_m = attempt.before_metrics.min_radius_m;
  diagnostic.min_radius_after_m = attempt.after_metrics.min_radius_m;
  diagnostic.min_speed_before_mps = attempt.before_metrics.min_speed_limit_mps;
  diagnostic.min_speed_after_mps = attempt.after_metrics.min_speed_limit_mps;
  diagnostic.local_time_before_s = attempt.before_metrics.estimated_time_s;
  diagnostic.local_time_after_s = attempt.after_metrics.estimated_time_s;
  diagnostic.curvature_jump_before_1pm =
      attempt.before_metrics.shape.max_curvature_jump_1pm;
  diagnostic.curvature_jump_after_1pm =
      attempt.after_metrics.shape.max_curvature_jump_1pm;
  diagnostic.heading_delta_before_rad =
      attempt.before_metrics.shape.max_heading_delta_rad;
  diagnostic.heading_delta_after_rad =
      attempt.after_metrics.shape.max_heading_delta_rad;
  return diagnostic;
}

void incrementRejectStat(TurnSmoothingStats& stats,
                         const SmoothingRejectReason reason) noexcept {
  switch (reason) {
    case SmoothingRejectReason::kProhibited:
      ++stats.rejected_prohibited;
      break;
    case SmoothingRejectReason::kNotImproved:
      ++stats.rejected_not_improved;
      break;
    case SmoothingRejectReason::kCurvatureRegression:
      ++stats.rejected_curvature_regression;
      break;
    case SmoothingRejectReason::kRadiusRegression:
      ++stats.rejected_radius_regression;
      break;
    case SmoothingRejectReason::kSpeedRegression:
      ++stats.rejected_speed_regression;
      break;
    case SmoothingRejectReason::kNone:
      break;
  }
}

} // namespace

TurnSmoothingResult
smoothTrajectoryTurns(const std::span<const TrajectoryPointSample> samples,
                      const std::span<const CorridorSample> corridor_samples,
                      const OccupancyGrid2D& prohibited_grid,
                      const TurnSmoothingConfig& config,
                      const VelocityFollowerConfig& speed_config) {
  TurnSmoothingResult result{};
  result.stats.input_samples = samples.size();
  result.samples.assign(samples.begin(), samples.end());
  if (samples.size() < 3U || corridor_samples.empty()) {
    result.stats.output_samples = result.samples.size();
    result.valid = trajectorySamplesAreUsable(result.samples);
    return result;
  }
  populateSampleGeometry(result.samples);

  const TrajectoryShapeDiagnostics initial_shape =
      computeTrajectoryShapeDiagnostics(result.samples);
  result.stats.max_heading_delta_before_rad = initial_shape.max_heading_delta_rad;
  result.stats.max_curvature_jump_before_1pm = initial_shape.max_curvature_jump_1pm;

  const std::size_t max_passes =
      std::clamp<std::size_t>(config.max_passes, 0U, static_cast<std::size_t>(100U));
  for (std::size_t pass = 0U; pass < max_passes; ++pass) {
    TurnSmoothingStats detection_stats{};
    const std::optional<CornerCandidate> corner =
        worstCorner(result.samples, config, speed_config, detection_stats);
    result.stats.detected_corners += detection_stats.detected_corners;
    if (std::isfinite(detection_stats.min_inner_margin_m)) {
      if (!std::isfinite(result.stats.min_inner_margin_m)) {
        result.stats.min_inner_margin_m = detection_stats.min_inner_margin_m;
      } else {
        result.stats.min_inner_margin_m = std::min(result.stats.min_inner_margin_m,
                                                   detection_stats.min_inner_margin_m);
      }
    }
    if (!corner.has_value()) {
      break;
    }

    TurnSmoothingWorkBuffer work_buffer{};
    work_buffer.before_local.reserve(result.samples.size());
    work_buffer.replacement.reserve(result.samples.size());
    work_buffer.candidate.reserve(result.samples.size() * 2U);
    work_buffer.after_local.reserve(result.samples.size());
    ++result.stats.attempted_corners;
    const double entry_distance =
        sanitizedPositive(config.entry_distance_m, 45.0, 0.1, 5000.0);
    const double exit_distance =
        sanitizedPositive(config.exit_distance_m, 45.0, 0.1, 5000.0);
    const std::vector<double> entry_candidates =
        distanceFallbackCandidates(entry_distance);
    constexpr std::array<double, 4U> kShiftScales = {1.0, 0.5, 0.25, 0.0};
    SmoothingRejectReason dominant_reject_reason = SmoothingRejectReason::kNone;
    std::optional<SmoothingAttempt> accepted_attempt;
    std::optional<SmoothingAttempt> rejected_attempt;
    std::optional<std::size_t> selected_candidate_diagnostic_index;
    const double corner_s_m = corner->index < result.samples.size()
                                  ? result.samples[corner->index].s_m
                                  : std::numeric_limits<double>::quiet_NaN();
    const auto record_attempt = [&](const SmoothingAttempt& attempt,
                                    const std::size_t attempt_index) {
      result.stats.candidate_diagnostics.push_back(candidateDiagnosticFromAttempt(
          attempt, *corner, pass, attempt_index, corner_s_m));
      return result.stats.candidate_diagnostics.size() - 1U;
    };
    const auto consider_attempt = [&](SmoothingAttempt&& attempt,
                                      const std::size_t diagnostic_index) {
      if (attempt.accepted) {
        if (!accepted_attempt.has_value() || attempt.score < accepted_attempt->score) {
          accepted_attempt = std::move(attempt);
          selected_candidate_diagnostic_index = diagnostic_index;
        }
        return;
      }
      if (rejectReasonDominates(attempt.reject_reason, dominant_reject_reason)) {
        dominant_reject_reason = attempt.reject_reason;
      }
      if (!rejected_attempt.has_value() ||
          (std::isfinite(attempt.score) && attempt.score < rejected_attempt->score)) {
        rejected_attempt = std::move(attempt);
      }
    };
    for (const double entry_candidate : entry_candidates) {
      const double distance_scale =
          entry_distance > kTinyDistanceM ? entry_candidate / entry_distance : 1.0;
      for (const double shift_scale : kShiftScales) {
        const std::size_t attempt_index = result.stats.candidate_attempts++;
        SmoothingAttempt attempt = trySmoothCorner(
            result.samples, corridor_samples, prohibited_grid, *corner, config,
            entry_distance * distance_scale, exit_distance * distance_scale,
            shift_scale, 0.0, speed_config, work_buffer, result.stats);
        const std::size_t diagnostic_index = record_attempt(attempt, attempt_index);
        consider_attempt(std::move(attempt), diagnostic_index);
      }
    }
    for (const double entry_candidate : entry_candidates) {
      const double distance_scale =
          entry_distance > kTinyDistanceM ? entry_candidate / entry_distance : 1.0;
      for (const double relaxed_angle : relaxedTangentAngleCandidatesRad()) {
        for (const double shift_scale : kShiftScales) {
          const std::size_t attempt_index = result.stats.candidate_attempts++;
          ++result.stats.relaxed_candidate_attempts;
          SmoothingAttempt attempt = trySmoothCorner(
              result.samples, corridor_samples, prohibited_grid, *corner, config,
              entry_distance * distance_scale, exit_distance * distance_scale,
              shift_scale, relaxed_angle, speed_config, work_buffer, result.stats);
          const std::size_t diagnostic_index = record_attempt(attempt, attempt_index);
          consider_attempt(std::move(attempt), diagnostic_index);
        }
      }
    }
    if (!accepted_attempt.has_value()) {
      incrementRejectStat(result.stats,
                          dominant_reject_reason == SmoothingRejectReason::kNone
                              ? SmoothingRejectReason::kNotImproved
                              : dominant_reject_reason);
      if (rejected_attempt.has_value()) {
        result.stats.corner_diagnostics.push_back(
            cornerDiagnosticFromAttempt(*rejected_attempt, corner_s_m));
      }
      break;
    }

    if (selected_candidate_diagnostic_index.has_value() &&
        *selected_candidate_diagnostic_index <
            result.stats.candidate_diagnostics.size()) {
      result.stats.candidate_diagnostics[*selected_candidate_diagnostic_index]
          .decision = "selected";
    }
    result.stats.corner_diagnostics.push_back(
        cornerDiagnosticFromAttempt(*accepted_attempt, corner_s_m));
    result.samples = std::move(accepted_attempt->samples);
    result.changed = true;
    ++result.stats.smoothed_corners;
    result.stats.max_applied_outer_shift_m = std::max(
        result.stats.max_applied_outer_shift_m, accepted_attempt->applied_shift_m);
    result.stats.accepted_entry_distance_m = accepted_attempt->entry_distance_m;
    result.stats.accepted_exit_distance_m = accepted_attempt->exit_distance_m;
    result.stats.accepted_shift_scale = accepted_attempt->shift_scale;
    result.stats.accepted_relaxed_angle_deg =
        radiansToDegrees(accepted_attempt->relaxed_angle_rad);
    result.stats.accepted_score = accepted_attempt->score;
    result.stats.accepted_min_radius_before_m =
        accepted_attempt->before_metrics.min_radius_m;
    result.stats.accepted_min_radius_after_m =
        accepted_attempt->after_metrics.min_radius_m;
    result.stats.accepted_min_speed_before_mps =
        accepted_attempt->before_metrics.min_speed_limit_mps;
    result.stats.accepted_min_speed_after_mps =
        accepted_attempt->after_metrics.min_speed_limit_mps;
    result.stats.accepted_local_time_before_s =
        accepted_attempt->before_metrics.estimated_time_s;
    result.stats.accepted_local_time_after_s =
        accepted_attempt->after_metrics.estimated_time_s;
  }

  populateSampleGeometry(result.samples);
  const TrajectoryShapeDiagnostics final_shape =
      computeTrajectoryShapeDiagnostics(result.samples);
  result.stats.output_samples = result.samples.size();
  result.stats.max_heading_delta_after_rad = final_shape.max_heading_delta_rad;
  result.stats.max_curvature_jump_after_1pm = final_shape.max_curvature_jump_1pm;
  result.valid = trajectorySamplesAreUsable(result.samples) &&
                 pathIsTraversable(prohibited_grid, result.samples);
  return result;
}

} // namespace drone_city_nav
