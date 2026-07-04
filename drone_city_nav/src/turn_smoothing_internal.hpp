#pragma once

#include "drone_city_nav/trajectory_speed_planner.hpp"
#include "drone_city_nav/turn_smoothing.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace drone_city_nav::turn_smoothing_detail {

inline constexpr double kTinyDistanceM = 1.0e-6;

[[nodiscard]] bool finite2D(Point2 point) noexcept;
[[nodiscard]] Point2 operator+(Point2 lhs, Point2 rhs) noexcept;
[[nodiscard]] Point2 operator-(Point2 lhs, Point2 rhs) noexcept;
[[nodiscard]] Point2 operator*(Point2 point, double scale) noexcept;
[[nodiscard]] double dot(Point2 lhs, Point2 rhs) noexcept;
[[nodiscard]] double cross(Point2 lhs, Point2 rhs) noexcept;
[[nodiscard]] double norm(Point2 point) noexcept;
[[nodiscard]] Point2 normalized(Point2 point) noexcept;
[[nodiscard]] Point2 leftNormal(Point2 tangent) noexcept;
[[nodiscard]] double normalizeAngle(double angle_rad) noexcept;
[[nodiscard]] double headingOf(Point2 vector) noexcept;
[[nodiscard]] double signedHeadingDelta(Point2 previous, Point2 next) noexcept;
[[nodiscard]] double radiansToDegrees(double radians) noexcept;
[[nodiscard]] double sanitizedPositive(double value, double fallback, double min_value,
                                       double max_value) noexcept;
[[nodiscard]] double
curvatureSpeedLimitMps(double radius_m, const VelocityFollowerConfig& config) noexcept;
[[nodiscard]] double elapsedMilliseconds(std::chrono::steady_clock::time_point start);
[[nodiscard]] std::int64_t quantizedCacheValue(double value) noexcept;
[[nodiscard]] std::vector<double> distanceFallbackCandidates(double max_distance_m);
[[nodiscard]] std::array<double, 6U> relaxedTangentAngleCandidatesRad() noexcept;
[[nodiscard]] double discreteCurvature(Point2 previous, Point2 current, Point2 next);
void populateSampleGeometry(std::vector<TrajectoryPointSample>& samples);
[[nodiscard]] double pathLength(std::span<const TrajectoryPointSample> samples,
                                std::size_t start_index, std::size_t end_index);
[[nodiscard]] double pathLength(std::span<const TrajectoryPointSample> samples);
[[nodiscard]] std::optional<CorridorSample>
corridorSampleAtS(std::span<const CorridorSample> corridor_samples, double s_m);

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
  std::size_t entry_index{0U};
  std::size_t exit_index{0U};
  std::size_t replacement_sample_count{0U};
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

[[nodiscard]] bool segmentIsTraversable(const OccupancyGrid2D& grid, Point2 start,
                                        Point2 end);
[[nodiscard]] SegmentCellKey orderedSegmentKey(GridIndex start, GridIndex end) noexcept;
[[nodiscard]] bool cachedSegmentIsTraversable(const OccupancyGrid2D& grid, Point2 start,
                                              Point2 end,
                                              SegmentTraversabilityCache& cache,
                                              TurnSmoothingStats& stats);
[[nodiscard]] bool pathIsTraversable(const OccupancyGrid2D& grid,
                                     std::span<const TrajectoryPointSample> samples);
[[nodiscard]] bool
pathIsTraversableCached(const OccupancyGrid2D& grid,
                        std::span<const TrajectoryPointSample> samples,
                        SegmentTraversabilityCache& cache, TurnSmoothingStats& stats);
[[nodiscard]] double innerMarginM(const TrajectoryPointSample& sample,
                                  double turn_sign) noexcept;
void updateMinInnerMargin(TurnSmoothingStats& stats,
                          const TrajectoryPointSample& sample,
                          double turn_sign) noexcept;

[[nodiscard]] CornerCandidate
cornerCandidateAt(std::span<const TrajectoryPointSample> samples, std::size_t index,
                  const TurnSmoothingConfig& config,
                  const VelocityFollowerConfig& speed_config);
[[nodiscard]] std::size_t findEntryIndex(std::span<const TrajectoryPointSample> samples,
                                         std::size_t corner, double entry_distance_m);
[[nodiscard]] std::size_t findExitIndex(std::span<const TrajectoryPointSample> samples,
                                        std::size_t corner, double exit_distance_m);
[[nodiscard]] double outwardShiftFor(const TrajectoryPointSample& sample,
                                     Point2 outward, const TurnSmoothingConfig& config);
[[nodiscard]] Point2 cubicBezier(Point2 p0, Point2 p1, Point2 p2, Point2 p3,
                                 double t) noexcept;
[[nodiscard]] Point2 tangentRelaxedOutward(Point2 tangent, Point2 outward,
                                           double angle_rad) noexcept;
[[nodiscard]] TrajectoryPointSample
sampleForPoint(Point2 point, double station_hint_m,
               std::span<const CorridorSample> corridor_samples);
[[nodiscard]] std::vector<TrajectoryPointSample>
buildBezierSamples(std::span<const TrajectoryPointSample> samples,
                   std::span<const CorridorSample> corridor_samples,
                   std::size_t entry_index, std::size_t corner_index,
                   std::size_t exit_index, const CornerCandidate& corner,
                   const TurnSmoothingConfig& config, double outward_shift_scale,
                   double relaxed_angle_rad, double& applied_shift_m);
void cachedBezierSamples(std::span<const TrajectoryPointSample> samples,
                         std::span<const CorridorSample> corridor_samples,
                         std::size_t entry_index, std::size_t corner_index,
                         std::size_t exit_index, const CornerCandidate& corner,
                         const TurnSmoothingConfig& config, double outward_shift_scale,
                         double relaxed_angle_rad, double& applied_shift_m,
                         TurnSmoothingWorkBuffer& buffer, TurnSmoothingStats& stats);
[[nodiscard]] SmoothingAttempt
trySmoothCorner(std::span<const TrajectoryPointSample> samples,
                std::span<const CorridorSample> corridor_samples,
                const OccupancyGrid2D& prohibited_grid, const CornerCandidate& corner,
                const TurnSmoothingConfig& config, double entry_distance_m,
                double exit_distance_m, double outward_shift_scale,
                double relaxed_angle_rad, const VelocityFollowerConfig& speed_config,
                TurnSmoothingWorkBuffer& buffer, TurnSmoothingStats& stats);

void sampleRangeInto(std::span<const TrajectoryPointSample> samples,
                     std::size_t start_index, std::size_t end_index,
                     std::vector<TrajectoryPointSample>& result);
void replaceRangeInto(std::span<const TrajectoryPointSample> samples,
                      std::size_t entry_index, std::size_t exit_index,
                      std::span<const TrajectoryPointSample> replacement,
                      std::vector<TrajectoryPointSample>& result);
[[nodiscard]] LocalTrajectoryMetrics
localTrajectoryMetrics(std::span<const TrajectoryPointSample> samples,
                       const VelocityFollowerConfig& speed_config,
                       TurnSmoothingStats* stats = nullptr,
                       bool include_speed_profile = false);
[[nodiscard]] LocalTrajectoryMetrics
cachedBeforeMetrics(std::span<const TrajectoryPointSample> samples,
                    std::size_t entry_index, std::size_t exit_index,
                    const VelocityFollowerConfig& speed_config,
                    TurnSmoothingWorkBuffer& buffer, TurnSmoothingStats& stats);
[[nodiscard]] double smoothingAttemptScore(const LocalTrajectoryMetrics& metrics);
[[nodiscard]] std::optional<CornerCandidate>
worstCorner(std::span<const TrajectoryPointSample> samples,
            const TurnSmoothingConfig& config,
            const VelocityFollowerConfig& speed_config, TurnSmoothingStats& stats);
[[nodiscard]] const char*
shapeImprovementRejectDetail(const TrajectoryShapeDiagnostics& before,
                             const TrajectoryShapeDiagnostics& after,
                             const TurnSmoothingConfig& config);
[[nodiscard]] SmoothingRejectReason
candidateRegressionReason(const LocalTrajectoryMetrics& before,
                          const LocalTrajectoryMetrics& after) noexcept;
[[nodiscard]] const char* regressionRejectDetail(SmoothingRejectReason reason) noexcept;

void populateAttemptSpeedDiagnostics(
    std::span<const TrajectoryPointSample> before_samples, SmoothingAttempt& attempt,
    const VelocityFollowerConfig& speed_config, TurnSmoothingWorkBuffer& buffer,
    TurnSmoothingStats& stats);
void updateCandidateSpeedDiagnostics(TurnSmoothingCandidateDiagnostic& diagnostic,
                                     const SmoothingAttempt& attempt) noexcept;
[[nodiscard]] bool rejectReasonDominates(SmoothingRejectReason candidate,
                                         SmoothingRejectReason current) noexcept;
[[nodiscard]] const char* rejectReasonName(SmoothingRejectReason reason) noexcept;
[[nodiscard]] TurnSmoothingCornerDiagnostic
cornerDiagnosticFromAttempt(const SmoothingAttempt& attempt,
                            double corner_s_m) noexcept;
[[nodiscard]] TurnSmoothingCandidateDiagnostic
candidateDiagnosticFromAttempt(const SmoothingAttempt& attempt,
                               const CornerCandidate& corner, std::size_t pass,
                               std::size_t attempt_index, double corner_s_m);
void incrementRejectStat(TurnSmoothingStats& stats,
                         SmoothingRejectReason reason) noexcept;

} // namespace drone_city_nav::turn_smoothing_detail
