#pragma once

#include "drone_city_nav/trajectory_diagnostics.hpp"
#include "drone_city_nav/trajectory_optimizer.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <mutex>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace drone_city_nav::trajectory_optimizer_detail {

inline constexpr double kTinyDistanceM = 1.0e-6;
inline constexpr double kCollisionPenalty = 1.0e9;
inline constexpr double kOutsideGridPenalty = 1.0e9;
inline constexpr double kHeadingJumpPenalty = 1.0e6;
inline constexpr double kHeadingJumpHardPenalty = 1.0e9;
inline constexpr double kHeadingJumpSoftLimitRad = std::numbers::pi / 2.0;
inline constexpr double kHeadingJumpHardLimitRad = 2.0 * std::numbers::pi / 3.0;

struct PathEvaluation {
  double length_m{0.0};
  std::size_t prohibited_cells{0U};
  std::size_t outside_grid_segments{0U};
  std::size_t blocked_segment_count{0U};
  std::size_t blocked_span_count{0U};
  std::size_t first_blocked_segment_index{0U};
  std::size_t last_blocked_segment_index{0U};
  double first_blocked_s_m{std::numeric_limits<double>::quiet_NaN()};
  double last_blocked_s_m{std::numeric_limits<double>::quiet_NaN()};
  Point2 first_blocked_point{};
  Point2 last_blocked_point{};
  bool has_first_blocked_point{false};
  bool has_last_blocked_point{false};
  bool first_blocked_outside_grid{false};
  bool last_blocked_outside_grid{false};
  std::size_t blocked_span_diagnostic_count{0U};
  std::array<TrajectoryOptimizerBlockedSpanDiagnostic,
             kMaxCenterlineBlockedSpanDiagnostics>
      blocked_span_diagnostics{};

  [[nodiscard]] bool traversable() const noexcept {
    return prohibited_cells == 0U && outside_grid_segments == 0U;
  }
};

struct CostBreakdown {
  double curvature_cost{0.0};
  double curvature_change_cost{0.0};
  double radius_shortfall_cost{0.0};
  double heading_jump_cost{0.0};
  double offset_change_cost{0.0};
  double offset_second_change_cost{0.0};
  double offset_slope_cost{0.0};
  double collision_cost{0.0};
  double outside_grid_cost{0.0};

  [[nodiscard]] double total() const noexcept {
    return curvature_cost + curvature_change_cost + radius_shortfall_cost +
           heading_jump_cost + offset_change_cost + offset_second_change_cost +
           offset_slope_cost + collision_cost + outside_grid_cost;
  }
};

struct CandidateScore {
  double score{std::numeric_limits<double>::infinity()};
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
  double local_point_build_duration_ms{0.0};
  double local_path_evaluation_duration_ms{0.0};
  double local_score_duration_ms{0.0};
  double full_score_duration_ms{0.0};
  std::size_t local_segment_cache_hits{0U};
  std::size_t local_segment_cache_misses{0U};
  std::size_t full_path_segment_cache_hits{0U};
  std::size_t full_path_segment_cache_misses{0U};
  std::size_t offset_changed_samples{0U};
  std::size_t offset_changed_span_samples{0U};
  std::size_t local_speed_window_samples{0U};
  std::size_t shadow_boundary_clamped_window_samples{0U};
  bool local_evaluated{false};
  bool requires_full_score{false};
  bool full_score_used{false};
  LocalFullScoreReason local_full_score_reason{LocalFullScoreReason::kNone};
  bool scratch_reused{false};
  bool snapshot_allocation_avoided{false};
  bool shadow_segment_score_valid{false};
  bool shadow_segment_score_would_prune{false};
  double shadow_segment_score_estimated_score{std::numeric_limits<double>::quiet_NaN()};
  double shadow_segment_score_incumbent_score{std::numeric_limits<double>::quiet_NaN()};
  std::size_t shadow_segment_score_window_samples{0U};
};

struct CandidateTask {
  std::size_t order{0U};
  std::size_t center_index{0U};
  double delta_m{0.0};
};

struct CandidateBatchResult {
  std::size_t order{0U};
  std::size_t center_index{0U};
  double delta_m{0.0};
  EvaluatedCandidate candidate{};
};

struct OffsetChangeDiagnostics {
  std::size_t changed_samples{0U};
  std::size_t changed_span_samples{0U};
  std::size_t first_changed_index{0U};
  std::size_t last_changed_index{0U};
};

struct IndexRange {
  std::size_t begin{0U};
  std::size_t end{0U};
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
  std::vector<TrajectoryPointSample> samples;
  SegmentTraversabilityCache candidate_segment_cache;
  SegmentProhibitedCountCache full_path_segment_cache;
};

struct TrajectoryOptimizerScratch {
  std::vector<double> candidate_offsets;
  std::vector<double> accepted_offsets;
  std::vector<double> iteration_best_offsets;
  std::vector<double> smoothed_offsets;
  std::vector<Point2> candidate_points;
  std::vector<Point2> accepted_points;
  std::vector<TrajectoryPointSample> candidate_samples;
  SegmentProhibitedCountCache full_path_segment_cache;
};

struct CandidateBatchWorkspace {
  std::vector<CandidateBatchResult> results;
  std::vector<CandidateWorkBuffer> worker_buffers;
};

[[nodiscard]] double elapsedMilliseconds(std::chrono::steady_clock::time_point start);

class TrajectoryOptimizerCandidateWorkerPool {
public:
  TrajectoryOptimizerCandidateWorkerPool(const std::size_t worker_count,
                                         TrajectoryOptimizerStats& stats)
      : stats_(stats) {
    const auto started_at = std::chrono::steady_clock::now();
    workers_.reserve(worker_count);
    for (std::size_t worker_index = 0U; worker_index < worker_count; ++worker_index) {
      workers_.emplace_back([this, worker_index] { workerLoop(worker_index); });
    }
    stats_.candidate_threads_launched += worker_count;
    stats_.candidate_thread_launch_duration_ms += elapsedMilliseconds(started_at);
  }

  TrajectoryOptimizerCandidateWorkerPool(
      const TrajectoryOptimizerCandidateWorkerPool&) = delete;
  TrajectoryOptimizerCandidateWorkerPool&
  operator=(const TrajectoryOptimizerCandidateWorkerPool&) = delete;
  TrajectoryOptimizerCandidateWorkerPool(TrajectoryOptimizerCandidateWorkerPool&&) =
      delete;
  TrajectoryOptimizerCandidateWorkerPool&
  operator=(TrajectoryOptimizerCandidateWorkerPool&&) = delete;

  ~TrajectoryOptimizerCandidateWorkerPool() {
    const auto started_at = std::chrono::steady_clock::now();
    {
      std::lock_guard lock(mutex_);
      stop_requested_ = true;
      ++generation_;
    }
    job_available_.notify_all();
    for (std::thread& worker : workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
    stats_.candidate_thread_join_wait_duration_ms += elapsedMilliseconds(started_at);
  }

  template<typename EvaluateFn>
  void run(const std::size_t active_workers, const std::size_t task_count,
           const EvaluateFn& evaluate) {
    if (active_workers == 0U || task_count == 0U) {
      return;
    }

    const std::size_t bounded_workers =
        std::min<std::size_t>({active_workers, workers_.size(), task_count});
    const auto started_at = std::chrono::steady_clock::now();
    {
      std::lock_guard lock(mutex_);
      active_workers_ = bounded_workers;
      remaining_workers_ = bounded_workers;
      worker_exception_ = nullptr;
      job_ = [bounded_workers, task_count, &evaluate](const std::size_t worker_index) {
        const std::size_t begin = task_count * worker_index / bounded_workers;
        const std::size_t end = task_count * (worker_index + 1U) / bounded_workers;
        for (std::size_t task_index = begin; task_index < end; ++task_index) {
          evaluate(task_index, worker_index);
        }
      };
      ++generation_;
    }

    job_available_.notify_all();
    std::unique_lock lock(mutex_);
    job_finished_.wait(lock, [this] { return remaining_workers_ == 0U; });
    const std::exception_ptr exception = worker_exception_;
    lock.unlock();
    stats_.candidate_batch_wait_duration_ms += elapsedMilliseconds(started_at);
    if (exception != nullptr) {
      std::rethrow_exception(exception);
    }
  }

private:
  void workerLoop(const std::size_t worker_index) {
    std::size_t observed_generation = 0U;
    while (true) {
      std::function<void(std::size_t)> job;
      {
        std::unique_lock lock(mutex_);
        job_available_.wait(lock, [this, observed_generation] {
          return stop_requested_ || generation_ != observed_generation;
        });
        if (stop_requested_) {
          return;
        }
        observed_generation = generation_;
        if (worker_index >= active_workers_) {
          continue;
        }
        job = job_;
      }

      try {
        job(worker_index);
      } catch (...) {
        std::lock_guard lock(mutex_);
        if (worker_exception_ == nullptr) {
          worker_exception_ = std::current_exception();
        }
      }

      {
        std::lock_guard lock(mutex_);
        if (remaining_workers_ > 0U) {
          --remaining_workers_;
        }
        if (remaining_workers_ == 0U) {
          job_finished_.notify_one();
        }
      }
    }
  }

  TrajectoryOptimizerStats& stats_;
  std::vector<std::thread> workers_;
  std::mutex mutex_;
  std::condition_variable job_available_;
  std::condition_variable job_finished_;
  std::function<void(std::size_t)> job_;
  std::exception_ptr worker_exception_{nullptr};
  std::size_t active_workers_{0U};
  std::size_t remaining_workers_{0U};
  std::size_t generation_{0U};
  bool stop_requested_{false};
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
  double point_build_duration_ms{0.0};
  double path_evaluation_duration_ms{0.0};
  double score_duration_ms{0.0};
  std::size_t segment_cache_hits{0U};
  std::size_t segment_cache_misses{0U};
  std::size_t shadow_boundary_clamped_window_samples{0U};
};

[[nodiscard]] const char*
localFullScoreReasonName(const LocalFullScoreReason reason) noexcept;
void reserveCandidateWorkBuffer(CandidateWorkBuffer& buffer,
                                const std::size_t offset_count,
                                const std::size_t sample_count);
void prepareCandidateWorkspace(CandidateBatchWorkspace& workspace,
                               const std::size_t min_worker_count,
                               const std::size_t offset_count,
                               const std::size_t sample_count,
                               TrajectoryOptimizerStats& stats);
[[nodiscard]] Point2 operator+(const Point2 lhs, const Point2 rhs) noexcept;
[[nodiscard]] Point2 operator-(const Point2 lhs, const Point2 rhs) noexcept;
[[nodiscard]] Point2 operator*(const Point2 point, const double scale) noexcept;
[[nodiscard]] double norm(const Point2 point) noexcept;
[[nodiscard]] Point2 normalized(const Point2 point) noexcept;
[[nodiscard]] double cross(const Point2 lhs, const Point2 rhs) noexcept;
[[nodiscard]] double dot(const Point2 lhs, const Point2 rhs) noexcept;
void startBlockedSpanDiagnostic(PathEvaluation& evaluation,
                                const std::size_t segment_index,
                                const double segment_start_s_m,
                                const Point2 segment_start);
void updateBlockedSpanDiagnostic(PathEvaluation& evaluation,
                                 const std::size_t diagnostic_index,
                                 const std::size_t segment_index,
                                 const double segment_end_s_m, const Point2 segment_end,
                                 const std::size_t prohibited_cells,
                                 const std::size_t outside_grid_segments);
void recordBlockedPoint(PathEvaluation& evaluation, const std::size_t segment_index,
                        const double segment_start_s_m, const Point2 segment_start,
                        const Point2 segment_end, const Point2 blocked_point,
                        const bool outside_grid);
[[nodiscard]] double sanitizedPositive(const double value, const double fallback,
                                       const double min_value,
                                       const double max_value) noexcept;
[[nodiscard]] std::size_t desiredWorkerCount(const std::size_t requested_workers,
                                             const std::size_t work_items) noexcept;
[[nodiscard]] double
elapsedMilliseconds(const std::chrono::steady_clock::time_point start);
[[nodiscard]] double percentileValue(std::vector<double>& values,
                                     const double percentile);
void pointsFromOffsets(const std::span<const CorridorSample> corridor_samples,
                       const std::span<const double> offsets,
                       std::vector<Point2>& points);
[[nodiscard]] std::vector<Point2>
pointsFromOffsets(const std::span<const CorridorSample> corridor_samples,
                  const std::span<const double> offsets);
[[nodiscard]] bool offsetsNearlyEqual(const std::span<const double> lhs,
                                      const std::span<const double> rhs) noexcept;
[[nodiscard]] OffsetChangeDiagnostics
offsetChangeDiagnostics(const std::span<const double> base_offsets,
                        const std::span<const double> candidate_offsets) noexcept;
[[nodiscard]] std::size_t
estimatedLocalSpeedWindowSamples(const OffsetChangeDiagnostics& diagnostics,
                                 const std::size_t sample_count) noexcept;
void samplesFromPointsAndOffsets(const std::span<const CorridorSample> corridor_samples,
                                 const std::span<const Point2> points,
                                 const std::span<const double> offsets,
                                 std::vector<TrajectoryPointSample>& samples);
[[nodiscard]] std::vector<TrajectoryPointSample>
samplesFromPointsAndOffsets(const std::span<const CorridorSample> corridor_samples,
                            const std::span<const Point2> points,
                            const std::span<const double> offsets);
void applyOffsetDelta(std::vector<double>& offsets,
                      const std::span<const CorridorSample> corridor_samples,
                      const std::size_t center_index, const double delta_m,
                      const std::span<const std::uint8_t> mutable_indices = {});
enum class InitialOffsetSeed : std::uint8_t {
  kCenterline,
  kCorridorMidline,
  kLeftBiased,
  kRightBiased,
};
[[nodiscard]] const char* initialOffsetSeedName(const InitialOffsetSeed seed) noexcept;
[[nodiscard]] double offsetForSeed(const CorridorSample& sample,
                                   const InitialOffsetSeed seed) noexcept;
void offsetsFromSeed(const std::span<const CorridorSample> corridor_samples,
                     const InitialOffsetSeed seed, std::vector<double>& offsets);
[[nodiscard]] std::vector<CorridorSample>
optimizerCorridorSamples(const std::span<const CorridorSample> corridor_samples,
                         const TrajectoryOptimizerConfig& config);
[[nodiscard]] double pathLength(const std::span<const Point2> points);
[[nodiscard]] double headingDeltaRad(const Point2 lhs, const Point2 rhs) noexcept;
[[nodiscard]] double discreteCurvature(const Point2 previous, const Point2 current,
                                       const Point2 next);
[[nodiscard]] double edgeMarginM(const CorridorSample& sample,
                                 const double offset_m) noexcept;
[[nodiscard]] CostBreakdown
costBreakdownForPoints(std::span<const Point2> points, std::span<const double> offsets,
                       const TrajectoryOptimizerConfig& config);

[[nodiscard]] PathEvaluation evaluatePath(const OccupancyGrid2D& grid,
                                          const std::span<const Point2> points);
[[nodiscard]] SegmentCellKey orderedSegmentKey(GridIndex start, GridIndex end) noexcept;
[[nodiscard]] bool cachedSegmentTraversable(const OccupancyGrid2D& grid,
                                            const Point2 start, const Point2 end,
                                            SegmentTraversabilityCache& cache,
                                            std::size_t& hits, std::size_t& misses);
[[nodiscard]] std::pair<std::size_t, std::size_t>
localScoreWindowForCenter(const std::size_t center_index,
                          const std::size_t sample_count,
                          const std::size_t radius_samples) noexcept;
void pointsFromOffsetsRange(const std::span<const CorridorSample> corridor_samples,
                            const std::span<const double> offsets,
                            const std::size_t begin_index, const std::size_t end_index,
                            std::vector<Point2>& points);
void copyRange(const std::span<const Point2> source, const std::size_t begin_index,
               const std::size_t end_index, std::vector<Point2>& destination);
void copyRange(const std::span<const double> source, const std::size_t begin_index,
               const std::size_t end_index, std::vector<double>& destination);
void copyRange(const std::span<const CorridorSample> source,
               const std::size_t begin_index, const std::size_t end_index,
               std::vector<CorridorSample>& destination);
[[nodiscard]] PathEvaluation
evaluateLocalPathWindowCached(const OccupancyGrid2D& grid,
                              const std::span<const Point2> local_points,
                              SegmentTraversabilityCache& cache,
                              std::size_t& cache_hits, std::size_t& cache_misses);
[[nodiscard]] PathEvaluation evaluatePathCached(const OccupancyGrid2D& grid,
                                                const std::span<const Point2> points,
                                                SegmentProhibitedCountCache& cache,
                                                std::size_t& cache_hits,
                                                std::size_t& cache_misses);
[[nodiscard]] LocalCandidateScore
evaluateLocalOffsetPath(const std::span<const CorridorSample> corridor_samples,
                        const std::span<const Point2> base_points,
                        const std::span<const double> base_offsets,
                        const std::span<const double> candidate_offsets,
                        const OccupancyGrid2D& prohibited_grid,
                        const CandidateScore& base_score, const double base_length_m,
                        const std::size_t center_index, CandidateWorkBuffer& buffer);
void addActiveWindow(std::vector<ActiveWindow>& windows,
                     const std::span<const CorridorSample> samples,
                     const std::size_t center_index, const double pre_margin_m,
                     const double post_margin_m);
[[nodiscard]] bool addActiveWindowRange(std::vector<ActiveWindow>& windows,
                                        const std::span<const CorridorSample> samples,
                                        const double begin_s_m, const double end_s_m);
void mergeActiveWindows(std::vector<ActiveWindow>& windows);
[[nodiscard]] std::size_t
activeWindowControlSampleCount(const std::span<const ActiveWindow> windows);
[[nodiscard]] std::size_t fullPathActiveSampleCount(const std::size_t sample_count);
[[nodiscard]] double headingSpanAround(const std::span<const CorridorSample> samples,
                                       const std::size_t center_index,
                                       const double pre_margin_m,
                                       const double post_margin_m);
[[nodiscard]] std::vector<ActiveWindow> detectActiveWindows(
    const std::span<const CorridorSample> samples,
    const std::span<const Point2> centerline, const OccupancyGrid2D& prohibited_grid,
    const TrajectoryOptimizerConfig& config, TrajectoryOptimizerStats& stats);
[[nodiscard]] std::vector<std::size_t>
activeControlIndices(const std::span<const ActiveWindow> windows,
                     const std::size_t sample_count,
                     std::vector<std::uint8_t>& mutable_indices);
[[nodiscard]] std::vector<TrajectoryOptimizerWindowMetadata>
windowMetadata(const std::span<const ActiveWindow> windows,
               const std::span<const CorridorSample> samples);
[[nodiscard]] CostBreakdown
costBreakdownForPoints(const std::span<const Point2> points,
                       const std::span<const double> offsets,
                       const TrajectoryOptimizerConfig& config);
[[nodiscard]] double geometrySubtotal(const CostBreakdown& breakdown) noexcept;
[[nodiscard]] bool geometryCostsAreFinite(const CostBreakdown& breakdown) noexcept;
[[nodiscard]] std::optional<IndexRange>
boundedIndexRange(std::size_t begin, std::size_t end, const std::size_t max_index);
[[nodiscard]] double curvatureAtIndex(const std::span<const Point2> points,
                                      const std::size_t index);
[[nodiscard]] CostBreakdown localGeometryCostForChangedSpan(
    const std::span<const Point2> points, const std::span<const double> offsets,
    const OffsetChangeDiagnostics& changed, const TrajectoryOptimizerConfig& config);
[[nodiscard]] std::size_t
shadowSegmentWindowSamples(const OffsetChangeDiagnostics& changed,
                           const std::size_t sample_count);
[[nodiscard]] std::optional<CostBreakdown> incrementalGeometryBreakdownForChangedSpan(
    const CandidateScore& base_score, const std::span<const Point2> base_points,
    const std::span<const double> base_offsets,
    const std::span<const Point2> candidate_points,
    const std::span<const double> candidate_offsets,
    const OffsetChangeDiagnostics& changed, const TrajectoryOptimizerConfig& config);
void populateShadowSegmentScoreDiagnostics(
    EvaluatedCandidate& result, const CandidateScore& base_score,
    const std::span<const Point2> base_points,
    const std::span<const double> base_offsets,
    const std::span<const Point2> candidate_points,
    const std::span<const double> candidate_offsets,
    const OffsetChangeDiagnostics& changed, const TrajectoryOptimizerConfig& config,
    const double incumbent_score,
    const std::optional<CostBreakdown>& incremental_geometry_breakdown);
[[nodiscard]] CandidateScore scoreForCandidate(
    const std::span<const CorridorSample> corridor_samples,
    const std::span<const Point2> points, const std::span<const double> offsets,
    const PathEvaluation& evaluation, const TrajectoryOptimizerConfig& config,
    std::vector<TrajectoryPointSample>& scratch_samples,
    TrajectoryOptimizerStats& stats, const CostBreakdown* geometry_breakdown_override);
void populateSampleGeometry(std::vector<TrajectoryPointSample>& samples);
void updateCurvatureStats(const std::span<const TrajectoryPointSample> samples,
                          TrajectoryOptimizerStats& stats);
void copyTraversalEstimateToFinalStats(const TraversalTimeEstimate& estimate,
                                       TrajectoryOptimizerStats& stats);
void copyCostBreakdownToStats(const CostBreakdown& breakdown,
                              TrajectoryOptimizerStats& stats);
void copyCostBreakdownToCandidateDiagnostic(
    const CostBreakdown& breakdown, TrajectoryOptimizerCandidateDiagnostic& diagnostic);
void populateCandidateDiagnosticFromScore(
    TrajectoryOptimizerCandidateDiagnostic& diagnostic, const CandidateScore& score,
    const PathEvaluation& evaluation, const double incumbent_score, const bool accepted,
    const double point_build_duration_ms, const double path_evaluation_duration_ms,
    const double score_duration_ms, const double full_score_duration_ms);
void updateEdgeMarginStats(const std::span<const TrajectoryPointSample> samples,
                           TrajectoryOptimizerStats& stats);
[[nodiscard]] bool updateBestCandidate(
    const std::span<const CorridorSample> corridor_samples,
    const std::span<const double> candidate_offsets,
    const std::span<const Point2> candidate_points,
    const OccupancyGrid2D& prohibited_grid, const TrajectoryOptimizerConfig& config,
    double& best_cost, std::vector<double>& offsets, std::vector<Point2>& best_points,
    CandidateScore& best_score, double& best_length_m,
    std::vector<TrajectoryPointSample>& scratch_samples,
    SegmentProhibitedCountCache& segment_cache, TrajectoryOptimizerStats& stats,
    TrajectoryOptimizerCandidateDiagnostic* diagnostic = nullptr);
[[nodiscard]] EvaluatedCandidate evaluateCandidateSnapshot(
    const std::span<const CorridorSample> corridor_samples,
    const std::span<const double> base_offsets,
    const std::span<const Point2> base_points, const CandidateScore& base_score,
    const double base_length_m, const std::size_t center_index, const double delta_m,
    const OccupancyGrid2D& prohibited_grid, const TrajectoryOptimizerConfig& config,
    const std::span<const std::uint8_t> mutable_indices, const double incumbent_score,
    CandidateWorkBuffer& buffer);
[[nodiscard]] std::vector<double> offsetCandidatesForSample(
    const CorridorSample& sample, const double offset_step_m,
    const std::optional<double> center_offset = std::nullopt,
    const double radius_m = std::numeric_limits<double>::infinity());
[[nodiscard]] bool buildDpSeedForWindow(
    const std::span<const CorridorSample> corridor_samples, const ActiveWindow& window,
    const OccupancyGrid2D& prohibited_grid, const TrajectoryOptimizerConfig& config,
    const double requested_step_m, const std::span<const double> base_offsets,
    const std::span<const double> guide_offsets, const double guide_radius_m,
    std::vector<double>& output_offsets, TrajectoryOptimizerStats& stats);
void smoothedOffsets(const std::span<const double> offsets,
                     const std::span<const CorridorSample> corridor_samples,
                     std::vector<double>& smoothed);
[[nodiscard]] std::vector<CandidateTask>
candidateTasksForStep(const std::span<const std::size_t> control_indices,
                      const double step);
[[nodiscard]] std::span<const CandidateBatchResult> evaluateCandidateBatch(
    const std::span<const CandidateTask> tasks,
    const std::span<const CorridorSample> corridor_samples,
    const std::span<const double> base_offsets,
    const std::span<const Point2> base_points, const CandidateScore& base_score,
    const double base_length_m, const OccupancyGrid2D& prohibited_grid,
    const TrajectoryOptimizerConfig& config,
    const std::span<const std::uint8_t> mutable_indices, const double incumbent_score,
    CandidateBatchWorkspace& workspace, TrajectoryOptimizerCandidateWorkerPool* pool,
    TrajectoryOptimizerStats& stats, const std::size_t worker_count);
void incrementLocalFullScoreReason(const LocalFullScoreReason reason,
                                   TrajectoryOptimizerStats& stats);
void mergeCandidateStats(const EvaluatedCandidate& candidate,
                         TrajectoryOptimizerStats& stats);
[[nodiscard]] TrajectoryOptimizerCandidateDiagnostic candidateDiagnosticFromBatchResult(
    const CandidateBatchResult& batch_result,
    const std::span<const CorridorSample> corridor_samples, const std::size_t iteration,
    const double step_m, const double incumbent_score, const bool selected);

} // namespace drone_city_nav::trajectory_optimizer_detail
