#include "drone_city_nav/trajectory_optimizer.hpp"

#include "drone_city_nav/trajectory_diagnostics.hpp"

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
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;
constexpr double kCollisionPenalty = 1.0e9;
constexpr double kOutsideGridPenalty = 1.0e9;
constexpr double kHeadingJumpPenalty = 1.0e6;
constexpr double kHeadingJumpHardPenalty = 1.0e9;
constexpr double kHeadingJumpSoftLimitRad = std::numbers::pi / 2.0;
constexpr double kHeadingJumpHardLimitRad = 2.0 * std::numbers::pi / 3.0;

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

[[nodiscard]] const char*
localFullScoreReasonName(const LocalFullScoreReason reason) noexcept {
  switch (reason) {
    case LocalFullScoreReason::kNone:
      return "none";
    case LocalFullScoreReason::kInvalidInput:
      return "invalid_input";
    case LocalFullScoreReason::kBoundaryWindow:
      return "boundary_window";
    case LocalFullScoreReason::kUnsafeBase:
      return "unsafe_base";
    case LocalFullScoreReason::kWindowInvalid:
      return "window_invalid";
  }
  return "unknown";
}

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
  bool shadow_lower_bound_valid{false};
  bool shadow_lower_bound_would_prune{false};
  double shadow_lower_bound_score{std::numeric_limits<double>::quiet_NaN()};
  double shadow_lower_bound_incumbent_score{std::numeric_limits<double>::quiet_NaN()};
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

void reserveCandidateWorkBuffer(CandidateWorkBuffer& buffer,
                                const std::size_t offset_count,
                                const std::size_t sample_count) {
  buffer.offsets.reserve(offset_count);
  buffer.points.reserve(sample_count);
  const std::size_t local_capacity = std::min<std::size_t>(sample_count, 16U);
  buffer.local_base_points.reserve(local_capacity);
  buffer.local_candidate_points.reserve(local_capacity);
  buffer.local_corridor_samples.reserve(local_capacity);
  buffer.local_base_offsets.reserve(local_capacity);
  buffer.local_candidate_offsets.reserve(local_capacity);
  buffer.samples.reserve(sample_count);
}

void prepareCandidateWorkspace(CandidateBatchWorkspace& workspace,
                               const std::size_t min_worker_count,
                               const std::size_t offset_count,
                               const std::size_t sample_count,
                               TrajectoryOptimizerStats& stats) {
  const auto started_at = std::chrono::steady_clock::now();
  if (workspace.worker_buffers.size() < min_worker_count) {
    workspace.worker_buffers.resize(min_worker_count);
  }
  for (CandidateWorkBuffer& buffer : workspace.worker_buffers) {
    reserveCandidateWorkBuffer(buffer, offset_count, sample_count);
  }
  stats.candidate_worker_buffer_prepare_duration_ms += elapsedMilliseconds(started_at);
}

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

[[nodiscard]] double dot(const Point2 lhs, const Point2 rhs) noexcept {
  return lhs.x * rhs.x + lhs.y * rhs.y;
}

void startBlockedSpanDiagnostic(PathEvaluation& evaluation,
                                const std::size_t segment_index,
                                const double segment_start_s_m,
                                const Point2 segment_start) {
  if (evaluation.blocked_span_diagnostic_count >=
      kMaxCenterlineBlockedSpanDiagnostics) {
    return;
  }
  TrajectoryOptimizerBlockedSpanDiagnostic& span =
      evaluation.blocked_span_diagnostics.at(evaluation.blocked_span_diagnostic_count);
  span.begin_segment_index = segment_index;
  span.end_segment_index = segment_index;
  span.begin_s_m = segment_start_s_m;
  span.end_s_m = segment_start_s_m;
  span.length_m = 0.0;
  span.begin_x_m = segment_start.x;
  span.begin_y_m = segment_start.y;
  span.end_x_m = segment_start.x;
  span.end_y_m = segment_start.y;
  ++evaluation.blocked_span_diagnostic_count;
}

void updateBlockedSpanDiagnostic(PathEvaluation& evaluation,
                                 const std::size_t diagnostic_index,
                                 const std::size_t segment_index,
                                 const double segment_end_s_m, const Point2 segment_end,
                                 const std::size_t prohibited_cells,
                                 const std::size_t outside_grid_segments) {
  if (diagnostic_index >= evaluation.blocked_span_diagnostic_count) {
    return;
  }
  TrajectoryOptimizerBlockedSpanDiagnostic& span =
      evaluation.blocked_span_diagnostics.at(diagnostic_index);
  span.end_segment_index = segment_index;
  span.end_s_m = segment_end_s_m;
  span.length_m = span.end_s_m - span.begin_s_m;
  span.end_x_m = segment_end.x;
  span.end_y_m = segment_end.y;
  span.prohibited_cells += prohibited_cells;
  span.outside_grid_segments += outside_grid_segments;
}

void recordBlockedPoint(PathEvaluation& evaluation, const std::size_t segment_index,
                        const double segment_start_s_m, const Point2 segment_start,
                        const Point2 segment_end, const Point2 blocked_point,
                        const bool outside_grid) {
  const Point2 segment = segment_end - segment_start;
  const double segment_length_sq = squaredDistance(segment_start, segment_end);
  const double t =
      segment_length_sq > kTinyDistanceM * kTinyDistanceM
          ? std::clamp(dot(blocked_point - segment_start, segment) / segment_length_sq,
                       0.0, 1.0)
          : 0.0;
  const double blocked_s_m =
      segment_start_s_m + t * distance(segment_start, segment_end);
  if (!evaluation.has_first_blocked_point ||
      blocked_s_m < evaluation.first_blocked_s_m) {
    evaluation.first_blocked_segment_index = segment_index;
    evaluation.first_blocked_s_m = blocked_s_m;
    evaluation.first_blocked_point = blocked_point;
    evaluation.has_first_blocked_point = true;
    evaluation.first_blocked_outside_grid = outside_grid;
  }
  if (!evaluation.has_last_blocked_point ||
      blocked_s_m >= evaluation.last_blocked_s_m) {
    evaluation.last_blocked_segment_index = segment_index;
    evaluation.last_blocked_s_m = blocked_s_m;
    evaluation.last_blocked_point = blocked_point;
    evaluation.has_last_blocked_point = true;
    evaluation.last_blocked_outside_grid = outside_grid;
  }
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

[[nodiscard]] double percentileValue(std::vector<double>& values,
                                     const double percentile) {
  if (values.empty()) {
    return 0.0;
  }
  std::ranges::sort(values);
  const double bounded_percentile =
      std::isfinite(percentile) ? std::clamp(percentile, 0.0, 1.0) : 1.0;
  const std::size_t index =
      bounded_percentile <= 0.0
          ? 0U
          : std::min<std::size_t>(
                values.size() - 1U,
                static_cast<std::size_t>(std::ceil(
                    bounded_percentile * static_cast<double>(values.size()))) -
                    1U);
  return values[index];
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

[[nodiscard]] OffsetChangeDiagnostics
offsetChangeDiagnostics(const std::span<const double> base_offsets,
                        const std::span<const double> candidate_offsets) noexcept {
  OffsetChangeDiagnostics diagnostics{};
  if (base_offsets.size() != candidate_offsets.size() || base_offsets.empty()) {
    return diagnostics;
  }
  for (std::size_t i = 0U; i < base_offsets.size(); ++i) {
    if (std::abs(base_offsets[i] - candidate_offsets[i]) <= 1.0e-9) {
      continue;
    }
    if (diagnostics.changed_samples == 0U) {
      diagnostics.first_changed_index = i;
    }
    diagnostics.last_changed_index = i;
    ++diagnostics.changed_samples;
  }
  if (diagnostics.changed_samples > 0U) {
    diagnostics.changed_span_samples =
        diagnostics.last_changed_index - diagnostics.first_changed_index + 1U;
  }
  return diagnostics;
}

[[nodiscard]] std::size_t
estimatedLocalSpeedWindowSamples(const OffsetChangeDiagnostics& diagnostics,
                                 const std::size_t sample_count) noexcept {
  if (diagnostics.changed_samples == 0U || sample_count == 0U) {
    return 0U;
  }
  constexpr std::size_t kGeometryNeighborSamples = 2U;
  constexpr std::size_t kProfileContextSamples = 12U;
  const std::size_t context = kGeometryNeighborSamples + kProfileContextSamples;
  const std::size_t begin = diagnostics.first_changed_index > context
                                ? diagnostics.first_changed_index - context
                                : 0U;
  const std::size_t end =
      std::min(sample_count - 1U, diagnostics.last_changed_index + context);
  return end >= begin ? end - begin + 1U : 0U;
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
    sample.lateral_offset_m = offsets[i];
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

[[nodiscard]] const char* initialOffsetSeedName(const InitialOffsetSeed seed) noexcept {
  switch (seed) {
    case InitialOffsetSeed::kCenterline:
      return "seed_centerline";
    case InitialOffsetSeed::kCorridorMidline:
      return "seed_corridor_midline";
    case InitialOffsetSeed::kLeftBiased:
      return "seed_left_biased";
    case InitialOffsetSeed::kRightBiased:
      return "seed_right_biased";
  }
  return "seed_unknown";
}

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
                         const TrajectoryOptimizerConfig& config) {
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

[[nodiscard]] CostBreakdown
costBreakdownForPoints(std::span<const Point2> points, std::span<const double> offsets,
                       const TrajectoryOptimizerConfig& config);

[[nodiscard]] PathEvaluation evaluatePath(const OccupancyGrid2D& grid,
                                          const std::span<const Point2> points) {
  PathEvaluation evaluation{};
  if (points.size() < 2U) {
    ++evaluation.outside_grid_segments;
    return evaluation;
  }

  bool previous_segment_blocked = false;
  std::size_t current_span_diagnostic_index = kMaxCenterlineBlockedSpanDiagnostics;
  for (std::size_t i = 1U; i < points.size(); ++i) {
    const Point2 start = points[i - 1U];
    const Point2 end = points[i];
    const double segment_start_s_m = evaluation.length_m;
    evaluation.length_m += distance(start, end);
    const std::optional<GridIndex> start_cell = grid.worldToCell(start);
    const std::optional<GridIndex> end_cell = grid.worldToCell(end);
    bool segment_blocked = false;
    std::size_t segment_prohibited_cells = 0U;
    std::size_t segment_outside_grid_segments = 0U;
    if (!start_cell.has_value() || !end_cell.has_value()) {
      ++evaluation.outside_grid_segments;
      segment_outside_grid_segments = 1U;
      segment_blocked = true;
      recordBlockedPoint(evaluation, i - 1U, segment_start_s_m, start, end, start,
                         true);
      recordBlockedPoint(evaluation, i - 1U, segment_start_s_m, start, end, end, true);
    } else {
      const std::vector<GridIndex> cells = grid.cellsOnLine(*start_cell, *end_cell);
      for (const GridIndex cell : cells) {
        if (grid.isProhibited(cell)) {
          ++evaluation.prohibited_cells;
          ++segment_prohibited_cells;
          segment_blocked = true;
          recordBlockedPoint(evaluation, i - 1U, segment_start_s_m, start, end,
                             grid.cellCenter(cell), false);
        }
      }
    }
    if (segment_blocked) {
      ++evaluation.blocked_segment_count;
      if (!previous_segment_blocked) {
        ++evaluation.blocked_span_count;
        const std::size_t previous_diagnostic_count =
            evaluation.blocked_span_diagnostic_count;
        startBlockedSpanDiagnostic(evaluation, i - 1U, segment_start_s_m, start);
        current_span_diagnostic_index =
            evaluation.blocked_span_diagnostic_count > previous_diagnostic_count
                ? evaluation.blocked_span_diagnostic_count - 1U
                : kMaxCenterlineBlockedSpanDiagnostics;
      }
      updateBlockedSpanDiagnostic(evaluation, current_span_diagnostic_index, i - 1U,
                                  evaluation.length_m, end, segment_prohibited_cells,
                                  segment_outside_grid_segments);
    } else {
      current_span_diagnostic_index = kMaxCenterlineBlockedSpanDiagnostics;
    }
    previous_segment_blocked = segment_blocked;
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

[[nodiscard]] LocalCandidateScore
evaluateLocalOffsetPath(const std::span<const CorridorSample> corridor_samples,
                        const std::span<const Point2> base_points,
                        const std::span<const double> base_offsets,
                        const std::span<const double> candidate_offsets,
                        const OccupancyGrid2D& prohibited_grid,
                        const CandidateScore& base_score, const double base_length_m,
                        const std::size_t center_index, CandidateWorkBuffer& buffer) {
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
    result.shadow_boundary_clamped_window_samples =
        end_index >= begin_index ? end_index - begin_index + 1U : 0U;
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
  if (!windows.empty() && begin >= windows.back().begin_index &&
      begin <= windows.back().end_index + 1U) {
    windows.back().end_index = std::max(windows.back().end_index, end);
    return;
  }
  windows.push_back(ActiveWindow{begin, end});
}

[[nodiscard]] bool addActiveWindowRange(std::vector<ActiveWindow>& windows,
                                        const std::span<const CorridorSample> samples,
                                        const double begin_s_m, const double end_s_m) {
  if (samples.empty() || !std::isfinite(begin_s_m) || !std::isfinite(end_s_m) ||
      begin_s_m >= end_s_m) {
    return false;
  }

  std::size_t begin = 0U;
  while (begin + 1U < samples.size() && samples[begin + 1U].s_m <= begin_s_m) {
    ++begin;
  }
  std::size_t end = begin;
  while (end + 1U < samples.size() && samples[end].s_m < end_s_m) {
    ++end;
  }
  if (begin >= end) {
    return false;
  }
  if (!windows.empty() && begin >= windows.back().begin_index &&
      begin <= windows.back().end_index + 1U) {
    windows.back().end_index = std::max(windows.back().end_index, end);
    return true;
  }
  windows.push_back(ActiveWindow{begin, end});
  return true;
}

void mergeActiveWindows(std::vector<ActiveWindow>& windows) {
  if (windows.size() < 2U) {
    return;
  }
  std::sort(windows.begin(), windows.end(),
            [](const ActiveWindow& lhs, const ActiveWindow& rhs) {
              if (lhs.begin_index == rhs.begin_index) {
                return lhs.end_index < rhs.end_index;
              }
              return lhs.begin_index < rhs.begin_index;
            });
  std::vector<ActiveWindow> merged;
  merged.reserve(windows.size());
  for (const ActiveWindow& window : windows) {
    if (window.begin_index >= window.end_index) {
      continue;
    }
    if (!merged.empty() && window.begin_index <= merged.back().end_index + 1U) {
      merged.back().end_index = std::max(merged.back().end_index, window.end_index);
      continue;
    }
    merged.push_back(window);
  }
  windows = std::move(merged);
}

[[nodiscard]] std::size_t
activeWindowControlSampleCount(const std::span<const ActiveWindow> windows) {
  std::size_t count = 0U;
  for (const ActiveWindow& window : windows) {
    if (window.end_index > window.begin_index + 1U) {
      count += window.end_index - window.begin_index - 1U;
    }
  }
  return count;
}

[[nodiscard]] std::size_t fullPathActiveSampleCount(const std::size_t sample_count) {
  return sample_count > 2U ? sample_count - 2U : 0U;
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

[[nodiscard]] std::vector<ActiveWindow> detectActiveWindows(
    const std::span<const CorridorSample> samples,
    const std::span<const Point2> centerline, const OccupancyGrid2D& prohibited_grid,
    const TrajectoryOptimizerConfig& config, TrajectoryOptimizerStats& stats) {
  const auto started_at = std::chrono::steady_clock::now();
  std::vector<ActiveWindow> windows;
  std::vector<ActiveWindow> shadow_no_width_asymmetry_windows;
  std::vector<ActiveWindow> shadow_no_width_triggers_windows;
  std::vector<ActiveWindow> shadow_no_heading_span_windows;
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
    stats.active_window_centerline_blocked = 1U;
    stats.centerline_blocked_prohibited_cells = centerline_evaluation.prohibited_cells;
    stats.centerline_blocked_outside_grid_segments =
        centerline_evaluation.outside_grid_segments;
    stats.centerline_blocked_segment_count =
        centerline_evaluation.blocked_segment_count;
    stats.centerline_blocked_span_count = centerline_evaluation.blocked_span_count;
    stats.centerline_blocked_span_diagnostic_count =
        centerline_evaluation.blocked_span_diagnostic_count;
    stats.centerline_blocked_span_diagnostics =
        centerline_evaluation.blocked_span_diagnostics;
    if (centerline_evaluation.has_first_blocked_point) {
      stats.centerline_blocked_first_segment_index =
          centerline_evaluation.first_blocked_segment_index;
      stats.centerline_blocked_first_s_m = centerline_evaluation.first_blocked_s_m;
      stats.centerline_blocked_first_x_m = centerline_evaluation.first_blocked_point.x;
      stats.centerline_blocked_first_y_m = centerline_evaluation.first_blocked_point.y;
      stats.centerline_blocked_first_outside_grid =
          centerline_evaluation.first_blocked_outside_grid;
    }
    if (centerline_evaluation.has_last_blocked_point) {
      stats.centerline_blocked_last_segment_index =
          centerline_evaluation.last_blocked_segment_index;
      stats.centerline_blocked_last_s_m = centerline_evaluation.last_blocked_s_m;
      stats.centerline_blocked_last_x_m = centerline_evaluation.last_blocked_point.x;
      stats.centerline_blocked_last_y_m = centerline_evaluation.last_blocked_point.y;
      stats.centerline_blocked_last_outside_grid =
          centerline_evaluation.last_blocked_outside_grid;
    }
    if (std::isfinite(stats.centerline_blocked_first_s_m) &&
        std::isfinite(stats.centerline_blocked_last_s_m)) {
      stats.centerline_blocked_span_length_m =
          stats.centerline_blocked_last_s_m - stats.centerline_blocked_first_s_m;
    }

    bool use_full_path_fallback = centerline_evaluation.outside_grid_segments > 0U ||
                                  centerline_evaluation.blocked_span_count == 0U ||
                                  centerline_evaluation.blocked_span_count >
                                      kMaxCenterlineBlockedSpanDiagnostics ||
                                  centerline_evaluation.blocked_span_diagnostic_count !=
                                      centerline_evaluation.blocked_span_count;
    if (!use_full_path_fallback) {
      for (std::size_t i = 0U; i < centerline_evaluation.blocked_span_diagnostic_count;
           ++i) {
        const TrajectoryOptimizerBlockedSpanDiagnostic& span =
            centerline_evaluation.blocked_span_diagnostics.at(i);
        if (span.outside_grid_segments > 0U || !std::isfinite(span.begin_s_m) ||
            !std::isfinite(span.end_s_m) || span.begin_s_m >= span.end_s_m) {
          use_full_path_fallback = true;
          break;
        }
        if (!addActiveWindowRange(windows, samples, span.begin_s_m - pre_margin_m,
                                  span.end_s_m + post_margin_m)) {
          use_full_path_fallback = true;
          break;
        }
        ++stats.centerline_blocked_windows;
      }
    }

    if (!use_full_path_fallback) {
      mergeActiveWindows(windows);
      stats.centerline_blocked_window_merged_count = windows.size();
      stats.centerline_blocked_window_samples = activeWindowControlSampleCount(windows);
      const std::size_t full_path_samples = fullPathActiveSampleCount(samples.size());
      if (windows.empty() || full_path_samples == 0U ||
          stats.centerline_blocked_window_samples * 10U >= full_path_samples * 9U) {
        use_full_path_fallback = true;
      }
    }

    if (use_full_path_fallback) {
      windows.clear();
      windows.push_back(ActiveWindow{0U, samples.size() - 1U});
      stats.window_count = 1U;
      stats.active_window_count = 1U;
      stats.active_window_samples = fullPathActiveSampleCount(samples.size());
      stats.shadow_active_window_no_width_asymmetry_count = 1U;
      stats.shadow_active_window_no_width_asymmetry_samples =
          stats.active_window_samples;
      stats.shadow_active_window_no_width_triggers_count = 1U;
      stats.shadow_active_window_no_width_triggers_samples =
          stats.active_window_samples;
      stats.shadow_active_window_no_heading_span_count = 1U;
      stats.shadow_active_window_no_heading_span_samples = stats.active_window_samples;
      stats.window_detection_duration_ms = elapsedMilliseconds(started_at);
      return windows;
    }
  }
  shadow_no_width_asymmetry_windows = windows;
  shadow_no_width_triggers_windows = windows;
  shadow_no_heading_span_windows = windows;

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
    const bool heading_change_trigger = heading_change >= heading_threshold_rad;
    const bool heading_span_trigger = heading_span >= heading_span_threshold_rad;
    const bool curvature_trigger = curvature >= curvature_threshold;
    const bool width_change_trigger =
        std::abs(next_width - previous_width) >= width_threshold_m;
    const bool width_asymmetry_trigger = width_asymmetry >= width_asymmetry_threshold_m;
    if (heading_change_trigger) {
      ++stats.active_window_heading_change_samples;
    }
    if (heading_span_trigger) {
      ++stats.active_window_heading_span_samples;
    }
    if (curvature_trigger) {
      ++stats.active_window_curvature_samples;
    }
    if (width_change_trigger) {
      ++stats.active_window_width_change_samples;
    }
    if (width_asymmetry_trigger) {
      ++stats.active_window_width_asymmetry_samples;
    }
    const bool turn_zone =
        heading_change_trigger || heading_span_trigger || curvature_trigger;
    const bool width_zone = width_change_trigger || width_asymmetry_trigger;
    if (turn_zone || width_zone) {
      addActiveWindow(windows, samples, i, pre_margin_m, post_margin_m);
    }
    if (turn_zone || width_change_trigger) {
      addActiveWindow(shadow_no_width_asymmetry_windows, samples, i, pre_margin_m,
                      post_margin_m);
    }
    if (turn_zone) {
      addActiveWindow(shadow_no_width_triggers_windows, samples, i, pre_margin_m,
                      post_margin_m);
    }
    if (heading_change_trigger || curvature_trigger || width_zone) {
      addActiveWindow(shadow_no_heading_span_windows, samples, i, pre_margin_m,
                      post_margin_m);
    }
  }

  mergeActiveWindows(windows);
  mergeActiveWindows(shadow_no_width_asymmetry_windows);
  mergeActiveWindows(shadow_no_width_triggers_windows);
  mergeActiveWindows(shadow_no_heading_span_windows);
  stats.window_count = windows.size();
  stats.active_window_count = windows.size();
  stats.active_window_samples = activeWindowControlSampleCount(windows);
  stats.shadow_active_window_no_width_asymmetry_count =
      shadow_no_width_asymmetry_windows.size();
  stats.shadow_active_window_no_width_asymmetry_samples =
      activeWindowControlSampleCount(shadow_no_width_asymmetry_windows);
  stats.shadow_active_window_no_width_triggers_count =
      shadow_no_width_triggers_windows.size();
  stats.shadow_active_window_no_width_triggers_samples =
      activeWindowControlSampleCount(shadow_no_width_triggers_windows);
  stats.shadow_active_window_no_heading_span_count =
      shadow_no_heading_span_windows.size();
  stats.shadow_active_window_no_heading_span_samples =
      activeWindowControlSampleCount(shadow_no_heading_span_windows);
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

[[nodiscard]] std::vector<TrajectoryOptimizerWindowMetadata>
windowMetadata(const std::span<const ActiveWindow> windows,
               const std::span<const CorridorSample> samples) {
  std::vector<TrajectoryOptimizerWindowMetadata> metadata;
  metadata.reserve(windows.size());
  for (std::size_t i = 0U; i < windows.size(); ++i) {
    const ActiveWindow& window = windows[i];
    if (window.begin_index >= samples.size() || window.end_index >= samples.size()) {
      continue;
    }
    metadata.push_back(TrajectoryOptimizerWindowMetadata{
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
                       const TrajectoryOptimizerConfig& config) {
  CostBreakdown breakdown{};
  if (points.size() < 2U) {
    breakdown.outside_grid_cost = kOutsideGridPenalty;
    return breakdown;
  }

  const double weight_curvature =
      sanitizedPositive(config.weight_curvature, 300.0, 0.0, 1.0e9);
  const double weight_curvature_change =
      sanitizedPositive(config.weight_curvature_change, 180.0, 0.0, 1.0e9);
  const double preferred_min_radius =
      sanitizedPositive(config.preferred_min_radius_m, 24.0, 0.0, 100000.0);
  const double weight_radius_shortfall =
      sanitizedPositive(config.weight_radius_shortfall, 40.0, 0.0, 1.0e9);
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
  double radius_shortfall_cost = 0.0;
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
    if (preferred_min_radius > kTinyDistanceM) {
      const double target_curvature = 1.0 / preferred_min_radius;
      const double shortfall_ratio =
          std::max(0.0, std::abs(curvature) - target_curvature) / target_curvature;
      radius_shortfall_cost += shortfall_ratio * shortfall_ratio;
    }
    if (previous_curvature_valid) {
      const double change = curvature - previous_curvature;
      curvature_change_cost += change * change;
    }
    previous_curvature = curvature;
    previous_curvature_valid = true;
  }

  breakdown.curvature_cost = weight_curvature * curvature_cost;
  breakdown.curvature_change_cost = weight_curvature_change * curvature_change_cost;
  breakdown.radius_shortfall_cost = weight_radius_shortfall * radius_shortfall_cost;
  breakdown.offset_change_cost = weight_offset_change * offset_change_cost;
  breakdown.offset_second_change_cost =
      weight_offset_second_change * offset_second_change_cost;
  breakdown.offset_slope_cost = weight_offset_slope * offset_slope_cost;
  return breakdown;
}

[[nodiscard]] double geometrySubtotal(const CostBreakdown& breakdown) noexcept {
  return breakdown.curvature_cost + breakdown.curvature_change_cost +
         breakdown.radius_shortfall_cost + breakdown.offset_change_cost +
         breakdown.offset_second_change_cost + breakdown.offset_slope_cost;
}

[[nodiscard]] bool geometryCostsAreFinite(const CostBreakdown& breakdown) noexcept {
  return std::isfinite(breakdown.curvature_cost) &&
         std::isfinite(breakdown.curvature_change_cost) &&
         std::isfinite(breakdown.radius_shortfall_cost) &&
         std::isfinite(breakdown.offset_change_cost) &&
         std::isfinite(breakdown.offset_second_change_cost) &&
         std::isfinite(breakdown.offset_slope_cost);
}

[[nodiscard]] std::optional<IndexRange>
boundedIndexRange(std::size_t begin, std::size_t end, const std::size_t max_index) {
  if (end > max_index) {
    end = max_index;
  }
  if (begin > end) {
    return std::nullopt;
  }
  return IndexRange{.begin = begin, .end = end};
}

[[nodiscard]] double curvatureAtIndex(const std::span<const Point2> points,
                                      const std::size_t index) {
  if (index == 0U || index + 1U >= points.size()) {
    return 0.0;
  }
  return discreteCurvature(points[index - 1U], points[index], points[index + 1U]);
}

[[nodiscard]] CostBreakdown localGeometryCostForChangedSpan(
    const std::span<const Point2> points, const std::span<const double> offsets,
    const OffsetChangeDiagnostics& changed, const TrajectoryOptimizerConfig& config) {
  CostBreakdown breakdown{};
  if (points.size() < 2U || points.size() != offsets.size() ||
      changed.changed_samples == 0U || changed.last_changed_index >= points.size()) {
    breakdown.outside_grid_cost = kOutsideGridPenalty;
    return breakdown;
  }

  const double weight_curvature =
      sanitizedPositive(config.weight_curvature, 300.0, 0.0, 1.0e9);
  const double weight_curvature_change =
      sanitizedPositive(config.weight_curvature_change, 180.0, 0.0, 1.0e9);
  const double preferred_min_radius =
      sanitizedPositive(config.preferred_min_radius_m, 24.0, 0.0, 100000.0);
  const double weight_radius_shortfall =
      sanitizedPositive(config.weight_radius_shortfall, 40.0, 0.0, 1.0e9);
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
  double radius_shortfall_cost = 0.0;
  double offset_change_cost = 0.0;
  double offset_second_change_cost = 0.0;
  double offset_slope_cost = 0.0;

  const std::size_t first = changed.first_changed_index;
  const std::size_t last = changed.last_changed_index;
  const std::optional<IndexRange> segment_range = boundedIndexRange(
      std::max<std::size_t>(1U, first), last + 1U, points.size() - 1U);
  if (segment_range.has_value()) {
    for (std::size_t i = segment_range->begin; i <= segment_range->end; ++i) {
      const double ds = distance(points[i - 1U], points[i]);
      const double change = offsets[i] - offsets[i - 1U];
      offset_change_cost += change * change;
      if (ds > kTinyDistanceM) {
        const double slope_violation =
            std::max(0.0, std::abs(change) / ds - max_offset_slope);
        offset_slope_cost += slope_violation * slope_violation * ds;
      }
    }
  }

  const std::optional<IndexRange> triple_range =
      points.size() >= 3U
          ? boundedIndexRange(std::max<std::size_t>(1U, first > 0U ? first - 1U : 1U),
                              last + 1U, points.size() - 2U)
          : std::nullopt;
  if (triple_range.has_value()) {
    for (std::size_t i = triple_range->begin; i <= triple_range->end; ++i) {
      const double second_change = offsets[i + 1U] - 2.0 * offsets[i] + offsets[i - 1U];
      offset_second_change_cost += second_change * second_change;
      const double curvature = curvatureAtIndex(points, i);
      curvature_cost += curvature * curvature;
      if (preferred_min_radius > kTinyDistanceM) {
        const double target_curvature = 1.0 / preferred_min_radius;
        const double shortfall_ratio =
            std::max(0.0, std::abs(curvature) - target_curvature) / target_curvature;
        radius_shortfall_cost += shortfall_ratio * shortfall_ratio;
      }
    }

    const std::optional<IndexRange> curvature_change_range =
        points.size() >= 4U
            ? boundedIndexRange(std::max<std::size_t>(2U, triple_range->begin),
                                triple_range->end + 1U, points.size() - 2U)
            : std::nullopt;
    if (curvature_change_range.has_value()) {
      for (std::size_t i = curvature_change_range->begin;
           i <= curvature_change_range->end; ++i) {
        const double change =
            curvatureAtIndex(points, i) - curvatureAtIndex(points, i - 1U);
        curvature_change_cost += change * change;
      }
    }
  }

  breakdown.curvature_cost = weight_curvature * curvature_cost;
  breakdown.curvature_change_cost = weight_curvature_change * curvature_change_cost;
  breakdown.radius_shortfall_cost = weight_radius_shortfall * radius_shortfall_cost;
  breakdown.offset_change_cost = weight_offset_change * offset_change_cost;
  breakdown.offset_second_change_cost =
      weight_offset_second_change * offset_second_change_cost;
  breakdown.offset_slope_cost = weight_offset_slope * offset_slope_cost;
  return breakdown;
}

[[nodiscard]] std::size_t
shadowSegmentWindowSamples(const OffsetChangeDiagnostics& changed,
                           const std::size_t sample_count) {
  if (sample_count == 0U || changed.changed_samples == 0U ||
      changed.last_changed_index >= sample_count) {
    return 0U;
  }
  const std::size_t begin =
      changed.first_changed_index > 2U ? changed.first_changed_index - 2U : 0U;
  const std::size_t end = std::min(sample_count - 1U, changed.last_changed_index + 2U);
  return end >= begin ? end - begin + 1U : 0U;
}

[[nodiscard]] std::optional<CostBreakdown> incrementalGeometryBreakdownForChangedSpan(
    const CandidateScore& base_score, const std::span<const Point2> base_points,
    const std::span<const double> base_offsets,
    const std::span<const Point2> candidate_points,
    const std::span<const double> candidate_offsets,
    const OffsetChangeDiagnostics& changed, const TrajectoryOptimizerConfig& config) {
  if (changed.changed_samples == 0U || base_points.size() != candidate_points.size() ||
      base_offsets.size() != candidate_offsets.size() ||
      base_points.size() != base_offsets.size() || !std::isfinite(base_score.score)) {
    return std::nullopt;
  }

  const CostBreakdown base_local =
      localGeometryCostForChangedSpan(base_points, base_offsets, changed, config);
  const CostBreakdown candidate_local = localGeometryCostForChangedSpan(
      candidate_points, candidate_offsets, changed, config);
  if (base_local.outside_grid_cost > 0.0 || candidate_local.outside_grid_cost > 0.0 ||
      !geometryCostsAreFinite(base_score.breakdown) ||
      !geometryCostsAreFinite(base_local) || !geometryCostsAreFinite(candidate_local)) {
    return std::nullopt;
  }

  CostBreakdown breakdown{};
  breakdown.curvature_cost = base_score.breakdown.curvature_cost -
                             base_local.curvature_cost + candidate_local.curvature_cost;
  breakdown.curvature_change_cost = base_score.breakdown.curvature_change_cost -
                                    base_local.curvature_change_cost +
                                    candidate_local.curvature_change_cost;
  breakdown.radius_shortfall_cost = base_score.breakdown.radius_shortfall_cost -
                                    base_local.radius_shortfall_cost +
                                    candidate_local.radius_shortfall_cost;
  breakdown.offset_change_cost = base_score.breakdown.offset_change_cost -
                                 base_local.offset_change_cost +
                                 candidate_local.offset_change_cost;
  breakdown.offset_second_change_cost = base_score.breakdown.offset_second_change_cost -
                                        base_local.offset_second_change_cost +
                                        candidate_local.offset_second_change_cost;
  breakdown.offset_slope_cost = base_score.breakdown.offset_slope_cost -
                                base_local.offset_slope_cost +
                                candidate_local.offset_slope_cost;
  if (!geometryCostsAreFinite(breakdown)) {
    return std::nullopt;
  }
  return breakdown;
}

void populateShadowSegmentScoreDiagnostics(
    EvaluatedCandidate& result, const CandidateScore& base_score,
    const std::span<const Point2> base_points,
    const std::span<const double> base_offsets,
    const std::span<const Point2> candidate_points,
    const std::span<const double> candidate_offsets,
    const OffsetChangeDiagnostics& changed, const TrajectoryOptimizerConfig& config,
    const double incumbent_score,
    const std::optional<CostBreakdown>& incremental_geometry_breakdown) {
  if (changed.changed_samples == 0U || base_points.size() != candidate_points.size() ||
      base_offsets.size() != candidate_offsets.size() ||
      base_points.size() != base_offsets.size() || !std::isfinite(base_score.score) ||
      !std::isfinite(result.score.score)) {
    return;
  }
  const std::optional<CostBreakdown> fallback_geometry =
      incremental_geometry_breakdown.has_value()
          ? std::nullopt
          : incrementalGeometryBreakdownForChangedSpan(
                base_score, base_points, base_offsets, candidate_points,
                candidate_offsets, changed, config);
  const CostBreakdown* estimated_geometry = nullptr;
  if (incremental_geometry_breakdown.has_value()) {
    estimated_geometry = &*incremental_geometry_breakdown;
  } else if (fallback_geometry.has_value()) {
    estimated_geometry = &*fallback_geometry;
  }
  if (estimated_geometry == nullptr) {
    return;
  }

  const double full_candidate_geometry_score = geometrySubtotal(result.score.breakdown);
  const double estimated_score = result.score.score - full_candidate_geometry_score +
                                 geometrySubtotal(*estimated_geometry);
  if (!std::isfinite(estimated_score)) {
    return;
  }

  result.shadow_segment_score_valid = true;
  result.shadow_segment_score_estimated_score = estimated_score;
  result.shadow_segment_score_incumbent_score = incumbent_score;
  result.shadow_segment_score_window_samples =
      shadowSegmentWindowSamples(changed, base_points.size());
  result.shadow_segment_score_would_prune =
      std::isfinite(incumbent_score) && estimated_score + 1.0e-9 >= incumbent_score;
}

[[nodiscard]] double
conservativeShadowLowerBoundScore(const std::span<const Point2> points,
                                  const std::span<const double> offsets,
                                  const TrajectoryOptimizerConfig& config) {
  if (points.size() < 2U || points.size() != offsets.size()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const CostBreakdown geometry = costBreakdownForPoints(points, offsets, config);
  return geometry.curvature_cost + geometry.curvature_change_cost +
         geometry.offset_change_cost + geometry.offset_second_change_cost +
         geometry.offset_slope_cost;
}

[[nodiscard]] CandidateScore scoreForCandidate(
    const std::span<const CorridorSample> corridor_samples,
    const std::span<const Point2> points, const std::span<const double> offsets,
    const PathEvaluation& evaluation, const TrajectoryOptimizerConfig& config,
    std::vector<TrajectoryPointSample>& scratch_samples,
    TrajectoryOptimizerStats& stats, const CostBreakdown* geometry_breakdown_override) {
  CandidateScore result{};
  const auto cost_started_at = std::chrono::steady_clock::now();
  if (geometry_breakdown_override != nullptr) {
    result.breakdown = *geometry_breakdown_override;
  } else {
    result.breakdown = costBreakdownForPoints(points, offsets, config);
  }
  result.breakdown.collision_cost =
      static_cast<double>(evaluation.prohibited_cells) * kCollisionPenalty;
  result.breakdown.outside_grid_cost =
      static_cast<double>(evaluation.outside_grid_segments) * kOutsideGridPenalty;
  stats.candidate_cost_breakdown_duration_ms += elapsedMilliseconds(cost_started_at);
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
                          TrajectoryOptimizerStats& stats) {
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
                                       TrajectoryOptimizerStats& stats) {
  stats.estimated_time_s = estimate.estimated_time_s;
  stats.min_speed_limit_mps = estimate.min_speed_limit_mps;
  stats.max_speed_limit_mps = estimate.max_speed_limit_mps;
  stats.curvature_limited_samples = estimate.curvature_limited_samples;
}

void copyCostBreakdownToStats(const CostBreakdown& breakdown,
                              TrajectoryOptimizerStats& stats) {
  stats.cost_curvature = breakdown.curvature_cost;
  stats.cost_curvature_change = breakdown.curvature_change_cost;
  stats.cost_radius_shortfall = breakdown.radius_shortfall_cost;
  stats.cost_heading_jump = breakdown.heading_jump_cost;
  stats.cost_offset_change = breakdown.offset_change_cost;
  stats.cost_offset_second_change = breakdown.offset_second_change_cost;
  stats.cost_offset_slope = breakdown.offset_slope_cost;
  stats.cost_collision = breakdown.collision_cost;
  stats.cost_outside_grid = breakdown.outside_grid_cost;
}

void copyCostBreakdownToCandidateDiagnostic(
    const CostBreakdown& breakdown,
    TrajectoryOptimizerCandidateDiagnostic& diagnostic) {
  diagnostic.cost_curvature = breakdown.curvature_cost;
  diagnostic.cost_curvature_change = breakdown.curvature_change_cost;
  diagnostic.cost_radius_shortfall = breakdown.radius_shortfall_cost;
  diagnostic.cost_heading_jump = breakdown.heading_jump_cost;
  diagnostic.cost_offset_change = breakdown.offset_change_cost;
  diagnostic.cost_offset_second_change = breakdown.offset_second_change_cost;
  diagnostic.cost_offset_slope = breakdown.offset_slope_cost;
  diagnostic.cost_collision = breakdown.collision_cost;
  diagnostic.cost_outside_grid = breakdown.outside_grid_cost;
}

void populateCandidateDiagnosticFromScore(
    TrajectoryOptimizerCandidateDiagnostic& diagnostic, const CandidateScore& score,
    const PathEvaluation& evaluation, const double incumbent_score, const bool accepted,
    const double point_build_duration_ms, const double path_evaluation_duration_ms,
    const double score_duration_ms, const double full_score_duration_ms) {
  diagnostic.score = score.score;
  diagnostic.incumbent_score = incumbent_score;
  diagnostic.length_m = evaluation.length_m;
  diagnostic.traversable = evaluation.traversable();
  diagnostic.full_score_used = true;
  diagnostic.prohibited_cells = evaluation.prohibited_cells;
  diagnostic.outside_grid_segments = evaluation.outside_grid_segments;
  diagnostic.point_build_duration_ms = point_build_duration_ms;
  diagnostic.path_evaluation_duration_ms = path_evaluation_duration_ms;
  diagnostic.score_duration_ms = score_duration_ms;
  diagnostic.full_score_duration_ms = full_score_duration_ms;
  copyCostBreakdownToCandidateDiagnostic(score.breakdown, diagnostic);
  if (accepted) {
    diagnostic.decision = "selected";
  } else if (evaluation.outside_grid_segments > 0U) {
    diagnostic.decision = "outside_grid";
  } else if (evaluation.prohibited_cells > 0U) {
    diagnostic.decision = "prohibited";
  } else {
    diagnostic.decision = "not_better_than_incumbent";
  }
}

void updateEdgeMarginStats(const std::span<const TrajectoryPointSample> samples,
                           TrajectoryOptimizerStats& stats) {
  double margin_sum = 0.0;
  std::size_t margin_count = 0U;
  for (const TrajectoryPointSample& sample : samples) {
    CorridorSample bounds{};
    bounds.left_bound_m = sample.left_bound_m;
    bounds.right_bound_m = sample.right_bound_m;
    const double margin = edgeMarginM(bounds, sample.lateral_offset_m);
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
    const OccupancyGrid2D& prohibited_grid, const TrajectoryOptimizerConfig& config,
    double& best_cost, std::vector<double>& offsets, std::vector<Point2>& best_points,
    CandidateScore& best_score, double& best_length_m,
    std::vector<TrajectoryPointSample>& scratch_samples,
    SegmentProhibitedCountCache& segment_cache, TrajectoryOptimizerStats& stats,
    TrajectoryOptimizerCandidateDiagnostic* diagnostic = nullptr) {
  ++stats.candidate_evaluations;
  ++stats.scratch_reused_candidates;
  const double incumbent_score = best_cost;
  const auto evaluation_started_at = std::chrono::steady_clock::now();
  std::size_t cache_hits = 0U;
  std::size_t cache_misses = 0U;
  const PathEvaluation evaluation = evaluatePathCached(
      prohibited_grid, candidate_points, segment_cache, cache_hits, cache_misses);
  const double path_evaluation_duration_ms = elapsedMilliseconds(evaluation_started_at);
  stats.full_path_segment_cache_hits += cache_hits;
  stats.full_path_segment_cache_misses += cache_misses;
  stats.candidate_path_evaluation_duration_ms += path_evaluation_duration_ms;
  if (!evaluation.traversable()) {
    ++stats.collision_rejections;
  }
  const auto score_started_at = std::chrono::steady_clock::now();
  const CandidateScore candidate_score =
      scoreForCandidate(corridor_samples, candidate_points, candidate_offsets,
                        evaluation, config, scratch_samples, stats, nullptr);
  const double score_duration_ms = elapsedMilliseconds(score_started_at);
  stats.candidate_score_duration_ms += score_duration_ms;
  stats.full_candidate_score_duration_ms += score_duration_ms;
  const bool accepted = candidate_score.score + 1.0e-9 < best_cost;
  if (diagnostic != nullptr) {
    populateCandidateDiagnosticFromScore(
        *diagnostic, candidate_score, evaluation, incumbent_score, accepted, 0.0,
        path_evaluation_duration_ms, score_duration_ms, score_duration_ms);
  }
  if (accepted) {
    best_cost = candidate_score.score;
    offsets.assign(candidate_offsets.begin(), candidate_offsets.end());
    best_points.assign(candidate_points.begin(), candidate_points.end());
    best_score = candidate_score;
    best_length_m = evaluation.length_m;
    stats.best_candidate_score = candidate_score.score;
    return true;
  }
  return false;
}

[[nodiscard]] EvaluatedCandidate evaluateCandidateSnapshot(
    const std::span<const CorridorSample> corridor_samples,
    const std::span<const double> base_offsets,
    const std::span<const Point2> base_points, const CandidateScore& base_score,
    const double base_length_m, const std::size_t center_index, const double delta_m,
    const OccupancyGrid2D& prohibited_grid, const TrajectoryOptimizerConfig& config,
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
  const OffsetChangeDiagnostics offset_diagnostics =
      offsetChangeDiagnostics(base_offsets, buffer.offsets);
  result.offset_changed_samples = offset_diagnostics.changed_samples;
  result.offset_changed_span_samples = offset_diagnostics.changed_span_samples;
  result.local_speed_window_samples =
      estimatedLocalSpeedWindowSamples(offset_diagnostics, base_offsets.size());
  if (offset_diagnostics.changed_samples == 0U ||
      offsetsNearlyEqual(buffer.offsets, base_offsets)) {
    result.noop = true;
    return result;
  }

  result.local_evaluated = true;
  LocalCandidateScore local_score = evaluateLocalOffsetPath(
      corridor_samples, base_points, base_offsets, buffer.offsets, prohibited_grid,
      base_score, base_length_m, center_index, buffer);
  result.point_build_duration_ms = local_score.point_build_duration_ms;
  result.path_evaluation_duration_ms = local_score.path_evaluation_duration_ms;
  result.score_duration_ms = local_score.score_duration_ms;
  result.local_point_build_duration_ms = local_score.point_build_duration_ms;
  result.local_path_evaluation_duration_ms = local_score.path_evaluation_duration_ms;
  result.local_score_duration_ms =
      local_score.path_evaluation_duration_ms + local_score.score_duration_ms;
  result.local_segment_cache_hits = local_score.segment_cache_hits;
  result.local_segment_cache_misses = local_score.segment_cache_misses;
  result.local_full_score_reason = local_score.full_score_reason;
  result.shadow_boundary_clamped_window_samples =
      local_score.shadow_boundary_clamped_window_samples;
  if (local_score.valid && !local_score.requires_full_score) {
    result.path = local_score.path;
    result.offsets.assign(buffer.offsets.begin(), buffer.offsets.end());
    if (!result.path.traversable()) {
      return result;
    }
    TrajectoryOptimizerStats local_stats{};
    const auto points_started_at = std::chrono::steady_clock::now();
    pointsFromOffsets(corridor_samples, buffer.offsets, buffer.points);
    result.point_build_duration_ms += elapsedMilliseconds(points_started_at);
    const auto prefilter_started_at = std::chrono::steady_clock::now();
    result.shadow_lower_bound_score = conservativeShadowLowerBoundScore(
        std::span<const Point2>{buffer.points.data(), buffer.points.size()},
        std::span<const double>{buffer.offsets.data(), buffer.offsets.size()}, config);
    result.shadow_lower_bound_valid = std::isfinite(result.shadow_lower_bound_score);
    result.shadow_lower_bound_incumbent_score = incumbent_score;
    result.shadow_lower_bound_would_prune =
        std::isfinite(result.shadow_lower_bound_score) &&
        std::isfinite(incumbent_score) &&
        result.shadow_lower_bound_score + 1.0e-9 >= incumbent_score;
    result.score_duration_ms += elapsedMilliseconds(prefilter_started_at);
    const auto score_started_at = std::chrono::steady_clock::now();
    const auto geometry_started_at = std::chrono::steady_clock::now();
    const std::optional<CostBreakdown> incremental_geometry =
        incrementalGeometryBreakdownForChangedSpan(
            base_score, base_points, base_offsets, buffer.points, buffer.offsets,
            offset_diagnostics, config);
    local_stats.candidate_cost_breakdown_duration_ms +=
        elapsedMilliseconds(geometry_started_at);
    result.score =
        scoreForCandidate(corridor_samples, buffer.points, buffer.offsets, result.path,
                          config, buffer.samples, local_stats,
                          incremental_geometry ? &*incremental_geometry : nullptr);
    populateShadowSegmentScoreDiagnostics(
        result, base_score, base_points, base_offsets, buffer.points, buffer.offsets,
        offset_diagnostics, config, incumbent_score, incremental_geometry);
    const double full_score_duration_ms = elapsedMilliseconds(score_started_at);
    result.score_duration_ms += full_score_duration_ms;
    result.full_score_duration_ms += full_score_duration_ms;
    result.sample_build_duration_ms = local_stats.candidate_sample_build_duration_ms;
    result.cost_breakdown_duration_ms =
        local_stats.candidate_cost_breakdown_duration_ms;
    result.shape_diagnostics_duration_ms =
        local_stats.candidate_shape_diagnostics_duration_ms;
    result.full_score_used = true;
    return result;
  }

  result.requires_full_score = true;
  result.offsets.assign(buffer.offsets.begin(), buffer.offsets.end());

  TrajectoryOptimizerStats local_stats{};
  const auto points_started_at = std::chrono::steady_clock::now();
  pointsFromOffsets(corridor_samples, buffer.offsets, buffer.points);
  result.point_build_duration_ms += elapsedMilliseconds(points_started_at);
  const auto full_evaluation_started_at = std::chrono::steady_clock::now();
  result.path = evaluatePathCached(
      prohibited_grid, buffer.points, buffer.full_path_segment_cache,
      result.full_path_segment_cache_hits, result.full_path_segment_cache_misses);
  result.path_evaluation_duration_ms += elapsedMilliseconds(full_evaluation_started_at);
  const auto score_started_at = std::chrono::steady_clock::now();
  const auto geometry_started_at = std::chrono::steady_clock::now();
  const std::optional<CostBreakdown> incremental_geometry =
      incrementalGeometryBreakdownForChangedSpan(base_score, base_points, base_offsets,
                                                 buffer.points, buffer.offsets,
                                                 offset_diagnostics, config);
  local_stats.candidate_cost_breakdown_duration_ms +=
      elapsedMilliseconds(geometry_started_at);
  result.score =
      scoreForCandidate(corridor_samples, buffer.points, buffer.offsets, result.path,
                        config, buffer.samples, local_stats,
                        incremental_geometry ? &*incremental_geometry : nullptr);
  populateShadowSegmentScoreDiagnostics(
      result, base_score, base_points, base_offsets, buffer.points, buffer.offsets,
      offset_diagnostics, config, incumbent_score, incremental_geometry);
  const double full_score_duration_ms = elapsedMilliseconds(score_started_at);
  result.score_duration_ms += full_score_duration_ms;
  result.full_score_duration_ms += full_score_duration_ms;
  result.sample_build_duration_ms = local_stats.candidate_sample_build_duration_ms;
  result.cost_breakdown_duration_ms = local_stats.candidate_cost_breakdown_duration_ms;
  result.shape_diagnostics_duration_ms =
      local_stats.candidate_shape_diagnostics_duration_ms;
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
    const OccupancyGrid2D& prohibited_grid, const TrajectoryOptimizerConfig& config,
    const double requested_step_m, const std::span<const double> base_offsets,
    const std::span<const double> guide_offsets, const double guide_radius_m,
    std::vector<double>& output_offsets, TrajectoryOptimizerStats& stats) {
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
    cost.front()[candidate_index] = weight_offset_change * offset_delta * offset_delta;
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
            cost[row - 1U][previous_candidate_index] +
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

[[nodiscard]] std::span<const CandidateBatchResult> evaluateCandidateBatch(
    const std::span<const CandidateTask> tasks,
    const std::span<const CorridorSample> corridor_samples,
    const std::span<const double> base_offsets,
    const std::span<const Point2> base_points, const CandidateScore& base_score,
    const double base_length_m, const OccupancyGrid2D& prohibited_grid,
    const TrajectoryOptimizerConfig& config,
    const std::span<const std::uint8_t> mutable_indices, const double incumbent_score,
    CandidateBatchWorkspace& workspace, TrajectoryOptimizerCandidateWorkerPool* pool,
    TrajectoryOptimizerStats& stats, const std::size_t worker_count) {
  const auto batch_started_at = std::chrono::steady_clock::now();
  workspace.results.resize(tasks.size());
  if (tasks.empty()) {
    stats.candidate_batch_wall_duration_ms += elapsedMilliseconds(batch_started_at);
    return std::span<const CandidateBatchResult>{workspace.results};
  }

  const std::size_t resolved_workers = std::clamp<std::size_t>(
      worker_count, 1U, std::max<std::size_t>(1U, tasks.size()));
  prepareCandidateWorkspace(workspace, resolved_workers, base_offsets.size(),
                            corridor_samples.size(), stats);

  const auto evaluate_one = [&](const std::size_t task_index,
                                CandidateWorkBuffer& buffer) {
    const CandidateTask& task = tasks[task_index];
    workspace.results[task_index].order = task.order;
    workspace.results[task_index].center_index = task.center_index;
    workspace.results[task_index].delta_m = task.delta_m;
    workspace.results[task_index].candidate = evaluateCandidateSnapshot(
        corridor_samples, base_offsets, base_points, base_score, base_length_m,
        task.center_index, task.delta_m, prohibited_grid, config, mutable_indices,
        incumbent_score, buffer);
  };

  if (resolved_workers == 1U || pool == nullptr) {
    for (std::size_t task_index = 0U; task_index < tasks.size(); ++task_index) {
      evaluate_one(task_index, workspace.worker_buffers.front());
    }
    stats.candidate_batch_wall_duration_ms += elapsedMilliseconds(batch_started_at);
    return std::span<const CandidateBatchResult>{workspace.results};
  }

  ++stats.candidate_parallel_batches;
  pool->run(resolved_workers, tasks.size(),
            [&](const std::size_t task_index, const std::size_t worker_index) {
              evaluate_one(task_index, workspace.worker_buffers[worker_index]);
            });
  stats.candidate_batch_wall_duration_ms += elapsedMilliseconds(batch_started_at);
  return std::span<const CandidateBatchResult>{workspace.results};
}

void incrementLocalFullScoreReason(const LocalFullScoreReason reason,
                                   TrajectoryOptimizerStats& stats) {
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

void mergeCandidateStats(const EvaluatedCandidate& candidate,
                         TrajectoryOptimizerStats& stats) {
  if (candidate.scratch_reused) {
    ++stats.worker_scratch_reuses;
  }
  if (candidate.snapshot_allocation_avoided) {
    ++stats.candidate_snapshot_allocations_avoided;
  }
  stats.candidate_offset_changed_samples_total += candidate.offset_changed_samples;
  stats.candidate_offset_changed_samples_max = std::max(
      stats.candidate_offset_changed_samples_max, candidate.offset_changed_samples);
  stats.candidate_offset_changed_span_samples_total +=
      candidate.offset_changed_span_samples;
  stats.candidate_offset_changed_span_samples_max =
      std::max(stats.candidate_offset_changed_span_samples_max,
               candidate.offset_changed_span_samples);
  stats.candidate_local_speed_window_samples_total +=
      candidate.local_speed_window_samples;
  stats.candidate_local_speed_window_samples_max =
      std::max(stats.candidate_local_speed_window_samples_max,
               candidate.local_speed_window_samples);
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
    stats.candidate_segment_cache_hits += candidate.local_segment_cache_hits;
    stats.candidate_segment_cache_misses += candidate.local_segment_cache_misses;
    if (candidate.requires_full_score) {
      ++stats.local_candidate_full_score_required;
      incrementLocalFullScoreReason(candidate.local_full_score_reason, stats);
      if (candidate.local_full_score_reason == LocalFullScoreReason::kBoundaryWindow) {
        ++stats.shadow_boundary_clamped_local_candidates;
        stats.shadow_boundary_clamped_window_samples_total +=
            candidate.shadow_boundary_clamped_window_samples;
        stats.shadow_boundary_clamped_window_samples_max =
            std::max(stats.shadow_boundary_clamped_window_samples_max,
                     candidate.shadow_boundary_clamped_window_samples);
      }
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
  if (candidate.shadow_segment_score_valid) {
    ++stats.shadow_segment_score_evaluations;
    stats.shadow_segment_score_window_samples_total +=
        candidate.shadow_segment_score_window_samples;
    stats.shadow_segment_score_window_samples_max =
        std::max(stats.shadow_segment_score_window_samples_max,
                 candidate.shadow_segment_score_window_samples);
    const double score_delta =
        candidate.shadow_segment_score_estimated_score - candidate.score.score;
    if (std::isfinite(score_delta)) {
      stats.shadow_segment_score_abs_error_sum += std::abs(score_delta);
      stats.shadow_segment_score_max_overestimate =
          std::max(stats.shadow_segment_score_max_overestimate, score_delta);
      stats.shadow_segment_score_max_underestimate =
          std::max(stats.shadow_segment_score_max_underestimate, -score_delta);
    }
    if (candidate.shadow_segment_score_would_prune) {
      ++stats.shadow_segment_score_prunable;
      if (std::isfinite(candidate.score.score) &&
          std::isfinite(candidate.shadow_segment_score_incumbent_score) &&
          candidate.score.score + 1.0e-9 <
              candidate.shadow_segment_score_incumbent_score) {
        ++stats.shadow_segment_score_false_prunes;
        stats.shadow_segment_score_max_false_prune_improvement_score = std::max(
            stats.shadow_segment_score_max_false_prune_improvement_score,
            candidate.shadow_segment_score_incumbent_score - candidate.score.score);
      }
    }
  } else if (candidate.local_evaluated) {
    ++stats.shadow_segment_score_unavailable;
  }
  ++stats.candidate_evaluations;
  stats.candidate_point_build_duration_ms += candidate.point_build_duration_ms;
  stats.candidate_path_evaluation_duration_ms += candidate.path_evaluation_duration_ms;
  stats.candidate_score_duration_ms += candidate.score_duration_ms;
  stats.candidate_sample_build_duration_ms += candidate.sample_build_duration_ms;
  stats.candidate_cost_breakdown_duration_ms += candidate.cost_breakdown_duration_ms;
  stats.candidate_shape_diagnostics_duration_ms +=
      candidate.shape_diagnostics_duration_ms;
  if (!candidate.path.traversable()) {
    ++stats.collision_rejections;
  }
}

[[nodiscard]] TrajectoryOptimizerCandidateDiagnostic candidateDiagnosticFromBatchResult(
    const CandidateBatchResult& batch_result,
    const std::span<const CorridorSample> corridor_samples, const std::size_t iteration,
    const double step_m, const double incumbent_score, const bool selected) {
  const EvaluatedCandidate& candidate = batch_result.candidate;
  TrajectoryOptimizerCandidateDiagnostic diagnostic{};
  diagnostic.phase = "iteration";
  diagnostic.iteration = iteration;
  diagnostic.order = batch_result.order;
  diagnostic.center_index = batch_result.center_index;
  diagnostic.step_m = step_m;
  diagnostic.delta_m = batch_result.delta_m;
  if (batch_result.center_index < corridor_samples.size()) {
    diagnostic.center_s_m = corridor_samples[batch_result.center_index].s_m;
  }
  diagnostic.score = candidate.score.score;
  diagnostic.incumbent_score = incumbent_score;
  diagnostic.length_m = candidate.path.length_m;
  diagnostic.noop = candidate.noop;
  diagnostic.traversable = candidate.path.traversable();
  diagnostic.local_evaluated = candidate.local_evaluated;
  diagnostic.requires_full_score = candidate.requires_full_score;
  diagnostic.full_score_used = candidate.full_score_used;
  diagnostic.local_full_score_reason =
      localFullScoreReasonName(candidate.local_full_score_reason);
  diagnostic.prohibited_cells = candidate.path.prohibited_cells;
  diagnostic.outside_grid_segments = candidate.path.outside_grid_segments;
  diagnostic.changed_samples = candidate.offset_changed_samples;
  diagnostic.changed_span_samples = candidate.offset_changed_span_samples;
  diagnostic.point_build_duration_ms = candidate.point_build_duration_ms;
  diagnostic.path_evaluation_duration_ms = candidate.path_evaluation_duration_ms;
  diagnostic.score_duration_ms = candidate.score_duration_ms;
  diagnostic.full_score_duration_ms = candidate.full_score_duration_ms;
  copyCostBreakdownToCandidateDiagnostic(candidate.score.breakdown, diagnostic);

  if (selected) {
    diagnostic.decision = "selected";
  } else if (candidate.noop) {
    diagnostic.decision = "noop";
  } else if (candidate.path.outside_grid_segments > 0U) {
    diagnostic.decision = "outside_grid";
  } else if (candidate.path.prohibited_cells > 0U) {
    diagnostic.decision = "prohibited";
  } else if (!candidate.full_score_used) {
    diagnostic.decision = "not_scored";
  } else if (std::isfinite(candidate.score.score) && std::isfinite(incumbent_score) &&
             candidate.score.score + 1.0e-9 < incumbent_score) {
    diagnostic.decision = "valid_not_iteration_best";
  } else {
    diagnostic.decision = "not_better_than_incumbent";
  }
  return diagnostic;
}

} // namespace

TrajectoryOptimizerResult
optimizeTrajectory(const std::span<const CorridorSample> corridor_samples,
                   const OccupancyGrid2D& prohibited_grid,
                   const TrajectoryOptimizerConfig& config,
                   const VelocityFollowerConfig& speed_config) {
  TrajectoryOptimizerResult result{};
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
  result.stats.candidate_diagnostics.reserve(
      control_indices.size() * 2U * max_iterations + active_windows.size() + 8U);

  TrajectoryOptimizerScratch scratch{};
  scratch.candidate_offsets.reserve(sample_count);
  scratch.accepted_offsets.reserve(sample_count);
  scratch.iteration_best_offsets.reserve(sample_count);
  scratch.smoothed_offsets.reserve(sample_count);
  scratch.candidate_points.reserve(sample_count);
  scratch.accepted_points.reserve(sample_count);
  scratch.candidate_samples.reserve(sample_count);
  std::size_t candidate_worker_count = 1U;
  CandidateBatchWorkspace candidate_workspace{};

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
  std::size_t seed_order = 0U;
  for (const InitialOffsetSeed seed : kInitialSeeds) {
    offsetsFromSeed(optimizer_samples, seed, scratch.candidate_offsets);
    const auto points_started_at = std::chrono::steady_clock::now();
    pointsFromOffsets(optimizer_samples, scratch.candidate_offsets,
                      scratch.candidate_points);
    result.stats.candidate_point_build_duration_ms +=
        elapsedMilliseconds(points_started_at);
    TrajectoryOptimizerCandidateDiagnostic diagnostic{};
    diagnostic.phase = initialOffsetSeedName(seed);
    diagnostic.order = seed_order++;
    diagnostic.local_full_score_reason = "none";
    (void)updateBestCandidate(
        optimizer_samples, scratch.candidate_offsets, scratch.candidate_points,
        prohibited_grid, config, best_cost, offsets, best_points, best_score,
        best_length_m, scratch.candidate_samples, scratch.full_path_segment_cache,
        result.stats, &diagnostic);
    result.stats.candidate_diagnostics.push_back(std::move(diagnostic));
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
    const bool used_fallback_dp = !dp_ok;
    const auto points_started_at = std::chrono::steady_clock::now();
    pointsFromOffsets(optimizer_samples, scratch.candidate_offsets,
                      scratch.candidate_points);
    result.stats.candidate_point_build_duration_ms +=
        elapsedMilliseconds(points_started_at);
    TrajectoryOptimizerCandidateDiagnostic diagnostic{};
    diagnostic.phase = used_fallback_dp ? "dp_seed_fallback" : "dp_seed_coarse_to_fine";
    diagnostic.order = result.stats.candidate_diagnostics.size();
    diagnostic.center_index =
        window.begin_index + (window.end_index - window.begin_index) / 2U;
    if (diagnostic.center_index < optimizer_samples.size()) {
      diagnostic.center_s_m = optimizer_samples[diagnostic.center_index].s_m;
    }
    diagnostic.local_full_score_reason = "none";
    const bool accepted = updateBestCandidate(
        optimizer_samples, scratch.candidate_offsets, scratch.candidate_points,
        prohibited_grid, config, best_cost, offsets, best_points, best_score,
        best_length_m, scratch.candidate_samples, scratch.full_path_segment_cache,
        result.stats, &diagnostic);
    result.stats.candidate_diagnostics.push_back(std::move(diagnostic));
    if (accepted) {
      result.stats.initial_cost = best_cost;
    }
  }
  result.stats.window_eval_duration_ms += elapsedMilliseconds(window_eval_started_at);

  const std::size_t max_candidate_worker_count =
      desiredWorkerCount(config.parallel_workers, control_indices.size() * 2U);
  std::optional<TrajectoryOptimizerCandidateWorkerPool> candidate_worker_pool;
  if (max_candidate_worker_count > 1U) {
    candidate_worker_pool.emplace(max_candidate_worker_count, result.stats);
  }
  std::vector<double> shadow_segment_score_abs_errors;
  shadow_segment_score_abs_errors.reserve(control_indices.size() * max_iterations);

  for (std::size_t iteration = 0U; iteration < max_iterations && step >= min_step;
       ++iteration) {
    if (control_indices.empty()) {
      break;
    }
    const double incumbent_before_iteration = best_cost;
    const std::vector<CandidateTask> tasks =
        candidateTasksForStep(control_indices, step);
    candidate_worker_count = desiredWorkerCount(config.parallel_workers, tasks.size());
    result.stats.parallel_candidate_evaluation_used =
        result.stats.parallel_candidate_evaluation_used || candidate_worker_count > 1U;
    result.stats.parallel_workers_used =
        std::max(result.stats.parallel_workers_used, candidate_worker_count);
    ++result.stats.candidate_chunks;
    const std::span<const CandidateBatchResult> candidates = evaluateCandidateBatch(
        tasks, optimizer_samples, offsets, best_points, best_score, best_length_m,
        prohibited_grid, config, mutable_indices, best_cost, candidate_workspace,
        candidate_worker_pool.has_value() ? &candidate_worker_pool.value() : nullptr,
        result.stats, candidate_worker_count);

    bool changed = false;
    const EvaluatedCandidate* iteration_winner = nullptr;
    std::optional<std::size_t> iteration_winner_order;
    std::optional<std::size_t> shadow_segment_score_winner_order;
    double best_estimated_score = best_cost;
    double best_shadow_segment_score = best_cost;
    for (const CandidateBatchResult& batch_result : candidates) {
      const EvaluatedCandidate& candidate = batch_result.candidate;
      mergeCandidateStats(candidate, result.stats);
      if (candidate.noop || candidate.offsets.empty()) {
        continue;
      }
      if (candidate.shadow_segment_score_valid) {
        const double score_delta =
            candidate.shadow_segment_score_estimated_score - candidate.score.score;
        if (std::isfinite(score_delta)) {
          shadow_segment_score_abs_errors.push_back(std::abs(score_delta));
        }
      }
      if (candidate.shadow_segment_score_valid &&
          candidate.shadow_segment_score_estimated_score + 1.0e-9 <
              best_shadow_segment_score) {
        best_shadow_segment_score = candidate.shadow_segment_score_estimated_score;
        shadow_segment_score_winner_order = batch_result.order;
      }
      if (!candidate.full_score_used) {
        continue;
      }
      if (candidate.score.score + 1.0e-9 < best_cost) {
        if (candidate.score.score + 1.0e-9 < best_estimated_score) {
          best_estimated_score = candidate.score.score;
          iteration_winner = &candidate;
          iteration_winner_order = batch_result.order;
        }
      }
    }
    if (shadow_segment_score_winner_order.has_value() &&
        shadow_segment_score_winner_order != iteration_winner_order) {
      ++result.stats.shadow_segment_score_winner_mismatches;
    }
    for (const CandidateBatchResult& batch_result : candidates) {
      result.stats.candidate_diagnostics.push_back(candidateDiagnosticFromBatchResult(
          batch_result, optimizer_samples, iteration, step, incumbent_before_iteration,
          iteration_winner_order.has_value() &&
              batch_result.order == *iteration_winner_order));
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
      result.stats.best_candidate_score = iteration_winner->score.score;
      changed = true;
    }
    ++result.stats.iterations;
    if (!changed) {
      step *= cooling;
    }
  }
  result.stats.shadow_segment_score_abs_error_p95 =
      percentileValue(shadow_segment_score_abs_errors, 0.95);
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
    if (candidate_diagnostics.max_curvature_jump_1pm + 1.0e-9 >=
        pre_diagnostics.max_curvature_jump_1pm) {
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
      scratch.candidate_samples, result.stats, nullptr);
  result.stats.full_final_score_duration_ms =
      elapsedMilliseconds(final_score_started_at);

  result.samples.reserve(sample_count);
  for (std::size_t i = 0U; i < sample_count; ++i) {
    TrajectoryPointSample sample{};
    sample.point = final_points[i];
    sample.left_bound_m = optimizer_samples[i].left_bound_m;
    sample.right_bound_m = optimizer_samples[i].right_bound_m;
    sample.lateral_offset_m = final_offsets[i];
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
  updateCurvatureStats(result.samples, result.stats);
  updateEdgeMarginStats(result.samples, result.stats);
  result.valid = trajectorySamplesAreUsable(result.samples);
  return result;
}

} // namespace drone_city_nav
