#include "trajectory_optimizer_internal.hpp"

namespace drone_city_nav::trajectory_optimizer_detail {

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

} // namespace drone_city_nav::trajectory_optimizer_detail
