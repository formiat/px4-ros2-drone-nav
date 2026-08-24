#include "drone_city_nav/bounded_worker_pool.hpp"

#include <algorithm>
#include <numeric>

namespace drone_city_nav {
namespace {

constexpr std::size_t kRouteLaneIndex{0U};
constexpr std::size_t kWorldLaneIndex{1U};
constexpr std::size_t kBackgroundLaneIndex{2U};
constexpr std::size_t kMaximumConsecutiveRouteTasks{8U};
constexpr std::size_t kMaximumConsecutiveNonBackgroundTasks{12U};

} // namespace

std::size_t BoundedWorkerPoolSnapshot::pendingTasks() const noexcept {
  return std::accumulate(
      lanes.begin(), lanes.end(), std::size_t{0U},
      [](const std::size_t total, const WorkerTaskLaneSnapshot& lane) {
        return total + lane.pending;
      });
}

thread_local const BoundedWorkerPool* BoundedWorkerPool::current_pool_{nullptr};

BoundedWorkerPool::BoundedWorkerPool(const std::size_t worker_count,
                                     const std::size_t maximum_pending_tasks)
    : maximum_pending_tasks_{std::max<std::size_t>(1U, maximum_pending_tasks)} {
  if (worker_count == 0U) {
    throw std::invalid_argument{"worker pool requires at least one worker"};
  }
  if (maximum_pending_tasks_ >= 3U) {
    route_reserved_tasks_ = std::max<std::size_t>(1U, maximum_pending_tasks_ / 4U);
    world_reserved_tasks_ = std::max<std::size_t>(1U, maximum_pending_tasks_ / 4U);
    if (route_reserved_tasks_ + world_reserved_tasks_ >= maximum_pending_tasks_) {
      world_reserved_tasks_ = maximum_pending_tasks_ - route_reserved_tasks_ - 1U;
    }
  } else if (maximum_pending_tasks_ == 2U) {
    route_reserved_tasks_ = 1U;
  }
  workers_.reserve(worker_count);
  for (std::size_t index = 0U; index < worker_count; ++index) {
    workers_.emplace_back([this]() { workerLoop(); });
  }
}

BoundedWorkerPool::~BoundedWorkerPool() {
  {
    const std::scoped_lock lock{mutex_};
    stopping_ = true;
  }
  task_available_.notify_all();
  space_available_.notify_all();
}

std::size_t BoundedWorkerPool::workerCount() const noexcept {
  return workers_.size();
}

bool BoundedWorkerPool::canParallelizeFromCurrentThread() const noexcept {
  return workers_.size() > 1U && current_pool_ != this;
}

BoundedWorkerPoolSnapshot BoundedWorkerPool::snapshot() const noexcept {
  const std::scoped_lock lock{mutex_};
  BoundedWorkerPoolSnapshot result{
      .lanes = lane_snapshots_,
      .maximum_pending_tasks = maximum_pending_tasks_,
      .route_reserved_tasks = route_reserved_tasks_,
      .world_reserved_tasks = world_reserved_tasks_,
  };
  for (std::size_t index = 0U; index < tasks_.size(); ++index) {
    result.lanes[index].pending = tasks_[index].size();
  }
  return result;
}

std::size_t BoundedWorkerPool::laneIndex(const WorkerTaskLane lane) noexcept {
  switch (lane) {
    case WorkerTaskLane::kRouteCritical:
      return kRouteLaneIndex;
    case WorkerTaskLane::kWorldUpdate:
      return kWorldLaneIndex;
    case WorkerTaskLane::kBackground:
      return kBackgroundLaneIndex;
  }
  return kBackgroundLaneIndex;
}

std::size_t BoundedWorkerPool::pendingTasksLocked() const noexcept {
  return std::accumulate(
      tasks_.begin(), tasks_.end(), std::size_t{0U},
      [](const std::size_t total, const auto& lane) { return total + lane.size(); });
}

bool BoundedWorkerPool::canAcceptLocked(const WorkerTaskLane lane) const noexcept {
  std::size_t capacity = maximum_pending_tasks_;
  if (lane == WorkerTaskLane::kWorldUpdate) {
    capacity -= route_reserved_tasks_;
  } else if (lane == WorkerTaskLane::kBackground) {
    capacity -= route_reserved_tasks_ + world_reserved_tasks_;
  }
  return pendingTasksLocked() < capacity;
}

std::size_t BoundedWorkerPool::selectLaneLocked() noexcept {
  const bool route_pending = !tasks_[kRouteLaneIndex].empty();
  const bool world_pending = !tasks_[kWorldLaneIndex].empty();
  const bool background_pending = !tasks_[kBackgroundLaneIndex].empty();

  if (background_pending &&
      consecutive_non_background_tasks_ >= kMaximumConsecutiveNonBackgroundTasks) {
    consecutive_route_tasks_ = 0U;
    consecutive_non_background_tasks_ = 0U;
    return kBackgroundLaneIndex;
  }
  if (route_pending && consecutive_route_tasks_ < kMaximumConsecutiveRouteTasks) {
    ++consecutive_route_tasks_;
    ++consecutive_non_background_tasks_;
    return kRouteLaneIndex;
  }
  if (world_pending) {
    consecutive_route_tasks_ = 0U;
    ++consecutive_non_background_tasks_;
    return kWorldLaneIndex;
  }
  if (background_pending) {
    consecutive_route_tasks_ = 0U;
    consecutive_non_background_tasks_ = 0U;
    return kBackgroundLaneIndex;
  }
  if (route_pending) {
    ++consecutive_route_tasks_;
    ++consecutive_non_background_tasks_;
    return kRouteLaneIndex;
  }
  consecutive_route_tasks_ = 0U;
  consecutive_non_background_tasks_ = 0U;
  return kBackgroundLaneIndex;
}

void BoundedWorkerPool::workerLoop() {
  while (true) {
    std::function<void()> task;
    {
      std::unique_lock lock{mutex_};
      task_available_.wait(
          lock, [this]() { return stopping_ || pendingTasksLocked() != 0U; });
      if (stopping_ && pendingTasksLocked() == 0U) {
        return;
      }
      const std::size_t lane_index = selectLaneLocked();
      task = std::move(tasks_[lane_index].front());
      tasks_[lane_index].pop_front();
      ++lane_snapshots_[lane_index].started;
    }
    space_available_.notify_all();
    const BoundedWorkerPool* const previous_pool = current_pool_;
    current_pool_ = this;
    task();
    current_pool_ = previous_pool;
  }
}

} // namespace drone_city_nav
