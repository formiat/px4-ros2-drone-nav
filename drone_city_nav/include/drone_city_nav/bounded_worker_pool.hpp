#pragma once

#include <algorithm>
#include <array>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <future>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace drone_city_nav {

enum class WorkerTaskLane : std::uint8_t {
  kRouteCritical,
  kWorldUpdate,
  kBackground,
};

struct WorkerTaskLaneSnapshot {
  std::size_t pending{0U};
  std::uint64_t submitted{0U};
  std::uint64_t started{0U};
  std::uint64_t completed{0U};
  std::uint64_t capacity_waits{0U};
};

struct BoundedWorkerPoolSnapshot {
  std::array<WorkerTaskLaneSnapshot, 3U> lanes{};
  std::size_t maximum_pending_tasks{0U};
  std::size_t route_reserved_tasks{0U};
  std::size_t world_reserved_tasks{0U};

  [[nodiscard]] std::size_t pendingTasks() const noexcept;
};

class BoundedWorkerPool final {
public:
  explicit BoundedWorkerPool(std::size_t worker_count,
                             std::size_t maximum_pending_tasks = 256U);
  ~BoundedWorkerPool();

  BoundedWorkerPool(const BoundedWorkerPool&) = delete;
  BoundedWorkerPool& operator=(const BoundedWorkerPool&) = delete;
  BoundedWorkerPool(BoundedWorkerPool&&) = delete;
  BoundedWorkerPool& operator=(BoundedWorkerPool&&) = delete;

  [[nodiscard]] std::size_t workerCount() const noexcept;
  [[nodiscard]] bool canParallelizeFromCurrentThread() const noexcept;
  [[nodiscard]] BoundedWorkerPoolSnapshot snapshot() const noexcept;

  template<typename Function>
  [[nodiscard]] auto
  submit(Function&& function) -> std::future<std::invoke_result_t<Function>> {
    return submit(WorkerTaskLane::kRouteCritical, std::forward<Function>(function));
  }

  template<typename Function>
  [[nodiscard]] auto submit(const WorkerTaskLane lane, Function&& function)
      -> std::future<std::invoke_result_t<Function>> {
    using Result = std::invoke_result_t<Function>;
    auto task = std::make_shared<std::packaged_task<Result()>>(
        std::forward<Function>(function));
    std::future<Result> future = task->get_future();
    {
      std::unique_lock lock{mutex_};
      const std::size_t lane_index = laneIndex(lane);
      if (!canAcceptLocked(lane)) {
        ++lane_snapshots_[lane_index].capacity_waits;
      }
      space_available_.wait(
          lock, [this, lane]() { return stopping_ || canAcceptLocked(lane); });
      if (stopping_) {
        throw std::runtime_error{"submit on stopped worker pool"};
      }
      ++lane_snapshots_[lane_index].submitted;
      tasks_[lane_index].emplace_back([this, lane_index, task]() {
        (*task)();
        const std::scoped_lock completion_lock{mutex_};
        ++lane_snapshots_[lane_index].completed;
      });
    }
    task_available_.notify_one();
    return future;
  }

  template<typename Function>
  void parallelFor(const std::size_t count, Function&& function) {
    parallelFor(count, WorkerTaskLane::kRouteCritical,
                std::forward<Function>(function));
  }

  template<typename Function>
  void parallelFor(const std::size_t count, const WorkerTaskLane lane,
                   Function&& function) {
    if (count == 0U) {
      return;
    }
    if (current_pool_ == this) {
      for (std::size_t index = 0U; index < count; ++index) {
        function(index);
      }
      return;
    }
    const std::size_t task_count = std::min(count, workerCount());
    const std::size_t chunk_size = 1U + (count - 1U) / task_count;
    std::vector<std::future<void>> futures;
    futures.reserve(task_count);
    for (std::size_t task_index = 0U; task_index < task_count; ++task_index) {
      const std::size_t begin = task_index * chunk_size;
      const std::size_t end = std::min(count, begin + chunk_size);
      if (begin >= end) {
        break;
      }
      futures.push_back(submit(lane, [begin, end, &function]() {
        for (std::size_t index = begin; index < end; ++index) {
          function(index);
        }
      }));
    }
    for (std::future<void>& future : futures) {
      future.get();
    }
  }

private:
  [[nodiscard]] static std::size_t laneIndex(WorkerTaskLane lane) noexcept;
  [[nodiscard]] std::size_t pendingTasksLocked() const noexcept;
  [[nodiscard]] bool canAcceptLocked(WorkerTaskLane lane) const noexcept;
  [[nodiscard]] std::size_t selectLaneLocked() noexcept;
  void workerLoop();

  static thread_local const BoundedWorkerPool* current_pool_;

  std::size_t maximum_pending_tasks_{0U};
  std::size_t route_reserved_tasks_{0U};
  std::size_t world_reserved_tasks_{0U};
  mutable std::mutex mutex_;
  std::condition_variable task_available_;
  std::condition_variable space_available_;
  std::array<std::deque<std::function<void()>>, 3U> tasks_;
  std::array<WorkerTaskLaneSnapshot, 3U> lane_snapshots_{};
  std::size_t consecutive_route_tasks_{0U};
  std::size_t consecutive_non_background_tasks_{0U};
  bool stopping_{false};
  std::vector<std::jthread> workers_;
};

} // namespace drone_city_nav
