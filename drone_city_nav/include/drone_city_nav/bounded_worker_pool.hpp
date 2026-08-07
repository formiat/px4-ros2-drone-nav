#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace drone_city_nav {

struct BoundedWorkerPoolStatistics {
  std::size_t worker_count{0U};
  std::size_t client_count{0U};
  std::size_t pending_tasks{0U};
  std::size_t active_tasks{0U};
  std::size_t peak_pending_tasks{0U};
  std::size_t peak_active_tasks{0U};
  std::uint64_t submitted_tasks{0U};
  std::uint64_t completed_tasks{0U};
};

class BoundedWorkerPool final {
public:
  explicit BoundedWorkerPool(std::size_t worker_count,
                             std::size_t maximum_pending_tasks = 256U);
  ~BoundedWorkerPool();

  [[nodiscard]] static std::shared_ptr<BoundedWorkerPool>
  acquireShared(std::string scheduler_id, std::size_t worker_count,
                std::size_t maximum_pending_tasks = 256U);

  BoundedWorkerPool(const BoundedWorkerPool&) = delete;
  BoundedWorkerPool& operator=(const BoundedWorkerPool&) = delete;
  BoundedWorkerPool(BoundedWorkerPool&&) = delete;
  BoundedWorkerPool& operator=(BoundedWorkerPool&&) = delete;

  [[nodiscard]] std::size_t workerCount() const noexcept;
  [[nodiscard]] bool canParallelizeFromCurrentThread() const noexcept;
  [[nodiscard]] bool isSharedScheduler() const noexcept;
  [[nodiscard]] const std::string& schedulerId() const noexcept;
  [[nodiscard]] bool sharesSchedulerWith(const BoundedWorkerPool& other) const noexcept;
  [[nodiscard]] BoundedWorkerPoolStatistics statistics() const;

  template<typename Function>
  [[nodiscard]] auto
  submit(Function&& function) -> std::future<std::invoke_result_t<Function>> {
    using Result = std::invoke_result_t<Function>;
    auto task = std::make_shared<std::packaged_task<Result()>>(
        std::forward<Function>(function));
    std::future<Result> future = task->get_future();
    enqueue([task]() { (*task)(); });
    return future;
  }

  template<typename Function>
  void parallelFor(const std::size_t count, Function&& function) {
    if (count == 0U) {
      return;
    }
    if (current_state_ == state_.get()) {
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
      futures.push_back(submit([begin, end, &function]() {
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
  struct SharedState;

  BoundedWorkerPool(std::shared_ptr<SharedState> state, std::string scheduler_id,
                    bool shared_scheduler);
  void enqueue(std::function<void()> task);

  static thread_local const SharedState* current_state_;

  std::shared_ptr<SharedState> state_;
  std::size_t lane_index_{0U};
  std::string scheduler_id_;
  bool shared_scheduler_{false};
};

} // namespace drone_city_nav
