#pragma once

#include <algorithm>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace drone_city_nav {

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

  template<typename Function>
  [[nodiscard]] auto
  submit(Function&& function) -> std::future<std::invoke_result_t<Function>> {
    using Result = std::invoke_result_t<Function>;
    auto task = std::make_shared<std::packaged_task<Result()>>(
        std::forward<Function>(function));
    std::future<Result> future = task->get_future();
    {
      std::unique_lock lock{mutex_};
      space_available_.wait(lock, [this]() {
        return stopping_ || tasks_.size() < maximum_pending_tasks_;
      });
      if (stopping_) {
        throw std::runtime_error{"submit on stopped worker pool"};
      }
      tasks_.emplace([task]() { (*task)(); });
    }
    task_available_.notify_one();
    return future;
  }

  template<typename Function>
  void parallelFor(const std::size_t count, Function&& function) {
    if (count == 0U) {
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
  void workerLoop();

  std::size_t maximum_pending_tasks_{0U};
  mutable std::mutex mutex_;
  std::condition_variable task_available_;
  std::condition_variable space_available_;
  std::queue<std::function<void()>> tasks_;
  std::vector<std::jthread> workers_;
  bool stopping_{false};
};

} // namespace drone_city_nav
