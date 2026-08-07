#include "drone_city_nav/bounded_worker_pool.hpp"

#include <algorithm>

namespace drone_city_nav {

thread_local const BoundedWorkerPool* BoundedWorkerPool::current_pool_{nullptr};

BoundedWorkerPool::BoundedWorkerPool(const std::size_t worker_count,
                                     const std::size_t maximum_pending_tasks)
    : maximum_pending_tasks_{std::max<std::size_t>(1U, maximum_pending_tasks)} {
  if (worker_count == 0U) {
    throw std::invalid_argument{"worker pool requires at least one worker"};
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

void BoundedWorkerPool::workerLoop() {
  while (true) {
    std::function<void()> task;
    {
      std::unique_lock lock{mutex_};
      task_available_.wait(lock, [this]() { return stopping_ || !tasks_.empty(); });
      if (stopping_ && tasks_.empty()) {
        return;
      }
      task = std::move(tasks_.front());
      tasks_.pop();
    }
    space_available_.notify_one();
    const BoundedWorkerPool* const previous_pool = current_pool_;
    current_pool_ = this;
    task();
    current_pool_ = previous_pool;
  }
}

} // namespace drone_city_nav
