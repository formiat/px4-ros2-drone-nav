#include "drone_city_nav/bounded_worker_pool.hpp"

#include <algorithm>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace drone_city_nav {

struct BoundedWorkerPool::SharedState final {
  SharedState(const std::size_t worker_count,
              const std::size_t maximum_pending_tasks_value)
      : maximum_pending_tasks{std::max<std::size_t>(1U, maximum_pending_tasks_value)} {
    if (worker_count == 0U) {
      throw std::invalid_argument{"worker pool requires at least one worker"};
    }
    workers.reserve(worker_count);
    for (std::size_t index = 0U; index < worker_count; ++index) {
      workers.emplace_back([this]() { workerLoop(); });
    }
  }

  SharedState(const SharedState&) = delete;
  SharedState& operator=(const SharedState&) = delete;
  SharedState(SharedState&&) = delete;
  SharedState& operator=(SharedState&&) = delete;

  ~SharedState() {
    {
      const std::scoped_lock lock{mutex};
      stopping = true;
    }
    task_available.notify_all();
    space_available.notify_all();
    workers.clear();
  }

  [[nodiscard]] std::size_t registerClient() {
    const std::scoped_lock lock{mutex};
    if (stopping) {
      throw std::runtime_error{"register client on stopped worker pool"};
    }
    lanes.emplace_back();
    ++live_clients;
    return lanes.size() - 1U;
  }

  void releaseClient() noexcept {
    const std::scoped_lock lock{mutex};
    if (live_clients > 0U) {
      --live_clients;
    }
  }

  void enqueue(const std::size_t lane_index, std::function<void()> task) {
    {
      std::unique_lock lock{mutex};
      space_available.wait(
          lock, [this]() { return stopping || pending_tasks < maximum_pending_tasks; });
      if (stopping) {
        throw std::runtime_error{"submit on stopped worker pool"};
      }
      if (lane_index >= lanes.size()) {
        throw std::logic_error{"worker pool client lane is invalid"};
      }
      lanes[lane_index].emplace(std::move(task));
      ++pending_tasks;
      ++submitted_tasks;
      peak_pending_tasks = std::max(peak_pending_tasks, pending_tasks);
    }
    task_available.notify_one();
  }

  [[nodiscard]] BoundedWorkerPoolStatistics statistics() const {
    const std::scoped_lock lock{mutex};
    return BoundedWorkerPoolStatistics{
        .worker_count = workers.size(),
        .client_count = live_clients,
        .pending_tasks = pending_tasks,
        .active_tasks = active_tasks,
        .peak_pending_tasks = peak_pending_tasks,
        .peak_active_tasks = peak_active_tasks,
        .submitted_tasks = submitted_tasks,
        .completed_tasks = completed_tasks,
    };
  }

  void workerLoop() {
    while (true) {
      std::function<void()> task;
      {
        std::unique_lock lock{mutex};
        task_available.wait(lock, [this]() { return stopping || pending_tasks > 0U; });
        if (stopping && pending_tasks == 0U) {
          return;
        }

        const std::size_t lane_count = lanes.size();
        for (std::size_t offset = 0U; offset < lane_count; ++offset) {
          const std::size_t lane = (next_lane + offset) % lane_count;
          if (lanes[lane].empty()) {
            continue;
          }
          task = std::move(lanes[lane].front());
          lanes[lane].pop();
          next_lane = (lane + 1U) % lane_count;
          break;
        }
        if (!task) {
          throw std::logic_error{"worker pool pending-task count is inconsistent"};
        }
        --pending_tasks;
        ++active_tasks;
        peak_active_tasks = std::max(peak_active_tasks, active_tasks);
      }
      space_available.notify_one();

      const SharedState* const previous_state = BoundedWorkerPool::current_state_;
      BoundedWorkerPool::current_state_ = this;
      task();
      BoundedWorkerPool::current_state_ = previous_state;

      {
        const std::scoped_lock lock{mutex};
        --active_tasks;
        ++completed_tasks;
      }
    }
  }

  const std::size_t maximum_pending_tasks;
  mutable std::mutex mutex;
  std::condition_variable task_available;
  std::condition_variable space_available;
  std::vector<std::queue<std::function<void()>>> lanes;
  std::vector<std::jthread> workers;
  std::size_t next_lane{0U};
  std::size_t live_clients{0U};
  std::size_t pending_tasks{0U};
  std::size_t active_tasks{0U};
  std::size_t peak_pending_tasks{0U};
  std::size_t peak_active_tasks{0U};
  std::uint64_t submitted_tasks{0U};
  std::uint64_t completed_tasks{0U};
  bool stopping{false};
};

thread_local const BoundedWorkerPool::SharedState* BoundedWorkerPool::current_state_{
    nullptr};

BoundedWorkerPool::BoundedWorkerPool(const std::size_t worker_count,
                                     const std::size_t maximum_pending_tasks)
    : BoundedWorkerPool{
          std::make_shared<SharedState>(worker_count, maximum_pending_tasks), "",
          false} {
}

BoundedWorkerPool::BoundedWorkerPool(std::shared_ptr<SharedState> state,
                                     std::string scheduler_id,
                                     const bool shared_scheduler)
    : state_{std::move(state)},
      scheduler_id_{std::move(scheduler_id)},
      shared_scheduler_{shared_scheduler} {
  if (!state_) {
    throw std::invalid_argument{"worker pool state must be available"};
  }
  lane_index_ = state_->registerClient();
}

BoundedWorkerPool::~BoundedWorkerPool() {
  state_->releaseClient();
}

std::shared_ptr<BoundedWorkerPool>
BoundedWorkerPool::acquireShared(std::string scheduler_id,
                                 const std::size_t worker_count,
                                 const std::size_t maximum_pending_tasks) {
  if (scheduler_id.empty()) {
    throw std::invalid_argument{"shared worker scheduler id must not be empty"};
  }
  if (worker_count == 0U) {
    throw std::invalid_argument{"worker pool requires at least one worker"};
  }

  static std::mutex registry_mutex;
  static std::unordered_map<std::string, std::weak_ptr<SharedState>> registry;

  std::shared_ptr<SharedState> state;
  {
    const std::scoped_lock lock{registry_mutex};
    std::weak_ptr<SharedState>& entry = registry[scheduler_id];
    state = entry.lock();
    if (state) {
      const std::size_t normalized_maximum =
          std::max<std::size_t>(1U, maximum_pending_tasks);
      if (state->workers.size() != worker_count ||
          state->maximum_pending_tasks != normalized_maximum) {
        throw std::invalid_argument{"shared worker scheduler configuration mismatch"};
      }
    } else {
      state = std::make_shared<SharedState>(worker_count, maximum_pending_tasks);
      entry = state;
    }
  }

  return std::shared_ptr<BoundedWorkerPool>{
      new BoundedWorkerPool{std::move(state), std::move(scheduler_id), true}};
}

std::size_t BoundedWorkerPool::workerCount() const noexcept {
  return state_->workers.size();
}

bool BoundedWorkerPool::canParallelizeFromCurrentThread() const noexcept {
  return workerCount() > 1U && current_state_ != state_.get();
}

bool BoundedWorkerPool::isSharedScheduler() const noexcept {
  return shared_scheduler_;
}

const std::string& BoundedWorkerPool::schedulerId() const noexcept {
  return scheduler_id_;
}

bool BoundedWorkerPool::sharesSchedulerWith(
    const BoundedWorkerPool& other) const noexcept {
  return state_.get() == other.state_.get();
}

BoundedWorkerPoolStatistics BoundedWorkerPool::statistics() const {
  return state_->statistics();
}

void BoundedWorkerPool::enqueue(std::function<void()> task) {
  state_->enqueue(lane_index_, std::move(task));
}

} // namespace drone_city_nav
