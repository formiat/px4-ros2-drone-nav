#include "drone_city_nav/bounded_worker_pool.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <future>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace drone_city_nav {
namespace {

TEST(BoundedWorkerPoolTest, SharedClientsUseOneConfiguredBackend) {
  const auto first = BoundedWorkerPool::acquireShared("shared_backend_test", 3U, 16U);
  const auto second = BoundedWorkerPool::acquireShared("shared_backend_test", 3U, 16U);

  EXPECT_TRUE(first->isSharedScheduler());
  EXPECT_EQ(first->schedulerId(), "shared_backend_test");
  EXPECT_TRUE(first->sharesSchedulerWith(*second));
  EXPECT_EQ(first->workerCount(), 3U);
  EXPECT_EQ(first->statistics().client_count, 2U);
}

TEST(BoundedWorkerPoolTest, SharedBackendRejectsConfigurationMismatch) {
  const auto scheduler =
      BoundedWorkerPool::acquireShared("configuration_mismatch_test", 2U, 8U);

  EXPECT_THROW(BoundedWorkerPool::acquireShared("configuration_mismatch_test", 3U, 8U),
               std::invalid_argument);
  EXPECT_THROW(BoundedWorkerPool::acquireShared("configuration_mismatch_test", 2U, 9U),
               std::invalid_argument);
  EXPECT_EQ(scheduler->statistics().client_count, 1U);
}

TEST(BoundedWorkerPoolTest, SharedBackendDispatchesClientLanesRoundRobin) {
  const auto first = BoundedWorkerPool::acquireShared("round_robin_test", 1U, 16U);
  const auto second = BoundedWorkerPool::acquireShared("round_robin_test", 1U, 16U);

  std::promise<void> blocker_started;
  std::future<void> blocker_started_future = blocker_started.get_future();
  std::promise<void> release_blocker;
  std::shared_future<void> release_future = release_blocker.get_future().share();
  std::future<void> blocker = first->submit([&]() {
    blocker_started.set_value();
    release_future.wait();
  });
  blocker_started_future.wait();

  std::mutex order_mutex;
  std::vector<int> order;
  const auto record = [&](const int value) {
    const std::scoped_lock lock{order_mutex};
    order.push_back(value);
  };
  std::future<void> first_one = first->submit([&]() { record(1); });
  std::future<void> first_two = first->submit([&]() { record(2); });
  std::future<void> second_one = second->submit([&]() { record(3); });
  std::future<void> second_two = second->submit([&]() { record(4); });

  release_blocker.set_value();
  blocker.get();
  first_one.get();
  first_two.get();
  second_one.get();
  second_two.get();

  EXPECT_EQ(order, (std::vector<int>{3, 1, 4, 2}));
  const BoundedWorkerPoolStatistics statistics = first->statistics();
  EXPECT_EQ(statistics.submitted_tasks, 5U);
  EXPECT_EQ(statistics.completed_tasks, 5U);
  EXPECT_EQ(statistics.peak_active_tasks, 1U);
  EXPECT_GE(statistics.peak_pending_tasks, 4U);
}

TEST(BoundedWorkerPoolTest, NestedParallelForRunsWithoutReentrantSubmission) {
  BoundedWorkerPool pool{2U};
  std::atomic<std::size_t> calls{0U};
  std::atomic<bool> nested_parallelism_observed{false};

  pool.parallelFor(2U, [&](const std::size_t) {
    if (pool.canParallelizeFromCurrentThread()) {
      nested_parallelism_observed.store(true, std::memory_order_relaxed);
    }
    pool.parallelFor(
        3U, [&](const std::size_t) { calls.fetch_add(1U, std::memory_order_relaxed); });
  });

  EXPECT_FALSE(nested_parallelism_observed.load(std::memory_order_relaxed));
  EXPECT_EQ(calls.load(std::memory_order_relaxed), 6U);
}

} // namespace
} // namespace drone_city_nav
