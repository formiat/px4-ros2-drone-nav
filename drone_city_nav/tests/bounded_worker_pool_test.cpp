#include "drone_city_nav/bounded_worker_pool.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <future>
#include <mutex>
#include <vector>

namespace drone_city_nav {
namespace {

TEST(BoundedWorkerPoolTest, RouteAndWorldLanesPrecedeQueuedBackgroundWork) {
  BoundedWorkerPool pool{1U, 32U};
  std::promise<void> blocker_started;
  std::promise<void> release_blocker;
  const std::shared_future<void> release = release_blocker.get_future().share();
  std::mutex order_mutex;
  std::vector<int> order;
  const auto record = [&](const int value) {
    const std::scoped_lock lock{order_mutex};
    order.push_back(value);
  };

  std::future<void> blocker = pool.submit(WorkerTaskLane::kBackground, [&]() {
    blocker_started.set_value();
    release.wait();
  });
  blocker_started.get_future().wait();
  std::future<void> background =
      pool.submit(WorkerTaskLane::kBackground, [&]() { record(4); });
  std::future<void> world =
      pool.submit(WorkerTaskLane::kWorldUpdate, [&]() { record(3); });
  std::future<void> first_route =
      pool.submit(WorkerTaskLane::kRouteCritical, [&]() { record(1); });
  std::future<void> second_route =
      pool.submit(WorkerTaskLane::kRouteCritical, [&]() { record(2); });

  release_blocker.set_value();
  blocker.get();
  first_route.get();
  second_route.get();
  world.get();
  background.get();

  EXPECT_EQ(order, (std::vector<int>{1, 2, 3, 4}));
  const BoundedWorkerPoolSnapshot snapshot = pool.snapshot();
  EXPECT_EQ(snapshot.pendingTasks(), 0U);
  EXPECT_EQ(snapshot.lanes[0U].started, 2U);
  EXPECT_EQ(snapshot.lanes[1U].started, 1U);
  EXPECT_EQ(snapshot.lanes[2U].started, 2U);
}

TEST(BoundedWorkerPoolTest, BackgroundLaneReceivesBoundedFairService) {
  BoundedWorkerPool pool{1U, 64U};
  std::promise<void> blocker_started;
  std::promise<void> release_blocker;
  const std::shared_future<void> release = release_blocker.get_future().share();
  std::mutex order_mutex;
  std::vector<int> order;
  const auto record = [&](const int value) {
    const std::scoped_lock lock{order_mutex};
    order.push_back(value);
  };

  std::future<void> blocker = pool.submit(WorkerTaskLane::kBackground, [&]() {
    blocker_started.set_value();
    release.wait();
  });
  blocker_started.get_future().wait();
  std::future<void> background =
      pool.submit(WorkerTaskLane::kBackground, [&]() { record(100); });
  std::vector<std::future<void>> route_tasks;
  for (int index = 0; index < 16; ++index) {
    route_tasks.push_back(
        pool.submit(WorkerTaskLane::kRouteCritical, [&, index]() { record(index); }));
  }

  release_blocker.set_value();
  blocker.get();
  background.get();
  for (std::future<void>& task : route_tasks) {
    task.get();
  }

  const auto background_position =
      std::find(order.begin(), order.end(), 100) - order.begin();
  ASSERT_NE(background_position, static_cast<std::ptrdiff_t>(order.size()));
  EXPECT_LE(background_position, 8);
}

TEST(BoundedWorkerPoolTest, HigherLanesCanUseReservedPendingCapacity) {
  BoundedWorkerPool pool{1U, 4U};
  std::promise<void> blocker_started;
  std::promise<void> release_blocker;
  const std::shared_future<void> release = release_blocker.get_future().share();
  std::future<void> blocker = pool.submit(WorkerTaskLane::kBackground, [&]() {
    blocker_started.set_value();
    release.wait();
  });
  blocker_started.get_future().wait();

  std::future<void> first_background =
      pool.submit(WorkerTaskLane::kBackground, []() {});
  std::future<void> second_background =
      pool.submit(WorkerTaskLane::kBackground, []() {});
  std::future<void> world = pool.submit(WorkerTaskLane::kWorldUpdate, []() {});
  std::future<void> route = pool.submit(WorkerTaskLane::kRouteCritical, []() {});
  const BoundedWorkerPoolSnapshot saturated = pool.snapshot();

  EXPECT_EQ(saturated.pendingTasks(), saturated.maximum_pending_tasks);
  EXPECT_EQ(saturated.route_reserved_tasks, 1U);
  EXPECT_EQ(saturated.world_reserved_tasks, 1U);

  release_blocker.set_value();
  blocker.get();
  route.get();
  world.get();
  first_background.get();
  second_background.get();
}

} // namespace
} // namespace drone_city_nav
