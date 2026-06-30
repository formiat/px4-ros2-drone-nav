#include "drone_city_nav/trajectory_refinement_scheduler.hpp"

#include <gtest/gtest.h>

#include <optional>

namespace drone_city_nav {
namespace {

void expectJobEquals(const std::optional<TrajectoryRefinementJob>& actual,
                     const TrajectoryRefinementJob& expected) {
  if (!actual.has_value()) {
    ADD_FAILURE() << "expected trajectory refinement job";
    return;
  }
  EXPECT_EQ(actual.value(), expected);
}

} // namespace

TEST(TrajectoryRefinementScheduler, ZeroWorkersDisablesAsyncRefinement) {
  TrajectoryRefinementScheduler scheduler;
  scheduler.configure(0U);

  const TrajectoryRefinementScheduleDecision decision =
      scheduler.submit(TrajectoryRefinementJob{1U, 10U});

  EXPECT_EQ(scheduler.workerCount(), 0U);
  EXPECT_FALSE(scheduler.enabled());
  EXPECT_EQ(decision.action, TrajectoryRefinementScheduleAction::kDisabled);
  EXPECT_FALSE(scheduler.activeJob().has_value());
  EXPECT_FALSE(scheduler.queuedJob().has_value());
}

TEST(TrajectoryRefinementScheduler,
     SingleWorkerQueuesLatestRequestAndStartsItAfterCompletion) {
  TrajectoryRefinementScheduler scheduler;
  scheduler.configure(1U);

  const TrajectoryRefinementScheduleDecision first =
      scheduler.submit(TrajectoryRefinementJob{1U, 10U});
  const TrajectoryRefinementScheduleDecision second =
      scheduler.submit(TrajectoryRefinementJob{2U, 20U});
  const TrajectoryRefinementScheduleDecision third =
      scheduler.submit(TrajectoryRefinementJob{3U, 30U});

  EXPECT_EQ(first.action, TrajectoryRefinementScheduleAction::kStartNow);
  EXPECT_EQ(second.action, TrajectoryRefinementScheduleAction::kQueuedLatest);
  EXPECT_EQ(third.action, TrajectoryRefinementScheduleAction::kReplacedQueuedLatest);
  expectJobEquals(scheduler.activeJob(), TrajectoryRefinementJob{1U, 10U});
  expectJobEquals(scheduler.queuedJob(), TrajectoryRefinementJob{3U, 30U});

  const std::optional<TrajectoryRefinementJob> next =
      scheduler.completeActive(TrajectoryRefinementJob{1U, 10U});

  expectJobEquals(next, TrajectoryRefinementJob{3U, 30U});
  expectJobEquals(scheduler.activeJob(), TrajectoryRefinementJob{3U, 30U});
  EXPECT_FALSE(scheduler.queuedJob().has_value());
}

TEST(TrajectoryRefinementScheduler, HigherWorkerCountsAreClampedToSingleWorker) {
  TrajectoryRefinementScheduler scheduler;
  scheduler.configure(8U);

  EXPECT_EQ(scheduler.workerCount(), 1U);
  EXPECT_TRUE(scheduler.enabled());

  const TrajectoryRefinementScheduleDecision first =
      scheduler.submit(TrajectoryRefinementJob{1U, 10U});
  const TrajectoryRefinementScheduleDecision second =
      scheduler.submit(TrajectoryRefinementJob{2U, 20U});

  EXPECT_EQ(first.action, TrajectoryRefinementScheduleAction::kStartNow);
  EXPECT_EQ(second.action, TrajectoryRefinementScheduleAction::kQueuedLatest);
}

TEST(TrajectoryRefinementScheduler,
     CompletingUnexpectedJobDoesNotAdvanceQueuedLatestRequest) {
  TrajectoryRefinementScheduler scheduler;
  scheduler.configure(1U);

  EXPECT_EQ(scheduler.submit(TrajectoryRefinementJob{1U, 10U}).action,
            TrajectoryRefinementScheduleAction::kStartNow);
  EXPECT_EQ(scheduler.submit(TrajectoryRefinementJob{2U, 20U}).action,
            TrajectoryRefinementScheduleAction::kQueuedLatest);

  const std::optional<TrajectoryRefinementJob> next =
      scheduler.completeActive(TrajectoryRefinementJob{99U, 99U});

  EXPECT_FALSE(next.has_value());
  expectJobEquals(scheduler.activeJob(), TrajectoryRefinementJob{1U, 10U});
  expectJobEquals(scheduler.queuedJob(), TrajectoryRefinementJob{2U, 20U});
}

} // namespace drone_city_nav
