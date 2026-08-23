#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/world_generation.hpp"

#include <gtest/gtest.h>

#include <chrono>

namespace drone_city_nav {
namespace {

TEST(WorldGenerationTest, StableWorldHeartbeatRefreshesSensorLiveness) {
  LatestObservationTracker tracker;
  ASSERT_TRUE(tracker.observe(7U, 12U, 1'000'000'000LL));
  EXPECT_DOUBLE_EQ(tracker.latest().ageMs(1'500'000'000LL), 500.0);

  ASSERT_TRUE(tracker.observe(7U, 13U, 2'000'000'000LL));
  EXPECT_DOUBLE_EQ(tracker.latest().ageMs(2'100'000'000LL), 100.0);
}

TEST(WorldGenerationTest, OutOfOrderHeartbeatCannotRewindLatestObservation) {
  LatestObservationTracker tracker;
  ASSERT_TRUE(tracker.observe(7U, 12U, 1'000'000'000LL));

  EXPECT_FALSE(tracker.observe(7U, 11U, 2'000'000'000LL));
  EXPECT_EQ(tracker.latest().sequence, 12U);
  EXPECT_EQ(tracker.latest().receive_stamp_ns, 1'000'000'000LL);
}

TEST(WorldGenerationTest, LocalWorldGenerationLinksRawPoseEsdfGpuAndTopology) {
  LocalWorldGenerationCounter counter;
  const RawMapVersion raw{
      .producer_instance_id = 7U, .base_snapshot_revision = 10U, .revision = 14U};

  const std::optional<LocalWorldGeneration> first =
      counter.issue(raw, 21U, 99U, 99U, 13U);
  const std::optional<LocalWorldGeneration> second =
      counter.issue(raw, 22U, 100U, 100U, 14U);

  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  const LocalWorldGeneration first_generation = first.value_or(LocalWorldGeneration{});
  const LocalWorldGeneration second_generation =
      second.value_or(LocalWorldGeneration{});
  EXPECT_TRUE(first_generation.coherent());
  EXPECT_TRUE(second_generation.coherent());
  EXPECT_EQ(first_generation.generation, 1U);
  EXPECT_EQ(second_generation.generation, 2U);
  EXPECT_EQ(second_generation.pose_revision, 22U);
}

TEST(WorldGenerationTest, MixedFutureTopologyRevisionIsRejected) {
  LocalWorldGenerationCounter counter;
  const RawMapVersion raw{
      .producer_instance_id = 7U, .base_snapshot_revision = 10U, .revision = 14U};

  EXPECT_FALSE(counter.issue(raw, 21U, 99U, 99U, 15U).has_value());
  EXPECT_EQ(counter.lastIssued(), 0U);
}

TEST(WorldGenerationTest, CpuWorldMustMatchTheActiveGpuEsdfRevision) {
  EXPECT_TRUE(mppi::mppiEsdfRevisionMatches(55U, 55U));
  EXPECT_FALSE(mppi::mppiEsdfRevisionMatches(55U, 56U));
}

TEST(WorldGenerationTest, DeferredFinalUpdateRunsWhenRateDeadlineArrives) {
  using Scheduler = LatestWinsDeferredScheduler<int>;
  Scheduler scheduler;
  const Scheduler::TimePoint start{};
  const Scheduler::TimePoint deadline = start + std::chrono::milliseconds{100};
  ASSERT_TRUE(scheduler.submit(1).ready_immediately);
  ASSERT_EQ(scheduler.takeReady(start), 1);

  scheduler.defer(1, deadline);

  EXPECT_FALSE(
      scheduler.takeReady(deadline - std::chrono::milliseconds{1}).has_value());
  EXPECT_EQ(scheduler.takeReady(deadline), 1);
  EXPECT_FALSE(scheduler.hasPending());
}

TEST(WorldGenerationTest, NewestDeferredUpdateSupersedesOlderValue) {
  using Scheduler = LatestWinsDeferredScheduler<int>;
  Scheduler scheduler;
  const Scheduler::TimePoint start{};
  const Scheduler::TimePoint deadline = start + std::chrono::milliseconds{100};
  scheduler.defer(1, deadline);

  EXPECT_TRUE(scheduler.submit(2).replaced_pending);
  EXPECT_EQ(scheduler.takeReady(deadline), 2);
}

TEST(WorldGenerationTest, PoseDrivenUrgentRequestBypassesRateDeadline) {
  using Scheduler = LatestWinsDeferredScheduler<int>;
  Scheduler scheduler;
  const Scheduler::TimePoint start{};
  scheduler.defer(1, start + std::chrono::seconds{1});

  const Scheduler::Submission submission = scheduler.submit(1, true);

  EXPECT_TRUE(submission.replaced_pending);
  EXPECT_TRUE(submission.ready_immediately);
  EXPECT_EQ(scheduler.takeReady(start), 1);
}

} // namespace
} // namespace drone_city_nav
