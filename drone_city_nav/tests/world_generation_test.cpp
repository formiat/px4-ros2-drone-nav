#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/world_generation.hpp"

#include <gtest/gtest.h>

#include <chrono>

namespace drone_city_nav {
namespace {

constexpr ProducerEpochAdmissionConfig kObservationAdmissionConfig{
    .maximum_observation_age_ns = 5'000'000'000LL,
    .maximum_confirmation_interval_ns = 1'000'000'000LL,
};

[[nodiscard]] ProducerEpochObservation
observation(const std::uint64_t producer, const std::uint64_t sequence,
            const std::int64_t source_stamp_ns, const std::int64_t receive_stamp_ns,
            const std::uint64_t fingerprint = 99U) noexcept {
  return ProducerEpochObservation{
      .producer_instance_id = producer,
      .sequence = sequence,
      .source_stamp_ns = source_stamp_ns,
      .receive_stamp_ns = receive_stamp_ns,
      .content_fingerprint = fingerprint,
  };
}

TEST(WorldGenerationTest, StableWorldHeartbeatRefreshesSensorLiveness) {
  LatestObservationTracker tracker;
  ASSERT_TRUE(tracker
                  .observe(kObservationAdmissionConfig,
                           observation(7U, 12U, 990'000'000LL, 1'000'000'000LL),
                           1'000'000'000LL)
                  .install_observation);
  EXPECT_DOUBLE_EQ(tracker.latest().ageMs(1'500'000'000LL), 510.0);

  ASSERT_TRUE(tracker
                  .observe(kObservationAdmissionConfig,
                           observation(7U, 13U, 1'990'000'000LL, 2'000'000'000LL),
                           2'000'000'000LL)
                  .install_observation);
  EXPECT_DOUBLE_EQ(tracker.latest().ageMs(2'100'000'000LL), 110.0);
}

TEST(WorldGenerationTest, OutOfOrderHeartbeatCannotRewindLatestObservation) {
  LatestObservationTracker tracker;
  ASSERT_TRUE(tracker
                  .observe(kObservationAdmissionConfig,
                           observation(7U, 12U, 990'000'000LL, 1'000'000'000LL),
                           1'000'000'000LL)
                  .install_observation);

  EXPECT_FALSE(tracker
                   .observe(kObservationAdmissionConfig,
                            observation(7U, 11U, 1'990'000'000LL, 2'000'000'000LL),
                            2'000'000'000LL)
                   .install_observation);
  EXPECT_EQ(tracker.latest().sequence, 12U);
  EXPECT_EQ(tracker.latest().receive_stamp_ns, 1'000'000'000LL);
}

TEST(WorldGenerationTest, ExactHeartbeatReplayDoesNotRefreshObservationAge) {
  LatestObservationTracker tracker;
  ASSERT_TRUE(tracker
                  .observe(kObservationAdmissionConfig,
                           observation(7U, 12U, 990'000'000LL, 1'000'000'000LL),
                           1'000'000'000LL)
                  .install_observation);

  const ProducerEpochAdmissionResult replay = tracker.observe(
      kObservationAdmissionConfig, observation(7U, 12U, 990'000'000LL, 1'500'000'000LL),
      1'500'000'000LL);

  EXPECT_EQ(replay.status, ProducerEpochAdmissionStatus::kIdempotentReplay);
  EXPECT_EQ(tracker.latest().receive_stamp_ns, 1'000'000'000LL);
  EXPECT_DOUBLE_EQ(tracker.latest().ageMs(1'500'000'000LL), 510.0);
}

TEST(WorldGenerationTest, ObservationAgeIncludesDelayedSourceContent) {
  LatestObservationTracker tracker;
  ASSERT_TRUE(tracker
                  .observe(kObservationAdmissionConfig,
                           observation(7U, 12U, 100'000'000LL, 4'900'000'000LL),
                           5'000'000'000LL)
                  .install_observation);

  EXPECT_DOUBLE_EQ(tracker.latest().ageMs(5'050'000'000LL), 4'950.0);
}

TEST(WorldGenerationTest, ConflictedHeartbeatIdentityIsUnavailableUntilNewer) {
  LatestObservationTracker tracker;
  ASSERT_TRUE(tracker
                  .observe(kObservationAdmissionConfig,
                           observation(7U, 12U, 990'000'000LL, 1'000'000'000LL),
                           1'000'000'000LL)
                  .install_observation);

  const ProducerEpochAdmissionResult conflict = tracker.observe(
      kObservationAdmissionConfig,
      observation(7U, 12U, 990'000'000LL, 1'010'000'000LL, 100U), 1'010'000'000LL);
  ASSERT_EQ(conflict.status, ProducerEpochAdmissionStatus::kRejectedIdentityConflict);
  EXPECT_FALSE(tracker.latest().available());
  EXPECT_TRUE(tracker.latest().identity_conflicted);

  const ProducerEpochAdmissionResult recovered = tracker.observe(
      kObservationAdmissionConfig,
      observation(7U, 13U, 1'020'000'000LL, 1'030'000'000LL, 101U), 1'030'000'000LL);
  ASSERT_TRUE(recovered.install_observation);
  EXPECT_TRUE(tracker.latest().available());
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
