#include "drone_city_nav/raw_obstacle_snapshot_tracker.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] RawObstacleSnapshotMetadata metadata(const std::uint64_t producer,
                                                   const std::uint64_t revision,
                                                   const std::uint64_t policy = 3U) {
  return RawObstacleSnapshotMetadata{
      .identity =
          RawObstacleSnapshotIdentity{
              .producer_instance_id = producer,
              .revision = revision,
              .policy_fingerprint = policy,
          },
      .policy =
          ObstacleRiskPolicy{
              .critical_distance_m = 1.0,
              .preferred_distance_m = 4.0,
          },
      .grid_valid = true,
  };
}

} // namespace

TEST(RawObstacleSnapshotTracker, ClassifiesExactNewerAndPending) {
  RawObstacleSnapshotTracker tracker;
  ASSERT_TRUE(tracker.accept(metadata(7U, 10U)));

  EXPECT_EQ(tracker.relation({7U, 10U, 3U}), RawSnapshotRelation::kExact);
  EXPECT_EQ(tracker.relation({7U, 9U, 3U}), RawSnapshotRelation::kRuntimeNewer);
  EXPECT_EQ(tracker.relation({7U, 11U, 3U}), RawSnapshotRelation::kRuntimeOlder);
}

TEST(RawObstacleSnapshotTracker, ValidTrajectoryWaitsForFirstSnapshotThenRetries) {
  RawObstacleSnapshotTracker tracker;
  const RawObstacleSnapshotIdentity trajectory{7U, 10U, 3U};

  EXPECT_EQ(tracker.relation(trajectory), RawSnapshotRelation::kNoSnapshot);
  ASSERT_TRUE(tracker.accept(metadata(7U, 10U)));
  EXPECT_EQ(tracker.relation(trajectory), RawSnapshotRelation::kExact);
}

TEST(RawObstacleSnapshotTracker, ZeroTrajectoryIdentityIsMalformedWithoutSnapshot) {
  RawObstacleSnapshotTracker tracker;

  EXPECT_EQ(tracker.relation({}), RawSnapshotRelation::kMalformed);
  EXPECT_EQ(tracker.relation({7U, 0U, 3U}), RawSnapshotRelation::kMalformed);
}

TEST(RawObstacleSnapshotTracker, RejectsOutOfOrderAndMalformedSnapshots) {
  RawObstacleSnapshotTracker tracker;
  ASSERT_TRUE(tracker.accept(metadata(7U, 10U)));
  EXPECT_FALSE(tracker.accept(metadata(7U, 9U)));
  EXPECT_FALSE(tracker.accept(metadata(0U, 11U)));
  ASSERT_NE(tracker.current(), nullptr);
  EXPECT_EQ(tracker.current()->identity.revision, 10U);
}

TEST(RawObstacleSnapshotTracker, ClassifiesProducerAndPolicyMismatch) {
  RawObstacleSnapshotTracker tracker;
  ASSERT_TRUE(tracker.accept(metadata(7U, 10U)));

  EXPECT_EQ(tracker.relation({8U, 10U, 3U}), RawSnapshotRelation::kDifferentProducer);
  EXPECT_EQ(tracker.relation({7U, 10U, 4U}), RawSnapshotRelation::kPolicyMismatch);
}

TEST(RawObstacleSnapshotTracker, MapsEveryRelationToTerminalOrPendingDisposition) {
  EXPECT_EQ(classifyRawSnapshotTrajectoryDisposition(RawSnapshotRelation::kExact),
            RawSnapshotTrajectoryDisposition::kValidate);
  EXPECT_EQ(
      classifyRawSnapshotTrajectoryDisposition(RawSnapshotRelation::kRuntimeNewer),
      RawSnapshotTrajectoryDisposition::kValidate);
  for (const RawSnapshotRelation relation :
       {RawSnapshotRelation::kRuntimeOlder, RawSnapshotRelation::kNoSnapshot,
        RawSnapshotRelation::kDifferentProducer}) {
    EXPECT_EQ(classifyRawSnapshotTrajectoryDisposition(relation),
              RawSnapshotTrajectoryDisposition::kWait);
  }
  for (const RawSnapshotRelation relation :
       {RawSnapshotRelation::kRetiredProducer, RawSnapshotRelation::kPolicyMismatch,
        RawSnapshotRelation::kMalformed}) {
    EXPECT_EQ(classifyRawSnapshotTrajectoryDisposition(relation),
              RawSnapshotTrajectoryDisposition::kReject);
  }
}

TEST(RawObstacleSnapshotTracker, ProducerSwitchRetiresOldIdentity) {
  RawObstacleSnapshotTracker tracker;
  ASSERT_TRUE(tracker.accept(metadata(7U, 10U)));
  ASSERT_TRUE(tracker.accept(metadata(8U, 1U)));

  EXPECT_EQ(tracker.relation({7U, 10U, 3U}), RawSnapshotRelation::kRetiredProducer);
  EXPECT_EQ(tracker.relation({8U, 1U, 3U}), RawSnapshotRelation::kExact);
  EXPECT_FALSE(tracker.accept(metadata(7U, 11U)));
  ASSERT_NE(tracker.current(), nullptr);
  EXPECT_EQ(tracker.current()->identity.producer_instance_id, 8U);
}

TEST(RawObstacleSnapshotTracker, ProcessInstanceIdentityDoesNotDependOnRosTime) {
  const std::uint64_t first = generateRawObstacleProducerInstanceId();
  const std::uint64_t second = generateRawObstacleProducerInstanceId();

  EXPECT_NE(first, 0U);
  EXPECT_NE(second, 0U);
  EXPECT_NE(first, second);
}

TEST(PendingRawObstacleSnapshotDeadline, PreservesOneWaitAndResetsAfterCompletion) {
  PendingRawObstacleSnapshotDeadline deadline;

  deadline.startIfIdle(100);
  deadline.startIfIdle(200);
  EXPECT_EQ(deadline.startedAtNs(), 100);
  EXPECT_FALSE(deadline.expired(1099, 1000));
  EXPECT_TRUE(deadline.expired(1100, 1000));

  deadline.complete();
  EXPECT_EQ(deadline.startedAtNs(), 0);
  deadline.startIfIdle(2000);
  EXPECT_EQ(deadline.startedAtNs(), 2000);
  EXPECT_FALSE(deadline.expired(2500, 1000));
}

} // namespace drone_city_nav
