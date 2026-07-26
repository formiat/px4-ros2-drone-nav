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

} // namespace drone_city_nav
