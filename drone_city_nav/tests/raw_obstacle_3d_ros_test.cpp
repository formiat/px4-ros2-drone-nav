#include "drone_city_nav/raw_obstacle_3d_ros.hpp"

#include <gtest/gtest.h>

#include <array>

namespace drone_city_nav {
namespace {

constexpr GridBounds3D kBounds{-10.0, -20.0, 0.0, 0.5, 64, 80, 40};

TEST(RawObstacle3DRos, SnapshotAndCumulativeDeltaRoundTripTriStateChunks) {
  ObservedOccupancyGrid3D producer{kBounds};
  static_cast<void>(producer.setState({2, 3, 4}, ObservedVoxelState::kFree));
  static_cast<void>(producer.setState({20, 3, 4}, ObservedVoxelState::kOccupied));
  std_msgs::msg::Header header;
  header.frame_id = "map";
  const msg::RawObstacleSnapshot3D snapshot =
      makeRawObstacleSnapshot3D(producer, header, 42U, 1U);

  RawObstacleDeltaAccumulator3D accumulator;
  const RawObstacleGridUpdate3D initial = accumulator.apply(snapshot);
  ASSERT_TRUE(initial.accepted());
  EXPECT_TRUE(initial.full_reset);
  EXPECT_TRUE(initial.dirty_chunks.empty());
  ASSERT_NE(initial.state.occupancy, nullptr);
  EXPECT_TRUE(initial.state.occupancy->isKnownFree({2, 3, 4}));
  EXPECT_TRUE(initial.state.occupancy->isOccupied({20, 3, 4}));
  EXPECT_FALSE(initial.state.occupancy->isKnown({3, 3, 4}));

  static_cast<void>(producer.setState({20, 3, 4}, ObservedVoxelState::kFree));
  static_cast<void>(producer.setState({35, 3, 4}, ObservedVoxelState::kOccupied));
  const std::array dirty{ObservedOccupancyGrid3D::chunkIndex({20, 3, 4}),
                         ObservedOccupancyGrid3D::chunkIndex({35, 3, 4})};
  const msg::RawObstacleDelta3D delta =
      makeRawObstacleDelta3D(producer, header, 42U, 1U, 2U, dirty);
  const RawObstacleGridUpdate3D updated = accumulator.apply(delta);

  ASSERT_TRUE(updated.accepted());
  EXPECT_FALSE(updated.full_reset);
  ASSERT_EQ(updated.dirty_chunks.size(), 2U);
  EXPECT_EQ(updated.dirty_chunks[0], ObservedOccupancyGrid3D::chunkIndex({20, 3, 4}));
  EXPECT_EQ(updated.dirty_chunks[1], ObservedOccupancyGrid3D::chunkIndex({35, 3, 4}));
  ASSERT_NE(updated.state.occupancy, nullptr);
  EXPECT_TRUE(updated.state.occupancy->isKnownFree({20, 3, 4}));
  EXPECT_TRUE(updated.state.occupancy->isOccupied({35, 3, 4}));
  EXPECT_EQ(updated.state.base_snapshot_revision, 1U);
  EXPECT_EQ(updated.state.obstacle_snapshot_revision, 2U);
}

TEST(RawObstacle3DRos, RejectsDeltaFromDifferentBase) {
  ObservedOccupancyGrid3D producer{kBounds};
  static_cast<void>(producer.setState({2, 3, 4}, ObservedVoxelState::kFree));
  std_msgs::msg::Header header;
  RawObstacleDeltaAccumulator3D accumulator;
  ASSERT_TRUE(accumulator.apply(makeRawObstacleSnapshot3D(producer, header, 42U, 5U))
                  .accepted());
  const std::array dirty{ObservedOccupancyGrid3D::chunkIndex({2, 3, 4})};

  const RawObstacleGridUpdate3D update =
      accumulator.apply(makeRawObstacleDelta3D(producer, header, 42U, 4U, 6U, dirty));

  EXPECT_EQ(update.status, RawObstacleGridUpdateStatus3D::kBaseUnavailable);
}

TEST(RawObstacle3DRos, RejectsDuplicateChunkInDelta) {
  ObservedOccupancyGrid3D producer{kBounds};
  static_cast<void>(producer.setState({2, 3, 4}, ObservedVoxelState::kFree));
  std_msgs::msg::Header header;
  RawObstacleDeltaAccumulator3D accumulator;
  ASSERT_TRUE(accumulator.apply(makeRawObstacleSnapshot3D(producer, header, 42U, 1U))
                  .accepted());
  const OccupancyChunkIndex3D chunk = ObservedOccupancyGrid3D::chunkIndex({2, 3, 4});
  const std::array dirty{chunk, chunk};

  const RawObstacleGridUpdate3D update =
      accumulator.apply(makeRawObstacleDelta3D(producer, header, 42U, 1U, 2U, dirty));

  EXPECT_EQ(update.status, RawObstacleGridUpdateStatus3D::kInvalidMessage);
}

TEST(RawObstacle3DRos, NewerSnapshotUsesDirtyChunksWhenGeometryIsStable) {
  ObservedOccupancyGrid3D producer{kBounds};
  static_cast<void>(producer.setState({2, 3, 4}, ObservedVoxelState::kFree));
  static_cast<void>(producer.setState({20, 3, 4}, ObservedVoxelState::kOccupied));
  std_msgs::msg::Header header;
  RawObstacleDeltaAccumulator3D accumulator;

  ASSERT_TRUE(accumulator.apply(makeRawObstacleSnapshot3D(producer, header, 42U, 1U))
                  .accepted());

  const RawObstacleGridUpdate3D unchanged =
      accumulator.apply(makeRawObstacleSnapshot3D(producer, header, 42U, 2U));
  ASSERT_TRUE(unchanged.accepted());
  EXPECT_FALSE(unchanged.full_reset);
  EXPECT_TRUE(unchanged.dirty_chunks.empty());

  static_cast<void>(producer.setState({2, 3, 4}, ObservedVoxelState::kOccupied));
  static_cast<void>(producer.setState({35, 3, 4}, ObservedVoxelState::kFree));
  const RawObstacleGridUpdate3D changed =
      accumulator.apply(makeRawObstacleSnapshot3D(producer, header, 42U, 3U));

  ASSERT_TRUE(changed.accepted());
  EXPECT_FALSE(changed.full_reset);
  const std::array expected{ObservedOccupancyGrid3D::chunkIndex({2, 3, 4}),
                            ObservedOccupancyGrid3D::chunkIndex({35, 3, 4})};
  ASSERT_EQ(changed.dirty_chunks.size(), expected.size());
  EXPECT_EQ(changed.dirty_chunks[0], expected[0]);
  EXPECT_EQ(changed.dirty_chunks[1], expected[1]);
  ASSERT_NE(changed.state.occupancy, nullptr);
  EXPECT_TRUE(changed.state.occupancy->isOccupied({2, 3, 4}));
  EXPECT_TRUE(changed.state.occupancy->isKnownFree({35, 3, 4}));
}

} // namespace
} // namespace drone_city_nav
