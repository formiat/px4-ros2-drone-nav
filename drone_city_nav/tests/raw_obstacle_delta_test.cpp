#include "drone_city_nav/raw_obstacle_delta.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] msg::RawObstacleSnapshot makeSnapshot() {
  msg::RawObstacleSnapshot snapshot;
  snapshot.producer_instance_id = 7U;
  snapshot.obstacle_snapshot_revision = 10U;
  snapshot.risk_policy_fingerprint = 1234U;
  snapshot.risk_critical_distance_m = 1.0;
  snapshot.risk_preferred_distance_m = 6.0;
  snapshot.grid.header.frame_id = "map";
  snapshot.grid.info.resolution = 1.0F;
  snapshot.grid.info.width = 5U;
  snapshot.grid.info.height = 3U;
  snapshot.grid.info.origin.orientation.w = 1.0;
  snapshot.grid.data.assign(15U, static_cast<std::int8_t>(CellState::kUnknown));
  return snapshot;
}

[[nodiscard]] msg::RawObstacleDelta
requireDelta(std::optional<msg::RawObstacleDelta> delta) {
  if (!delta.has_value()) {
    ADD_FAILURE() << "expected a valid raw obstacle delta";
    return msg::RawObstacleDelta{};
  }
  return std::move(delta).value();
}

TEST(RawObstacleDelta, ReconstructsCumulativeAbsoluteChunks) {
  RawObstacleDeltaAccumulator accumulator;
  const msg::RawObstacleSnapshot snapshot = makeSnapshot();
  const RawObstacleGridUpdate initial = accumulator.apply(snapshot);
  ASSERT_TRUE(initial.accepted());
  ASSERT_TRUE(initial.state.occupancy);

  OccupancyGrid2D source = *initial.state.occupancy;
  source.setFree(GridIndex{1, 1});
  source.setOccupied(GridIndex{4, 2});
  const std::vector<std::size_t> changed_cells{6U, 14U};
  const std::vector<std::uint32_t> chunks =
      rawObstacleChunkIndices(source.bounds(), changed_cells, 2U);
  ASSERT_EQ(chunks, (std::vector<std::uint32_t>{0U, 5U}));

  const msg::RawObstacleDelta first = requireDelta(makeRawObstacleDelta(
      source, snapshot.grid.header, 7U, 10U, 11U, 1234U, 1.0, 6.0, 2U, chunks));
  const RawObstacleGridUpdate first_update = accumulator.apply(first);
  ASSERT_TRUE(first_update.accepted());
  EXPECT_EQ(first_update.state.occupancy->state(GridIndex{1, 1}), CellState::kFree);
  EXPECT_EQ(first_update.state.occupancy->state(GridIndex{4, 2}), CellState::kOccupied);

  source.setOccupied(GridIndex{1, 1});
  const msg::RawObstacleDelta second = requireDelta(makeRawObstacleDelta(
      source, snapshot.grid.header, 7U, 10U, 12U, 1234U, 1.0, 6.0, 2U, chunks));
  const RawObstacleGridUpdate second_update = accumulator.apply(second);
  ASSERT_TRUE(second_update.accepted());
  EXPECT_EQ(second_update.state.obstacle_snapshot_revision, 12U);
  EXPECT_EQ(second_update.state.occupancy->state(GridIndex{1, 1}),
            CellState::kOccupied);
  EXPECT_EQ(second_update.state.occupancy->state(GridIndex{4, 2}),
            CellState::kOccupied);

  EXPECT_EQ(accumulator.apply(first).status, RawObstacleGridUpdateStatus::kStale);
}

TEST(RawObstacleDelta, RejectsMissingBaseAndInvalidGeometry) {
  RawObstacleDeltaAccumulator accumulator;
  const msg::RawObstacleSnapshot snapshot = makeSnapshot();
  ASSERT_TRUE(accumulator.apply(snapshot).accepted());
  OccupancyGrid2D source = *accumulator.state().occupancy;
  source.setOccupied(GridIndex{1, 1});
  const std::vector<std::uint32_t> chunks{0U};
  msg::RawObstacleDelta delta = requireDelta(makeRawObstacleDelta(
      source, snapshot.grid.header, 7U, 9U, 11U, 1234U, 1.0, 6.0, 2U, chunks));
  EXPECT_EQ(accumulator.apply(delta).status,
            RawObstacleGridUpdateStatus::kBaseUnavailable);

  delta = requireDelta(makeRawObstacleDelta(source, snapshot.grid.header, 7U, 10U, 11U,
                                            1234U, 1.0, 6.0, 2U, chunks));
  delta.map_info.width = 6U;
  EXPECT_EQ(accumulator.apply(delta).status,
            RawObstacleGridUpdateStatus::kInvalidMessage);
}

TEST(RawObstacleDelta, NewFullSnapshotChangesTheRequiredBase) {
  RawObstacleDeltaAccumulator accumulator;
  msg::RawObstacleSnapshot first_snapshot = makeSnapshot();
  ASSERT_TRUE(accumulator.apply(first_snapshot).accepted());
  OccupancyGrid2D source = *accumulator.state().occupancy;
  source.setOccupied(GridIndex{1, 1});
  const std::vector<std::uint32_t> chunks{0U};
  const msg::RawObstacleDelta old_delta = requireDelta(makeRawObstacleDelta(
      source, first_snapshot.grid.header, 7U, 10U, 11U, 1234U, 1.0, 6.0, 2U, chunks));

  msg::RawObstacleSnapshot resync = first_snapshot;
  resync.obstacle_snapshot_revision = 20U;
  ASSERT_TRUE(accumulator.apply(resync).accepted());
  EXPECT_EQ(accumulator.apply(old_delta).status,
            RawObstacleGridUpdateStatus::kBaseUnavailable);
}

} // namespace
} // namespace drone_city_nav
