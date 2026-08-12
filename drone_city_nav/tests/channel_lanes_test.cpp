#include "drone_city_nav/channel_lanes.hpp"
#include "drone_city_nav/distance_field_3d.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <iterator>
#include <ranges>

namespace drone_city_nav {
namespace {

[[nodiscard]] ConstrainedFreeSpaceEdge straightChannel() {
  return ConstrainedFreeSpaceEdge{
      .id = "test:straight",
      .centerline = sampleRoute3D(
          std::vector<Point3>{{1.5, 6.5, 5.5}, {18.5, 6.5, 5.5}}, 0.5, 10.0),
      .min_z_m = 1.0,
      .max_z_m = 10.0,
      .width_m = 12.0,
      .height_m = 9.0,
      .minimum_clearance_m = 4.5,
      .speed_limit_mps = 10.0,
  };
}

[[nodiscard]] ChannelLaneConfig laneConfig() {
  return ChannelLaneConfig{
      .minimum_center_separation_m = 4.0,
      .minimum_wall_clearance_m = 0.25,
      .maximum_lane_count = 5U,
      .footprint =
          SweptFootprintConfig{
              .radius_m = 0.5,
              .lower_extent_m = 0.25,
              .upper_extent_m = 0.25,
              .perimeter_samples = 12U,
              .radial_rings = 2U,
              .axial_samples = 3U,
              .sweep_step_m = 0.25,
          },
  };
}

TEST(ChannelLanes, PreservesTrueWidthWhenDerivingParallelLanes) {
  ConstrainedFreeSpaceEdge channel = straightChannel();
  channel.width_m = 24.0;
  const ChannelLaneSet lanes = makeGeometricChannelLanes(channel, laneConfig());

  ASSERT_EQ(lanes.lanes.size(), 5U);
  EXPECT_DOUBLE_EQ(lanes.physical_width_m, 24.0);
  EXPECT_DOUBLE_EQ(lanes.lanes[0].lateral_offset_m, -8.0);
  EXPECT_DOUBLE_EQ(lanes.lanes[2].lateral_offset_m, 0.0);
  EXPECT_DOUBLE_EQ(lanes.lanes[4].lateral_offset_m, 8.0);
}

TEST(ChannelLanes, UsesMiteredOffsetsThroughRightAngleTurns) {
  const std::vector<RouteSample3D> centerline = sampleRoute3D(
      std::vector<Point3>{{0.0, 0.0, 5.0}, {10.0, 0.0, 5.0}, {10.0, 10.0, 5.0}}, 1.0,
      10.0);
  const std::vector<RouteSample3D> offset = offsetChannelCenterline(centerline, 5.0);

  ASSERT_EQ(offset.size(), centerline.size());
  const auto corner = std::ranges::find_if(centerline, [](const RouteSample3D& sample) {
    return std::abs(sample.position.x - 10.0) < 1.0e-9 &&
           std::abs(sample.position.y) < 1.0e-9;
  });
  ASSERT_NE(corner, centerline.end());
  const std::size_t index =
      static_cast<std::size_t>(std::distance(centerline.begin(), corner));
  EXPECT_NEAR(offset[index].position.x, 5.0, 1.0e-9);
  EXPECT_NEAR(offset[index].position.y, 5.0, 1.0e-9);
}

TEST(ChannelLanes, ReducesCapacityWhenAnOuterLaneIsNotRawSafe) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 20, 14, 12}};
  for (int x = 1; x < 19; ++x) {
    occupancy.setOccupied(GridIndex3D{x, 12, 5});
  }
  const DistanceField3D field = DistanceField3D::build(occupancy, 30.0);
  const GridBounds3D& bounds = field.bounds();
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};

  const ChannelLaneSet lanes = makeCollisionValidatedChannelLanes(
      straightChannel(), laneConfig(), grid, field.distancesM());

  ASSERT_EQ(lanes.lanes.size(), 2U);
  EXPECT_DOUBLE_EQ(lanes.lanes[0].lateral_offset_m, -2.0);
  EXPECT_DOUBLE_EQ(lanes.lanes[1].lateral_offset_m, 2.0);
}

TEST(ChannelLanes, RawValidationReducesCapacityAtOccupiedOuterLane) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 20, 14, 12}};
  for (int x = 1; x < 19; ++x) {
    occupancy.setOccupied(GridIndex3D{x, 10, 5});
  }

  const ChannelLaneSet lanes =
      makeRawCollisionValidatedChannelLanes(straightChannel(), laneConfig(), occupancy);

  ASSERT_EQ(lanes.lanes.size(), 2U);
  EXPECT_DOUBLE_EQ(lanes.lanes[0].lateral_offset_m, -2.0);
  EXPECT_DOUBLE_EQ(lanes.lanes[1].lateral_offset_m, 2.0);
}

TEST(ChannelLanes, GroupsAlternativeMovementsIntoOneConflictResource) {
  EXPECT_EQ(channelConflictResourceId("channel_t:west_north"), "channel_t");
  EXPECT_EQ(channelConflictResourceId("channel_straight"), "channel_straight");
}

} // namespace
} // namespace drone_city_nav
