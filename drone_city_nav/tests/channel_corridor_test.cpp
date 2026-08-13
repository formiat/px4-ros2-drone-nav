#include "drone_city_nav/channel_corridor.hpp"

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

[[nodiscard]] ChannelCorridorConfig corridorConfig() {
  return ChannelCorridorConfig{
      .desired_center_separation_m = 4.0,
      .minimum_wall_clearance_m = 0.25,
      .lateral_probe_step_m = 0.5,
      .directional_offset_fraction = 0.5,
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

TEST(ChannelCorridor, PreservesContinuousUsableWidth) {
  ConstrainedFreeSpaceEdge channel = straightChannel();
  channel.width_m = 24.0;
  const ChannelCorridor corridor =
      makeGeometricChannelCorridor(channel, corridorConfig());

  EXPECT_DOUBLE_EQ(corridor.physical_width_m, 24.0);
  EXPECT_DOUBLE_EQ(corridor.minimum_lateral_offset_m, -11.25);
  EXPECT_DOUBLE_EQ(corridor.maximum_lateral_offset_m, 11.25);
  EXPECT_DOUBLE_EQ(corridor.usableWidthM(), 22.5);
  EXPECT_FALSE(corridor.raw_validated);
}

TEST(ChannelCorridor, UsesMiteredOffsetsThroughRightAngleTurns) {
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

TEST(ChannelCorridor, RawValidationClipsOnlyTheBlockedSide) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 20, 14, 12}};
  for (int x = 1; x < 19; ++x) {
    occupancy.setOccupied(GridIndex3D{x, 10, 5});
  }

  const ChannelCorridor geometric =
      makeGeometricChannelCorridor(straightChannel(), corridorConfig());
  const ChannelCorridor validated = makeRawCollisionValidatedChannelCorridor(
      straightChannel(), corridorConfig(), occupancy);

  ASSERT_TRUE(validated.raw_validated);
  EXPECT_LE(validated.minimum_lateral_offset_m,
            geometric.minimum_lateral_offset_m + 0.51);
  EXPECT_LT(validated.maximum_lateral_offset_m, geometric.maximum_lateral_offset_m);
  EXPECT_GT(validated.usableWidthM(), corridorConfig().desired_center_separation_m);
}

TEST(ChannelCorridor, SelectsContinuousOppositeDirectionalOffsets) {
  ChannelCorridor corridor =
      makeGeometricChannelCorridor(straightChannel(), corridorConfig());
  corridor.raw_validated = true;

  const double forward =
      preferredDirectionalChannelOffset(corridor, 1, corridorConfig());
  const double reverse =
      preferredDirectionalChannelOffset(corridor, -1, corridorConfig());

  EXPECT_LT(forward, 0.0);
  EXPECT_GT(reverse, 0.0);
  EXPECT_NEAR(forward + reverse, 0.0, 1.0e-9);
  EXPECT_GE(reverse - forward, corridorConfig().desired_center_separation_m);
}

TEST(ChannelCorridor, GroupsAlternativeMovementsIntoOneConflictResource) {
  EXPECT_EQ(channelConflictResourceId("channel_t:west_north"), "channel_t");
  EXPECT_EQ(channelConflictResourceId("channel_straight"), "channel_straight");
}

TEST(ChannelCorridor, ReusesImmutableResourceForIdenticalWorldAndConfiguration) {
  const std::vector<ConstrainedFreeSpaceEdge> channels{straightChannel()};
  const OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 20, 14, 12}};

  const ChannelCorridorResource first =
      acquireRawValidatedChannelCorridors(channels, corridorConfig(), occupancy);
  const ChannelCorridorResource second =
      acquireRawValidatedChannelCorridors(channels, corridorConfig(), occupancy);

  ASSERT_TRUE(first.corridors);
  ASSERT_TRUE(second.corridors);
  EXPECT_FALSE(first.shared_resource_reused);
  EXPECT_TRUE(second.shared_resource_reused);
  EXPECT_EQ(first.corridors.get(), second.corridors.get());

  ChannelCorridorConfig changed = corridorConfig();
  changed.minimum_wall_clearance_m += 0.1;
  const ChannelCorridorResource different =
      acquireRawValidatedChannelCorridors(channels, changed, occupancy);
  ASSERT_TRUE(different.corridors);
  EXPECT_FALSE(different.shared_resource_reused);
  EXPECT_NE(first.corridors.get(), different.corridors.get());
}

} // namespace
} // namespace drone_city_nav
