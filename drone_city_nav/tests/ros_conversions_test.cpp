#include "drone_city_nav/ros_conversions.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] nav_msgs::msg::OccupancyGrid makeRosGrid(const std::uint32_t width,
                                                       const std::uint32_t height) {
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = "map";
  msg.info.resolution = 1.0F;
  msg.info.width = width;
  msg.info.height = height;
  msg.info.origin.position.x = -1.0;
  msg.info.origin.position.y = -2.0;
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(static_cast<std::size_t>(width) * height,
                  static_cast<std::int8_t>(-1));
  return msg;
}

} // namespace

TEST(RosConversions, RejectsInvalidOccupancyGridMetadata) {
  nav_msgs::msg::OccupancyGrid msg = makeRosGrid(2U, 2U);
  msg.info.resolution = 0.0F;

  const OccupancyGridFromRosResult result =
      occupancyGridFromRos(msg, OccupancyGridFromRosConfig{});

  EXPECT_FALSE(result.grid.has_value());
  ASSERT_TRUE(result.error.has_value());
  if (!result.error.has_value()) {
    return;
  }
  EXPECT_EQ(*result.error, OccupancyGridFromRosError::kInvalidMetadata);
}

TEST(RosConversions, RejectsMismatchedOccupancyGridDataSize) {
  nav_msgs::msg::OccupancyGrid msg = makeRosGrid(3U, 2U);
  msg.data.pop_back();

  const OccupancyGridFromRosResult result =
      occupancyGridFromRos(msg, OccupancyGridFromRosConfig{});

  EXPECT_FALSE(result.grid.has_value());
  ASSERT_TRUE(result.error.has_value());
  if (!result.error.has_value()) {
    return;
  }
  EXPECT_EQ(*result.error, OccupancyGridFromRosError::kMismatchedDataSize);
  EXPECT_EQ(result.expected_data_size, 6U);
  EXPECT_EQ(result.actual_data_size, 5U);
}

TEST(RosConversions, ConvertsUnknownFreeOccupiedByThresholds) {
  nav_msgs::msg::OccupancyGrid msg = makeRosGrid(3U, 1U);
  msg.data = {static_cast<std::int8_t>(-1), static_cast<std::int8_t>(0),
              static_cast<std::int8_t>(65)};

  const OccupancyGridFromRosResult result =
      occupancyGridFromRos(msg, OccupancyGridFromRosConfig{65, 0});

  ASSERT_TRUE(result.grid.has_value());
  if (!result.grid.has_value()) {
    return;
  }
  const OccupancyGrid2D& grid = *result.grid;
  EXPECT_EQ(grid.state(GridIndex{0, 0}), CellState::kUnknown);
  EXPECT_EQ(grid.state(GridIndex{1, 0}), CellState::kFree);
  EXPECT_EQ(grid.state(GridIndex{2, 0}), CellState::kOccupied);
  EXPECT_DOUBLE_EQ(grid.originX(), -1.0);
  EXPECT_DOUBLE_EQ(grid.originY(), -2.0);
}

TEST(RosConversions, SerializesInflatedCellsOnlyWhenRequested) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 3, 1}};
  grid.setFree(GridIndex{0, 0});
  grid.setOccupied(GridIndex{1, 0});
  grid.rebuildInflation(1.1);

  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp.sec = 42;

  nav_msgs::msg::OccupancyGrid with_inflation =
      occupancyGridToRos(grid, OccupancyGridToRosConfig{header, true});
  nav_msgs::msg::OccupancyGrid without_inflation =
      occupancyGridToRos(grid, OccupancyGridToRosConfig{header, false});

  EXPECT_EQ(with_inflation.header.frame_id, "map");
  EXPECT_EQ(with_inflation.info.map_load_time.sec, 42);
  EXPECT_EQ(with_inflation.data[0], 80);
  EXPECT_EQ(with_inflation.data[1], 100);
  EXPECT_EQ(without_inflation.data[0], 0);
  EXPECT_EQ(without_inflation.data[1], 100);
}

TEST(RosConversions, BuildsPathWithHeaderAltitudeAndIdentityOrientation) {
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp.sec = 7;
  const std::vector<Point2> points{Point2{1.0, 2.0}, Point2{3.0, 4.0}};

  const nav_msgs::msg::Path path = pathToRos(points, header, 12.5);

  EXPECT_EQ(path.header.frame_id, "map");
  EXPECT_EQ(path.header.stamp.sec, 7);
  ASSERT_EQ(path.poses.size(), 2U);
  EXPECT_DOUBLE_EQ(path.poses[0].pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(path.poses[0].pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(path.poses[0].pose.position.z, 12.5);
  EXPECT_DOUBLE_EQ(path.poses[0].pose.orientation.w, 1.0);
  EXPECT_EQ(path.poses[0].header.frame_id, "map");
}

TEST(RosConversions, BuildsEmptyPathWithoutSideEffects) {
  std_msgs::msg::Header header;
  header.frame_id = "map";
  const std::vector<Point2> points;

  const nav_msgs::msg::Path path = pathToRos(points, header, 10.0);

  EXPECT_EQ(path.header.frame_id, "map");
  EXPECT_TRUE(path.poses.empty());
}

} // namespace drone_city_nav
