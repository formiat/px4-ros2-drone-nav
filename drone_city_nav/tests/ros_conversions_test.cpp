#include "drone_city_nav/ros_conversions.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

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

  const RawOccupancyGridFromRosResult result =
      rawOccupancyGridFromRos(msg, RawOccupancyGridFromRosConfig{});

  EXPECT_FALSE(result.grid.has_value());
  ASSERT_TRUE(result.error.has_value());
  if (!result.error.has_value()) {
    return;
  }
  EXPECT_EQ(*result.error, OccupancyGridFromRosError::kInvalidMetadata);
}

TEST(RosConversions, RejectsNonFiniteAndOversizedOccupancyGridMetadata) {
  nav_msgs::msg::OccupancyGrid nonfinite_origin = makeRosGrid(2U, 2U);
  nonfinite_origin.info.origin.position.x = std::numeric_limits<double>::quiet_NaN();

  const RawOccupancyGridFromRosResult nonfinite_result =
      rawOccupancyGridFromRos(nonfinite_origin, RawOccupancyGridFromRosConfig{});

  EXPECT_FALSE(nonfinite_result.grid.has_value());
  ASSERT_TRUE(nonfinite_result.error.has_value());
  if (!nonfinite_result.error.has_value()) {
    return;
  }
  EXPECT_EQ(*nonfinite_result.error, OccupancyGridFromRosError::kInvalidMetadata);

  nav_msgs::msg::OccupancyGrid infinite_resolution = makeRosGrid(2U, 2U);
  infinite_resolution.info.resolution = std::numeric_limits<float>::infinity();

  const RawOccupancyGridFromRosResult infinite_result =
      rawOccupancyGridFromRos(infinite_resolution, RawOccupancyGridFromRosConfig{});

  EXPECT_FALSE(infinite_result.grid.has_value());
  ASSERT_TRUE(infinite_result.error.has_value());
  if (!infinite_result.error.has_value()) {
    return;
  }
  EXPECT_EQ(*infinite_result.error, OccupancyGridFromRosError::kInvalidMetadata);

  nav_msgs::msg::OccupancyGrid oversized = makeRosGrid(1U, 1U);
  oversized.info.width = std::numeric_limits<std::uint32_t>::max();
  oversized.data.clear();

  const RawOccupancyGridFromRosResult oversized_result =
      rawOccupancyGridFromRos(oversized, RawOccupancyGridFromRosConfig{});

  EXPECT_FALSE(oversized_result.grid.has_value());
  ASSERT_TRUE(oversized_result.error.has_value());
  if (!oversized_result.error.has_value()) {
    return;
  }
  EXPECT_EQ(*oversized_result.error, OccupancyGridFromRosError::kInvalidMetadata);
}

TEST(RosConversions, RejectsMismatchedOccupancyGridDataSize) {
  nav_msgs::msg::OccupancyGrid msg = makeRosGrid(3U, 2U);
  msg.data.pop_back();

  const RawOccupancyGridFromRosResult result =
      rawOccupancyGridFromRos(msg, RawOccupancyGridFromRosConfig{});

  EXPECT_FALSE(result.grid.has_value());
  ASSERT_TRUE(result.error.has_value());
  if (!result.error.has_value()) {
    return;
  }
  EXPECT_EQ(*result.error, OccupancyGridFromRosError::kMismatchedDataSize);
  EXPECT_EQ(result.expected_data_size, 6U);
  EXPECT_EQ(result.actual_data_size, 5U);
}

TEST(RosConversions, ConvertsRawUnknownFreeOccupiedValues) {
  nav_msgs::msg::OccupancyGrid msg = makeRosGrid(4U, 1U);
  msg.data = {static_cast<std::int8_t>(-1), static_cast<std::int8_t>(0),
              static_cast<std::int8_t>(80), static_cast<std::int8_t>(100)};

  const RawOccupancyGridFromRosResult result =
      rawOccupancyGridFromRos(msg, RawOccupancyGridFromRosConfig{100, 0});

  ASSERT_TRUE(result.grid.has_value());
  if (!result.grid.has_value()) {
    return;
  }
  const OccupancyGrid2D& grid = *result.grid;
  EXPECT_EQ(grid.state(GridIndex{0, 0}), CellState::kUnknown);
  EXPECT_EQ(grid.state(GridIndex{1, 0}), CellState::kFree);
  EXPECT_EQ(grid.state(GridIndex{2, 0}), CellState::kUnknown);
  EXPECT_EQ(grid.state(GridIndex{3, 0}), CellState::kOccupied);
  EXPECT_EQ(result.intermediate_value_cells, 1U);
  EXPECT_DOUBLE_EQ(grid.originX(), -1.0);
  EXPECT_DOUBLE_EQ(grid.originY(), -2.0);
}

TEST(RosConversions, SerializesRawGridWithoutInflationValues) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 3, 1}};
  grid.setFree(GridIndex{0, 0});
  grid.setOccupied(GridIndex{1, 0});
  grid.rebuildInflation(1.1);

  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp.sec = 42;

  nav_msgs::msg::OccupancyGrid raw =
      rawOccupancyGridToRos(grid, RawOccupancyGridToRosConfig{header});

  EXPECT_EQ(raw.header.frame_id, "map");
  EXPECT_EQ(raw.info.map_load_time.sec, 42);
  EXPECT_EQ(raw.data[0], 0);
  EXPECT_EQ(raw.data[1], 100);
}

TEST(RosConversions, SerializesProhibitedGridWithInflationValues) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 3, 1}};
  grid.setFree(GridIndex{0, 0});
  grid.setOccupied(GridIndex{1, 0});
  grid.rebuildInflation(1.1);

  std_msgs::msg::Header header;
  header.frame_id = "map";

  nav_msgs::msg::OccupancyGrid prohibited =
      prohibitedGridToRos(grid, ProhibitedGridToRosConfig{header});

  EXPECT_EQ(prohibited.data[0], 80);
  EXPECT_EQ(prohibited.data[1], 100);
}

TEST(RosConversions, ClearanceUsesOccupiedCellsAtThreshold) {
  nav_msgs::msg::OccupancyGrid msg = makeRosGrid(5U, 5U);
  msg.info.origin.position.x = 0.0;
  msg.info.origin.position.y = 0.0;
  msg.data[static_cast<std::size_t>(2U) * 5U + 3U] = 100;

  const double clearance_m = occupancyGridClearanceM(msg, Point2{2.5, 2.5}, 4.0, 100);

  EXPECT_DOUBLE_EQ(clearance_m, 1.0);
}

TEST(RosConversions, ClearanceTreatsInflatedCellsAsSafetyBlockers) {
  nav_msgs::msg::OccupancyGrid msg = makeRosGrid(5U, 5U);
  msg.info.origin.position.x = 0.0;
  msg.info.origin.position.y = 0.0;
  msg.data[static_cast<std::size_t>(2U) * 5U + 3U] = 80;

  const double inflated_clearance_m =
      occupancyGridClearanceM(msg, Point2{2.5, 2.5}, 4.0, 80);
  const double occupied_clearance_m =
      occupancyGridClearanceM(msg, Point2{2.5, 2.5}, 4.0, 100);

  EXPECT_DOUBLE_EQ(inflated_clearance_m, 1.0);
  EXPECT_TRUE(std::isinf(occupied_clearance_m));
}

TEST(RosConversions, ClearanceIgnoresUnknownAndFreeCells) {
  nav_msgs::msg::OccupancyGrid msg = makeRosGrid(5U, 5U);
  msg.info.origin.position.x = 0.0;
  msg.info.origin.position.y = 0.0;
  msg.data[static_cast<std::size_t>(2U) * 5U + 2U] = -1;
  msg.data[static_cast<std::size_t>(2U) * 5U + 3U] = 0;

  const double clearance_m = occupancyGridClearanceM(msg, Point2{2.5, 2.5}, 4.0, 80);

  EXPECT_TRUE(std::isinf(clearance_m));
}

TEST(RosConversions, ClearanceReturnsNanForInvalidGridOrOutsidePoint) {
  nav_msgs::msg::OccupancyGrid msg = makeRosGrid(5U, 5U);

  nav_msgs::msg::OccupancyGrid invalid = msg;
  invalid.data.pop_back();

  EXPECT_TRUE(std::isnan(occupancyGridClearanceM(invalid, Point2{0.0, 0.0}, 4.0, 80)));
  EXPECT_TRUE(std::isnan(occupancyGridClearanceM(msg, Point2{100.0, 100.0}, 4.0, 80)));
  EXPECT_TRUE(std::isnan(occupancyGridClearanceM(
      msg, Point2{std::numeric_limits<double>::quiet_NaN(), 0.0}, 4.0, 80)));

  nav_msgs::msg::OccupancyGrid invalid_metadata = msg;
  invalid_metadata.info.origin.position.y = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(
      std::isnan(occupancyGridClearanceM(invalid_metadata, Point2{0.0, 0.0}, 4.0, 80)));
  EXPECT_TRUE(std::isnan(occupancyGridClearanceM(
      msg, Point2{0.0, 0.0}, std::numeric_limits<double>::max(), 80)));
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

TEST(RosConversions, BuildsPathFromTrajectorySamplesWithPerSampleAltitude) {
  std_msgs::msg::Header header;
  header.frame_id = "map";
  std::vector<TrajectoryPointSample> samples(2U);
  samples[0].point = Point2{1.0, 2.0};
  samples[0].z_m = 12.0;
  samples[1].point = Point2{3.0, 4.0};
  samples[1].z_m = 18.0;

  const nav_msgs::msg::Path path = pathToRos(samples, header);

  ASSERT_EQ(path.poses.size(), 2U);
  EXPECT_DOUBLE_EQ(path.poses[0].pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(path.poses[0].pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(path.poses[0].pose.position.z, 12.0);
  EXPECT_DOUBLE_EQ(path.poses[1].pose.position.x, 3.0);
  EXPECT_DOUBLE_EQ(path.poses[1].pose.position.y, 4.0);
  EXPECT_DOUBLE_EQ(path.poses[1].pose.position.z, 18.0);
  EXPECT_DOUBLE_EQ(path.poses[1].pose.orientation.w, 1.0);
}

} // namespace drone_city_nav
