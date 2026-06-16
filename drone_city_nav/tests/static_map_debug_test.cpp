#include "drone_city_nav/static_map_debug.hpp"

#include <gtest/gtest.h>

#include <cstring>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupancyGrid2D makeDebugGrid() {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 3, 2}};
  grid.setFree(GridIndex{0, 0});
  grid.setOccupied(GridIndex{1, 0});
  grid.setOccupied(GridIndex{2, 1});
  grid.rebuildInflation(1.1);
  return grid;
}

[[nodiscard]] std_msgs::msg::Header makeHeader() {
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp.sec = 9;
  return header;
}

[[nodiscard]] float readFloat(const std::vector<unsigned char>& data,
                              const std::size_t offset) {
  float value = 0.0F;
  std::memcpy(&value, &data[offset], sizeof(float));
  return value;
}

} // namespace

TEST(StaticMapDebug, GridMessageUsesStaticMapFrameAndNoInflation) {
  const OccupancyGrid2D grid = makeDebugGrid();

  const nav_msgs::msg::OccupancyGrid msg =
      staticMapGridMessage(grid, StaticMapDebugConfig{makeHeader(), 0.05F});

  EXPECT_EQ(msg.header.frame_id, "map");
  EXPECT_EQ(msg.header.stamp.sec, 9);
  EXPECT_EQ(msg.data[0], 0);
  EXPECT_EQ(msg.data[1], 100);
  EXPECT_EQ(msg.data[2], static_cast<std::int8_t>(-1));
}

TEST(StaticMapDebug, PointCloudContainsOnlyOccupiedCells) {
  const OccupancyGrid2D grid = makeDebugGrid();

  const sensor_msgs::msg::PointCloud2 cloud =
      staticMapPointCloud(grid, StaticMapDebugConfig{makeHeader(), 0.25F});

  EXPECT_EQ(cloud.header.frame_id, "map");
  EXPECT_EQ(cloud.height, 1U);
  EXPECT_EQ(cloud.width, 2U);
  EXPECT_TRUE(cloud.is_dense);
  EXPECT_EQ(cloud.point_step, 12U);
  EXPECT_EQ(cloud.row_step, 24U);
  ASSERT_EQ(cloud.data.size(), 24U);
  EXPECT_FLOAT_EQ(readFloat(cloud.data, 0U), 1.5F);
  EXPECT_FLOAT_EQ(readFloat(cloud.data, 4U), 0.5F);
  EXPECT_FLOAT_EQ(readFloat(cloud.data, 8U), 0.25F);
  EXPECT_FLOAT_EQ(readFloat(cloud.data, 12U), 2.5F);
  EXPECT_FLOAT_EQ(readFloat(cloud.data, 16U), 1.5F);
  EXPECT_FLOAT_EQ(readFloat(cloud.data, 20U), 0.25F);
}

TEST(StaticMapDebug, PointCloudFieldsAreXyzFloat32) {
  const OccupancyGrid2D grid = makeDebugGrid();

  const sensor_msgs::msg::PointCloud2 cloud =
      staticMapPointCloud(grid, StaticMapDebugConfig{makeHeader(), 0.05F});

  ASSERT_EQ(cloud.fields.size(), 3U);
  EXPECT_EQ(cloud.fields[0].name, "x");
  EXPECT_EQ(cloud.fields[0].offset, 0U);
  EXPECT_EQ(cloud.fields[0].datatype, sensor_msgs::msg::PointField::FLOAT32);
  EXPECT_EQ(cloud.fields[1].name, "y");
  EXPECT_EQ(cloud.fields[1].offset, 4U);
  EXPECT_EQ(cloud.fields[1].datatype, sensor_msgs::msg::PointField::FLOAT32);
  EXPECT_EQ(cloud.fields[2].name, "z");
  EXPECT_EQ(cloud.fields[2].offset, 8U);
  EXPECT_EQ(cloud.fields[2].datatype, sensor_msgs::msg::PointField::FLOAT32);
}

TEST(StaticMapDebug, EmptyMapPublishesValidEmptyPointCloud) {
  const OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 2, 2}};

  const sensor_msgs::msg::PointCloud2 cloud =
      staticMapPointCloud(grid, StaticMapDebugConfig{makeHeader(), 0.05F});

  EXPECT_EQ(cloud.height, 1U);
  EXPECT_EQ(cloud.width, 0U);
  EXPECT_EQ(cloud.point_step, 12U);
  EXPECT_EQ(cloud.row_step, 0U);
  EXPECT_TRUE(cloud.data.empty());
}

} // namespace drone_city_nav
