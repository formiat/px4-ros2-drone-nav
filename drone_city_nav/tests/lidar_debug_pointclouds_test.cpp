#include "drone_city_nav/lidar_debug_pointclouds.hpp"

#include <gtest/gtest.h>

#include <cstring>
#include <limits>
#include <unordered_map>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] float readFloat(const std::vector<std::uint8_t>& data,
                              const std::size_t offset) {
  if (offset + sizeof(float) > data.size()) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  float value{0.0F};
  std::memcpy(&value, &data.at(offset), sizeof(float));
  return value;
}

[[nodiscard]] MemoryCellProvenance makeProvenance(const Point3 trigger_endpoint,
                                                  const bool endpoint_xyz_valid) {
  MemoryCellProvenance provenance;
  provenance.occupancy_trigger.beam.projection.endpoint_map_m = trigger_endpoint;
  provenance.occupancy_trigger.beam.projection.endpoint_xyz_valid = endpoint_xyz_valid;
  provenance.last_hit = provenance.occupancy_trigger;
  return provenance;
}

[[nodiscard]] sensor_msgs::msg::PointCloud2
buildTriggerCloud(const Point3 endpoint, const bool endpoint_xyz_valid = true) {
  std::unordered_map<std::size_t, MemoryCellProvenance> provenance;
  provenance.emplace(1U, makeProvenance(endpoint, endpoint_xyz_valid));
  const builtin_interfaces::msg::Time stamp;
  return buildObstacleMemoryTriggerPointCloud(provenance, stamp, "map");
}

TEST(LidarDebugPointcloudsTest, CollectsOnlyRequestedOccupancyRange) {
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.resolution = 0.5F;
  grid.info.width = 3U;
  grid.info.height = 2U;
  grid.info.origin.position.x = 10.0;
  grid.info.origin.position.y = -2.0;
  grid.data = {
      -1, 0, 80, 99, 100, 42,
  };

  const std::vector<Point2> prohibited = collectProhibitedGridPoints(grid);
  ASSERT_EQ(prohibited.size(), 2U);
  EXPECT_DOUBLE_EQ(prohibited[0].x, 11.25);
  EXPECT_DOUBLE_EQ(prohibited[0].y, -1.75);
  EXPECT_DOUBLE_EQ(prohibited[1].x, 10.25);
  EXPECT_DOUBLE_EQ(prohibited[1].y, -1.25);

  const std::vector<Point2> occupied = collectOccupiedGridPoints(grid);
  ASSERT_EQ(occupied.size(), 1U);
  EXPECT_DOUBLE_EQ(occupied[0].x, 10.75);
  EXPECT_DOUBLE_EQ(occupied[0].y, -1.25);
}

TEST(LidarDebugPointcloudsTest, RejectsInvalidGridGeometry) {
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.resolution = 0.0F;
  grid.info.width = 10U;
  grid.info.height = 10U;
  EXPECT_TRUE(collectOccupiedGridPoints(grid).empty());
}

TEST(LidarDebugPointcloudsTest, BuildsPointCloud2WithXyzFloatLayout) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 12;
  stamp.nanosec = 34U;
  const std::vector<Point2> points{Point2{1.5, -2.0}, Point2{3.0, 4.5}};

  const sensor_msgs::msg::PointCloud2 cloud = buildLidarDebugPointCloud(
      points, std::numeric_limits<double>::quiet_NaN(), stamp, "map");

  EXPECT_EQ(cloud.header.stamp.sec, 12);
  EXPECT_EQ(cloud.header.stamp.nanosec, 34U);
  EXPECT_EQ(cloud.header.frame_id, "map");
  EXPECT_EQ(cloud.height, 1U);
  EXPECT_EQ(cloud.width, 2U);
  EXPECT_EQ(cloud.point_step, 12U);
  EXPECT_EQ(cloud.row_step, 24U);
  ASSERT_EQ(cloud.fields.size(), 3U);
  EXPECT_EQ(cloud.fields[0].name, "x");
  EXPECT_EQ(cloud.fields[1].name, "y");
  EXPECT_EQ(cloud.fields[2].name, "z");
  ASSERT_EQ(cloud.data.size(), 24U);
  EXPECT_FLOAT_EQ(readFloat(cloud.data, 0U), 1.5F);
  EXPECT_FLOAT_EQ(readFloat(cloud.data, 4U), -2.0F);
  EXPECT_FLOAT_EQ(readFloat(cloud.data, 8U), 0.0F);
  EXPECT_FLOAT_EQ(readFloat(cloud.data, 12U), 3.0F);
  EXPECT_FLOAT_EQ(readFloat(cloud.data, 16U), 4.5F);
  EXPECT_FLOAT_EQ(readFloat(cloud.data, 20U), 0.0F);
}

TEST(LidarDebugPointcloudsTest, CompensatesZForGazeboAlignedRvizFrame) {
  builtin_interfaces::msg::Time stamp;
  const std::vector<Point2> points{Point2{1.5, -2.0}};

  const sensor_msgs::msg::PointCloud2 cloud =
      buildLidarDebugPointCloud(points, 2.5, stamp, "map");

  ASSERT_EQ(cloud.data.size(), 12U);
  EXPECT_FLOAT_EQ(readFloat(cloud.data, 8U), -2.5F);
}

TEST(LidarDebugPointcloudsTest, Builds3dCurrentLidarCloudWithTrueEndpointAltitude) {
  const builtin_interfaces::msg::Time stamp;
  const std::vector<Point3> points{Point3{1.5, -2.0, 7.25}};

  const sensor_msgs::msg::PointCloud2 cloud =
      buildLidarDebugPointCloud(points, stamp, "map");

  ASSERT_EQ(cloud.width, 1U);
  ASSERT_EQ(cloud.data.size(), 12U);
  EXPECT_FLOAT_EQ(readFloat(cloud.data, 0U), 1.5F);
  EXPECT_FLOAT_EQ(readFloat(cloud.data, 4U), -2.0F);
  EXPECT_FLOAT_EQ(readFloat(cloud.data, 8U), -7.25F);
}

TEST(LidarDebugPointcloudsTest, BuildsSortedCloudFromOccupancyTriggerEndpoints) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 45;
  stamp.nanosec = 67U;
  std::unordered_map<std::size_t, MemoryCellProvenance> provenance;
  provenance.emplace(9U, makeProvenance(Point3{9.0, 10.0, 11.0}, true));
  provenance.emplace(2U, makeProvenance(Point3{2.0, 3.0, 4.0}, true));
  provenance.at(2U).last_hit.beam.projection.endpoint_map_m = Point3{20.0, 30.0, 40.0};

  const sensor_msgs::msg::PointCloud2 cloud =
      buildObstacleMemoryTriggerPointCloud(provenance, stamp, "map");

  EXPECT_EQ(cloud.header.stamp.sec, 45);
  EXPECT_EQ(cloud.header.stamp.nanosec, 67U);
  EXPECT_EQ(cloud.header.frame_id, "map");
  ASSERT_EQ(cloud.width, 2U);
  ASSERT_EQ(cloud.data.size(), 24U);
  EXPECT_FLOAT_EQ(readFloat(cloud.data, 0U), 2.0F);
  EXPECT_FLOAT_EQ(readFloat(cloud.data, 4U), 3.0F);
  EXPECT_FLOAT_EQ(readFloat(cloud.data, 8U), -4.0F);
  EXPECT_FLOAT_EQ(readFloat(cloud.data, 12U), 9.0F);
  EXPECT_FLOAT_EQ(readFloat(cloud.data, 16U), 10.0F);
  EXPECT_FLOAT_EQ(readFloat(cloud.data, 20U), -11.0F);
}

TEST(LidarDebugPointcloudsTest, OmitsTriggerWhenEndpointXyzIsInvalid) {
  const sensor_msgs::msg::PointCloud2 cloud =
      buildTriggerCloud(Point3{1.0, 2.0, 3.0}, false);

  EXPECT_EQ(cloud.width, 0U);
  EXPECT_TRUE(cloud.data.empty());
}

TEST(LidarDebugPointcloudsTest, OmitsTriggerWhenXIsNotFinite) {
  const sensor_msgs::msg::PointCloud2 cloud =
      buildTriggerCloud(Point3{std::numeric_limits<double>::quiet_NaN(), 2.0, 3.0});

  EXPECT_EQ(cloud.width, 0U);
  EXPECT_TRUE(cloud.data.empty());
}

TEST(LidarDebugPointcloudsTest, OmitsTriggerWhenYIsNotFinite) {
  const sensor_msgs::msg::PointCloud2 cloud =
      buildTriggerCloud(Point3{1.0, std::numeric_limits<double>::infinity(), 3.0});

  EXPECT_EQ(cloud.width, 0U);
  EXPECT_TRUE(cloud.data.empty());
}

TEST(LidarDebugPointcloudsTest, OmitsTriggerWhenZIsNotFinite) {
  const sensor_msgs::msg::PointCloud2 cloud =
      buildTriggerCloud(Point3{1.0, 2.0, -std::numeric_limits<double>::infinity()});

  EXPECT_EQ(cloud.width, 0U);
  EXPECT_TRUE(cloud.data.empty());
}

TEST(LidarDebugPointcloudsTest, BuildsEmptyCloudForEmptyProvenance) {
  const std::unordered_map<std::size_t, MemoryCellProvenance> provenance;
  const builtin_interfaces::msg::Time stamp;

  const sensor_msgs::msg::PointCloud2 cloud =
      buildObstacleMemoryTriggerPointCloud(provenance, stamp, "map");

  EXPECT_EQ(cloud.height, 1U);
  EXPECT_EQ(cloud.width, 0U);
  EXPECT_EQ(cloud.point_step, 12U);
  EXPECT_EQ(cloud.row_step, 0U);
  EXPECT_TRUE(cloud.data.empty());
}

TEST(LidarDebugPointcloudsTest, EmitsOnePointPerActiveCell) {
  std::unordered_map<std::size_t, MemoryCellProvenance> provenance;
  MemoryCellProvenance record = makeProvenance(Point3{1.0, 2.0, 3.0}, true);
  record.min_endpoint_z_m = -100.0;
  record.max_endpoint_z_m = 100.0;
  record.accepted_hit_count = 500U;
  provenance.emplace(1U, record);
  const builtin_interfaces::msg::Time stamp;

  const sensor_msgs::msg::PointCloud2 cloud =
      buildObstacleMemoryTriggerPointCloud(provenance, stamp, "map");

  EXPECT_EQ(cloud.width, 1U);
  EXPECT_EQ(cloud.data.size(), 12U);
}

} // namespace
} // namespace drone_city_nav
