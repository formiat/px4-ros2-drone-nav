#include "drone_city_nav/lidar_debug_pointclouds.hpp"

#include <gtest/gtest.h>

#include <cstring>
#include <limits>
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

TEST(LidarDebugPointcloudsTest, Builds3dCurrentLidarCloudWithTrueEndpointAltitude) {
  const builtin_interfaces::msg::Time stamp;
  const std::vector<Point3> points{Point3{1.5, -2.0, 7.25}};

  const sensor_msgs::msg::PointCloud2 cloud =
      buildLidarDebugPointCloud(points, stamp, "map", true);

  ASSERT_EQ(cloud.width, 1U);
  ASSERT_EQ(cloud.data.size(), 12U);
  EXPECT_FLOAT_EQ(readFloat(cloud.data, 0U), 1.5F);
  EXPECT_FLOAT_EQ(readFloat(cloud.data, 4U), -2.0F);
  EXPECT_FLOAT_EQ(readFloat(cloud.data, 8U), -7.25F);
}

} // namespace
} // namespace drone_city_nav
