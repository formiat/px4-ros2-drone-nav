#include "drone_city_nav/lidar_scan_3d.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace drone_city_nav {
namespace {

TEST(LidarScan3D, PreservesNoReturnBeamsAsMisses) {
  OrganizedLidarScan3DConfig config;
  config.horizontal_samples = 4U;
  config.vertical_samples = 2U;
  config.horizontal_min_angle_rad = -1.0;
  config.horizontal_max_angle_rad = 1.0;
  config.vertical_min_angle_rad = -0.5;
  config.vertical_max_angle_rad = 0.5;
  config.minimum_range_m = 0.2;
  config.maximum_range_m = 10.0;
  std::vector<Point3> points(
      8U, Point3{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0});
  points.at(3U) = Point3{3.0, 4.0, 0.0};

  const OrganizedLidarScan3DResult result = decodeOrganizedLidarScan3D(points, config);

  ASSERT_TRUE(result.organized_dimensions_match);
  ASSERT_EQ(result.beams.size(), 8U);
  EXPECT_EQ(result.hit_beams, 1U);
  EXPECT_EQ(result.miss_beams, 7U);
  EXPECT_EQ(result.invalid_beams, 0U);
  EXPECT_TRUE(result.beams.at(3U).hit);
  EXPECT_DOUBLE_EQ(result.beams.at(3U).range_m, 5.0);
  EXPECT_FALSE(result.beams.at(0U).hit);
  EXPECT_TRUE(result.beams.at(0U).valid);
  EXPECT_DOUBLE_EQ(result.beams.at(0U).range_m, 10.0);
}

TEST(LidarScan3D, RejectsUnorganizedCloudInsteadOfDroppingMissGeometry) {
  OrganizedLidarScan3DConfig config;
  config.horizontal_samples = 4U;
  config.vertical_samples = 2U;
  const std::vector points(7U, Point3{1.0, 0.0, 0.0});

  const OrganizedLidarScan3DResult result = decodeOrganizedLidarScan3D(points, config);

  EXPECT_FALSE(result.organized_dimensions_match);
  EXPECT_TRUE(result.beams.empty());
}

TEST(LidarScan3D, ReconstructsThreeDimensionalBeamDirections) {
  OrganizedLidarScan3DConfig config;
  config.horizontal_samples = 3U;
  config.vertical_samples = 3U;
  config.horizontal_min_angle_rad = -0.5;
  config.horizontal_max_angle_rad = 0.5;
  config.vertical_min_angle_rad = -0.5;
  config.vertical_max_angle_rad = 0.5;
  config.maximum_range_m = 10.0;
  const std::vector points(9U,
                           Point3{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0});

  const OrganizedLidarScan3DResult result = decodeOrganizedLidarScan3D(points, config);

  ASSERT_EQ(result.beams.size(), 9U);
  EXPECT_LT(result.beams.front().direction_lidar_flu.z, 0.0);
  EXPECT_NEAR(result.beams.at(4U).direction_lidar_flu.x, 1.0, 1.0e-9);
  EXPECT_NEAR(result.beams.at(4U).direction_lidar_flu.y, 0.0, 1.0e-9);
  EXPECT_NEAR(result.beams.at(4U).direction_lidar_flu.z, 0.0, 1.0e-9);
  EXPECT_GT(result.beams.back().direction_lidar_flu.z, 0.0);
}

} // namespace
} // namespace drone_city_nav
