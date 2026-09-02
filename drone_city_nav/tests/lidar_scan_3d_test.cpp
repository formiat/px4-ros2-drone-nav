#include "drone_city_nav/lidar_scan_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
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

[[nodiscard]] OrganizedLidarScan3DConfig threeRowScanConfig() {
  OrganizedLidarScan3DConfig config;
  config.horizontal_samples = 1U;
  config.vertical_samples = 3U;
  config.horizontal_min_angle_rad = -0.05;
  config.horizontal_max_angle_rad = 0.05;
  config.vertical_min_angle_rad = -0.17453292519943295;
  config.vertical_max_angle_rad = 0.17453292519943295;
  config.minimum_range_m = 0.2;
  config.maximum_range_m = 35.0;
  return config;
}

TEST(LidarScan3D, JoinsAdjacentReturnsOfOneWallWithSurfaceSamples) {
  const OrganizedLidarScan3DConfig config = threeRowScanConfig();
  // A vertical wall ten metres ahead sampled by the -10, 0 and +10 degree
  // beams: rows 1.76 m apart, the body-height band between them unsampled.
  const double wall_x = 10.0;
  const double row_z = wall_x * std::tan(0.17453292519943295);
  const std::vector<Point3> points{Point3{wall_x, 0.0, -row_z},
                                   Point3{wall_x, 0.0, 0.0},
                                   Point3{wall_x, 0.0, row_z}};
  const OrganizedLidarScan3DResult decoded = decodeOrganizedLidarScan3D(points, config);
  ASSERT_EQ(decoded.hit_beams, 3U);

  const LidarSurfaceInterpolation3DConfig interpolation{.sample_spacing_m = 0.175};
  const std::vector<LidarBeamSample3D> samples =
      interpolateOrganizedLidarSurfaces3D(decoded.beams, config, interpolation);

  // Each row pair spans ten 0.175 m steps, so nine samples strictly between.
  ASSERT_EQ(samples.size(), 20U);
  double previous_z = -row_z;
  for (const LidarBeamSample3D& sample : samples) {
    EXPECT_TRUE(sample.hit);
    EXPECT_TRUE(sample.valid);
    EXPECT_TRUE(sample.interpolated);
    const double x = sample.direction_lidar_flu.x * sample.range_m;
    const double z = sample.direction_lidar_flu.z * sample.range_m;
    EXPECT_NEAR(x, wall_x, 1.0e-6);
    EXPECT_GT(z, -row_z);
    EXPECT_LT(z, row_z);
    EXPECT_GT(z, previous_z - 1.0e-9);
    if (z < previous_z) {
      previous_z = z;
    }
    previous_z = std::max(previous_z, z);
  }
}

TEST(LidarScan3D, DoesNotJoinReturnsWhoseRangeStepExceedsOneSurface) {
  const OrganizedLidarScan3DConfig config = threeRowScanConfig();
  // The -10 degree beam reaches the ground well in front of the wall the
  // horizontal beam hits; the +10 degree beam misses. A ramp between the
  // ground return and the wall return would block free street volume.
  const std::vector<Point3> points{
      Point3{4.0, 0.0, -4.0 * std::tan(0.17453292519943295)}, Point3{10.0, 0.0, 0.0},
      Point3{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}};
  const OrganizedLidarScan3DResult decoded = decodeOrganizedLidarScan3D(points, config);
  ASSERT_EQ(decoded.hit_beams, 2U);

  const std::vector<LidarBeamSample3D> samples = interpolateOrganizedLidarSurfaces3D(
      decoded.beams, config, LidarSurfaceInterpolation3DConfig{});

  EXPECT_TRUE(samples.empty());
}

TEST(LidarScan3D, LeavesNearHorizontalSurfacesToTheBeams) {
  OrganizedLidarScan3DConfig config = threeRowScanConfig();
  config.vertical_min_angle_rad = -1.3962634015954636;
  config.vertical_max_angle_rad = -1.0471975511965976;
  // Flat ground twenty metres below sampled at 80, 70 and 60 degrees of
  // depression: consecutive rings 3.7 m and 4.3 m apart on one surface.
  const double height = 20.0;
  const std::vector<Point3> points{
      Point3{height / std::tan(1.3962634015954636), 0.0, -height},
      Point3{height / std::tan(1.2217304763960306), 0.0, -height},
      Point3{height / std::tan(1.0471975511965976), 0.0, -height}};
  const OrganizedLidarScan3DResult decoded = decodeOrganizedLidarScan3D(points, config);
  ASSERT_EQ(decoded.hit_beams, 3U);

  const std::vector<LidarBeamSample3D> samples = interpolateOrganizedLidarSurfaces3D(
      decoded.beams, config, LidarSurfaceInterpolation3DConfig{});

  EXPECT_TRUE(samples.empty());
}

TEST(LidarScan3D, DoesNotJoinACeilingReturnToTheWallBehindIt) {
  const OrganizedLidarScan3DConfig config = threeRowScanConfig();
  // Inside a passage: the horizontal beam reaches the far wall at 33.5 m, the
  // +10 degree beam meets the ceiling at 26.8 m. The range step is a fifth of
  // the range; a ramp between the two would cross the passage's free volume.
  const double ceiling_range = 26.8;
  const std::vector<Point3> points{
      Point3{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
      Point3{33.5, 0.0, 0.0},
      Point3{ceiling_range * std::cos(0.17453292519943295), 0.0,
             ceiling_range * std::sin(0.17453292519943295)}};
  const OrganizedLidarScan3DResult decoded = decodeOrganizedLidarScan3D(points, config);
  ASSERT_EQ(decoded.hit_beams, 2U);

  const std::vector<LidarBeamSample3D> samples = interpolateOrganizedLidarSurfaces3D(
      decoded.beams, config, LidarSurfaceInterpolation3DConfig{});

  EXPECT_TRUE(samples.empty());
}

TEST(LidarScan3D, JoinsNeighbouringColumnsOfAWallSeenAtGrazingIncidence) {
  OrganizedLidarScan3DConfig config;
  config.horizontal_samples = 2U;
  config.vertical_samples = 1U;
  config.horizontal_min_angle_rad = 0.0;
  config.horizontal_max_angle_rad = 0.02617993877991494;
  config.vertical_min_angle_rad = -0.05;
  config.vertical_max_angle_rad = 0.05;
  config.minimum_range_m = 0.2;
  config.maximum_range_m = 35.0;
  // A wall along the line y = 2 seen from the side: the beam at azimuth 0
  // never meets it, so the wall is placed as x = 20 for the first beam and the
  // second beam, 1.5 degrees further, meets the same plane x = 20 at grazing
  // incidence with a range step well inside the row limit.
  const double azimuth = 0.02617993877991494;
  const std::vector<Point3> points{Point3{20.0, 0.0, 0.0},
                                   Point3{20.0, 20.0 * std::tan(azimuth), 0.0}};
  const OrganizedLidarScan3DResult decoded = decodeOrganizedLidarScan3D(points, config);
  ASSERT_EQ(decoded.hit_beams, 2U);

  const std::vector<LidarBeamSample3D> samples = interpolateOrganizedLidarSurfaces3D(
      decoded.beams, config, LidarSurfaceInterpolation3DConfig{});

  ASSERT_FALSE(samples.empty());
  for (const LidarBeamSample3D& sample : samples) {
    EXPECT_NEAR(sample.direction_lidar_flu.x * sample.range_m, 20.0, 1.0e-6);
    EXPECT_GT(sample.direction_lidar_flu.y * sample.range_m, 0.0);
    EXPECT_LT(sample.direction_lidar_flu.y * sample.range_m, 20.0 * std::tan(azimuth));
  }
}

TEST(LidarScan3D, SurfaceInterpolationCanBeDisabled) {
  const OrganizedLidarScan3DConfig config = threeRowScanConfig();
  const double row_z = 10.0 * std::tan(0.17453292519943295);
  const std::vector<Point3> points{Point3{10.0, 0.0, -row_z}, Point3{10.0, 0.0, 0.0},
                                   Point3{10.0, 0.0, row_z}};
  const OrganizedLidarScan3DResult decoded = decodeOrganizedLidarScan3D(points, config);

  const std::vector<LidarBeamSample3D> samples = interpolateOrganizedLidarSurfaces3D(
      decoded.beams, config, LidarSurfaceInterpolation3DConfig{.enabled = false});

  EXPECT_TRUE(samples.empty());
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
