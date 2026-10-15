#include "drone_city_nav/lidar_scan_patterns_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace drone_city_nav {
namespace {

// A small organized scan: 60 columns round, 31 rows.
const OrganizedLidarScan3DConfig kConfig{.horizontal_samples = 60U,
                                         .vertical_samples = 31U,
                                         .minimum_range_m = 0.2,
                                         .maximum_range_m = 35.0,
                                         .hit_epsilon_m = 0.05};

// Every beam a hit at `range_m`; returns the scan and its intensities at one.
OrganizedLidarScan3DResult wallScan(const double range_m,
                                    std::vector<float>& intensities) {
  OrganizedLidarScan3DResult scan;
  for (std::size_t row = 0U; row < kConfig.vertical_samples; ++row) {
    for (std::size_t column = 0U; column < kConfig.horizontal_samples; ++column) {
      scan.beams.push_back(LidarBeamSample3D{.direction_lidar_flu = Vec3{1.0, 0.0, 0.0},
                                             .range_m = range_m,
                                             .hit = true,
                                             .valid = true});
      intensities.push_back(1.0F);
    }
  }
  scan.hit_beams = scan.beams.size();
  return scan;
}

std::size_t at(const std::size_t row, const std::size_t column) {
  return row * kConfig.horizontal_samples + column;
}

TEST(LidarScanPatterns, WeakReturnsAmongWeakReturnsAreAnAerosol) {
  std::vector<float> intensities;
  OrganizedLidarScan3DResult scan = wallScan(10.0, intensities);
  // A 5 x 5 patch of weak returns at 4 m: smoke before the wall.
  for (std::size_t row = 10U; row < 15U; ++row) {
    for (std::size_t column = 20U; column < 25U; ++column) {
      scan.beams[at(row, column)].range_m = 4.0;
      intensities[at(row, column)] = 0.1F;
    }
  }
  // One weak return alone at 6 m: a dark surface, not an aerosol.
  intensities[at(25U, 40U)] = 0.1F;
  const LidarScanPatternsResult result = classifyLidarScanPatterns3D(
      scan, intensities, kConfig, LidarScanPatternsConfig{});
  EXPECT_EQ(result.aerosol_beams, 25U);
  EXPECT_EQ(scan.unobservable.size(), 25U);
  EXPECT_FALSE(scan.beams[at(12U, 22U)].hit);
  EXPECT_NEAR(scan.beams[at(12U, 22U)].range_m, 4.0, 1.0e-9);
  EXPECT_TRUE(scan.beams[at(25U, 40U)].hit);
  EXPECT_EQ(scan.hit_beams, scan.beams.size() - 25U);
}

TEST(LidarScanPatterns, AScanOfSmokeAloneIsAnAerosolAtTheBodysEdge) {
  std::vector<float> intensities;
  // Inside a dense plume: every beam returns from the smoke at 0.3 m, at a
  // thin fraction of a matte surface's intensity (tools/smoke_rig.py, 3/m).
  OrganizedLidarScan3DResult scan = wallScan(0.3, intensities);
  std::fill(intensities.begin(), intensities.end(), 0.018F);
  const LidarScanPatternsResult result = classifyLidarScanPatterns3D(
      scan, intensities, kConfig, LidarScanPatternsConfig{});
  EXPECT_EQ(result.aerosol_beams, scan.beams.size());
  EXPECT_EQ(scan.hit_beams, 0U);
  ASSERT_EQ(scan.unobservable.size(), scan.beams.size());
  EXPECT_NEAR(scan.unobservable.front().range_m, 1.2, 1.0e-9);
  EXPECT_NEAR(scan.beams.front().range_m, 1.2, 1.0e-9);
}

TEST(LidarScanPatterns, ANoReturnBundleInAFlatNearWallIsADropout) {
  std::vector<float> intensities;
  OrganizedLidarScan3DResult scan = wallScan(5.0, intensities);
  for (std::size_t row = 10U; row < 16U; ++row) {
    for (std::size_t column = 20U; column < 26U; ++column) {
      scan.beams[at(row, column)].range_m = kConfig.maximum_range_m;
      scan.beams[at(row, column)].hit = false;
    }
  }
  const LidarScanPatternsResult result =
      classifyLidarScanPatterns3D(scan, {}, kConfig, LidarScanPatternsConfig{});
  EXPECT_EQ(result.dropout_bundles, 1U);
  EXPECT_EQ(result.dropout_beams, 36U);
  EXPECT_EQ(scan.unobservable.size(), 36U);
  EXPECT_NEAR(scan.beams[at(12U, 22U)].range_m, 5.0, 1.0e-9);
  EXPECT_FALSE(scan.beams[at(12U, 22U)].hit);
}

TEST(LidarScanPatterns, OpenSpaceAndAnOpeningWhoseWallsRunOnStayFree) {
  std::vector<float> intensities;
  // The upper half of the scan returns nothing: open sky, no border round it.
  OrganizedLidarScan3DResult sky = wallScan(5.0, intensities);
  for (std::size_t row = 16U; row < kConfig.vertical_samples; ++row) {
    for (std::size_t column = 0U; column < kConfig.horizontal_samples; ++column) {
      sky.beams[at(row, column)].range_m = kConfig.maximum_range_m;
      sky.beams[at(row, column)].hit = false;
    }
  }
  EXPECT_EQ(classifyLidarScanPatterns3D(sky, {}, kConfig, LidarScanPatternsConfig{})
                .dropout_bundles,
            0U);
  EXPECT_NEAR(sky.beams[at(20U, 10U)].range_m, kConfig.maximum_range_m, 1.0e-9);
  // A corridor's far end: the bundle's border ranges run from 2 to 30 m.
  intensities.clear();
  OrganizedLidarScan3DResult corridor = wallScan(5.0, intensities);
  for (std::size_t row = 10U; row < 16U; ++row) {
    for (std::size_t column = 20U; column < 26U; ++column) {
      corridor.beams[at(row, column)].range_m = kConfig.maximum_range_m;
      corridor.beams[at(row, column)].hit = false;
    }
  }
  for (std::size_t row = 9U; row <= 16U; ++row) {
    for (std::size_t column = 19U; column <= 26U; ++column) {
      if (corridor.beams[at(row, column)].hit) {
        corridor.beams[at(row, column)].range_m =
            2.0 + 28.0 * static_cast<double>(column - 19U) / 7.0;
      }
    }
  }
  EXPECT_EQ(
      classifyLidarScanPatterns3D(corridor, {}, kConfig, LidarScanPatternsConfig{})
          .dropout_bundles,
      0U);
  EXPECT_NEAR(corridor.beams[at(12U, 22U)].range_m, kConfig.maximum_range_m, 1.0e-9);
}

} // namespace
} // namespace drone_city_nav
