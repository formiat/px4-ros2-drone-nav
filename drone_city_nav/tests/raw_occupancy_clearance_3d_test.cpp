#include "drone_city_nav/raw_occupancy_clearance_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupancyGrid3D scatteredGrid() {
  OccupancyGrid3D grid{GridBounds3D{
      .origin_x = -4.0,
      .origin_y = -4.0,
      .origin_z = 0.0,
      .resolution_m = 0.25,
      .width_cells = 96,
      .height_cells = 96,
      .depth_cells = 64,
  }};
  // A floor, one wall, and scattered voxels across several chunks, so the
  // nearest box lies in a near chunk for some points and far away for others.
  for (int x = 0; x < 96; ++x) {
    for (int y = 0; y < 96; ++y) {
      grid.setOccupied({x, y, 0});
    }
  }
  for (int y = 0; y < 96; ++y) {
    for (int z = 0; z < 64; ++z) {
      grid.setOccupied({80, y, z});
    }
  }
  std::mt19937 rng{11};
  std::uniform_int_distribution<int> xy(0, 95);
  std::uniform_int_distribution<int> z(1, 63);
  for (int index = 0; index < 400; ++index) {
    grid.setOccupied({xy(rng), xy(rng), z(rng)});
  }
  return grid;
}

// Brute force over every occupied box within the reach, in grid order.
[[nodiscard]] double bruteForceClearance(const OccupancyGrid3D& grid,
                                         const Point3& point, const double cap_m) {
  double best_squared = cap_m * cap_m;
  forEachRawOccupiedVoxelNear3D(
      grid, point, cap_m, nullptr,
      [&](const Point3& box_minimum, const Point3& box_maximum) {
        const auto gap = [](const double minimum, const double maximum,
                            const double coordinate) {
          return std::max({minimum - coordinate, 0.0, coordinate - maximum});
        };
        const double dx = gap(box_minimum.x, box_maximum.x, point.x);
        const double dy = gap(box_minimum.y, box_maximum.y, point.y);
        const double dz = gap(box_minimum.z, box_maximum.z, point.z);
        best_squared = std::min(best_squared, dx * dx + dy * dy + dz * dz);
      });
  return std::sqrt(best_squared);
}

[[nodiscard]] double bruteForceMargin(const OccupancyGrid3D& grid, const Point3& point,
                                      const RawClearanceBody3D& body,
                                      const double cap_m) {
  const double reach_m =
      cap_m + std::max({body.radius_m, body.lower_extent_m, body.upper_extent_m});
  double margin_m = cap_m;
  forEachRawOccupiedVoxelNear3D(
      grid, point, reach_m, nullptr,
      [&](const Point3& box_minimum, const Point3& box_maximum) {
        const auto gap = [](const double minimum, const double maximum,
                            const double coordinate) {
          return std::max({minimum - coordinate, 0.0, coordinate - maximum});
        };
        const double dx = gap(box_minimum.x, box_maximum.x, point.x);
        const double dy = gap(box_minimum.y, box_maximum.y, point.y);
        const double horizontal_gap_m = std::hypot(dx, dy) - body.radius_m;
        const double vertical_gap_m =
            std::max(box_minimum.z - (point.z + body.upper_extent_m),
                     (point.z - body.lower_extent_m) - box_maximum.z);
        margin_m =
            std::min(margin_m, std::max({horizontal_gap_m, vertical_gap_m, 0.0}));
      });
  return margin_m;
}

TEST(RawOccupancyClearance3DTest, NearestFirstSearchMatchesBruteForceClearance) {
  const OccupancyGrid3D grid = scatteredGrid();
  std::mt19937 rng{23};
  std::uniform_real_distribution<double> xy(-3.0, 19.0);
  std::uniform_real_distribution<double> z(0.5, 15.0);
  for (int sample = 0; sample < 300; ++sample) {
    const Point3 point{xy(rng), xy(rng), z(rng)};
    for (const double cap_m : {0.5, 3.0, 6.0}) {
      EXPECT_DOUBLE_EQ(rawEuclideanClearance3D(grid, point, cap_m),
                       bruteForceClearance(grid, point, cap_m))
          << "point " << point.x << ' ' << point.y << ' ' << point.z << " cap "
          << cap_m;
    }
  }
}

TEST(RawOccupancyClearance3DTest, NearestFirstSearchMatchesBruteForceBodyMargin) {
  const OccupancyGrid3D grid = scatteredGrid();
  const RawClearanceBody3D body{
      .radius_m = 0.82, .lower_extent_m = 0.23, .upper_extent_m = 0.35};
  std::mt19937 rng{29};
  std::uniform_real_distribution<double> xy(-3.0, 19.0);
  std::uniform_real_distribution<double> z(0.5, 15.0);
  for (int sample = 0; sample < 300; ++sample) {
    const Point3 point{xy(rng), xy(rng), z(rng)};
    for (const double cap_m : {0.5, 1.5, 6.0}) {
      EXPECT_DOUBLE_EQ(rawBodyInflationMargin3D(grid, point, body, cap_m),
                       bruteForceMargin(grid, point, body, cap_m))
          << "point " << point.x << ' ' << point.y << ' ' << point.z << " cap "
          << cap_m;
    }
  }
}

TEST(RawOccupancyClearance3DTest, OpenSpaceReturnsTheCap) {
  const OccupancyGrid3D grid{GridBounds3D{
      .origin_x = 0.0,
      .origin_y = 0.0,
      .origin_z = 0.0,
      .resolution_m = 0.25,
      .width_cells = 64,
      .height_cells = 64,
      .depth_cells = 64,
  }};
  EXPECT_DOUBLE_EQ(rawEuclideanClearance3D(grid, Point3{8.0, 8.0, 8.0}, 6.0), 6.0);
  EXPECT_DOUBLE_EQ(rawBodyInflationMargin3D(grid, Point3{8.0, 8.0, 8.0},
                                            RawClearanceBody3D{.radius_m = 0.5}, 2.0),
                   2.0);
}

} // namespace
} // namespace drone_city_nav
