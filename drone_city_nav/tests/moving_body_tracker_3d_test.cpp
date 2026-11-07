#include "drone_city_nav/moving_body_tracker_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr GridBounds3D kBounds{.origin_x = 0.0,
                               .origin_y = 0.0,
                               .origin_z = 0.0,
                               .resolution_m = 0.25,
                               .width_cells = 80,
                               .height_cells = 80,
                               .depth_cells = 20};

// A person's column: two cells wide, seven cells tall, standing at a cell.
std::vector<GridIndex3D> column(const int x, const int y) {
  std::vector<GridIndex3D> cells;
  for (int z = 4; z < 11; ++z) {
    cells.push_back({x, y, z});
    cells.push_back({x + 1, y, z});
  }
  return cells;
}

TEST(MovingBodyTracker3D, TwoColumnsApartAreTwoBodiesOfTheirSize) {
  MovingBodyTracker3D tracker;
  std::vector<GridIndex3D> cells = column(10, 10);
  const std::vector<GridIndex3D> other = column(30, 30);
  cells.insert(cells.end(), other.begin(), other.end());
  // Three stray voxels under the least a body has are no body, nor is a sheet
  // of six a cell thick (the range's noise over a far floor).
  cells.push_back({50, 50, 5});
  cells.push_back({51, 50, 5});
  cells.push_back({52, 50, 5});
  for (int x = 60; x < 66; ++x) {
    cells.push_back({x, 50, 5});
  }
  tracker.update(cells, kBounds, 1000);
  ASSERT_EQ(tracker.bodies().size(), 2U);
  const MovingBody3D& body = tracker.bodies().front();
  EXPECT_EQ(body.voxels, 14U);
  EXPECT_FALSE(body.moving());
  EXPECT_DOUBLE_EQ(body.bottom_m, 1.0);
  EXPECT_DOUBLE_EQ(body.top_m, 2.75);
  EXPECT_NEAR(body.radius_m, 0.125 + 0.125, 1.0e-9);
  EXPECT_DOUBLE_EQ(body.age_s, 0.0);
}

TEST(MovingBodyTracker3D, AWalkingColumnReadsItsSpeedAndAStandingOneNone) {
  MovingBodyTracker3D tracker;
  // 1.2 m/s along x: 0.12 m a scan, a cell every other scan.
  for (int scan = 0; scan <= 20; ++scan) {
    const int x = 10 + static_cast<int>(std::floor(0.12 * scan / 0.25));
    tracker.update(column(x, 10), kBounds,
                   1000 + static_cast<std::int64_t>(scan) * 100);
  }
  ASSERT_EQ(tracker.bodies().size(), 1U);
  const MovingBody3D walking = tracker.bodies().front();
  EXPECT_TRUE(walking.moving());
  EXPECT_NEAR(walking.velocity_map_mps.x, 1.2, 0.5);
  EXPECT_NEAR(walking.velocity_map_mps.y, 0.0, 1.0e-9);
  EXPECT_NEAR(walking.age_s, 2.0, 1.0e-9);
  // The column stops: a few scans later it stands.
  for (int scan = 21; scan <= 40; ++scan) {
    tracker.update(column(19, 10), kBounds,
                   1000 + static_cast<std::int64_t>(scan) * 100);
  }
  ASSERT_EQ(tracker.bodies().size(), 1U);
  EXPECT_FALSE(tracker.bodies().front().moving());
  EXPECT_NEAR(tracker.bodies().front().age_s, 4.0, 1.0e-9);
  EXPECT_EQ(tracker.cells(0).size(), 14U);
  EXPECT_EQ(tracker.previousCells(0).size(), 14U);
  // Gone from the layer, gone from the tracker.
  tracker.update({}, kBounds, 5100);
  EXPECT_TRUE(tracker.bodies().empty());
  EXPECT_TRUE(tracker.cells(0).empty());
}

TEST(MovingBodyTracker3D, ASurfaceComingIntoViewIsNoBodyAndNoStep) {
  MovingBodyTracker3D tracker;
  // A wall patch revealed cell by cell: its centre shifts as it grows, and it
  // is wider than any body once it has grown.
  std::vector<GridIndex3D> patch;
  for (int step = 0; step < 12; ++step) {
    for (int z = 4; z < 8; ++z) {
      patch.push_back({10 + step, 10, z});
    }
    tracker.update(patch, kBounds, 1000 + static_cast<std::int64_t>(step) * 100);
    if (!tracker.bodies().empty()) {
      EXPECT_FALSE(tracker.bodies().front().moving()) << step;
    }
  }
  // Wider than a body: not followed at all.
  EXPECT_TRUE(tracker.bodies().empty());
}

TEST(MovingBodyTracker3D, ASurfacesNoiseIsNoStep) {
  // r3025: a fresh surface's voxels come and go with the range's noise, and
  // the cluster that lost one was read as a body that stepped. Its centre
  // shifts by less than a cell, and that is no step.
  MovingBodyTracker3D tracker;
  for (int scan = 0; scan < 30; ++scan) {
    std::vector<GridIndex3D> patch;
    for (int x = 10; x < 13; ++x) {
      for (int z = 4; z < 8; ++z) {
        if (scan % 2 == 1 && x == 12 && z == 7) {
          continue;
        }
        patch.push_back({x, 10, z});
      }
    }
    tracker.update(patch, kBounds, 1000 + static_cast<std::int64_t>(scan) * 100);
    ASSERT_EQ(tracker.bodies().size(), 1U);
    EXPECT_FALSE(tracker.bodies().front().moving()) << scan;
  }
}

TEST(MovingBodyTracker3D, AColumnFarFromWhereItWasIsANewBody) {
  MovingBodyTracker3D tracker;
  tracker.update(column(10, 10), kBounds, 1000);
  tracker.update(column(30, 10), kBounds, 1100);
  ASSERT_EQ(tracker.bodies().size(), 1U);
  EXPECT_DOUBLE_EQ(tracker.bodies().front().age_s, 0.0);
  EXPECT_FALSE(tracker.bodies().front().moving());
}

TEST(MovingBodyTracker3D, TheSweptCellsLieAheadAlongTheVelocity) {
  MovingBody3D body{.centre_map = {5.0, 5.0, 1.875},
                    .velocity_map_mps = {1.2, 0.0, 0.0},
                    .radius_m = 0.3,
                    .bottom_m = 1.0,
                    .top_m = 2.75,
                    .age_s = 1.0,
                    .voxels = 14U};
  const std::vector<GridIndex3D> cells = sweptCells3D(body, 1.5, kBounds);
  ASSERT_FALSE(cells.empty());
  const auto [x_min, x_max] = std::ranges::minmax(cells, {}, &GridIndex3D::x);
  const auto [z_min, z_max] = std::ranges::minmax(cells, {}, &GridIndex3D::z);
  // From the body's own reach behind its centre to 1.8 m and its reach ahead.
  EXPECT_EQ(x_min.x, 19);
  EXPECT_EQ(x_max.x, 27);
  EXPECT_EQ(z_min.z, 4);
  EXPECT_EQ(z_max.z, 10);
  for (const GridIndex3D cell : cells) {
    EXPECT_TRUE(cell.y == 19 || cell.y == 20) << cell.y;
  }
  body.velocity_map_mps = {};
  EXPECT_TRUE(sweptCells3D(body, 1.5, kBounds).empty());
  body.velocity_map_mps = {1.2, 0.0, 0.0};
  EXPECT_TRUE(sweptCells3D(body, 0.0, kBounds).empty());
}

} // namespace
} // namespace drone_city_nav
