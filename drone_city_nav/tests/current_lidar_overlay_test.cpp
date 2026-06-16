#include "drone_city_nav/current_lidar_overlay.hpp"

#include <gtest/gtest.h>

#include <array>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupancyGrid2D makeOverlayGrid() {
  return OccupancyGrid2D{GridBounds{0.0, -5.0, 1.0, 20, 10}};
}

[[nodiscard]] LidarProjectionPose levelPose() {
  return LidarProjectionPose{Point2{1.5, 0.5}, 10.0, 0.0, 0.0, 0.0, true, true};
}

[[nodiscard]] LidarProjectionConfig overlayConfig() {
  LidarProjectionConfig config{};
  config.max_lidar_range_m = 10.0;
  config.range_hit_epsilon_m = 0.05;
  return config;
}

} // namespace

TEST(CurrentLidarOverlay, AcceptedHitMarksDepthCellsBehindEndpoint) {
  OccupancyGrid2D grid = makeOverlayGrid();
  const std::array<float, 1> ranges{4.0F};
  LidarProjectionConfig config = overlayConfig();

  const CurrentLidarOverlayStats stats = overlayCurrentLidarHits(
      grid, LidarScanView{ranges, 0.1, 10.0, 0.0, 1.0}, levelPose(), config, 2.0);

  EXPECT_TRUE(stats.used);
  EXPECT_EQ(stats.processed_beams, 1U);
  EXPECT_EQ(stats.hit_beams, 1U);
  EXPECT_EQ(stats.outside_hits, 0U);
  EXPECT_TRUE(grid.isOccupied(GridIndex{5, 5}));
  EXPECT_TRUE(grid.isOccupied(GridIndex{6, 5}));
  EXPECT_TRUE(grid.isOccupied(GridIndex{7, 5}));
}

TEST(CurrentLidarOverlay, MaxRangeBeamDoesNotMarkObstacle) {
  OccupancyGrid2D grid = makeOverlayGrid();
  const std::array<float, 1> ranges{10.0F};
  LidarProjectionConfig config = overlayConfig();

  const CurrentLidarOverlayStats stats = overlayCurrentLidarHits(
      grid, LidarScanView{ranges, 0.1, 10.0, 0.0, 1.0}, levelPose(), config, 2.0);

  EXPECT_TRUE(stats.used);
  EXPECT_EQ(stats.processed_beams, 1U);
  EXPECT_EQ(stats.hit_beams, 0U);
  EXPECT_EQ(stats.occupied_cells, 0U);
}

TEST(CurrentLidarOverlay, AltitudeRejectedBeamDoesNotMarkGrid) {
  OccupancyGrid2D grid = makeOverlayGrid();
  const std::array<float, 1> ranges{8.0F};
  LidarProjectionConfig config = overlayConfig();
  config.compensate_attitude = true;
  config.min_projected_altitude_m = 1.0;
  const LidarProjectionPose pose{Point2{1.5, 0.5}, 5.0, 0.0, 0.0, -0.8, true, true};

  const CurrentLidarOverlayStats stats = overlayCurrentLidarHits(
      grid, LidarScanView{ranges, 0.1, 10.0, 0.0, 1.0}, pose, config, 2.0);

  EXPECT_TRUE(stats.used);
  EXPECT_EQ(stats.processed_beams, 1U);
  EXPECT_EQ(stats.hit_beams, 0U);
  EXPECT_EQ(stats.altitude_rejected_beams, 1U);
  EXPECT_EQ(stats.occupied_cells, 0U);
}

} // namespace drone_city_nav
