#include "drone_city_nav/corridor.hpp"
#include "drone_city_nav/trajectory_straightening.hpp"

#include <gtest/gtest.h>

#include <span>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupancyGrid2D openGrid() {
  OccupancyGrid2D grid{GridBounds{-30.0, -30.0, 1.0, 160, 160}};
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      grid.setFree(GridIndex{x, y});
    }
  }
  return grid;
}

[[nodiscard]] std::vector<CorridorSample>
buildOpenCorridor(const std::vector<Point2>& route, const OccupancyGrid2D& grid) {
  CorridorConfig config{};
  config.max_radius_m = 20.0;
  config.sample_step_m = 5.0;
  config.safety_margin_m = 0.0;
  config.center_recovery_max_m = 0.0;
  config.lateral_limit_window_m = 0.0;
  const CorridorResult corridor =
      buildCorridor(std::span<const Point2>{route.data(), route.size()}, grid, config);
  EXPECT_TRUE(corridor.valid);
  return corridor.samples;
}

[[nodiscard]] TrajectoryStraighteningConfig straighteningConfig() {
  TrajectoryStraighteningConfig config{};
  config.min_segment_length_m = 10.0;
  config.validation_step_m = 2.0;
  config.min_corridor_margin_m = 0.5;
  config.max_path_length_ratio = 1.04;
  config.max_heading_error_rad = 0.25;
  return config;
}

} // namespace

TEST(TrajectoryStraightening, CollapsesNearlyStraightWavySection) {
  const OccupancyGrid2D grid = openGrid();
  const std::vector<Point2> route{{0.0, 0.0}, {0.0, 80.0}};
  const std::vector<CorridorSample> corridor = buildOpenCorridor(route, grid);
  const std::vector<Point2> wavy_points{{0.0, 0.0},   {0.6, 10.0},  {-0.4, 20.0},
                                        {0.8, 30.0},  {-0.7, 40.0}, {0.5, 50.0},
                                        {-0.3, 60.0}, {0.0, 80.0}};
  const std::vector<TrajectoryPointSample> samples = trajectoryPointSamplesFromPoints(
      std::span<const Point2>{wavy_points.data(), wavy_points.size()});

  const TrajectoryStraighteningResult result = straightenTrajectory(
      std::span<const TrajectoryPointSample>{samples.data(), samples.size()},
      std::span<const CorridorSample>{corridor.data(), corridor.size()}, grid,
      straighteningConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.changed);
  EXPECT_LT(result.samples.size(), samples.size());
  EXPECT_GT(result.stats.collapsed_segments, 0U);
  EXPECT_EQ(result.stats.rejected_prohibited, 0U);
  EXPECT_EQ(result.stats.rejected_corridor, 0U);
}

TEST(TrajectoryStraightening, DoesNotCollapseSharpCorner) {
  const OccupancyGrid2D grid = openGrid();
  const std::vector<Point2> route{{0.0, 0.0}, {0.0, 40.0}, {40.0, 40.0}};
  const std::vector<CorridorSample> corridor = buildOpenCorridor(route, grid);
  const std::vector<TrajectoryPointSample> samples = trajectoryPointSamplesFromPoints(
      std::span<const Point2>{route.data(), route.size()});

  const TrajectoryStraighteningResult result = straightenTrajectory(
      std::span<const TrajectoryPointSample>{samples.data(), samples.size()},
      std::span<const CorridorSample>{corridor.data(), corridor.size()}, grid,
      straighteningConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.samples.size(), samples.size());
  EXPECT_EQ(result.stats.collapsed_segments, 0U);
  EXPECT_GT(result.stats.rejected_shape, 0U);
}

} // namespace drone_city_nav
