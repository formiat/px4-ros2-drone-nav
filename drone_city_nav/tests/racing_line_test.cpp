#include "drone_city_nav/racing_line.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupancyGrid2D openGrid() {
  OccupancyGrid2D grid{GridBounds{-20.0, -20.0, 1.0, 60, 60}};
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      grid.setFree(GridIndex{x, y});
    }
  }
  return grid;
}

[[nodiscard]] std::vector<CorridorSample> wideLeftTurnCorridor() {
  std::vector<CorridorSample> samples;
  const std::vector<Point2> centers{
      {0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}, {10.0, 5.0}, {10.0, 10.0}};
  const std::vector<Point2> tangents{
      {1.0, 0.0}, {1.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {0.0, 1.0}};
  for (std::size_t i = 0U; i < centers.size(); ++i) {
    CorridorSample sample{};
    sample.s_m = static_cast<double>(i) * 5.0;
    sample.center = centers[i];
    sample.tangent = tangents[i];
    sample.normal = Point2{-tangents[i].y, tangents[i].x};
    sample.left_bound_m = 5.0;
    sample.right_bound_m = 1.0;
    sample.clearance_m = 5.0;
    samples.push_back(sample);
  }
  return samples;
}

[[nodiscard]] RacingLineConfig testConfig() {
  RacingLineConfig config{};
  config.max_iterations = 30U;
  config.initial_offset_step_m = 1.0;
  config.min_offset_step_m = 0.1;
  config.weight_length = 1.0;
  config.weight_curvature = 50.0;
  config.weight_curvature_change = 5.0;
  config.weight_center_bias = 0.0;
  return config;
}

} // namespace

TEST(RacingLine, WideCornerUsesLateralOffset) {
  const OccupancyGrid2D grid = openGrid();
  const RacingLineResult result =
      optimizeRacingLine(wideLeftTurnCorridor(), grid, testConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.samples.size(), wideLeftTurnCorridor().size());
  EXPECT_GT(result.stats.max_abs_offset_m, 0.5);
  EXPECT_LE(result.stats.final_length_m,
            result.stats.centerline_length_m * testConfig().max_length_ratio);
}

TEST(RacingLine, ResultIsDeterministic) {
  const OccupancyGrid2D grid = openGrid();
  const auto corridor = wideLeftTurnCorridor();
  const RacingLineResult first = optimizeRacingLine(corridor, grid, testConfig());
  const RacingLineResult second = optimizeRacingLine(corridor, grid, testConfig());

  ASSERT_TRUE(first.valid);
  ASSERT_TRUE(second.valid);
  ASSERT_EQ(first.samples.size(), second.samples.size());
  for (std::size_t i = 0U; i < first.samples.size(); ++i) {
    EXPECT_DOUBLE_EQ(first.samples[i].point.x, second.samples[i].point.x);
    EXPECT_DOUBLE_EQ(first.samples[i].point.y, second.samples[i].point.y);
  }
}

TEST(RacingLine, ProhibitedCenterlineReturnsInvalidResult) {
  OccupancyGrid2D grid = openGrid();
  for (int y = 15; y < 30; ++y) {
    grid.setOccupied(GridIndex{30, y});
  }
  const std::vector<CorridorSample> corridor = wideLeftTurnCorridor();

  const RacingLineResult result = optimizeRacingLine(corridor, grid, testConfig());

  EXPECT_FALSE(result.valid);
  EXPECT_GT(result.stats.collision_rejections, 0U);
}

} // namespace drone_city_nav
