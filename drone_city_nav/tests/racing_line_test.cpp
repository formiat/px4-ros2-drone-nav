#include "drone_city_nav/racing_line.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
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

[[nodiscard]] std::vector<CorridorSample>
straightCorridorWithBlockedCenterline(const double left_bound_m,
                                      const double right_bound_m) {
  std::vector<CorridorSample> samples;
  for (int i = 0; i <= 4; ++i) {
    CorridorSample sample{};
    sample.s_m = static_cast<double>(i) * 5.0;
    sample.center = Point2{sample.s_m, 0.0};
    sample.tangent = Point2{1.0, 0.0};
    sample.normal = Point2{0.0, 1.0};
    sample.left_bound_m = left_bound_m;
    sample.right_bound_m = right_bound_m;
    sample.clearance_m = std::max(left_bound_m, right_bound_m);
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
  config.weight_offset_change = 1.0;
  config.weight_offset_second_change = 10.0;
  config.weight_center_bias = 0.0;
  return config;
}

[[nodiscard]] double maxOffsetDelta(const std::vector<TrajectoryPointSample>& samples) {
  double max_delta = 0.0;
  for (std::size_t i = 1U; i < samples.size(); ++i) {
    max_delta = std::max(max_delta, std::abs(samples[i].racing_offset_m -
                                             samples[i - 1U].racing_offset_m));
  }
  return max_delta;
}

} // namespace

TEST(RacingLine, WideCornerProducesTraversableSmoothLine) {
  const OccupancyGrid2D grid = openGrid();
  const RacingLineResult result =
      optimizeRacingLine(wideLeftTurnCorridor(), grid, testConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.samples.size(), wideLeftTurnCorridor().size());
  EXPECT_LE(result.stats.final_length_m,
            result.stats.centerline_length_m * testConfig().max_length_ratio);
  EXPECT_LE(maxOffsetDelta(result.samples), 2.5);
}

TEST(RacingLine, PenalizesOffsetSpikes) {
  const OccupancyGrid2D grid = openGrid();
  const RacingLineResult result =
      optimizeRacingLine(wideLeftTurnCorridor(), grid, testConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_LE(maxOffsetDelta(result.samples), 2.5);
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

TEST(RacingLine, ProhibitedCenterlineCanUseLateralCorridorSeed) {
  OccupancyGrid2D grid = openGrid();
  for (int x = 28; x <= 32; ++x) {
    grid.setOccupied(GridIndex{x, 20});
  }
  const std::vector<CorridorSample> corridor =
      straightCorridorWithBlockedCenterline(5.0, 1.0);

  const RacingLineResult result = optimizeRacingLine(corridor, grid, testConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.stats.collision_rejections, 0U);
  EXPECT_GT(result.stats.max_abs_offset_m, 0.1);
}

TEST(RacingLine, ProhibitedCenterlineWithoutLateralRoomReturnsInvalidResult) {
  OccupancyGrid2D grid = openGrid();
  for (int x = 28; x <= 32; ++x) {
    grid.setOccupied(GridIndex{x, 20});
  }
  const std::vector<CorridorSample> corridor =
      straightCorridorWithBlockedCenterline(0.0, 0.0);

  const RacingLineResult result = optimizeRacingLine(corridor, grid, testConfig());

  EXPECT_FALSE(result.valid);
  EXPECT_GT(result.stats.collision_rejections, 0U);
}

} // namespace drone_city_nav
