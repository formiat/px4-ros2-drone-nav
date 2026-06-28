#include "drone_city_nav/corridor.hpp"
#include "drone_city_nav/trajectory_diagnostics.hpp"
#include "drone_city_nav/turn_smoothing.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <span>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupancyGrid2D openGrid() {
  OccupancyGrid2D grid{GridBounds{-30.0, -30.0, 1.0, 120, 120}};
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      grid.setFree(GridIndex{x, y});
    }
  }
  return grid;
}

[[nodiscard]] std::vector<TrajectoryPointSample>
samplesFromCorridor(const std::vector<CorridorSample>& corridor_samples) {
  std::vector<TrajectoryPointSample> samples;
  samples.reserve(corridor_samples.size());
  for (const CorridorSample& corridor : corridor_samples) {
    TrajectoryPointSample sample{};
    sample.s_m = corridor.s_m;
    sample.point = corridor.center;
    sample.tangent = corridor.tangent;
    sample.left_bound_m = corridor.left_bound_m;
    sample.right_bound_m = corridor.right_bound_m;
    sample.racing_offset_m = 0.0;
    samples.push_back(sample);
  }
  return samples;
}

[[nodiscard]] std::vector<CorridorSample>
wideCornerCorridor(const OccupancyGrid2D& grid) {
  const std::vector<Point2> route{{0.0, 0.0}, {40.0, 0.0}, {40.0, 40.0}};
  CorridorConfig config{};
  config.max_radius_m = 28.0;
  config.sample_step_m = 5.0;
  config.safety_margin_m = 0.0;
  config.center_recovery_max_m = 0.0;
  config.lateral_limit_window_m = 0.0;
  const CorridorResult result =
      buildCorridor(std::span<const Point2>{route.data(), route.size()}, grid, config);
  EXPECT_TRUE(result.valid);
  return result.samples;
}

[[nodiscard]] TurnSmoothingConfig smoothingConfig() {
  TurnSmoothingConfig config{};
  config.trigger_heading_delta_rad = 0.6;
  config.trigger_min_radius_m = 14.0;
  config.entry_distance_m = 25.0;
  config.exit_distance_m = 25.0;
  config.sample_step_m = 1.0;
  config.outer_bias_ratio = 0.35;
  config.min_outer_shift_m = 2.0;
  config.max_outer_shift_m = 8.0;
  config.min_corridor_margin_m = 0.5;
  config.max_length_ratio = 1.5;
  config.min_heading_improvement_rad = 0.05;
  config.max_passes = 4U;
  return config;
}

[[nodiscard]] double
maxAcceptedCurvatureJumpAfter(const TrajectoryShapeDiagnostics& before) {
  return std::max(before.max_curvature_jump_1pm + 0.25,
                  before.max_curvature_jump_1pm * 2.0);
}

} // namespace

TEST(TurnSmoothing, SmoothsSingleSharpCornerInsideCorridor) {
  const OccupancyGrid2D grid = openGrid();
  const std::vector<CorridorSample> corridor = wideCornerCorridor(grid);
  const std::vector<TrajectoryPointSample> samples = samplesFromCorridor(corridor);
  const TrajectoryShapeDiagnostics before = computeTrajectoryShapeDiagnostics(samples);

  const TurnSmoothingResult result = smoothTrajectoryTurns(
      std::span<const TrajectoryPointSample>{samples.data(), samples.size()},
      std::span<const CorridorSample>{corridor.data(), corridor.size()}, grid,
      smoothingConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.changed);
  EXPECT_GT(result.stats.attempted_corners, 0U);
  EXPECT_GT(result.stats.candidate_attempts, 0U);
  EXPECT_GT(result.stats.smoothed_corners, 0U);
  EXPECT_TRUE(std::isfinite(result.stats.accepted_entry_distance_m));
  EXPECT_TRUE(std::isfinite(result.stats.accepted_exit_distance_m));
  EXPECT_TRUE(std::isfinite(result.stats.accepted_shift_scale));
  EXPECT_GT(result.samples.size(), samples.size());
  EXPECT_LT(result.stats.max_heading_delta_after_rad,
            result.stats.max_heading_delta_before_rad);
  EXPECT_LE(result.stats.max_heading_delta_after_rad, std::numbers::pi / 2.0);
  EXPECT_LE(result.stats.max_curvature_jump_after_1pm,
            maxAcceptedCurvatureJumpAfter(before));
  EXPECT_LT(computeTrajectoryShapeDiagnostics(result.samples).max_heading_delta_rad,
            before.max_heading_delta_rad);
  EXPECT_EQ(result.stats.rejected_prohibited, 0U);
  EXPECT_EQ(result.stats.rejected_corridor, 0U);
}

TEST(TurnSmoothing, FallsBackWhenWideCandidateTouchesProhibited) {
  OccupancyGrid2D grid = openGrid();
  const std::vector<CorridorSample> corridor = wideCornerCorridor(grid);
  const std::vector<TrajectoryPointSample> samples = samplesFromCorridor(corridor);
  const TrajectoryShapeDiagnostics before = computeTrajectoryShapeDiagnostics(samples);
  const GridIndex obstacle_cell{58, 28};
  ASSERT_TRUE(grid.contains(obstacle_cell));
  grid.setOccupied(obstacle_cell);

  const TurnSmoothingResult result = smoothTrajectoryTurns(
      std::span<const TrajectoryPointSample>{samples.data(), samples.size()},
      std::span<const CorridorSample>{corridor.data(), corridor.size()}, grid,
      smoothingConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.changed);
  EXPECT_GT(result.stats.candidate_attempts, 0U);
  EXPECT_GT(result.stats.smoothed_corners, 0U);
  EXPECT_TRUE(std::isfinite(result.stats.accepted_entry_distance_m));
  EXPECT_EQ(result.stats.rejected_prohibited, 0U);
  EXPECT_LT(result.stats.max_heading_delta_after_rad,
            result.stats.max_heading_delta_before_rad);
  EXPECT_LE(result.stats.max_heading_delta_after_rad, std::numbers::pi / 2.0);
  EXPECT_LE(result.stats.max_curvature_jump_after_1pm,
            maxAcceptedCurvatureJumpAfter(before));
  EXPECT_LT(computeTrajectoryShapeDiagnostics(result.samples).max_heading_delta_rad,
            before.max_heading_delta_rad);
}

TEST(TurnSmoothing, TriesUnifiedFallbackWindowsFromSixtyToFiveMeters) {
  OccupancyGrid2D grid = openGrid();
  const std::vector<CorridorSample> corridor = wideCornerCorridor(grid);
  const std::vector<TrajectoryPointSample> samples = samplesFromCorridor(corridor);
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      grid.setOccupied(GridIndex{x, y});
    }
  }
  TurnSmoothingConfig config = smoothingConfig();
  config.entry_distance_m = 45.0;
  config.exit_distance_m = 45.0;
  config.max_passes = 1U;

  const TurnSmoothingResult result = smoothTrajectoryTurns(
      std::span<const TrajectoryPointSample>{samples.data(), samples.size()},
      std::span<const CorridorSample>{corridor.data(), corridor.size()}, grid, config);

  EXPECT_FALSE(result.changed);
  EXPECT_EQ(result.stats.attempted_corners, 1U);
  EXPECT_EQ(result.stats.candidate_attempts, 48U);
  EXPECT_EQ(result.stats.rejected_prohibited, 1U);
}

} // namespace drone_city_nav
