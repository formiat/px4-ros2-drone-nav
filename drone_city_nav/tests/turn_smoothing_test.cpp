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
    sample.lateral_offset_m = 0.0;
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
  config.center_recovery_max_m = 0.0;
  config.lateral_limit_window_m = 0.0;
  const CorridorResult result =
      buildCorridor(std::span<const Point2>{route.data(), route.size()}, grid, config);
  EXPECT_TRUE(result.valid);
  return result.samples;
}

[[nodiscard]] Point2 unitDirection(const Point2 start, const Point2 end) {
  const double length = distance(start, end);
  if (!(length > 1.0e-9)) {
    return Point2{1.0, 0.0};
  }
  return Point2{(end.x - start.x) / length, (end.y - start.y) / length};
}

[[nodiscard]] std::vector<CorridorSample>
manualWideCorridor(const std::vector<Point2>& points) {
  std::vector<CorridorSample> corridor;
  corridor.reserve(points.size());
  double station_m = 0.0;
  for (std::size_t i = 0U; i < points.size(); ++i) {
    if (i > 0U) {
      station_m += distance(points[i - 1U], points[i]);
    }
    Point2 tangent{};
    if (points.size() == 1U) {
      tangent = Point2{1.0, 0.0};
    } else if (i == 0U) {
      tangent = unitDirection(points[i], points[i + 1U]);
    } else if (i + 1U == points.size()) {
      tangent = unitDirection(points[i - 1U], points[i]);
    } else {
      tangent = unitDirection(points[i - 1U], points[i + 1U]);
    }
    CorridorSample sample{};
    sample.s_m = station_m;
    sample.route_center = points[i];
    sample.center = points[i];
    sample.tangent = tangent;
    sample.normal = Point2{-tangent.y, tangent.x};
    sample.left_bound_m = 20.0;
    sample.right_bound_m = 20.0;
    sample.clearance_m = 20.0;
    corridor.push_back(sample);
  }
  return corridor;
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
  config.min_heading_improvement_rad = 0.05;
  config.max_passes = 4U;
  return config;
}

[[nodiscard]] double
maxAcceptedCurvatureJumpAfter(const TrajectoryShapeDiagnostics& before) {
  return std::max(before.max_curvature_jump_1pm + 0.02,
                  before.max_curvature_jump_1pm * 1.3);
}

[[nodiscard]] VelocityFollowerConfig speedConfig() {
  VelocityFollowerConfig config{};
  config.cruise_speed_mps = 20.0;
  config.min_turn_speed_mps = 1.5;
  config.max_lateral_accel_mps2 = 5.0;
  return config;
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
      smoothingConfig(), speedConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.changed);
  EXPECT_GT(result.stats.attempted_corners, 0U);
  EXPECT_GT(result.stats.candidate_attempts, 0U);
  EXPECT_GT(result.stats.smoothed_corners, 0U);
  EXPECT_TRUE(std::isfinite(result.stats.accepted_entry_distance_m));
  EXPECT_TRUE(std::isfinite(result.stats.accepted_exit_distance_m));
  EXPECT_TRUE(std::isfinite(result.stats.accepted_shift_scale));
  EXPECT_DOUBLE_EQ(result.stats.accepted_relaxed_angle_deg, 0.0);
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
      smoothingConfig(), speedConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.changed);
  EXPECT_GT(result.stats.candidate_attempts, 0U);
  EXPECT_GT(result.stats.smoothed_corners, 0U);
  EXPECT_TRUE(std::isfinite(result.stats.accepted_entry_distance_m));
  EXPECT_GE(result.stats.accepted_relaxed_angle_deg, 0.0);
  EXPECT_EQ(result.stats.rejected_prohibited, 0U);
  EXPECT_LT(result.stats.max_heading_delta_after_rad,
            result.stats.max_heading_delta_before_rad);
  EXPECT_LE(result.stats.max_heading_delta_after_rad, std::numbers::pi / 2.0);
  EXPECT_LE(result.stats.max_curvature_jump_after_1pm,
            maxAcceptedCurvatureJumpAfter(before));
  EXPECT_LT(computeTrajectoryShapeDiagnostics(result.samples).max_heading_delta_rad,
            before.max_heading_delta_rad);
}

TEST(TurnSmoothing, SmoothsModerateSpeedBottleneckCorner) {
  const OccupancyGrid2D grid = openGrid();
  const double angle_rad = 25.0 * std::numbers::pi / 180.0;
  const std::vector<Point2> points{
      {0.0, 0.0},
      {5.0, 0.0},
      {5.0 + 5.0 * std::cos(angle_rad), 5.0 * std::sin(angle_rad)}};
  const std::vector<CorridorSample> corridor = manualWideCorridor(points);
  const std::vector<TrajectoryPointSample> samples = samplesFromCorridor(corridor);
  TurnSmoothingConfig config = smoothingConfig();
  config.trigger_heading_delta_rad = 37.0 * std::numbers::pi / 180.0;
  config.trigger_min_radius_m = 4.0;
  config.trigger_speed_limit_mps = 12.0;
  config.entry_distance_m = 5.0;
  config.exit_distance_m = 5.0;
  config.max_passes = 1U;

  const TurnSmoothingResult result = smoothTrajectoryTurns(
      std::span<const TrajectoryPointSample>{samples.data(), samples.size()},
      std::span<const CorridorSample>{corridor.data(), corridor.size()}, grid, config,
      speedConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.changed);
  EXPECT_GT(result.stats.detected_corners, 0U);
  EXPECT_GT(result.stats.attempted_corners, 0U);
  EXPECT_GT(result.stats.smoothed_corners, 0U);
}

TEST(TurnSmoothing, AcceptsLocalCornerImprovementWhenGlobalWorstRemains) {
  const OccupancyGrid2D grid = openGrid();
  const std::vector<Point2> points{
      {0.0, 0.0}, {20.0, 0.0}, {20.0, 20.0}, {40.0, 20.0}, {40.0, 40.0}};
  const std::vector<CorridorSample> corridor = manualWideCorridor(points);
  const std::vector<TrajectoryPointSample> samples = samplesFromCorridor(corridor);
  const TrajectoryShapeDiagnostics before = computeTrajectoryShapeDiagnostics(samples);
  TurnSmoothingConfig config = smoothingConfig();
  config.entry_distance_m = 15.0;
  config.exit_distance_m = 15.0;
  config.max_passes = 1U;

  const TurnSmoothingResult result = smoothTrajectoryTurns(
      std::span<const TrajectoryPointSample>{samples.data(), samples.size()},
      std::span<const CorridorSample>{corridor.data(), corridor.size()}, grid, config,
      speedConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.changed);
  EXPECT_EQ(result.stats.smoothed_corners, 1U);
  EXPECT_GT(result.stats.detected_corners, 1U);
  EXPECT_LE(result.stats.max_heading_delta_after_rad,
            before.max_heading_delta_rad + 1.0e-9);
  EXPECT_LT(result.stats.accepted_min_radius_before_m,
            result.stats.accepted_min_radius_after_m);
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
      std::span<const CorridorSample>{corridor.data(), corridor.size()}, grid, config,
      speedConfig());

  EXPECT_FALSE(result.changed);
  EXPECT_EQ(result.stats.attempted_corners, 1U);
  EXPECT_EQ(result.stats.candidate_attempts, 336U);
  EXPECT_EQ(result.stats.relaxed_candidate_attempts, 288U);
  EXPECT_EQ(result.stats.rejected_prohibited, 1U);
}

} // namespace drone_city_nav
