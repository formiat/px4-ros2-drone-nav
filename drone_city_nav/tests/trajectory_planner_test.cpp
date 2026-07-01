#include "drone_city_nav/clearance_field.hpp"
#include "drone_city_nav/trajectory_planner.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <span>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupancyGrid2D testGrid() {
  OccupancyGrid2D grid{GridBounds{-20.0, -20.0, 1.0, 80, 80}};
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      grid.setFree(GridIndex{x, y});
    }
  }
  return grid;
}

[[nodiscard]] TrajectoryPlannerConfig testConfig() {
  TrajectoryPlannerConfig config{};
  config.corridor.max_radius_m = 10.0;
  config.corridor.sample_step_m = 2.5;
  config.racing_line.max_iterations = 20U;
  config.racing_line.initial_offset_step_m = 1.0;
  config.racing_line.min_offset_step_m = 0.1;
  config.racing_line.weight_curvature = 30.0;
  config.debug_sample_step_m = 1.0;
  config.speed_profile.cruise_speed_mps = 12.0;
  config.speed_profile.max_lateral_accel_mps2 = 3.0;
  config.speed_profile.speed_profile_decel_mps2 = 4.0;
  config.speed_profile.speed_profile_sample_step_m = 1.0;
  return config;
}

[[nodiscard]] std::vector<Point2>
samplePoints(const std::span<const TrajectoryPointSample> samples) {
  std::vector<Point2> points;
  points.reserve(samples.size());
  for (const TrajectoryPointSample& sample : samples) {
    points.push_back(sample.point);
  }
  return points;
}

} // namespace

TEST(TrajectoryPlanner, EmptyRouteIsInvalid) {
  const TrajectoryPlannerResult result =
      planTrajectory(TrajectoryPlannerInput{}, testConfig());

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.stats.status, TrajectoryPlannerStatus::kInvalidRoute);
}

TEST(TrajectoryPlanner, MissingGridIsInvalid) {
  const std::vector<Point2> route{{0.0, 0.0}, {10.0, 0.0}};

  const TrajectoryPlannerResult result = planTrajectory(
      TrajectoryPlannerInput{std::span<const Point2>{route.data(), route.size()},
                             nullptr, nullptr, false, std::span<const CorridorSample>{},
                             nullptr},
      testConfig());

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.stats.status, TrajectoryPlannerStatus::kMissingGrid);
  EXPECT_TRUE(result.compact_segments.empty());
  EXPECT_TRUE(result.samples.empty());
  EXPECT_FALSE(result.speed_profile.valid);
}

TEST(TrajectoryPlanner, MissingGridDoesNotBuildUnknownCorners) {
  const std::vector<Point2> route{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};

  const TrajectoryPlannerResult result = planTrajectory(
      TrajectoryPlannerInput{std::span<const Point2>{route.data(), route.size()},
                             nullptr, nullptr, false, std::span<const CorridorSample>{},
                             nullptr},
      testConfig());

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.stats.status, TrajectoryPlannerStatus::kMissingGrid);
  EXPECT_EQ(result.stats.arc_segments, 0U);
  EXPECT_TRUE(result.compact_segments.empty());
}

TEST(TrajectoryPlanner, RacingTrajectoryProducesSamplesAndSpeedProfile) {
  const OccupancyGrid2D grid = testGrid();
  const std::vector<Point2> route{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};

  const TrajectoryPlannerResult result = planTrajectory(
      TrajectoryPlannerInput{std::span<const Point2>{route.data(), route.size()}, &grid,
                             nullptr, false, std::span<const CorridorSample>{},
                             nullptr},
      testConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.stats.status, TrajectoryPlannerStatus::kOk);
  EXPECT_EQ(result.stats.quality, TrajectoryQuality::kRefined);
  EXPECT_GE(result.samples.size(), 3U);
  EXPECT_EQ(result.corridor_samples.size(), result.stats.corridor.samples);
  EXPECT_TRUE(result.speed_profile.valid);
  EXPECT_GT(result.stats.corridor.samples, 0U);
  EXPECT_GT(result.stats.racing_line.candidate_evaluations, 0U);
  EXPECT_GT(result.stats.racing_line.active_window_count, 0U);
  EXPECT_GT(result.stats.racing_line.dp_states, 0U);
  EXPECT_TRUE(std::isfinite(result.stats.racing_line.estimated_time_s));
  EXPECT_TRUE(std::isfinite(result.stats.racing_line.centerline_estimated_time_s));
  EXPECT_TRUE(std::isfinite(result.stats.racing_line.time_gain_s));
}

TEST(TrajectoryPlanner, BaselineTrajectoryProducesSamplesAndSpeedProfile) {
  const OccupancyGrid2D grid = testGrid();
  const std::vector<Point2> route{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};

  const TrajectoryPlannerResult result = planBaselineTrajectory(
      TrajectoryPlannerInput{std::span<const Point2>{route.data(), route.size()}, &grid,
                             nullptr, false, std::span<const CorridorSample>{},
                             nullptr},
      testConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.stats.status, TrajectoryPlannerStatus::kOk);
  EXPECT_EQ(result.stats.quality, TrajectoryQuality::kBaseline);
  EXPECT_GE(result.samples.size(), 3U);
  EXPECT_TRUE(result.speed_profile.valid);
  EXPECT_EQ(result.stats.racing_line.candidate_evaluations, 0U);
  EXPECT_FALSE(result.stats.racing_line.async_refined);
  ASSERT_FALSE(result.samples.empty());
  EXPECT_NEAR(distance(result.samples.back().point, route.back()), 0.0, 1.0e-6);
}

TEST(TrajectoryPlanner, ReusesProvidedClearanceFieldForCorridor) {
  const OccupancyGrid2D grid = testGrid();
  const std::vector<Point2> route{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};
  const TrajectoryPlannerConfig config = testConfig();
  const ClearanceField2D clearance_field = ClearanceField2D::build(
      grid, config.corridor.max_radius_m, ClearanceSource::kProhibited);

  const TrajectoryPlannerResult result = planTrajectory(
      TrajectoryPlannerInput{std::span<const Point2>{route.data(), route.size()}, &grid,
                             &clearance_field, true, std::span<const CorridorSample>{},
                             nullptr},
      config);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.stats.corridor.clearance_field_reused);
  EXPECT_TRUE(result.stats.corridor.clearance_field_cache_hit);
  EXPECT_FALSE(result.stats.corridor.samples_reused);
  EXPECT_EQ(result.stats.corridor.reused_samples, 0U);
}

TEST(TrajectoryPlanner, ReusesPrecomputedCorridorForRacingTrajectory) {
  const OccupancyGrid2D grid = testGrid();
  const std::vector<Point2> route{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};
  const TrajectoryPlannerConfig config = testConfig();
  const TrajectoryPlannerInput input{
      std::span<const Point2>{route.data(), route.size()},
      &grid,
      nullptr,
      false,
      std::span<const CorridorSample>{},
      nullptr};
  const TrajectoryPlannerResult baseline = planBaselineTrajectory(input, config);
  ASSERT_TRUE(baseline.valid);
  ASSERT_GE(baseline.corridor_samples.size(), 2U);

  const TrajectoryPlannerResult normal = planRacingTrajectory(input, config);
  const TrajectoryPlannerResult reused = planRacingTrajectory(
      TrajectoryPlannerInput{
          std::span<const Point2>{route.data(), route.size()},
          &grid,
          nullptr,
          false,
          std::span<const CorridorSample>{baseline.corridor_samples.data(),
                                          baseline.corridor_samples.size()},
          &baseline.stats.corridor,
      },
      config);

  ASSERT_TRUE(normal.valid);
  ASSERT_TRUE(reused.valid);
  EXPECT_TRUE(reused.stats.corridor.samples_reused);
  EXPECT_EQ(reused.stats.corridor.reused_samples, baseline.corridor_samples.size());
  EXPECT_EQ(reused.stats.corridor.sample_build_duration_ms, 0.0);
  EXPECT_EQ(reused.stats.corridor.raycast_duration_ms, 0.0);
  EXPECT_EQ(reused.stats.corridor.lateral_limit_duration_ms, 0.0);
  EXPECT_EQ(reused.stats.corridor.clearance_field_build_duration_ms, 0.0);
  EXPECT_EQ(reused.corridor_samples.size(), baseline.corridor_samples.size());
  ASSERT_EQ(normal.samples.size(), reused.samples.size());
  for (std::size_t i = 0U; i < normal.samples.size(); ++i) {
    EXPECT_DOUBLE_EQ(normal.samples[i].point.x, reused.samples[i].point.x);
    EXPECT_DOUBLE_EQ(normal.samples[i].point.y, reused.samples[i].point.y);
    EXPECT_DOUBLE_EQ(normal.samples[i].racing_offset_m,
                     reused.samples[i].racing_offset_m);
  }
}

TEST(TrajectoryPlanner, EmptyPrecomputedCorridorFallsBackToBuild) {
  const OccupancyGrid2D grid = testGrid();
  const std::vector<Point2> route{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};

  const TrajectoryPlannerResult result = planRacingTrajectory(
      TrajectoryPlannerInput{
          std::span<const Point2>{route.data(), route.size()},
          &grid,
          nullptr,
          false,
          std::span<const CorridorSample>{},
          nullptr,
      },
      testConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.stats.corridor.samples_reused);
  EXPECT_EQ(result.stats.corridor.reused_samples, 0U);
  EXPECT_GT(result.stats.corridor.samples, 0U);
}

TEST(TrajectoryPlanner, MismatchedPrecomputedCorridorFallsBackToBuild) {
  const OccupancyGrid2D grid = testGrid();
  const std::vector<Point2> route{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};
  const std::vector<Point2> shifted_route{{1.0, 0.0}, {11.0, 0.0}, {11.0, 10.0}};
  const TrajectoryPlannerConfig config = testConfig();
  const TrajectoryPlannerInput input{
      std::span<const Point2>{route.data(), route.size()},
      &grid,
      nullptr,
      false,
      std::span<const CorridorSample>{},
      nullptr};
  const TrajectoryPlannerResult baseline = planBaselineTrajectory(input, config);
  ASSERT_TRUE(baseline.valid);
  ASSERT_GE(baseline.corridor_samples.size(), 2U);

  const TrajectoryPlannerResult result = planRacingTrajectory(
      TrajectoryPlannerInput{
          std::span<const Point2>{shifted_route.data(), shifted_route.size()},
          &grid,
          nullptr,
          false,
          std::span<const CorridorSample>{baseline.corridor_samples.data(),
                                          baseline.corridor_samples.size()},
          &baseline.stats.corridor,
      },
      config);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.stats.corridor.samples_reused);
  EXPECT_EQ(result.stats.corridor.reused_samples, 0U);
  EXPECT_GT(result.stats.corridor.sample_build_duration_ms, 0.0);
}

TEST(TrajectoryPlanner, RejectsStaleRefinedTrajectory) {
  const OccupancyGrid2D grid = testGrid();
  const std::vector<Point2> route{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};
  TrajectoryPlannerResult refined = planRacingTrajectory(
      TrajectoryPlannerInput{std::span<const Point2>{route.data(), route.size()}, &grid,
                             nullptr, false, std::span<const CorridorSample>{},
                             nullptr},
      testConfig());
  ASSERT_TRUE(refined.valid);
  const std::vector<Point2> refined_points = samplePoints(refined.samples);

  const TrajectoryRefinementDecision decision =
      evaluateTrajectoryRefinement(TrajectoryRefinementDecisionInput{
          .current_generation = 2U,
          .snapshot_generation = 1U,
          .expected_start = refined_points.front(),
          .expected_goal = refined_points.back(),
          .endpoint_tolerance_m = 0.5,
          .baseline_estimated_time_s = std::numeric_limits<double>::quiet_NaN(),
          .baseline_length_m = std::numeric_limits<double>::quiet_NaN(),
          .refined = &refined,
          .refined_points =
              std::span<const Point2>{refined_points.data(), refined_points.size()},
          .validation_grid = &grid,
      });

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.reason, TrajectoryRefinementDecisionReason::kStaleGeneration);
}

TEST(TrajectoryPlanner, RejectsInvalidRefinedTrajectory) {
  const OccupancyGrid2D grid = testGrid();
  TrajectoryPlannerResult refined;
  refined.valid = false;
  const std::vector<Point2> refined_points{{0.0, 0.0}, {10.0, 0.0}};

  const TrajectoryRefinementDecision decision =
      evaluateTrajectoryRefinement(TrajectoryRefinementDecisionInput{
          .current_generation = 1U,
          .snapshot_generation = 1U,
          .expected_start = refined_points.front(),
          .expected_goal = refined_points.back(),
          .endpoint_tolerance_m = 0.5,
          .baseline_estimated_time_s = std::numeric_limits<double>::quiet_NaN(),
          .baseline_length_m = std::numeric_limits<double>::quiet_NaN(),
          .refined = &refined,
          .refined_points =
              std::span<const Point2>{refined_points.data(), refined_points.size()},
          .validation_grid = &grid,
      });

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.reason, TrajectoryRefinementDecisionReason::kInvalidRefined);
}

TEST(TrajectoryPlanner, AcceptsValidRefinedTrajectoryAndPreservesGoalEndpoint) {
  const OccupancyGrid2D grid = testGrid();
  const std::vector<Point2> route{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};
  const TrajectoryPlannerConfig config = testConfig();
  const TrajectoryPlannerResult baseline = planBaselineTrajectory(
      TrajectoryPlannerInput{std::span<const Point2>{route.data(), route.size()}, &grid,
                             nullptr, false, std::span<const CorridorSample>{},
                             nullptr},
      config);
  TrajectoryPlannerResult refined = planRacingTrajectory(
      TrajectoryPlannerInput{std::span<const Point2>{route.data(), route.size()}, &grid,
                             nullptr, false, std::span<const CorridorSample>{},
                             nullptr},
      config);
  refined.stats.racing_line.async_refined = true;
  ASSERT_TRUE(baseline.valid);
  ASSERT_TRUE(refined.valid);
  const std::vector<Point2> refined_points = samplePoints(refined.samples);

  const TrajectoryRefinementDecision decision =
      evaluateTrajectoryRefinement(TrajectoryRefinementDecisionInput{
          .current_generation = 1U,
          .snapshot_generation = 1U,
          .expected_start = route.front(),
          .expected_goal = route.back(),
          .endpoint_tolerance_m = 0.5,
          .max_time_regression_s = 3600.0,
          .max_length_regression_ratio = 10.0,
          .baseline_estimated_time_s = baseline.stats.racing_line.estimated_time_s,
          .baseline_length_m = baseline.stats.length_m,
          .refined = &refined,
          .refined_points =
              std::span<const Point2>{refined_points.data(), refined_points.size()},
          .validation_grid = &grid,
      });

  EXPECT_TRUE(decision.accepted);
  EXPECT_EQ(decision.reason, TrajectoryRefinementDecisionReason::kAccepted);
  ASSERT_FALSE(refined_points.empty());
  EXPECT_NEAR(distance(refined_points.back(), route.back()), 0.0, 0.5);
  EXPECT_TRUE(refined.stats.racing_line.async_refined);
}

} // namespace drone_city_nav
