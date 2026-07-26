#include "drone_city_nav/receding_horizon_trajectory_planner.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupancyGrid2D freeGrid() {
  OccupancyGrid2D grid{GridBounds{-20.0, -20.0, 0.5, 160, 160}};
  grid.reset(CellState::kFree);
  return grid;
}

void occupyWorldPoint(OccupancyGrid2D& grid, const Point2 point) {
  const std::optional<GridIndex> cell = grid.worldToCell(point);
  ASSERT_TRUE(cell.has_value());
  grid.setOccupied(cell.value());
}

} // namespace

TEST(RecedingHorizonTrajectoryPlanner, SelectsDirectOpenSpaceCandidate) {
  OccupancyGrid2D prohibited = freeGrid();
  const RecedingHorizonTrajectoryPlanner planner;

  const RolloutResult result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {2.0, 0.0},
                                .preferred_target = {50.0, 0.0},
                                .grid = &prohibited});

  ASSERT_FALSE(result.ranked_candidates.empty());
  const RolloutCandidate& candidate = result.ranked_candidates.front();
  EXPECT_NEAR(candidate.heading_offset_rad, 0.0, 1.0e-9);
  EXPECT_GT(candidate.samples.back().point.x, 0.0);
  EXPECT_LE(candidate.samples.back().point.x, 25.0);
  EXPECT_DOUBLE_EQ(candidate.score,
                   candidate.progress_cost + candidate.lateral_deviation_cost +
                       candidate.heading_change_cost + candidate.curvature_cost);
}

TEST(RecedingHorizonTrajectoryPlanner, RejectsProhibitedCandidates) {
  OccupancyGrid2D prohibited = freeGrid();
  for (int y = 0; y < prohibited.height(); ++y) {
    for (int x = 0; x < prohibited.width(); ++x) {
      prohibited.setOccupied(GridIndex{x, y});
    }
  }
  prohibited.setFree(GridIndex{40, 40});
  const RecedingHorizonTrajectoryPlanner planner;

  const RolloutResult result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {2.0, 0.0},
                                .preferred_target = {50.0, 0.0},
                                .grid = &prohibited});

  EXPECT_TRUE(result.ranked_candidates.empty());
  EXPECT_EQ(result.reject_reason, RolloutRejectReason::kNoCandidate);
  EXPECT_GT(result.diagnostics.grid_rejections, 0U);
  ASSERT_TRUE(result.diagnostics.first_grid_rejection.has_value());
  const RolloutGridRejectionDiagnostic first_grid_rejection =
      result.diagnostics.first_grid_rejection.value_or(
          RolloutGridRejectionDiagnostic{});
  EXPECT_EQ(first_grid_rejection.reason, RolloutGridRejectReason::kRawOccupied);
  EXPECT_TRUE(first_grid_rejection.cell.has_value());
}

TEST(RecedingHorizonTrajectoryPlanner, UsesDeterministicTieBreak) {
  OccupancyGrid2D prohibited = freeGrid();
  const RecedingHorizonTrajectoryPlanner planner{
      RolloutPlannerConfig{.heading_samples = 3U, .max_heading_offset_rad = 0.5}};
  const RolloutInput input{.position = {0.0, 0.0},
                           .velocity = {0.0, 0.0},
                           .preferred_target = {30.0, 0.0},
                           .grid = &prohibited};

  const RolloutResult first = planner.plan(input);
  const RolloutResult second = planner.plan(input);

  ASSERT_EQ(first.ranked_candidates.size(), second.ranked_candidates.size());
  for (std::size_t index = 0U; index < first.ranked_candidates.size(); ++index) {
    EXPECT_EQ(first.ranked_candidates[index].deterministic_index,
              second.ranked_candidates[index].deterministic_index);
  }
}

TEST(RecedingHorizonTrajectoryPlanner, RejectsProhibitedObstacleBetweenSparseSamples) {
  OccupancyGrid2D prohibited = freeGrid();
  occupyWorldPoint(prohibited, Point2{1.5, 0.0});
  const RecedingHorizonTrajectoryPlanner planner{RolloutPlannerConfig{
      .horizon_m = 6.0,
      .sample_step_m = 5.0,
      .heading_samples = 1U,
      .speed_samples = 1U,
      .minimum_speed_mps = 2.0,
      .maximum_speed_mps = 2.0,
  }};

  const RolloutResult result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {2.0, 0.0},
                                .preferred_target = {20.0, 0.0},
                                .grid = &prohibited});

  EXPECT_TRUE(result.ranked_candidates.empty());
  EXPECT_GT(result.diagnostics.grid_rejections, 0U);
}

TEST(RecedingHorizonTrajectoryPlanner, FindsLeftOrRightAvoidanceCandidate) {
  OccupancyGrid2D prohibited = freeGrid();
  for (int y_index = -1; y_index <= 1; ++y_index) {
    occupyWorldPoint(prohibited, Point2{5.0, 0.5 * static_cast<double>(y_index)});
  }
  const RecedingHorizonTrajectoryPlanner planner{RolloutPlannerConfig{
      .horizon_m = 12.0,
      .heading_samples = 9U,
      .speed_samples = 1U,
      .minimum_speed_mps = 4.0,
      .maximum_speed_mps = 4.0,
      .maximum_lateral_acceleration_mps2 = 10.0,
  }};

  const RolloutResult result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {4.0, 0.0},
                                .preferred_target = {30.0, 0.0},
                                .grid = &prohibited});

  ASSERT_FALSE(result.ranked_candidates.empty());
  EXPECT_NE(result.ranked_candidates.front().heading_offset_rad, 0.0);
}

TEST(RecedingHorizonTrajectoryPlanner, RejectsSlowTargetTurnThatIsUnsafeAtEntrySpeed) {
  OccupancyGrid2D raw = freeGrid();
  const RecedingHorizonTrajectoryPlanner planner{RolloutPlannerConfig{
      .horizon_m = 20.0,
      .heading_samples = 1U,
      .speed_samples = 1U,
      .minimum_speed_mps = 2.0,
      .maximum_speed_mps = 2.0,
      .maximum_lateral_acceleration_mps2 = 4.0,
  }};

  const RolloutResult result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {10.0, 0.0},
                                .preferred_target = {20.0, 20.0},
                                .grid = &raw});

  EXPECT_TRUE(result.ranked_candidates.empty());
  EXPECT_EQ(result.diagnostics.lateral_acceleration_rejections, 1U);
}

TEST(RecedingHorizonTrajectoryPlanner, RiskTierDominatesDirectCandidateProgress) {
  OccupancyGrid2D raw = freeGrid();
  occupyWorldPoint(raw, Point2{7.0, 2.5});
  const ObstacleRiskField risk = ObstacleRiskField::build(
      raw, {.critical_distance_m = 1.0, .preferred_distance_m = 4.0});
  const RecedingHorizonTrajectoryPlanner planner{RolloutPlannerConfig{
      .horizon_m = 12.0,
      .heading_samples = 9U,
      .speed_samples = 1U,
      .minimum_speed_mps = 4.0,
      .maximum_speed_mps = 4.0,
      .maximum_lateral_acceleration_mps2 = 10.0,
  }};

  const RolloutResult result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {4.0, 0.0},
                                .preferred_target = {30.0, 0.0},
                                .grid = &raw,
                                .risk_field = &risk});

  ASSERT_FALSE(result.ranked_candidates.empty());
  EXPECT_LT(result.ranked_candidates.front().heading_offset_rad, 0.0);
  EXPECT_TRUE(result.ranked_candidates.front().risk.hardValid());
  EXPECT_EQ(result.ranked_candidates.front().risk.worst_tier,
            ObstacleRiskTier::kPreferred);
}

TEST(RecedingHorizonTrajectoryPlanner, EvaluatesOnlyTheRequestedGrid) {
  OccupancyGrid2D prohibited = freeGrid();
  OccupancyGrid2D planning = prohibited;
  occupyWorldPoint(planning, Point2{3.0, 0.0});
  const RecedingHorizonTrajectoryPlanner planner{RolloutPlannerConfig{
      .heading_samples = 1U,
      .speed_samples = 1U,
      .minimum_speed_mps = 2.0,
      .maximum_speed_mps = 2.0,
  }};

  const RolloutResult planning_result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {2.0, 0.0},
                                .preferred_target = {30.0, 0.0},
                                .grid = &planning});
  const RolloutResult prohibited_result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {2.0, 0.0},
                                .preferred_target = {30.0, 0.0},
                                .grid = &prohibited});

  EXPECT_TRUE(planning_result.ranked_candidates.empty());
  EXPECT_FALSE(prohibited_result.ranked_candidates.empty());
}

TEST(RecedingHorizonTrajectoryPlanner, RejectsDynamicLimitViolation) {
  OccupancyGrid2D prohibited = freeGrid();
  const RecedingHorizonTrajectoryPlanner planner{RolloutPlannerConfig{
      .heading_samples = 1U,
      .speed_samples = 1U,
      .minimum_speed_mps = 5.0,
      .maximum_speed_mps = 5.0,
      .maximum_curvature_1pm = 0.01,
  }};

  const RolloutResult result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {5.0, 0.0},
                                .preferred_target = {0.0, 30.0},
                                .grid = &prohibited});

  EXPECT_TRUE(result.ranked_candidates.empty());
  EXPECT_GT(result.diagnostics.dynamic_limit_rejections, 0U);
  EXPECT_GT(result.diagnostics.curvature_rejections, 0U);
  EXPECT_EQ(result.diagnostics.dynamic_limit_rejections,
            result.diagnostics.acceleration_rejections +
                result.diagnostics.curvature_rejections +
                result.diagnostics.lateral_acceleration_rejections);
}

TEST(RecedingHorizonTrajectoryPlanner, StopsAtGoalInsideHorizon) {
  OccupancyGrid2D prohibited = freeGrid();
  const RecedingHorizonTrajectoryPlanner planner;

  const RolloutResult result = planner.plan(RolloutInput{.position = {0.0, 0.0},
                                                         .velocity = {2.0, 0.0},
                                                         .preferred_target = {4.0, 0.0},
                                                         .grid = &prohibited});

  ASSERT_FALSE(result.ranked_candidates.empty());
  EXPECT_NEAR(result.ranked_candidates.front().samples.back().point.x, 4.0, 0.1);
}

TEST(RecedingHorizonTrajectoryPlanner, HandlesZeroVelocityStart) {
  OccupancyGrid2D prohibited = freeGrid();
  const RecedingHorizonTrajectoryPlanner planner;

  const RolloutResult result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {0.0, 0.0},
                                .preferred_target = {20.0, 0.0},
                                .grid = &prohibited});

  ASSERT_FALSE(result.ranked_candidates.empty());
  EXPECT_GT(result.ranked_candidates.front().progress_m, 0.0);
}

TEST(RecedingHorizonTrajectoryPlanner, ExtendsCandidateToMinimumTerminalLength) {
  OccupancyGrid2D prohibited = freeGrid();
  const RecedingHorizonTrajectoryPlanner planner{RolloutPlannerConfig{
      .horizon_m = 25.0,
      .sample_step_m = 1.0,
      .heading_samples = 1U,
      .speed_samples = 1U,
      .horizon_time_s = 3.0,
      .minimum_speed_mps = 2.0,
      .maximum_speed_mps = 2.0,
  }};

  const RolloutResult result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {2.0, 0.0},
                                .preferred_target = {40.0, 0.0},
                                .grid = &prohibited,
                                .minimum_length_m = 15.0});

  ASSERT_FALSE(result.ranked_candidates.empty());
  EXPECT_GE(result.ranked_candidates.front().samples.back().s_m, 15.0);
}

TEST(RecedingHorizonTrajectoryPlanner, RejectsUnsafeTerminalResponseEnvelope) {
  OccupancyGrid2D raw = freeGrid();
  occupyWorldPoint(raw, Point2{8.0, 0.0});
  const ObstacleRiskField risk = ObstacleRiskField::build(
      raw, {.critical_distance_m = 1.0, .preferred_distance_m = 4.0});
  const RecedingHorizonTrajectoryPlanner planner{RolloutPlannerConfig{
      .horizon_m = 6.0,
      .sample_step_m = 1.0,
      .heading_samples = 1U,
      .speed_samples = 1U,
      .minimum_speed_mps = 2.0,
      .maximum_speed_mps = 2.0,
  }};

  const RolloutResult unguarded =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {2.0, 0.0},
                                .preferred_target = {20.0, 0.0},
                                .grid = &raw,
                                .risk_field = &risk});
  const RolloutResult guarded =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {2.0, 0.0},
                                .preferred_target = {20.0, 0.0},
                                .grid = &raw,
                                .risk_field = &risk,
                                .minimum_terminal_clearance_m = 3.0});

  ASSERT_FALSE(unguarded.ranked_candidates.empty());
  EXPECT_TRUE(guarded.ranked_candidates.empty());
  ASSERT_TRUE(guarded.diagnostics.first_grid_rejection.has_value());
  EXPECT_EQ(guarded.diagnostics.first_grid_rejection->reason,
            RolloutGridRejectReason::kTerminalResponseEnvelope);
}

TEST(RecedingHorizonTrajectoryPlanner, RejectsUnsafeTerminalStoppingEnvelope) {
  OccupancyGrid2D raw = freeGrid();
  occupyWorldPoint(raw, Point2{8.0, 0.0});
  const ObstacleRiskField risk = ObstacleRiskField::build(
      raw, {.critical_distance_m = 1.0, .preferred_distance_m = 4.0});
  const RecedingHorizonTrajectoryPlanner planner{RolloutPlannerConfig{
      .horizon_m = 6.0,
      .sample_step_m = 1.0,
      .heading_samples = 1U,
      .speed_samples = 1U,
      .minimum_speed_mps = 2.0,
      .maximum_speed_mps = 2.0,
  }};

  const RolloutResult unguarded =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {2.0, 0.0},
                                .preferred_target = {20.0, 0.0},
                                .grid = &raw,
                                .risk_field = &risk});
  const RolloutResult guarded =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {2.0, 0.0},
                                .preferred_target = {20.0, 0.0},
                                .grid = &raw,
                                .risk_field = &risk,
                                .terminal_braking_deceleration_mps2 = 1.0,
                                .terminal_braking_margin_m = 2.0});

  ASSERT_FALSE(unguarded.ranked_candidates.empty());
  EXPECT_TRUE(guarded.ranked_candidates.empty());
  ASSERT_TRUE(guarded.diagnostics.first_grid_rejection.has_value());
  EXPECT_EQ(guarded.diagnostics.first_grid_rejection->reason,
            RolloutGridRejectReason::kTerminalStoppingEnvelope);
}

TEST(RecedingHorizonTrajectoryPlanner, UsesPerCandidateTerminalStoppingEnvelope) {
  OccupancyGrid2D raw = freeGrid();
  occupyWorldPoint(raw, Point2{11.0, 0.0});
  const ObstacleRiskField risk = ObstacleRiskField::build(
      raw, {.critical_distance_m = 1.0, .preferred_distance_m = 4.0});
  const RecedingHorizonTrajectoryPlanner planner{RolloutPlannerConfig{
      .horizon_m = 6.0,
      .sample_step_m = 1.0,
      .heading_samples = 1U,
      .speed_samples = 2U,
      .horizon_time_s = 3.0,
      .minimum_speed_mps = 2.0,
      .maximum_speed_mps = 10.0,
  }};

  const RolloutResult result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {4.0, 0.0},
                                .preferred_target = {20.0, 0.0},
                                .grid = &raw,
                                .risk_field = &risk,
                                .terminal_braking_deceleration_mps2 = 4.0,
                                .terminal_braking_margin_m = 2.0});

  ASSERT_EQ(result.ranked_candidates.size(), 1U);
  EXPECT_DOUBLE_EQ(result.ranked_candidates.front().target_speed_mps, 2.0);
  EXPECT_NEAR(result.ranked_candidates.front().terminal_stopping_distance_m, 4.0,
              1.0e-9);
  EXPECT_EQ(result.diagnostics.grid_rejections, 1U);
}

TEST(RecedingHorizonTrajectoryPlanner, RejectsVehicleFootprintClearanceViolation) {
  OccupancyGrid2D raw = freeGrid();
  for (int x_index = 2; x_index <= 20; ++x_index) {
    occupyWorldPoint(raw, Point2{0.5 * static_cast<double>(x_index), 0.5});
  }
  const ObstacleRiskField risk = ObstacleRiskField::build(
      raw, {.critical_distance_m = 1.0, .preferred_distance_m = 4.0});
  const RecedingHorizonTrajectoryPlanner planner{RolloutPlannerConfig{
      .horizon_m = 6.0,
      .sample_step_m = 1.0,
      .heading_samples = 1U,
      .speed_samples = 1U,
      .minimum_speed_mps = 2.0,
      .maximum_speed_mps = 2.0,
  }};

  const RolloutResult unguarded =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {2.0, 0.0},
                                .preferred_target = {20.0, 0.0},
                                .grid = &raw,
                                .risk_field = &risk});
  const RolloutResult guarded =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {2.0, 0.0},
                                .preferred_target = {20.0, 0.0},
                                .grid = &raw,
                                .risk_field = &risk,
                                .minimum_path_clearance_m = 0.85});

  ASSERT_FALSE(unguarded.ranked_candidates.empty());
  EXPECT_TRUE(guarded.ranked_candidates.empty());
  ASSERT_TRUE(guarded.diagnostics.first_grid_rejection.has_value());
  EXPECT_EQ(guarded.diagnostics.first_grid_rejection->reason,
            RolloutGridRejectReason::kVehicleClearanceEnvelope);
}

TEST(RecedingHorizonTrajectoryPlanner, RejectsTrackingSweepClearanceViolation) {
  OccupancyGrid2D raw = freeGrid();
  for (int x_index = 2; x_index <= 20; ++x_index) {
    occupyWorldPoint(raw, Point2{0.5 * static_cast<double>(x_index), 2.5});
  }
  const ObstacleRiskField risk = ObstacleRiskField::build(
      raw, {.critical_distance_m = 1.0, .preferred_distance_m = 4.0});
  const RecedingHorizonTrajectoryPlanner planner{RolloutPlannerConfig{
      .horizon_m = 6.0,
      .sample_step_m = 1.0,
      .heading_samples = 1U,
      .speed_samples = 1U,
      .minimum_speed_mps = 2.0,
      .maximum_speed_mps = 2.0,
  }};

  const RolloutResult centerline_only =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {2.0, 0.0},
                                .preferred_target = {20.0, 0.0},
                                .grid = &raw,
                                .risk_field = &risk,
                                .minimum_path_clearance_m = 0.85});
  const RolloutResult swept_envelope =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {2.0, 0.0},
                                .preferred_target = {20.0, 0.0},
                                .grid = &raw,
                                .risk_field = &risk,
                                .minimum_path_clearance_m = 2.85});

  ASSERT_FALSE(centerline_only.ranked_candidates.empty());
  EXPECT_TRUE(swept_envelope.ranked_candidates.empty());
  ASSERT_TRUE(swept_envelope.diagnostics.first_grid_rejection.has_value());
  EXPECT_EQ(swept_envelope.diagnostics.first_grid_rejection->reason,
            RolloutGridRejectReason::kVehicleClearanceEnvelope);
}

TEST(RecedingHorizonTrajectoryPlanner, AllowsClearanceEscapeFromGuardedStart) {
  OccupancyGrid2D raw = freeGrid();
  occupyWorldPoint(raw, Point2{0.0, 0.5});
  const ObstacleRiskField risk = ObstacleRiskField::build(
      raw, {.critical_distance_m = 1.0, .preferred_distance_m = 4.0});
  const RecedingHorizonTrajectoryPlanner planner{RolloutPlannerConfig{
      .horizon_m = 6.0,
      .sample_step_m = 1.0,
      .heading_samples = 1U,
      .speed_samples = 1U,
      .minimum_speed_mps = 2.0,
      .maximum_speed_mps = 2.0,
  }};

  const RolloutResult result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {2.0, 0.0},
                                .preferred_target = {20.0, 0.0},
                                .grid = &raw,
                                .risk_field = &risk,
                                .minimum_path_clearance_m = 0.85});

  ASSERT_FALSE(result.ranked_candidates.empty());
  EXPECT_NEAR(result.ranked_candidates.front().risk.minimum_raw_clearance_m, 0.5,
              1.0e-9);
}

TEST(RecedingHorizonTrajectoryPlanner, StationaryRestartIgnoresReturnVelocity) {
  OccupancyGrid2D prohibited = freeGrid();
  const RecedingHorizonTrajectoryPlanner planner{RolloutPlannerConfig{
      .horizon_m = 20.0,
      .sample_step_m = 1.0,
      .heading_samples = 1U,
      .speed_samples = 1U,
      .horizon_time_s = 3.0,
      .minimum_speed_mps = 2.0,
      .maximum_speed_mps = 2.0,
  }};

  const RolloutResult result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {-4.0, 0.0},
                                .preferred_target = {20.0, 0.0},
                                .grid = &prohibited,
                                .stationary_restart = true});

  ASSERT_FALSE(result.ranked_candidates.empty());
  EXPECT_GT(result.ranked_candidates.front().samples.back().point.x, 0.0);
}

TEST(RecedingHorizonTrajectoryPlanner, StationaryRestartHonorsMinimumLength) {
  OccupancyGrid2D prohibited = freeGrid();
  const RecedingHorizonTrajectoryPlanner planner{RolloutPlannerConfig{
      .horizon_m = 20.0,
      .sample_step_m = 1.0,
      .heading_samples = 1U,
      .speed_samples = 1U,
      .horizon_time_s = 3.0,
      .minimum_speed_mps = 2.0,
      .maximum_speed_mps = 2.0,
  }};

  const RolloutResult result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {-4.0, 0.0},
                                .preferred_target = {20.0, 0.0},
                                .grid = &prohibited,
                                .minimum_length_m = 7.0,
                                .stationary_restart = true});

  ASSERT_FALSE(result.ranked_candidates.empty());
  EXPECT_GE(result.ranked_candidates.front().samples.back().s_m, 7.0);
}

TEST(RecedingHorizonTrajectoryPlanner, StationaryRestartCanEscapeBehindObstacle) {
  OccupancyGrid2D raw = freeGrid();
  for (int y_index = -20; y_index <= 20; ++y_index) {
    occupyWorldPoint(raw, Point2{1.0, 0.5 * static_cast<double>(y_index)});
  }
  const RecedingHorizonTrajectoryPlanner planner{RolloutPlannerConfig{
      .horizon_m = 10.0,
      .sample_step_m = 1.0,
      .heading_samples = 9U,
      .speed_samples = 1U,
      .minimum_speed_mps = 2.0,
      .maximum_speed_mps = 2.0,
  }};

  const RolloutResult result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {0.0, 0.0},
                                .preferred_target = {20.0, 0.0},
                                .grid = &raw,
                                .minimum_length_m = 7.0,
                                .stationary_restart = true});

  ASSERT_FALSE(result.ranked_candidates.empty());
  EXPECT_LT(result.ranked_candidates.front().samples.back().point.x, 0.0);
}

} // namespace drone_city_nav
