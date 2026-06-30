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
  return config;
}

[[nodiscard]] VelocityFollowerConfig speedConfig() {
  VelocityFollowerConfig config{};
  config.cruise_speed_mps = 12.0;
  config.min_turn_speed_mps = 2.0;
  config.max_lateral_accel_mps2 = 3.0;
  config.speed_profile_decel_mps2 = 4.0;
  config.speed_profile_sample_step_m = 1.0;
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
      optimizeRacingLine(wideLeftTurnCorridor(), grid, testConfig(), speedConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.samples.size(), wideLeftTurnCorridor().size());
  EXPECT_LE(result.stats.final_length_m,
            result.stats.centerline_length_m * testConfig().max_length_ratio);
  EXPECT_LE(maxOffsetDelta(result.samples), 2.5);
  EXPECT_GT(result.stats.active_window_count, 0U);
  EXPECT_GT(result.stats.dp_states, 0U);
  EXPECT_GT(result.stats.dp_transitions, 0U);
}

TEST(RacingLine, PenalizesOffsetSpikes) {
  const OccupancyGrid2D grid = openGrid();
  const RacingLineResult result =
      optimizeRacingLine(wideLeftTurnCorridor(), grid, testConfig(), speedConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_LE(maxOffsetDelta(result.samples), 2.5);
}

TEST(RacingLine, ResultIsDeterministic) {
  const OccupancyGrid2D grid = openGrid();
  const auto corridor = wideLeftTurnCorridor();
  const RacingLineResult first =
      optimizeRacingLine(corridor, grid, testConfig(), speedConfig());
  const RacingLineResult second =
      optimizeRacingLine(corridor, grid, testConfig(), speedConfig());

  ASSERT_TRUE(first.valid);
  ASSERT_TRUE(second.valid);
  ASSERT_EQ(first.samples.size(), second.samples.size());
  for (std::size_t i = 0U; i < first.samples.size(); ++i) {
    EXPECT_DOUBLE_EQ(first.samples[i].point.x, second.samples[i].point.x);
    EXPECT_DOUBLE_EQ(first.samples[i].point.y, second.samples[i].point.y);
  }
  EXPECT_EQ(first.stats.candidate_evaluations, second.stats.candidate_evaluations);
  EXPECT_EQ(first.stats.collision_rejections, second.stats.collision_rejections);
  EXPECT_EQ(first.stats.skipped_noop_candidates, second.stats.skipped_noop_candidates);
  EXPECT_DOUBLE_EQ(first.stats.final_cost, second.stats.final_cost);
  EXPECT_DOUBLE_EQ(first.stats.final_length_m, second.stats.final_length_m);
  EXPECT_DOUBLE_EQ(first.stats.estimated_time_s, second.stats.estimated_time_s);
  EXPECT_TRUE(first.stats.parallel_candidate_evaluation_used);
  EXPECT_EQ(first.stats.active_window_count, second.stats.active_window_count);
  EXPECT_EQ(first.stats.dp_states, second.stats.dp_states);
  EXPECT_EQ(first.stats.dp_transitions, second.stats.dp_transitions);
}

TEST(RacingLine, DefaultParallelCandidateEvaluationMatchesSingleWorkerResult) {
  const OccupancyGrid2D grid = openGrid();
  const auto corridor = wideLeftTurnCorridor();
  RacingLineConfig single_worker_config = testConfig();
  single_worker_config.parallel_workers = 1U;
  RacingLineConfig parallel_config = testConfig();
  parallel_config.parallel_workers = 2U;

  const RacingLineResult sequential =
      optimizeRacingLine(corridor, grid, single_worker_config, speedConfig());
  const RacingLineResult parallel =
      optimizeRacingLine(corridor, grid, parallel_config, speedConfig());

  ASSERT_TRUE(sequential.valid);
  ASSERT_TRUE(parallel.valid);
  ASSERT_EQ(sequential.samples.size(), parallel.samples.size());
  for (std::size_t i = 0U; i < sequential.samples.size(); ++i) {
    EXPECT_DOUBLE_EQ(sequential.samples[i].point.x, parallel.samples[i].point.x);
    EXPECT_DOUBLE_EQ(sequential.samples[i].point.y, parallel.samples[i].point.y);
    EXPECT_DOUBLE_EQ(sequential.samples[i].racing_offset_m,
                     parallel.samples[i].racing_offset_m);
  }
  EXPECT_TRUE(parallel.stats.parallel_candidate_evaluation_used);
  EXPECT_EQ(parallel.stats.parallel_workers_used, 2U);
  EXPECT_GT(parallel.stats.candidate_chunks, 0U);
  EXPECT_GT(parallel.stats.worker_scratch_reuses, 0U);
  EXPECT_GT(parallel.stats.candidate_snapshot_allocations_avoided, 0U);
  EXPECT_EQ(sequential.stats.candidate_evaluations,
            parallel.stats.candidate_evaluations);
  EXPECT_EQ(sequential.stats.collision_rejections, parallel.stats.collision_rejections);
  EXPECT_EQ(sequential.stats.skipped_noop_candidates,
            parallel.stats.skipped_noop_candidates);
  EXPECT_DOUBLE_EQ(sequential.stats.final_cost, parallel.stats.final_cost);
  EXPECT_DOUBLE_EQ(sequential.stats.final_length_m, parallel.stats.final_length_m);
  EXPECT_DOUBLE_EQ(sequential.stats.estimated_time_s, parallel.stats.estimated_time_s);
}

TEST(RacingLine, ProhibitedCenterlineCanUseLateralCorridorSeed) {
  OccupancyGrid2D grid = openGrid();
  for (int x = 28; x <= 32; ++x) {
    grid.setOccupied(GridIndex{x, 20});
  }
  const std::vector<CorridorSample> corridor =
      straightCorridorWithBlockedCenterline(5.0, 1.0);

  const RacingLineResult result =
      optimizeRacingLine(corridor, grid, testConfig(), speedConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.stats.collision_rejections, 0U);
  EXPECT_GT(result.stats.max_abs_offset_m, 0.05);
  EXPECT_EQ(result.stats.active_window_count, 1U);
  EXPECT_GT(result.stats.dp_states, 0U);
}

TEST(RacingLine, ProhibitedCenterlineWithoutLateralRoomReturnsInvalidResult) {
  OccupancyGrid2D grid = openGrid();
  for (int x = 28; x <= 32; ++x) {
    grid.setOccupied(GridIndex{x, 20});
  }
  const std::vector<CorridorSample> corridor =
      straightCorridorWithBlockedCenterline(0.0, 0.0);

  const RacingLineResult result =
      optimizeRacingLine(corridor, grid, testConfig(), speedConfig());

  EXPECT_FALSE(result.valid);
  EXPECT_GT(result.stats.collision_rejections, 0U);
}

TEST(RacingLine, OptimizerSampleStepUsesCoarseCorridor) {
  const OccupancyGrid2D grid = openGrid();
  std::vector<CorridorSample> corridor;
  for (int i = 0; i <= 20; ++i) {
    CorridorSample sample{};
    sample.s_m = static_cast<double>(i);
    sample.center = Point2{sample.s_m, 0.0};
    sample.tangent = Point2{1.0, 0.0};
    sample.normal = Point2{0.0, 1.0};
    sample.left_bound_m = 4.0;
    sample.right_bound_m = 4.0;
    sample.clearance_m = 4.0;
    corridor.push_back(sample);
  }
  RacingLineConfig config = testConfig();
  config.optimizer_sample_step_m = 5.0;

  const RacingLineResult result =
      optimizeRacingLine(corridor, grid, config, speedConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.stats.input_samples, corridor.size());
  EXPECT_LT(result.stats.optimizer_samples, result.stats.input_samples);
  EXPECT_EQ(result.samples.size(), result.stats.optimizer_samples);
  EXPECT_EQ(result.samples.front().point.x, corridor.front().center.x);
  EXPECT_EQ(result.samples.back().point.x, corridor.back().center.x);
}

TEST(RacingLine, StraightOpenCorridorSkipsWindowOptimization) {
  const OccupancyGrid2D grid = openGrid();
  const std::vector<CorridorSample> corridor =
      straightCorridorWithBlockedCenterline(4.0, 4.0);

  const RacingLineResult result =
      optimizeRacingLine(corridor, grid, testConfig(), speedConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.stats.window_count, 0U);
  EXPECT_EQ(result.stats.active_window_count, 0U);
  EXPECT_EQ(result.stats.active_window_samples, 0U);
  EXPECT_EQ(result.stats.dp_states, 0U);
  EXPECT_EQ(result.stats.dp_transitions, 0U);
}

TEST(RacingLine, ReportsTraversalTimeAndRegularizationStats) {
  const OccupancyGrid2D grid = openGrid();
  RacingLineConfig config = testConfig();
  config.weight_time = 0.1;
  config.regularization_iterations = 2U;

  const RacingLineResult result =
      optimizeRacingLine(wideLeftTurnCorridor(), grid, config, speedConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(std::isfinite(result.stats.estimated_time_s));
  EXPECT_TRUE(std::isfinite(result.stats.centerline_estimated_time_s));
  EXPECT_TRUE(std::isfinite(result.stats.best_candidate_estimated_time_s));
  EXPECT_TRUE(std::isfinite(result.stats.best_candidate_score));
  EXPECT_TRUE(std::isfinite(result.stats.time_gain_s));
  EXPECT_TRUE(std::isfinite(result.stats.pre_regularization_max_curvature_jump_1pm));
  EXPECT_TRUE(std::isfinite(result.stats.post_regularization_max_curvature_jump_1pm));
  EXPECT_GE(result.stats.candidate_point_build_duration_ms, 0.0);
  EXPECT_GE(result.stats.candidate_sample_build_duration_ms, 0.0);
  EXPECT_GE(result.stats.regularization_duration_ms, 0.0);
}

TEST(RacingLine, ReportsTimeFirstCostBreakdownAndEdgeMarginDiagnostics) {
  const OccupancyGrid2D grid = openGrid();
  RacingLineConfig config = testConfig();
  config.weight_length = 0.02;
  config.weight_time = 50.0;

  const RacingLineResult result =
      optimizeRacingLine(wideLeftTurnCorridor(), grid, config, speedConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(std::isfinite(result.stats.final_length_ratio));
  EXPECT_TRUE(std::isfinite(result.stats.cost_length));
  EXPECT_TRUE(std::isfinite(result.stats.cost_time));
  EXPECT_TRUE(std::isfinite(result.stats.cost_curvature));
  EXPECT_TRUE(std::isfinite(result.stats.cost_heading_jump));
  EXPECT_TRUE(std::isfinite(result.stats.min_edge_margin_m));
  EXPECT_TRUE(std::isfinite(result.stats.mean_edge_margin_m));
  EXPECT_GT(result.stats.cost_time, result.stats.cost_length);
}

} // namespace drone_city_nav
