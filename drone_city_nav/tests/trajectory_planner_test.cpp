#include "drone_city_nav/clearance_field.hpp"
#include "drone_city_nav/planner_core.hpp"
#include "drone_city_nav/trajectory_planner.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <numbers>
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
  config.trajectory_optimizer.max_iterations = 20U;
  config.trajectory_optimizer.initial_offset_step_m = 1.0;
  config.trajectory_optimizer.min_offset_step_m = 0.1;
  config.trajectory_optimizer.weight_curvature = 30.0;
  config.debug_sample_step_m = 1.0;
  config.initial_altitude_m = 18.0;
  config.speed_profile.cruise_speed_mps = 12.0;
  config.speed_profile.turn_speed_lateral_accel_mps2 = 3.0;
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

[[nodiscard]] KnownPassageMap plannerValidationPassageMap() {
  PassageOpening opening{};
  opening.id = "window";
  opening.structure_id = "arch";
  opening.center = Point3{0.0, 0.0, 10.0};
  opening.normal_xy = Point2{1.0, 0.0};
  opening.width_m = 6.0;
  opening.height_m = 4.0;
  opening.depth_m = 6.0;
  opening.min_z_m = 8.0;
  opening.max_z_m = 12.0;
  opening.approach_distance_m = 8.0;
  opening.exit_distance_m = 8.0;

  PassageStructure structure{};
  structure.id = "arch";
  structure.center = Point2{0.0, 0.0};
  structure.size_x_m = 10.0;
  structure.size_y_m = 10.0;
  structure.z_min_m = 0.0;
  structure.z_max_m = 20.0;
  structure.openings.push_back(opening);

  KnownPassageMap map{};
  map.frame_id = "map";
  map.structures.push_back(structure);
  return map;
}

[[nodiscard]] TrajectoryPlannerResult
planWithRisk(TrajectoryPlannerInput input, const TrajectoryPlannerConfig& config,
             const bool baseline) {
  const OccupancyGrid2D* raw = input.prohibited_grid;
  const ClearanceField2D* clearance = input.prohibited_clearance_field;
  bool clearance_cache_hit = input.prohibited_clearance_field_cache_hit;
  if (raw == nullptr && !input.risk_contexts.empty()) {
    raw = input.risk_contexts.front().raw_occupancy;
    clearance = input.risk_contexts.front().raw_clearance;
    clearance_cache_hit = input.risk_contexts.front().raw_clearance_cache_hit;
  }
  if (raw == nullptr) {
    return baseline ? planBaselineTrajectory(input, config)
                    : planOptimizedTrajectory(input, config);
  }
  const ObstacleRiskField risk = ObstacleRiskField::build(
      *raw, {.critical_distance_m = 1.0, .preferred_distance_m = 4.0});
  const std::array contexts{TrajectoryRiskContext{
      .name = "raw_risk",
      .raw_occupancy = raw,
      .risk_field = &risk,
      .raw_clearance = clearance,
      .raw_clearance_cache_hit = clearance_cache_hit,
  }};
  input.risk_contexts = contexts;
  return baseline ? planBaselineTrajectory(input, config)
                  : planOptimizedTrajectory(input, config);
}

[[nodiscard]] TrajectoryPlannerResult
planTrajectoryWithRisk(TrajectoryPlannerInput input,
                       const TrajectoryPlannerConfig& config) {
  return planWithRisk(std::move(input), config, false);
}

[[nodiscard]] TrajectoryPlannerResult
planOptimizedTrajectoryWithRisk(TrajectoryPlannerInput input,
                                const TrajectoryPlannerConfig& config) {
  return planWithRisk(std::move(input), config, false);
}

[[nodiscard]] TrajectoryPlannerResult
planBaselineTrajectoryWithRisk(TrajectoryPlannerInput input,
                               const TrajectoryPlannerConfig& config) {
  return planWithRisk(std::move(input), config, true);
}

[[nodiscard]] TrajectoryPlannerResult
finalizeStitchedTrajectoryWithRisk(StitchedTrajectoryFinalizationInput input,
                                   const TrajectoryPlannerConfig& config) {
  if (input.risk_contexts.empty() ||
      input.risk_contexts.front().raw_occupancy == nullptr) {
    return finalizeStitchedTrajectory(input, config);
  }
  const OccupancyGrid2D* const raw = input.risk_contexts.front().raw_occupancy;
  const ObstacleRiskField risk = ObstacleRiskField::build(
      *raw, {.critical_distance_m = 1.0, .preferred_distance_m = 4.0});
  const std::array contexts{TrajectoryRiskContext{
      .name = "raw_risk",
      .raw_occupancy = raw,
      .risk_field = &risk,
      .raw_clearance = input.risk_contexts.front().raw_clearance,
      .raw_clearance_cache_hit = input.risk_contexts.front().raw_clearance_cache_hit,
  }};
  input.risk_contexts = contexts;
  return finalizeStitchedTrajectory(input, config);
}

} // namespace

TEST(TrajectoryPlanner, EmptyRouteIsInvalid) {
  const TrajectoryPlannerResult result =
      planTrajectoryWithRisk(TrajectoryPlannerInput{}, testConfig());

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.stats.status, TrajectoryPlannerStatus::kInvalidRoute);
}

TEST(TrajectoryPlanner, MissingGridIsInvalid) {
  const std::vector<Point2> route{{0.0, 0.0}, {10.0, 0.0}};

  const TrajectoryPlannerResult result = planTrajectoryWithRisk(
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

  const TrajectoryPlannerResult result = planTrajectoryWithRisk(
      TrajectoryPlannerInput{std::span<const Point2>{route.data(), route.size()},
                             nullptr, nullptr, false, std::span<const CorridorSample>{},
                             nullptr},
      testConfig());

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.stats.status, TrajectoryPlannerStatus::kMissingGrid);
  EXPECT_EQ(result.stats.arc_segments, 0U);
  EXPECT_TRUE(result.compact_segments.empty());
}

TEST(TrajectoryPlanner, TrajectoryOptimizerTrajectoryProducesSamplesAndSpeedProfile) {
  const OccupancyGrid2D grid = testGrid();
  const std::vector<Point2> route{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};

  const TrajectoryPlannerResult result = planTrajectoryWithRisk(
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
  EXPECT_GT(result.stats.trajectory_optimizer.candidate_evaluations, 0U);
  EXPECT_GT(result.stats.trajectory_optimizer.active_window_count, 0U);
  EXPECT_GT(result.stats.trajectory_optimizer.dp_states, 0U);
  EXPECT_TRUE(std::isfinite(result.stats.trajectory_optimizer.estimated_time_s));
  for (const TrajectoryPointSample& sample : result.samples) {
    EXPECT_DOUBLE_EQ(sample.z_m, 18.0);
  }
}

TEST(TrajectoryPlanner, GridDependentStagesUseSingleRawRiskContext) {
  OccupancyGrid2D runtime_grid = testGrid();
  for (int x = 5; x <= 35; ++x) {
    runtime_grid.setOccupied(GridIndex{x, 24});
    runtime_grid.setOccupied(GridIndex{x, 16});
  }
  OccupancyGrid2D planning_grid = runtime_grid;

  const std::vector<Point2> route{{-10.0, 0.0}, {10.0, 0.0}};
  const std::vector<TrajectoryRiskContext> candidates{
      TrajectoryRiskContext{"planning_clearance", &planning_grid},
      TrajectoryRiskContext{"runtime_prohibited", &runtime_grid},
  };
  TrajectoryPlannerInput input{};
  input.route_points = std::span<const Point2>{route.data(), route.size()};
  input.prohibited_grid = &planning_grid;
  input.risk_contexts = candidates;
  const TrajectoryPlannerResult result =
      planOptimizedTrajectoryWithRisk(input, testConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.stats.grid_stages.corridor, "raw_risk");
  EXPECT_EQ(result.stats.grid_stages.optimizer, "raw_risk");
  EXPECT_EQ(result.stats.grid_stages.turn_smoothing, "raw_risk");
  EXPECT_EQ(result.stats.grid_stages.trajectory_validation, "raw_risk");
  EXPECT_EQ(result.stats.grid_stages.shape_cleanup, "raw_risk");
  EXPECT_EQ(result.stats.grid_stages.passage_insertion, "raw_risk");
  EXPECT_EQ(result.stats.grid_stages.corridor_attempts, 1U);
  EXPECT_EQ(result.stats.grid_stages.optimizer_attempts, 1U);
  EXPECT_EQ(result.stats.grid_stages.turn_smoothing_attempts, 1U);
  EXPECT_EQ(result.stats.grid_stages.trajectory_validation_attempts, 1U);
  EXPECT_EQ(result.stats.grid_stages.shape_cleanup_attempts, 1U);
  EXPECT_EQ(result.stats.grid_stages.passage_insertion_attempts, 1U);
  const std::vector<Point2> result_points = samplePoints(result.samples);
  EXPECT_TRUE(pathIsTraversable(runtime_grid, result_points));
  EXPECT_TRUE(pathIsTraversable(planning_grid, result_points));
}

TEST(TrajectoryPlanner,
     PassageInsertionDoesNotBlockBaseTrajectoryWhenStrictGridCannotRepair) {
  OccupancyGrid2D planning_grid = testGrid();
  const GridIndex blocked_cell{20, 20};
  planning_grid.setOccupied(blocked_cell);
  const OccupancyGrid2D runtime_grid = testGrid();
  const std::vector<Point2> route{{-10.0, 2.8}, {10.0, 2.8}};
  const KnownPassageMap map = plannerValidationPassageMap();
  const std::vector<TrajectoryRiskContext> candidates{
      TrajectoryRiskContext{"planning_clearance", &planning_grid},
      TrajectoryRiskContext{"runtime_prohibited", &runtime_grid},
  };
  TrajectoryPlannerConfig config = testConfig();
  config.corridor.max_radius_m = 0.1;
  config.initial_altitude_m = 10.0;
  config.vertical_profile.enabled = false;
  config.passage_insertion.max_join_tangent_delta_rad = std::numbers::pi;
  config.passage_insertion.max_join_curvature_jump_1pm = 10.0;
  config.passage_insertion.max_lateral_shift_m = 20.0;
  TrajectoryPlannerInput input{};
  input.route_points = std::span<const Point2>{route.data(), route.size()};
  input.prohibited_grid = &planning_grid;
  input.known_passage_map = &map;
  input.risk_contexts = candidates;

  const TrajectoryPlannerResult result = planOptimizedTrajectoryWithRisk(input, config);

  EXPECT_TRUE(result.valid) << "status=" << static_cast<int>(result.stats.status)
                            << " optimizer_inputs="
                            << result.stats.trajectory_optimizer.input_samples
                            << " optimizer_outputs="
                            << result.stats.trajectory_optimizer.output_samples
                            << " collision_rejections="
                            << result.stats.trajectory_optimizer.collision_rejections;
  EXPECT_EQ(result.stats.grid_stages.passage_insertion, "raw_risk");
  ASSERT_EQ(result.stats.passage_insertion_risk_attempts.size(), 1U);
  const PassageInsertionGridAttempt& strict_attempt =
      result.stats.passage_insertion_risk_attempts.front();
  EXPECT_EQ(strict_attempt.grid_name, "raw_risk");
  EXPECT_TRUE(strict_attempt.valid);
  EXPECT_TRUE(strict_attempt.repair_required);
  EXPECT_FALSE(strict_attempt.repair_satisfied);
  EXPECT_FALSE(strict_attempt.applied);
  EXPECT_TRUE(strict_attempt.accepted);
}

TEST(TrajectoryPlanner, PassageInsertionFailureOnAllGridsKeepsBaseTrajectory) {
  OccupancyGrid2D planning_grid = testGrid();
  OccupancyGrid2D runtime_grid = testGrid();
  const GridIndex blocked_cell{20, 20};
  planning_grid.setOccupied(blocked_cell);
  runtime_grid.setOccupied(blocked_cell);
  const std::vector<Point2> route{{-10.0, 2.8}, {10.0, 2.8}};
  const KnownPassageMap map = plannerValidationPassageMap();
  const std::vector<TrajectoryRiskContext> candidates{
      TrajectoryRiskContext{"planning_clearance", &planning_grid},
      TrajectoryRiskContext{"runtime_prohibited", &runtime_grid},
  };
  TrajectoryPlannerConfig config = testConfig();
  config.corridor.max_radius_m = 0.1;
  config.initial_altitude_m = 10.0;
  config.vertical_profile.enabled = false;
  config.passage_insertion.max_join_tangent_delta_rad = std::numbers::pi;
  config.passage_insertion.max_join_curvature_jump_1pm = 10.0;
  config.passage_insertion.max_lateral_shift_m = 20.0;
  TrajectoryPlannerInput input{};
  input.route_points = std::span<const Point2>{route.data(), route.size()};
  input.prohibited_grid = &planning_grid;
  input.known_passage_map = &map;
  input.risk_contexts = candidates;

  const TrajectoryPlannerResult result = planOptimizedTrajectoryWithRisk(input, config);

  EXPECT_TRUE(result.valid) << "status=" << static_cast<int>(result.stats.status)
                            << " optimizer_inputs="
                            << result.stats.trajectory_optimizer.input_samples
                            << " optimizer_outputs="
                            << result.stats.trajectory_optimizer.output_samples
                            << " collision_rejections="
                            << result.stats.trajectory_optimizer.collision_rejections;
  EXPECT_EQ(result.stats.status, TrajectoryPlannerStatus::kOk);
  EXPECT_EQ(result.stats.grid_stages.passage_insertion, "raw_risk");
  ASSERT_EQ(result.stats.passage_insertion_risk_attempts.size(), 1U);
  const PassageInsertionGridAttempt& attempt =
      result.stats.passage_insertion_risk_attempts.front();
  EXPECT_TRUE(attempt.valid);
  EXPECT_TRUE(attempt.repair_required);
  EXPECT_FALSE(attempt.repair_satisfied);
  EXPECT_FALSE(attempt.applied);
  EXPECT_TRUE(attempt.accepted);
}

TEST(TrajectoryPlanner, PassageInsertionSkipsFallbackWhenRepairIsNotRequired) {
  const OccupancyGrid2D planning_grid = testGrid();
  const OccupancyGrid2D runtime_grid = testGrid();
  const std::vector<Point2> route{{-10.0, 0.0}, {10.0, 0.0}};
  const KnownPassageMap map = plannerValidationPassageMap();
  const std::vector<TrajectoryRiskContext> candidates{
      TrajectoryRiskContext{"planning_clearance", &planning_grid},
      TrajectoryRiskContext{"runtime_prohibited", &runtime_grid},
  };
  TrajectoryPlannerConfig config = testConfig();
  config.initial_altitude_m = 10.0;
  config.vertical_profile.enabled = false;
  TrajectoryPlannerInput input{};
  input.route_points = std::span<const Point2>{route.data(), route.size()};
  input.prohibited_grid = &planning_grid;
  input.known_passage_map = &map;
  input.risk_contexts = candidates;

  const TrajectoryPlannerResult result = planOptimizedTrajectoryWithRisk(input, config);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.stats.grid_stages.passage_insertion, "raw_risk");
  ASSERT_EQ(result.stats.passage_insertion_risk_attempts.size(), 1U);
  const PassageInsertionGridAttempt& attempt =
      result.stats.passage_insertion_risk_attempts.front();
  EXPECT_FALSE(attempt.repair_required);
  EXPECT_TRUE(attempt.repair_satisfied);
  EXPECT_FALSE(attempt.applied);
  EXPECT_TRUE(attempt.accepted);
}

TEST(TrajectoryPlanner, BaselineTrajectoryProducesSamplesAndSpeedProfile) {
  const OccupancyGrid2D grid = testGrid();
  const std::vector<Point2> route{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};

  const TrajectoryPlannerResult result = planBaselineTrajectoryWithRisk(
      TrajectoryPlannerInput{std::span<const Point2>{route.data(), route.size()}, &grid,
                             nullptr, false, std::span<const CorridorSample>{},
                             nullptr},
      testConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.stats.status, TrajectoryPlannerStatus::kOk);
  EXPECT_EQ(result.stats.quality, TrajectoryQuality::kBaseline);
  EXPECT_GE(result.samples.size(), 3U);
  EXPECT_TRUE(result.speed_profile.valid);
  EXPECT_EQ(result.stats.trajectory_optimizer.candidate_evaluations, 0U);
  EXPECT_FALSE(result.stats.trajectory_optimizer.async_refined);
  ASSERT_FALSE(result.samples.empty());
  EXPECT_NEAR(distance(result.samples.back().point, route.back()), 0.0, 1.0e-6);
  for (const TrajectoryPointSample& sample : result.samples) {
    EXPECT_DOUBLE_EQ(sample.z_m, 18.0);
  }
}

TEST(TrajectoryPlanner, StartInsideOpeningPublishesPartialTraversal) {
  const OccupancyGrid2D grid = testGrid();
  const std::vector<Point2> route{{0.0, 0.0}, {10.0, 0.0}};
  const KnownPassageMap map = plannerValidationPassageMap();
  TrajectoryPlannerConfig config = testConfig();
  config.initial_altitude_m = 10.0;
  TrajectoryPlannerInput input{};
  input.route_points = std::span<const Point2>{route.data(), route.size()};
  input.prohibited_grid = &grid;
  input.known_passage_map = &map;

  const TrajectoryPlannerResult result = planOptimizedTrajectoryWithRisk(input, config);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.stats.status, TrajectoryPlannerStatus::kOk);
  EXPECT_EQ(result.stats.known_passage_validation.worst_reason,
            KnownPassageValidationReason::kPartialFromInside);
  ASSERT_EQ(result.stats.known_passage_validation.diagnostics.size(), 1U);
  EXPECT_TRUE(
      result.stats.known_passage_validation.diagnostics.front().starts_inside_opening);
  ASSERT_FALSE(result.samples.empty());
  EXPECT_TRUE(result.samples.front().vertical_hard_window_active);
  EXPECT_DOUBLE_EQ(result.samples.front().z_m, 10.0);
}

TEST(TrajectoryPlanner, KnownSolidIntersectionInvalidatesTrajectory) {
  const OccupancyGrid2D grid = testGrid();
  const std::vector<Point2> route{{-10.0, 0.0}, {10.0, 0.0}};
  const KnownPassageMap map = plannerValidationPassageMap();
  TrajectoryPlannerConfig config = testConfig();
  config.vertical_profile.enabled = false;

  const TrajectoryPlannerResult result = planBaselineTrajectoryWithRisk(
      TrajectoryPlannerInput{std::span<const Point2>{route.data(), route.size()}, &grid,
                             nullptr, false, std::span<const CorridorSample>{}, nullptr,
                             &map},
      config);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.stats.status, TrajectoryPlannerStatus::kInvalidTrajectory);
  EXPECT_FALSE(result.stats.vertical_profile.active);
  EXPECT_FALSE(result.stats.known_passage_validation.valid);
  EXPECT_FALSE(result.stats.known_passage_solid_validation.valid);
  EXPECT_EQ(result.stats.known_passage_solid_validation.reason,
            KnownPassageSolidValidationReason::kIntersection);
  EXPECT_EQ(result.stats.known_passage_validation.worst_reason,
            KnownPassageValidationReason::kOpeningVolumeMiss);
}

TEST(TrajectoryPlanner, PassageQualityFailurePublishesSolidClearTrajectory) {
  const OccupancyGrid2D grid = testGrid();
  const std::vector<Point2> route{{-10.0, 0.0}, {10.0, 0.0}};
  const KnownPassageMap map = plannerValidationPassageMap();
  TrajectoryPlannerConfig config = testConfig();
  config.initial_altitude_m = 10.0;
  config.vertical_profile.enabled = false;
  config.known_passage_validation.min_opening_overlap_m = 100.0;

  const TrajectoryPlannerResult result = planBaselineTrajectoryWithRisk(
      TrajectoryPlannerInput{std::span<const Point2>{route.data(), route.size()}, &grid,
                             nullptr, false, std::span<const CorridorSample>{}, nullptr,
                             &map},
      config);

  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.stats.status, TrajectoryPlannerStatus::kOk);
  EXPECT_EQ(result.stats.quality, TrajectoryQuality::kDegradedPassage);
  EXPECT_FALSE(result.stats.known_passage_validation.valid);
  EXPECT_TRUE(result.stats.known_passage_solid_validation.valid);
  EXPECT_EQ(result.stats.known_passage_solid_validation.reason,
            KnownPassageSolidValidationReason::kClear);
}

TEST(TrajectoryPlanner, PassageInsertionRepairsKnownPassageBeforeVerticalProfile) {
  const OccupancyGrid2D grid = testGrid();
  const std::vector<Point2> route{{-15.0, 4.0}, {15.0, 4.0}};
  const KnownPassageMap map = plannerValidationPassageMap();
  TrajectoryPlannerConfig config = testConfig();
  config.initial_altitude_m = 10.0;
  config.vertical_profile.enabled = true;
  config.vertical_profile.max_climb_angle_rad = 80.0 * std::numbers::pi / 180.0;
  config.passage_insertion.enabled = false;

  const TrajectoryPlannerInput input{
      std::span<const Point2>{route.data(), route.size()},
      &grid,
      nullptr,
      false,
      std::span<const CorridorSample>{},
      nullptr,
      &map};
  const TrajectoryPlannerResult missed = planOptimizedTrajectoryWithRisk(input, config);
  EXPECT_FALSE(missed.valid);
  EXPECT_EQ(missed.stats.status, TrajectoryPlannerStatus::kInvalidTrajectory);
  EXPECT_FALSE(missed.stats.passage_insertion.applied);
  EXPECT_FALSE(missed.stats.known_passage_validation.valid);
  EXPECT_EQ(missed.stats.known_passage_validation.worst_reason,
            KnownPassageValidationReason::kOpeningVolumeMiss);

  config.passage_insertion.enabled = true;
  config.passage_insertion.max_join_tangent_delta_rad = std::numbers::pi;
  config.passage_insertion.max_join_curvature_jump_1pm = 10.0;
  config.passage_insertion.max_lateral_shift_m = 20.0;
  const TrajectoryPlannerResult repaired =
      planOptimizedTrajectoryWithRisk(input, config);

  ASSERT_TRUE(repaired.valid);
  EXPECT_TRUE(repaired.stats.passage_insertion.applied);
  EXPECT_TRUE(repaired.stats.vertical_profile.valid);
  EXPECT_TRUE(repaired.stats.known_passage_validation.valid);
  EXPECT_EQ(repaired.stats.known_passage_validation.worst_reason,
            KnownPassageValidationReason::kMatchedOpening);
  ASSERT_FALSE(repaired.samples.empty());
  EXPECT_NEAR(distance(repaired.samples.front().point, route.front()), 0.0, 1.0e-6);
  EXPECT_NEAR(distance(repaired.samples.back().point, route.back()), 0.0, 1.0e-6);
}

TEST(TrajectoryPlanner, ReusesProvidedClearanceFieldForCorridor) {
  const OccupancyGrid2D grid = testGrid();
  const std::vector<Point2> route{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};
  const TrajectoryPlannerConfig config = testConfig();
  const ClearanceField2D clearance_field = ClearanceField2D::build(
      grid, config.corridor.max_radius_m, ClearanceSource::kOccupied);

  const TrajectoryPlannerResult result = planTrajectoryWithRisk(
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

TEST(TrajectoryPlanner, ReusesPrecomputedCorridorForTrajectoryOptimizerTrajectory) {
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
  const TrajectoryPlannerResult baseline =
      planBaselineTrajectoryWithRisk(input, config);
  ASSERT_TRUE(baseline.valid);
  ASSERT_GE(baseline.corridor_samples.size(), 2U);

  const TrajectoryPlannerResult normal = planOptimizedTrajectoryWithRisk(input, config);
  const TrajectoryPlannerResult reused = planOptimizedTrajectoryWithRisk(
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
    EXPECT_DOUBLE_EQ(normal.samples[i].lateral_offset_m,
                     reused.samples[i].lateral_offset_m);
  }
}

TEST(TrajectoryPlanner, EmptyPrecomputedCorridorFallsBackToBuild) {
  const OccupancyGrid2D grid = testGrid();
  const std::vector<Point2> route{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};

  const TrajectoryPlannerResult result = planOptimizedTrajectoryWithRisk(
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
  const TrajectoryPlannerResult baseline =
      planBaselineTrajectoryWithRisk(input, config);
  ASSERT_TRUE(baseline.valid);
  ASSERT_GE(baseline.corridor_samples.size(), 2U);

  const TrajectoryPlannerResult result = planOptimizedTrajectoryWithRisk(
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

TEST(TrajectoryPlanner, SameEndpointDifferentRouteCorridorFallsBackToBuild) {
  const OccupancyGrid2D grid = testGrid();
  const std::vector<Point2> route{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};
  const std::vector<Point2> other_route{{0.0, 0.0}, {0.0, 10.0}, {10.0, 10.0}};
  const TrajectoryPlannerConfig config = testConfig();
  const TrajectoryPlannerInput input{
      std::span<const Point2>{route.data(), route.size()},
      &grid,
      nullptr,
      false,
      std::span<const CorridorSample>{},
      nullptr};
  const TrajectoryPlannerResult baseline =
      planBaselineTrajectoryWithRisk(input, config);
  ASSERT_TRUE(baseline.valid);
  ASSERT_GE(baseline.corridor_samples.size(), 2U);

  const TrajectoryPlannerResult result = planOptimizedTrajectoryWithRisk(
      TrajectoryPlannerInput{
          std::span<const Point2>{other_route.data(), other_route.size()},
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
  EXPECT_NE(result.stats.corridor.route_fingerprint,
            baseline.stats.corridor.route_fingerprint);
}

TEST(TrajectoryPlanner, GridMismatchPrecomputedCorridorFallsBackToBuild) {
  const OccupancyGrid2D grid = testGrid();
  OccupancyGrid2D changed_grid = testGrid();
  changed_grid.setOccupied(GridIndex{0, 0});
  const std::vector<Point2> route{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};
  const TrajectoryPlannerConfig config = testConfig();
  const TrajectoryPlannerInput input{
      std::span<const Point2>{route.data(), route.size()},
      &grid,
      nullptr,
      false,
      std::span<const CorridorSample>{},
      nullptr};
  const TrajectoryPlannerResult baseline =
      planBaselineTrajectoryWithRisk(input, config);
  ASSERT_TRUE(baseline.valid);
  ASSERT_GE(baseline.corridor_samples.size(), 2U);

  const TrajectoryPlannerResult result = planOptimizedTrajectoryWithRisk(
      TrajectoryPlannerInput{
          std::span<const Point2>{route.data(), route.size()},
          &changed_grid,
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
  EXPECT_FALSE(occupancyGridFingerprintsEqual(
      result.stats.corridor.raw_occupancy_fingerprint,
      baseline.stats.corridor.raw_occupancy_fingerprint));
}

TEST(TrajectoryPlanner, CorridorConfigMismatchPrecomputedCorridorFallsBackToBuild) {
  const OccupancyGrid2D grid = testGrid();
  const std::vector<Point2> route{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};
  const TrajectoryPlannerConfig baseline_config = testConfig();
  TrajectoryPlannerConfig changed_config = testConfig();
  changed_config.corridor.max_radius_m = baseline_config.corridor.max_radius_m + 1.0;
  const TrajectoryPlannerInput input{
      std::span<const Point2>{route.data(), route.size()},
      &grid,
      nullptr,
      false,
      std::span<const CorridorSample>{},
      nullptr};
  const TrajectoryPlannerResult baseline =
      planBaselineTrajectoryWithRisk(input, baseline_config);
  ASSERT_TRUE(baseline.valid);
  ASSERT_GE(baseline.corridor_samples.size(), 2U);

  const TrajectoryPlannerResult result = planOptimizedTrajectoryWithRisk(
      TrajectoryPlannerInput{
          std::span<const Point2>{route.data(), route.size()},
          &grid,
          nullptr,
          false,
          std::span<const CorridorSample>{baseline.corridor_samples.data(),
                                          baseline.corridor_samples.size()},
          &baseline.stats.corridor,
      },
      changed_config);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.stats.corridor.samples_reused);
  EXPECT_EQ(result.stats.corridor.reused_samples, 0U);
  EXPECT_NE(result.stats.corridor.config_fingerprint,
            baseline.stats.corridor.config_fingerprint);
}

TEST(TrajectoryPlanner, SnapshotHelperWiresClearanceAndCorridorReuse) {
  const OccupancyGrid2D grid = testGrid();
  const std::vector<Point2> route{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};
  const TrajectoryPlannerConfig config = testConfig();
  const ClearanceField2D clearance_field = ClearanceField2D::build(
      grid, config.corridor.max_radius_m, ClearanceSource::kOccupied);
  const TrajectoryPlannerResult baseline = planBaselineTrajectoryWithRisk(
      TrajectoryPlannerInput{std::span<const Point2>{route.data(), route.size()}, &grid,
                             &clearance_field, true, std::span<const CorridorSample>{},
                             nullptr},
      config);
  ASSERT_TRUE(baseline.valid);
  ASSERT_GE(baseline.corridor_samples.size(), 2U);

  const TrajectoryPlannerResult refined = planOptimizedTrajectoryWithRisk(
      TrajectoryPlannerInput{
          .route_points = std::span<const Point2>{route.data(), route.size()},
          .prohibited_grid = &grid,
          .prohibited_clearance_field = &clearance_field,
          .prohibited_clearance_field_cache_hit = true,
          .precomputed_corridor_samples =
              std::span<const CorridorSample>{baseline.corridor_samples},
          .precomputed_corridor_stats = &baseline.stats.corridor,
      },
      config);

  ASSERT_TRUE(refined.valid);
  EXPECT_TRUE(refined.stats.corridor.samples_reused);
  EXPECT_EQ(refined.stats.corridor.reused_samples, baseline.corridor_samples.size());
  EXPECT_TRUE(refined.stats.corridor.clearance_field_reused);
  EXPECT_TRUE(refined.stats.corridor.clearance_field_cache_hit);
  EXPECT_EQ(refined.stats.corridor.sample_build_duration_ms, 0.0);
  EXPECT_EQ(refined.stats.corridor.clearance_field_build_duration_ms, 0.0);
}

TEST(TrajectoryPlanner, RejectsStaleRefinedTrajectory) {
  const OccupancyGrid2D grid = testGrid();
  const std::vector<Point2> route{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};
  TrajectoryPlannerResult refined = planOptimizedTrajectoryWithRisk(
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
  const TrajectoryPlannerResult baseline = planBaselineTrajectoryWithRisk(
      TrajectoryPlannerInput{std::span<const Point2>{route.data(), route.size()}, &grid,
                             nullptr, false, std::span<const CorridorSample>{},
                             nullptr},
      config);
  TrajectoryPlannerResult refined = planOptimizedTrajectoryWithRisk(
      TrajectoryPlannerInput{std::span<const Point2>{route.data(), route.size()}, &grid,
                             nullptr, false, std::span<const CorridorSample>{},
                             nullptr},
      config);
  refined.stats.trajectory_optimizer.async_refined = true;
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
          .refined = &refined,
          .refined_points =
              std::span<const Point2>{refined_points.data(), refined_points.size()},
          .validation_grid = &grid,
      });

  EXPECT_TRUE(decision.accepted);
  EXPECT_EQ(decision.reason, TrajectoryRefinementDecisionReason::kAccepted);
  ASSERT_FALSE(refined_points.empty());
  EXPECT_NEAR(distance(refined_points.back(), route.back()), 0.0, 0.5);
  EXPECT_TRUE(refined.stats.trajectory_optimizer.async_refined);
}

TEST(TrajectoryPlanner, FinalizesStitchedTrajectoryThroughKnownOpening) {
  const OccupancyGrid2D grid = testGrid();
  const KnownPassageMap map = plannerValidationPassageMap();
  const std::vector<TrajectoryPointSample> samples =
      trajectoryPointSamplesFromPoints(std::vector<Point2>{{-10.0, 0.0}, {10.0, 0.0}});
  const std::array grids{TrajectoryRiskContext{
      .name = "runtime_prohibited",
      .raw_occupancy = &grid,
  }};
  TrajectoryPlannerConfig config = testConfig();
  config.initial_altitude_m = 10.0;

  const TrajectoryPlannerResult result = finalizeStitchedTrajectoryWithRisk(
      StitchedTrajectoryFinalizationInput{
          .geometry_samples = samples,
          .known_passage_map = &map,
          .risk_contexts = grids,
      },
      config);

  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.speed_profile.valid);
  EXPECT_TRUE(result.stats.known_passage_solid_validation.valid);
}

TEST(TrajectoryPlanner, RejectsStitchedTrajectoryThroughKnownSolid) {
  const OccupancyGrid2D grid = testGrid();
  const KnownPassageMap map = plannerValidationPassageMap();
  const std::vector<TrajectoryPointSample> samples =
      trajectoryPointSamplesFromPoints(std::vector<Point2>{{-10.0, 4.0}, {10.0, 4.0}});
  const std::array grids{TrajectoryRiskContext{
      .name = "runtime_prohibited",
      .raw_occupancy = &grid,
  }};
  TrajectoryPlannerConfig config = testConfig();
  config.initial_altitude_m = 10.0;

  const TrajectoryPlannerResult result = finalizeStitchedTrajectoryWithRisk(
      StitchedTrajectoryFinalizationInput{
          .geometry_samples = samples,
          .known_passage_map = &map,
          .risk_contexts = grids,
      },
      config);

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.stats.known_passage_solid_validation.valid);
  EXPECT_EQ(result.stats.status, TrajectoryPlannerStatus::kInvalidTrajectory);
}

TEST(TrajectoryPlanner, ShortlistAcceptsSecondPostVerticalValidFinalist) {
  const OccupancyGrid2D grid = testGrid();
  const KnownPassageMap map = plannerValidationPassageMap();
  const std::array grids{TrajectoryRiskContext{
      .name = "runtime_prohibited",
      .raw_occupancy = &grid,
  }};
  const std::vector candidates{
      trajectoryPointSamplesFromPoints(std::vector<Point2>{{-10.0, 4.0}, {10.0, 4.0}}),
      trajectoryPointSamplesFromPoints(std::vector<Point2>{{-10.0, 0.0}, {10.0, 0.0}}),
  };
  TrajectoryPlannerConfig config = testConfig();
  config.initial_altitude_m = 10.0;

  std::optional<std::size_t> accepted_index;
  std::size_t index = 0U;
  for (const std::vector<TrajectoryPointSample>& candidate : candidates) {
    const TrajectoryPlannerResult result = finalizeStitchedTrajectoryWithRisk(
        StitchedTrajectoryFinalizationInput{
            .geometry_samples = candidate,
            .known_passage_map = &map,
            .risk_contexts = grids,
        },
        config);
    if (result.valid) {
      accepted_index = index;
      break;
    }
    ++index;
  }

  EXPECT_EQ(accepted_index, std::optional<std::size_t>{1U});
}

TEST(TrajectoryPlanner, ShortlistWithAllSolidInvalidFinalistsHasNoSelection) {
  const OccupancyGrid2D grid = testGrid();
  const KnownPassageMap map = plannerValidationPassageMap();
  const std::array grids{TrajectoryRiskContext{
      .name = "runtime_prohibited",
      .raw_occupancy = &grid,
  }};
  const std::array candidates{
      trajectoryPointSamplesFromPoints(std::vector<Point2>{{-10.0, 4.0}, {10.0, 4.0}}),
      trajectoryPointSamplesFromPoints(
          std::vector<Point2>{{-10.0, -4.0}, {10.0, -4.0}}),
  };
  TrajectoryPlannerConfig config = testConfig();
  config.initial_altitude_m = 10.0;

  bool accepted = false;
  for (const std::vector<TrajectoryPointSample>& candidate : candidates) {
    accepted = accepted || finalizeStitchedTrajectoryWithRisk(
                               StitchedTrajectoryFinalizationInput{
                                   .geometry_samples = candidate,
                                   .known_passage_map = &map,
                                   .risk_contexts = grids,
                               },
                               config)
                               .valid;
  }

  EXPECT_FALSE(accepted);
}

TEST(TrajectoryPlanner, FinalizedStitchRejectsRawOccupiedPath) {
  OccupancyGrid2D planning_grid = testGrid();
  OccupancyGrid2D runtime_grid = testGrid();
  const auto blocked_cell = planning_grid.worldToCell(Point2{0.0, 0.0});
  ASSERT_TRUE(blocked_cell.has_value());
  planning_grid.setOccupied(*blocked_cell);
  const std::vector<TrajectoryPointSample> samples =
      trajectoryPointSamplesFromPoints(std::vector<Point2>{{-10.0, 0.0}, {10.0, 0.0}});
  const std::array grids{
      TrajectoryRiskContext{
          .name = "planning_clearance",
          .raw_occupancy = &planning_grid,
      },
      TrajectoryRiskContext{
          .name = "runtime_prohibited",
          .raw_occupancy = &runtime_grid,
      },
  };
  TrajectoryPlannerConfig config = testConfig();
  config.vertical_profile.enabled = false;
  config.known_passage_validation.enabled = false;
  config.passage_insertion.enabled = false;

  const TrajectoryPlannerResult result = finalizeStitchedTrajectoryWithRisk(
      StitchedTrajectoryFinalizationInput{
          .geometry_samples = samples,
          .risk_contexts = grids,
      },
      config);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.stats.status, TrajectoryPlannerStatus::kInvalidTrajectory);
}

} // namespace drone_city_nav
