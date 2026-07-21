#include "drone_city_nav/known_passage_matching.hpp"
#include "drone_city_nav/trajectory_passage_insertion.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupancyGrid2D makeGrid() {
  OccupancyGrid2D grid{GridBounds{-30.0, -30.0, 1.0, 80, 80}};
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      grid.setFree(GridIndex{x, y});
    }
  }
  return grid;
}

[[nodiscard]] PassageOpening makeOpening() {
  PassageOpening opening{};
  opening.id = "main";
  opening.structure_id = "arch";
  opening.center = Point3{0.0, 0.0, 10.0};
  opening.normal_xy = Point2{1.0, 0.0};
  opening.width_m = 4.0;
  opening.height_m = 4.0;
  opening.depth_m = 4.0;
  opening.min_z_m = 8.0;
  opening.max_z_m = 12.0;
  opening.approach_distance_m = 8.0;
  opening.exit_distance_m = 8.0;
  return opening;
}

[[nodiscard]] PassageOpening makeOpeningWithId(const std::string& id,
                                               const Point3 center) {
  PassageOpening opening = makeOpening();
  opening.id = id;
  opening.center = center;
  return opening;
}

[[nodiscard]] KnownPassageMap makeMap() {
  PassageStructure structure{};
  structure.id = "arch";
  structure.center = Point2{0.0, 0.0};
  structure.size_x_m = 10.0;
  structure.size_y_m = 12.0;
  structure.z_min_m = 0.0;
  structure.z_max_m = 20.0;
  structure.openings.push_back(makeOpening());

  KnownPassageMap map{};
  map.frame_id = "map";
  map.structures.push_back(structure);
  return map;
}

[[nodiscard]] KnownPassageMap
makeMapWithOpenings(const std::vector<PassageOpening>& openings) {
  PassageStructure structure{};
  structure.id = "arch";
  structure.center = Point2{0.0, 0.0};
  structure.size_x_m = 10.0;
  structure.size_y_m = 12.0;
  structure.z_min_m = 0.0;
  structure.z_max_m = 20.0;
  structure.openings = openings;

  KnownPassageMap map{};
  map.frame_id = "map";
  map.structures.push_back(structure);
  return map;
}

[[nodiscard]] KnownPassageMap makeDuplicateIdMap() {
  PassageStructure valid_structure{};
  valid_structure.id = "arch";
  valid_structure.center = Point2{-20.0, 0.0};
  valid_structure.size_x_m = 8.0;
  valid_structure.size_y_m = 8.0;
  valid_structure.z_min_m = 0.0;
  valid_structure.z_max_m = 20.0;
  valid_structure.openings.push_back(
      makeOpeningWithId("main", Point3{-20.0, 0.0, 10.0}));

  PassageStructure invalid_structure{};
  invalid_structure.id = "arch";
  invalid_structure.center = Point2{20.0, 0.0};
  invalid_structure.size_x_m = 8.0;
  invalid_structure.size_y_m = 8.0;
  invalid_structure.z_min_m = 0.0;
  invalid_structure.z_max_m = 20.0;
  invalid_structure.openings.push_back(
      makeOpeningWithId("main", Point3{20.0, 0.0, 10.0}));

  KnownPassageMap map{};
  map.frame_id = "map";
  map.structures.push_back(valid_structure);
  map.structures.push_back(invalid_structure);
  return map;
}

[[nodiscard]] std::vector<TrajectoryPointSample> makeLineSamples(const double y_m) {
  std::vector<TrajectoryPointSample> samples;
  for (int i = 0; i <= 20; ++i) {
    TrajectoryPointSample sample{};
    sample.point = Point2{-20.0 + static_cast<double>(i) * 2.0, y_m};
    sample.z_m = 10.0;
    sample.left_bound_m = 10.0;
    sample.right_bound_m = 10.0;
    sample.lateral_offset_m = 0.0;
    samples.push_back(sample);
  }
  populateTrajectorySampleGeometry(samples);
  return samples;
}

[[nodiscard]] std::vector<TrajectoryPointSample> makeTwoCrossingSamples() {
  std::vector<TrajectoryPointSample> samples;
  const std::vector<Point2> points{{-26.0, 0.0}, {-20.0, 0.0}, {-14.0, 0.0},
                                   {14.0, 4.0},  {20.0, 4.0},  {26.0, 4.0}};
  samples.reserve(points.size());
  for (const Point2 point : points) {
    TrajectoryPointSample sample{};
    sample.point = point;
    sample.z_m = 10.0;
    sample.left_bound_m = 10.0;
    sample.right_bound_m = 10.0;
    sample.lateral_offset_m = 0.0;
    samples.push_back(sample);
  }
  populateTrajectorySampleGeometry(samples);
  return samples;
}

[[nodiscard]] PassageInsertionConfig insertionConfig() {
  PassageInsertionConfig config{};
  config.enabled = true;
  config.sample_step_m = 1.0;
  config.min_anchor_margin_m = 8.0;
  config.max_anchor_margin_m = 20.0;
  config.max_join_tangent_delta_rad = std::numbers::pi;
  config.max_join_curvature_jump_1pm = 10.0;
  config.max_lateral_shift_m = 20.0;
  config.max_candidates = 8U;
  config.max_diagnostics = 8U;
  return config;
}

void expectSamePoints(const std::vector<TrajectoryPointSample>& actual,
                      const std::vector<TrajectoryPointSample>& expected) {
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t i = 0U; i < expected.size(); ++i) {
    EXPECT_DOUBLE_EQ(actual[i].point.x, expected[i].point.x);
    EXPECT_DOUBLE_EQ(actual[i].point.y, expected[i].point.y);
  }
}

void expectEndpointsPreserved(const std::vector<TrajectoryPointSample>& actual,
                              const std::vector<TrajectoryPointSample>& expected) {
  ASSERT_FALSE(actual.empty());
  ASSERT_FALSE(expected.empty());
  EXPECT_NEAR(distance(actual.front().point, expected.front().point), 0.0, 1.0e-6);
  EXPECT_NEAR(distance(actual.back().point, expected.back().point), 0.0, 1.0e-6);
}

void expectMonotonicStations(const std::vector<TrajectoryPointSample>& samples) {
  ASSERT_FALSE(samples.empty());
  EXPECT_DOUBLE_EQ(samples.front().s_m, 0.0);
  for (std::size_t i = 1U; i < samples.size(); ++i) {
    EXPECT_GT(samples[i].s_m, samples[i - 1U].s_m);
  }
}

} // namespace

TEST(TrajectoryPassageInsertion, DisabledReturnsNoopSamples) {
  const OccupancyGrid2D grid = makeGrid();
  const KnownPassageMap map = makeMap();
  const std::vector<TrajectoryPointSample> samples = makeLineSamples(4.0);
  PassageInsertionConfig config = insertionConfig();
  config.enabled = false;

  const PassageInsertionResult result = insertLocalPassageSegments(
      samples, grid, &map, KnownPassageValidationConfig{}, config, 10.0);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.applied);
  EXPECT_FALSE(result.stats.applied);
  EXPECT_EQ(result.stats.final_reason, PassageInsertionRejectReason::kDisabled);
  expectSamePoints(result.samples, samples);
}

TEST(TrajectoryPassageInsertion, NoMapReturnsNoopSamples) {
  const OccupancyGrid2D grid = makeGrid();
  const std::vector<TrajectoryPointSample> samples = makeLineSamples(4.0);

  const PassageInsertionResult result = insertLocalPassageSegments(
      samples, grid, nullptr, KnownPassageValidationConfig{}, insertionConfig(), 10.0);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.applied);
  EXPECT_FALSE(result.stats.applied);
  EXPECT_EQ(result.stats.final_reason, PassageInsertionRejectReason::kNoMap);
  expectSamePoints(result.samples, samples);
}

TEST(TrajectoryPassageInsertion, AlreadyValidOpeningTraversalReturnsNoopSamples) {
  const OccupancyGrid2D grid = makeGrid();
  const KnownPassageMap map = makeMap();
  const std::vector<TrajectoryPointSample> samples = makeLineSamples(0.0);
  const KnownPassageValidationSummary before =
      validateKnownPassageTraversal(samples, &map, KnownPassageValidationConfig{});
  ASSERT_TRUE(before.valid);

  const PassageInsertionResult result = insertLocalPassageSegments(
      samples, grid, &map, KnownPassageValidationConfig{}, insertionConfig(), 10.0);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.applied);
  EXPECT_FALSE(result.stats.applied);
  EXPECT_EQ(result.stats.final_reason, PassageInsertionRejectReason::kNoRepairNeeded);
  EXPECT_EQ(result.stats.candidates, 0U);
  expectSamePoints(result.samples, samples);
}

TEST(TrajectoryPassageInsertion, RepairsValidButLowClearanceOpeningTraversal) {
  const OccupancyGrid2D grid = makeGrid();
  const KnownPassageMap map = makeMap();
  const std::vector<TrajectoryPointSample> samples = makeLineSamples(1.4);
  KnownPassageValidationConfig validation_config{};
  validation_config.clearance_margin_m = 0.0;
  const KnownPassageValidationSummary before =
      validateKnownPassageTraversal(samples, &map, validation_config);
  ASSERT_TRUE(before.valid);
  ASSERT_EQ(before.diagnostics.size(), 1U);
  EXPECT_LT(before.diagnostics.front().clearance_m, 1.5);

  PassageInsertionConfig config = insertionConfig();
  config.opening_lateral_target_margin_m = 1.5;
  config.repair_clearance_margin_m = 1.5;
  const PassageInsertionResult result =
      insertLocalPassageSegments(samples, grid, &map, validation_config, config, 10.0);

  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(result.applied);
  ASSERT_FALSE(result.stats.diagnostics.empty());
  EXPECT_EQ(result.stats.final_reason, PassageInsertionRejectReason::kNone);
  EXPECT_LT(result.stats.diagnostics.front().lateral_miss_after_m,
            result.stats.diagnostics.front().lateral_miss_before_m);
  expectEndpointsPreserved(result.samples, samples);
  expectMonotonicStations(result.samples);
}

TEST(TrajectoryPassageInsertion, InsertsLocalSegmentThroughKnownOpening) {
  const OccupancyGrid2D grid = makeGrid();
  const KnownPassageMap map = makeMap();
  const std::vector<TrajectoryPointSample> samples = makeLineSamples(4.0);
  const KnownPassageValidationSummary before =
      validateKnownPassageTraversal(samples, &map, KnownPassageValidationConfig{});
  ASSERT_FALSE(before.valid);
  ASSERT_EQ(before.worst_reason, KnownPassageValidationReason::kOpeningVolumeMiss);

  const PassageInsertionResult result = insertLocalPassageSegments(
      samples, grid, &map, KnownPassageValidationConfig{}, insertionConfig(), 10.0);

  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(result.applied);
  EXPECT_TRUE(result.stats.applied);
  EXPECT_EQ(result.stats.inserted_count, 1U);
  EXPECT_EQ(result.stats.final_reason, PassageInsertionRejectReason::kNone);
  ASSERT_FALSE(result.stats.diagnostics.empty());
  EXPECT_TRUE(result.stats.diagnostics.front().accepted);
  EXPECT_LT(result.stats.diagnostics.front().lateral_miss_after_m,
            result.stats.diagnostics.front().lateral_miss_before_m);
  expectEndpointsPreserved(result.samples, samples);
  expectMonotonicStations(result.samples);

  const KnownPassageValidationSummary after = validateKnownPassageTraversal(
      result.samples, &map, KnownPassageValidationConfig{});
  EXPECT_TRUE(after.valid);
  EXPECT_EQ(after.worst_reason, KnownPassageValidationReason::kMatchedOpening);
}

TEST(TrajectoryPassageInsertion, RejectsCandidateThatCrossesProhibitedGrid) {
  OccupancyGrid2D grid = makeGrid();
  const GridIndex blocked_cell{30, 30};
  ASSERT_TRUE(grid.contains(blocked_cell));
  grid.setOccupied(blocked_cell);
  const KnownPassageMap map = makeMap();
  const std::vector<TrajectoryPointSample> samples = makeLineSamples(4.0);

  const PassageInsertionResult result = insertLocalPassageSegments(
      samples, grid, &map, KnownPassageValidationConfig{}, insertionConfig(), 10.0);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.applied);
  EXPECT_FALSE(result.stats.applied);
  EXPECT_GT(result.stats.rejected_traversability, 0U);
  ASSERT_FALSE(result.stats.diagnostics.empty());
  const PassageInsertionDiagnostic& diagnostic = result.stats.diagnostics.front();
  EXPECT_EQ(diagnostic.reason, PassageInsertionRejectReason::kNonTraversable);
  ASSERT_TRUE(diagnostic.blocked_segment.available);
  EXPECT_LT(diagnostic.blocked_segment.start_s_m, diagnostic.blocked_segment.end_s_m);
  EXPECT_TRUE(diagnostic.blocked_segment.start_cell_available);
  EXPECT_TRUE(diagnostic.blocked_segment.end_cell_available);
  ASSERT_TRUE(diagnostic.blocked_segment.blocked_cell_available);
  EXPECT_EQ(diagnostic.blocked_segment.blocked_cell.x, blocked_cell.x);
  EXPECT_EQ(diagnostic.blocked_segment.blocked_cell.y, blocked_cell.y);
  EXPECT_TRUE(diagnostic.blocked_segment.occupied);
  EXPECT_FALSE(diagnostic.blocked_segment.inflated);
}

TEST(TrajectoryPassageInsertion, CapturesInflationOnlyBlockedSegment) {
  OccupancyGrid2D grid = makeGrid();
  const GridIndex raw_obstacle_cell{30, 29};
  ASSERT_TRUE(grid.contains(raw_obstacle_cell));
  grid.setOccupied(raw_obstacle_cell);
  grid.rebuildInflation(1.0);
  const KnownPassageMap map = makeMap();
  const std::vector<TrajectoryPointSample> samples = makeLineSamples(4.0);

  const PassageInsertionResult result = insertLocalPassageSegments(
      samples, grid, &map, KnownPassageValidationConfig{}, insertionConfig(), 10.0);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.applied);
  ASSERT_FALSE(result.stats.diagnostics.empty());
  const PassageInsertionDiagnostic& diagnostic = result.stats.diagnostics.front();
  EXPECT_EQ(diagnostic.reason, PassageInsertionRejectReason::kNonTraversable);
  ASSERT_TRUE(diagnostic.blocked_segment.available);
  ASSERT_TRUE(diagnostic.blocked_segment.blocked_cell_available);
  EXPECT_FALSE(diagnostic.blocked_segment.occupied);
  EXPECT_TRUE(diagnostic.blocked_segment.inflated);
}

TEST(TrajectoryPassageInsertion, RejectsCandidateOnJoinTangentDelta) {
  const OccupancyGrid2D grid = makeGrid();
  const KnownPassageMap map = makeMap();
  const std::vector<TrajectoryPointSample> samples = makeLineSamples(4.0);
  PassageInsertionConfig config = insertionConfig();
  config.max_join_tangent_delta_rad = 1.0e-6;

  const PassageInsertionResult result = insertLocalPassageSegments(
      samples, grid, &map, KnownPassageValidationConfig{}, config, 10.0);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.applied);
  EXPECT_GT(result.stats.rejected_join, 0U);
  ASSERT_FALSE(result.stats.diagnostics.empty());
  EXPECT_EQ(result.stats.diagnostics.front().reason,
            PassageInsertionRejectReason::kJoinTangent);
}

TEST(TrajectoryPassageInsertion, RejectsCandidateOnCurvatureJump) {
  const OccupancyGrid2D grid = makeGrid();
  const KnownPassageMap map = makeMap();
  const std::vector<TrajectoryPointSample> samples = makeLineSamples(4.0);
  PassageInsertionConfig config = insertionConfig();
  config.max_join_tangent_delta_rad = std::numbers::pi;
  config.max_join_curvature_jump_1pm = 1.0e-6;

  const PassageInsertionResult result = insertLocalPassageSegments(
      samples, grid, &map, KnownPassageValidationConfig{}, config, 10.0);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.applied);
  EXPECT_GT(result.stats.rejected_join, 0U);
  ASSERT_FALSE(result.stats.diagnostics.empty());
  EXPECT_EQ(result.stats.diagnostics.front().reason,
            PassageInsertionRejectReason::kJoinCurvature);
}

TEST(TrajectoryPassageInsertion, RejectsCandidateBelowMinimumInsertedRadius) {
  const OccupancyGrid2D grid = makeGrid();
  const KnownPassageMap map = makeMap();
  const std::vector<TrajectoryPointSample> samples = makeLineSamples(4.0);
  PassageInsertionConfig config = insertionConfig();
  config.max_join_tangent_delta_rad = std::numbers::pi;
  config.max_join_curvature_jump_1pm = 10.0;
  config.min_inserted_radius_m = 100000.0;

  const PassageInsertionResult result = insertLocalPassageSegments(
      samples, grid, &map, KnownPassageValidationConfig{}, config, 10.0);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.applied);
  EXPECT_GT(result.stats.rejected_join, 0U);
  ASSERT_FALSE(result.stats.diagnostics.empty());
  EXPECT_EQ(result.stats.diagnostics.front().reason,
            PassageInsertionRejectReason::kInsertedRadius);
}

TEST(TrajectoryPassageInsertion, MaxCandidatesZeroDoesNotEvaluateCandidates) {
  const OccupancyGrid2D grid = makeGrid();
  const KnownPassageMap map = makeMap();
  const std::vector<TrajectoryPointSample> samples = makeLineSamples(4.0);
  PassageInsertionConfig config = insertionConfig();
  config.max_candidates = 0U;

  const PassageInsertionResult result = insertLocalPassageSegments(
      samples, grid, &map, KnownPassageValidationConfig{}, config, 10.0);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.applied);
  EXPECT_EQ(result.stats.final_reason,
            PassageInsertionRejectReason::kTooManyCandidates);
  EXPECT_EQ(result.stats.candidates, 0U);
  EXPECT_TRUE(result.stats.diagnostics.empty());
}

TEST(TrajectoryPassageInsertion, LimitsDiagnosticsAndReportsRejectedCandidates) {
  const OccupancyGrid2D grid = makeGrid();
  const KnownPassageMap map =
      makeMapWithOpenings({makeOpeningWithId("first", Point3{0.0, 0.0, 10.0}),
                           makeOpeningWithId("second", Point3{0.0, 1.0, 10.0})});
  const std::vector<TrajectoryPointSample> samples = makeLineSamples(4.0);
  PassageInsertionConfig config = insertionConfig();
  config.max_lateral_shift_m = 0.1;
  config.max_candidates = 8U;
  config.max_diagnostics = 1U;

  const PassageInsertionResult result = insertLocalPassageSegments(
      samples, grid, &map, KnownPassageValidationConfig{}, config, 10.0);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.applied);
  EXPECT_EQ(result.stats.candidates, 2U);
  EXPECT_EQ(result.stats.diagnostics.size(), 1U);
  EXPECT_EQ(result.stats.diagnostics_dropped, 1U);
  EXPECT_EQ(result.stats.diagnostics.front().reason,
            PassageInsertionRejectReason::kExcessiveLateralShift);
  EXPECT_FALSE(result.stats.diagnostics.front().accepted);
}

TEST(TrajectoryPassageInsertion,
     ExistingValidOpeningDoesNotMaskUnrepairedInvalidCrossing) {
  const OccupancyGrid2D grid = makeGrid();
  const KnownPassageMap map = makeDuplicateIdMap();
  const std::vector<TrajectoryPointSample> samples = makeTwoCrossingSamples();
  const std::vector<KnownPassageTraversalMatch> before_matches =
      findKnownPassageTraversalMatches(samples, map, KnownPassageValidationConfig{},
                                       true);
  ASSERT_TRUE(std::ranges::any_of(before_matches, [](const auto& match) {
    return match.valid && match.structure_id == "arch" && match.opening_id == "main";
  }));
  ASSERT_TRUE(std::ranges::any_of(before_matches, [](const auto& match) {
    return !match.valid && match.structure_id == "arch";
  }));
  PassageInsertionConfig config = insertionConfig();
  config.max_lateral_shift_m = 1000.0;
  config.max_join_tangent_delta_rad = std::numbers::pi;
  config.max_join_curvature_jump_1pm = 1000.0;

  const PassageInsertionResult result = insertLocalPassageSegments(
      samples, grid, &map, KnownPassageValidationConfig{}, config, 10.0);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.applied);
  EXPECT_GT(result.stats.rejected_validation, 0U);
  ASSERT_FALSE(result.stats.diagnostics.empty());
  EXPECT_TRUE(std::ranges::any_of(result.stats.diagnostics, [](const auto& diagnostic) {
    return diagnostic.reason == PassageInsertionRejectReason::kValidationNotImproved;
  }));
}

} // namespace drone_city_nav
