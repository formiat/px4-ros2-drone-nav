#include "drone_city_nav/bounded_worker_pool.hpp"
#include "drone_city_nav/distance_field.hpp"
#include "drone_city_nav/raw_guide_validation.hpp"
#include "drone_city_nav/risk_aware_lattice.hpp"
#include "drone_city_nav/route_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <numbers>
#include <ranges>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] mppi::EsdfGrid makeGrid() {
  return mppi::EsdfGrid{40, 30, 1.0F, 0.0F, 0.0F};
}

TEST(RiskAwareLattice, BuildsGuideThroughOpenSpace) {
  const mppi::EsdfGrid grid = makeGrid();
  const std::vector<float> esdf(static_cast<std::size_t>(grid.width * grid.height),
                                20.0F);

  const RiskAwareLatticeResult result = planRiskAwareMotionPrimitiveGuide(
      grid, esdf, Point2{2.5, 10.5}, 0.0, Point2{34.5, 10.5}, RiskAwareLatticeConfig{});

  ASSERT_TRUE(result.valid);
  ASSERT_GE(result.guide.size(), 2U);
  EXPECT_TRUE(result.planning_goal_reached);
  EXPECT_TRUE(result.exact_terminal_connector);
  EXPECT_EQ(result.status, LatticePlanStatus::kReachedPlanningGoal);
  EXPECT_EQ(result.termination, LatticeSearchTermination::kPlanningGoalReached);
  EXPECT_DOUBLE_EQ(result.guide.back().x, 34.5);
  EXPECT_DOUBLE_EQ(result.guide.back().y, 10.5);
  EXPECT_NEAR(result.guide.front().y, 10.5, 1.0);
}

TEST(RiskAwareLattice, RejectsRawCollisionAndRoutesAroundWall) {
  const mppi::EsdfGrid grid = makeGrid();
  std::vector<float> esdf(static_cast<std::size_t>(grid.width * grid.height), 20.0F);
  for (int y = 7; y <= 13; ++y) {
    esdf[static_cast<std::size_t>(y) * static_cast<std::size_t>(grid.width) + 18U] =
        0.0F;
  }

  const RiskAwareLatticeResult result = planRiskAwareMotionPrimitiveGuide(
      grid, esdf, Point2{2.5, 10.5}, 0.0, Point2{34.5, 10.5}, RiskAwareLatticeConfig{});

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(std::ranges::any_of(result.guide, [](const Point2 point) {
    return point.y < 7.0 || point.y > 14.0;
  }));
}

TEST(RiskAwareLattice, FailsWhenStartIsOutsideWorldModel) {
  const mppi::EsdfGrid grid = makeGrid();
  const std::vector<float> esdf(static_cast<std::size_t>(grid.width * grid.height),
                                20.0F);

  const RiskAwareLatticeResult result =
      planRiskAwareMotionPrimitiveGuide(grid, esdf, Point2{-1.0, 10.0}, 0.0,
                                        Point2{20.0, 10.0}, RiskAwareLatticeConfig{});

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.status, LatticePlanStatus::kInvalidInput);
  EXPECT_EQ(result.termination, LatticeSearchTermination::kInvalidInput);
}

TEST(RiskAwareLattice, ClassifiesExhaustedMotionGraphExplicitly) {
  const mppi::EsdfGrid grid{20, 20, 1.0F, 0.0F, 0.0F};
  std::vector<float> esdf(static_cast<std::size_t>(grid.width * grid.height), 0.0F);
  for (int x = 2; x <= 6; ++x) {
    esdf[10U * static_cast<std::size_t>(grid.width) + static_cast<std::size_t>(x)] =
        20.0F;
  }

  const RiskAwareLatticeResult result = planRiskAwareMotionPrimitiveGuide(
      grid, esdf, Point2{2.5, 10.5}, 0.0, Point2{18.5, 10.5}, RiskAwareLatticeConfig{});

  ASSERT_FALSE(result.valid);
  EXPECT_TRUE(result.guide.empty());
  EXPECT_FALSE(result.planning_goal_reached);
  EXPECT_EQ(result.status, LatticePlanStatus::kMotionGraphExhausted);
  EXPECT_EQ(result.termination, LatticeSearchTermination::kOpenSetExhausted);
  EXPECT_EQ(result.terminal_successor_count, 0U);
}

TEST(RiskAwareLattice, DoesNotReachGoalThroughBlockedTerminalConnector) {
  const mppi::EsdfGrid grid = makeGrid();
  std::vector<float> esdf(static_cast<std::size_t>(grid.width * grid.height), 20.0F);
  esdf[10U * static_cast<std::size_t>(grid.width) + 34U] = 0.0F;

  const RiskAwareLatticeResult result = planRiskAwareMotionPrimitiveGuide(
      grid, esdf, Point2{2.5, 10.5}, 0.0, Point2{34.5, 10.5}, RiskAwareLatticeConfig{});

  EXPECT_FALSE(result.planning_goal_reached);
  EXPECT_FALSE(result.exact_terminal_connector);
  EXPECT_NE(result.status, LatticePlanStatus::kReachedPlanningGoal);
  EXPECT_TRUE(result.guide.empty() ||
              distance(result.guide.back(), Point2{34.5, 10.5}) > 1.0e-6);
}

TEST(RiskAwareLattice, ClassifiesUsefulBudgetLimitedGuideAsViableFrontier) {
  const mppi::EsdfGrid grid{200, 30, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(static_cast<std::size_t>(grid.width * grid.height),
                                20.0F);
  RiskAwareLatticeConfig config;
  config.maximum_expansions = 30U;

  const RiskAwareLatticeResult result = planRiskAwareMotionPrimitiveGuide(
      grid, esdf, Point2{2.5, 10.5}, 0.0, Point2{150.5, 10.5}, config);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.planning_goal_reached);
  EXPECT_EQ(result.status, LatticePlanStatus::kViableFrontier);
  EXPECT_EQ(result.termination, LatticeSearchTermination::kExpansionBudgetExhausted);
  EXPECT_GE(result.guide.size(), config.minimum_frontier_guide_points);
  EXPECT_GE(result.guide_length_m, config.minimum_frontier_guide_length_m);
  EXPECT_GE(result.frontier_endpoint_displacement_m,
            config.minimum_frontier_endpoint_displacement_m);
  EXPECT_GT(result.terminal_successor_count, 0U);
  EXPECT_GT(result.continuation_reachable_states, 0U);
  EXPECT_GE(result.reachable_depth_m, config.minimum_frontier_reachable_depth_m);
}

TEST(RiskAwareLattice, AcceptsViableFrontierWithTemporaryNegativeGoalProgress) {
  const mppi::EsdfGrid grid{100, 80, 1.0F, 0.0F, 0.0F};
  std::vector<float> esdf(static_cast<std::size_t>(grid.width * grid.height), 20.0F);
  for (int y = 5; y <= 75; ++y) {
    esdf[static_cast<std::size_t>(y) * static_cast<std::size_t>(grid.width) + 30U] =
        0.0F;
  }
  RiskAwareLatticeConfig config;
  config.maximum_expansions = 60U;
  config.maximum_search_time_ms = 1000.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_samples = 0U;

  const Point2 start{29.5, 40.5};
  const RiskAwareLatticeResult result = planRiskAwareMotionPrimitiveGuide(
      grid, esdf, start, 0.0, Point2{80.5, 40.5}, config);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.status, LatticePlanStatus::kViableFrontier);
  EXPECT_LT(result.achieved_progress_m, 0.0);
  EXPECT_GE(result.frontier_endpoint_displacement_m,
            config.minimum_frontier_endpoint_displacement_m);
  EXPECT_GE(result.reachable_depth_m, config.minimum_frontier_reachable_depth_m);
  EXPECT_TRUE(std::ranges::any_of(result.guide, [&start](const Point2 point) {
    return std::abs(point.y - start.y) >= 4.0 || point.x < start.x - 1.0;
  }));
}

TEST(RiskAwareLattice, ReturnsRawSafeDetourPrefixWhenBudgetEndsEarly) {
  const mppi::EsdfGrid grid{200, 30, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(static_cast<std::size_t>(grid.width * grid.height),
                                20.0F);
  RiskAwareLatticeConfig config;
  config.maximum_expansions = 3U;

  const RiskAwareLatticeResult result = planRiskAwareMotionPrimitiveGuide(
      grid, esdf, Point2{2.5, 10.5}, 0.0, Point2{150.5, 10.5}, config);

  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.status, LatticePlanStatus::kRawSafeDetourPrefix);
  EXPECT_EQ(result.termination, LatticeSearchTermination::kExpansionBudgetExhausted);
  EXPECT_FALSE(result.search_session_complete);
  EXPECT_GE(result.guide.size(), 2U);
}

TEST(RiskAwareLattice, ResumesPersistentSearchSessionAcrossBudgetSlices) {
  const mppi::EsdfGrid grid{200, 30, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(static_cast<std::size_t>(grid.width * grid.height),
                                20.0F);
  RiskAwareLatticeConfig config;
  config.maximum_expansions = 3U;
  RiskAwareLatticeSearchSession session;

  const RiskAwareLatticeResult first = planRiskAwareMotionPrimitiveGuide(
      grid, esdf, Point2{2.5, 10.5}, 0.0, Point2{150.5, 10.5}, config, {}, &session);
  config.maximum_expansions = 12U;
  const RiskAwareLatticeResult resumed = planRiskAwareMotionPrimitiveGuide(
      grid, esdf, Point2{2.5, 10.5}, 0.0, Point2{150.5, 10.5}, config, {}, &session);

  EXPECT_FALSE(first.search_session_resumed);
  EXPECT_TRUE(resumed.search_session_resumed);
  EXPECT_GT(resumed.expansions, first.expansions);

  session.reset();
  const RiskAwareLatticeResult restarted = planRiskAwareMotionPrimitiveGuide(
      grid, esdf, Point2{2.5, 10.5}, 0.0, Point2{150.5, 10.5}, config, {}, &session);
  EXPECT_FALSE(restarted.search_session_resumed);
}

TEST(RiskAwareLattice, BoundedContinuationProvesDepthBeyondTwoPrimitives) {
  const mppi::EsdfGrid grid{100, 60, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(static_cast<std::size_t>(grid.width * grid.height),
                                20.0F);
  RiskAwareLatticeConfig config;
  config.maximum_expansions = 30U;
  config.minimum_frontier_reachable_depth_m = 20.0;
  config.frontier_validation_maximum_states = 2048U;

  const RiskAwareLatticeResult result = planRiskAwareMotionPrimitiveGuide(
      grid, esdf, Point2{2.5, 20.5}, 0.0, Point2{90.5, 20.5}, config);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.status, LatticePlanStatus::kViableFrontier);
  EXPECT_GE(result.reachable_depth_m, 20.0);
  EXPECT_GT(result.continuation_reachable_states, 0U);
  EXPECT_FALSE(result.search_session_complete);
}

TEST(RiskAwareLattice, DetoursAroundObservedCornerAt69By123) {
  const mppi::EsdfGrid grid{180, 220, 1.0F, 0.0F, 0.0F};
  OccupancyGrid2D occupancy{GridBounds{0.0, 0.0, 1.0, grid.width, grid.height}};
  occupancy.reset(CellState::kFree);
  for (int y = 123; y < 147; ++y) {
    for (int x = 69; x < 93; ++x) {
      occupancy.setOccupied(GridIndex{x, y});
    }
  }
  const DistanceField2D field =
      DistanceField2D::build(occupancy, 26.0, DistanceFieldSource::kOccupied);
  std::vector<float> esdf;
  esdf.reserve(field.distancesM().size());
  std::ranges::transform(field.distancesM(), std::back_inserter(esdf),
                         [](const double value) { return static_cast<float>(value); });
  RiskAwareLatticeConfig config;
  config.maximum_expansions = 3000U;
  config.maximum_search_time_ms = 1000.0;
  config.minimum_frontier_reachable_depth_m = 20.0;

  const Point2 start{68.5, 122.5};
  const RiskAwareLatticeResult result = planRiskAwareMotionPrimitiveGuide(
      grid, esdf, start, std::numbers::pi / 4.0, Point2{120.5, 180.5}, config);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.status, LatticePlanStatus::kReachedPlanningGoal);
  ASSERT_GE(result.guide.size(), 2U);
  EXPECT_TRUE(validateGuideAgainstRawOccupancy(result.guide, occupancy, 0.5).accepted);
  EXPECT_TRUE(std::ranges::any_of(result.guide, [](const Point2 point) {
    return (point.x < 69.0 && point.y > 147.0) || (point.y < 123.0 && point.x > 93.0);
  }));
  const RiskAwareLatticeResult repeated = planRiskAwareMotionPrimitiveGuide(
      grid, esdf, start, std::numbers::pi / 4.0, Point2{120.5, 180.5}, config);
  ASSERT_EQ(repeated.status, LatticePlanStatus::kReachedPlanningGoal);
  EXPECT_EQ(routeFingerprint(result.guide), routeFingerprint(repeated.guide));
}

TEST(RawGuideValidation, RejectsCandidateBlockedByNewRawSnapshot) {
  const GridBounds bounds{0.0, 0.0, 1.0, 40, 30};
  OccupancyGrid2D search_snapshot{bounds};
  search_snapshot.reset(CellState::kFree);
  OccupancyGrid2D latest_snapshot = search_snapshot;
  latest_snapshot.setOccupied(GridIndex{10, 10});
  const std::vector<Point2> candidate{{2.5, 10.5}, {20.5, 10.5}};

  EXPECT_TRUE(
      validateGuideAgainstRawOccupancy(candidate, search_snapshot, 0.5).accepted);
  const RawGuideValidationResult latest =
      validateGuideAgainstRawOccupancy(candidate, latest_snapshot, 0.5);
  EXPECT_FALSE(latest.accepted);
  EXPECT_EQ(latest.status, RawGuideValidationStatus::kRawCollision);
  EXPECT_NEAR(latest.failure_point.x, 10.0, 0.5);
  EXPECT_NEAR(latest.failure_point.y, 10.5, 0.5);
}

TEST(RawGuideValidation, IgnoresBlockedPrefixBehindCurrentStation) {
  OccupancyGrid2D occupancy{GridBounds{0.0, 0.0, 1.0, 40, 30}};
  occupancy.reset(CellState::kFree);
  occupancy.setOccupied(GridIndex{5, 10});
  const std::vector<Point2> candidate{{2.5, 10.5}, {20.5, 10.5}};

  EXPECT_FALSE(validateGuideAgainstRawOccupancy(candidate, occupancy, 0.5).accepted);
  EXPECT_TRUE(
      validateGuideAgainstRawOccupancy(candidate, occupancy, 0.5, 8.0).accepted);
}

TEST(RiskAwareLattice, EscalatesToPlanningStageForCompleteRoute) {
  const mppi::EsdfGrid grid = makeGrid();
  const std::vector<float> esdf(static_cast<std::size_t>(grid.width * grid.height),
                                2.70710678F);

  const RiskAwareLatticeResult result = planRiskAwareMotionPrimitiveGuide(
      grid, esdf, Point2{2.5, 10.5}, 0.0, Point2{34.5, 10.5}, RiskAwareLatticeConfig{});

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.planning_goal_reached);
  EXPECT_EQ(result.risk_stage, LatticeRiskStage::kPlanningAllowed);
}

TEST(RiskAwareLattice, EscalatesToCriticalStageButNeverRawCollision) {
  const mppi::EsdfGrid grid = makeGrid();
  const std::vector<float> critical_esdf(
      static_cast<std::size_t>(grid.width * grid.height), 2.4F);
  RiskAwareLatticeConfig critical_config;
  critical_config.critical_distance_m = 1.5;

  const RiskAwareLatticeResult critical_result = planRiskAwareMotionPrimitiveGuide(
      grid, critical_esdf, Point2{2.5, 10.5}, 0.0, Point2{34.5, 10.5}, critical_config);

  ASSERT_TRUE(critical_result.valid);
  EXPECT_EQ(critical_result.risk_stage, LatticeRiskStage::kCriticalAllowed);

  const std::vector<float> occupied_esdf(
      static_cast<std::size_t>(grid.width * grid.height), 0.0F);
  const RiskAwareLatticeResult occupied_result =
      planRiskAwareMotionPrimitiveGuide(grid, occupied_esdf, Point2{2.5, 10.5}, 0.0,
                                        Point2{34.5, 10.5}, RiskAwareLatticeConfig{});
  EXPECT_FALSE(occupied_result.valid);
  EXPECT_EQ(occupied_result.status, LatticePlanStatus::kMotionGraphExhausted);
}

TEST(RiskAwareLattice, AllowsFreeCellsWithZeroConservativeClearanceAtCriticalStage) {
  const mppi::EsdfGrid grid = makeGrid();
  const std::vector<float> esdf(static_cast<std::size_t>(grid.width * grid.height),
                                0.5F);

  const RiskAwareLatticeResult result = planRiskAwareMotionPrimitiveGuide(
      grid, esdf, Point2{2.5, 10.5}, 0.0, Point2{34.5, 10.5}, RiskAwareLatticeConfig{});

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.guide.empty());
  EXPECT_NE(result.status, LatticePlanStatus::kMotionGraphExhausted);
  EXPECT_EQ(result.risk_stage, LatticeRiskStage::kCriticalAllowed);
  EXPECT_EQ(result.successor_diagnostics.rejected_raw_collision, 0U);
}

TEST(RiskAwareLattice, EscapesWallEvenWhenPreferredHeadingPointsIntoIt) {
  const mppi::EsdfGrid grid{50, 40, 1.0F, 0.0F, 0.0F};
  std::vector<float> esdf(static_cast<std::size_t>(grid.width * grid.height), 20.0F);
  for (int y = 8; y <= 22; ++y) {
    esdf[static_cast<std::size_t>(y) * static_cast<std::size_t>(grid.width) + 18U] =
        0.0F;
  }

  const RiskAwareLatticeResult result =
      planRiskAwareMotionPrimitiveGuide(grid, esdf, Point2{16.5, 15.5}, 0.0,
                                        Point2{40.5, 15.5}, RiskAwareLatticeConfig{});

  ASSERT_TRUE(result.valid);
  EXPECT_NE(result.status, LatticePlanStatus::kMotionGraphExhausted);
  EXPECT_GT(result.successor_diagnostics.rejected_raw_collision, 0U);
  EXPECT_TRUE(std::ranges::any_of(result.guide, [](const Point2 point) {
    return point.y < 8.0 || point.y > 23.0;
  }));
}

TEST(RiskAwareLattice, FailurePointBlocksOnlyRepeatedApproach) {
  const mppi::EsdfGrid grid{200, 40, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(static_cast<std::size_t>(grid.width * grid.height),
                                20.0F);
  RiskAwareLatticeConfig config;
  config.maximum_expansions = 300U;
  config.frontier_blacklist_radius_m = 1.0;
  const std::vector<LatticeFrontierBlacklistEntry> blacklist{
      LatticeFrontierBlacklistEntry{
          .failure_point = Point2{6.5, 10.5},
          .approach_heading_rad = 0.0,
          .expires_at_ns = 1000,
          .soft_penalty_cost = 0.0,
      },
  };

  const RiskAwareLatticeResult result = planRiskAwareMotionPrimitiveGuide(
      grid, esdf, Point2{2.5, 10.5}, 0.0, Point2{100.0, 10.5}, config, blacklist);

  ASSERT_TRUE(result.valid);
  ASSERT_GE(result.guide.size(), 2U);
  EXPECT_FALSE(
      std::ranges::all_of(std::views::drop(result.guide, 1U), [](const Point2 point) {
        return std::abs(point.y - 10.5) < 0.1;
      }));
}

TEST(RiskAwareLattice, SoftTabuChangesCostWithoutRemovingReachability) {
  const mppi::EsdfGrid grid{60, 40, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(static_cast<std::size_t>(grid.width * grid.height),
                                20.0F);
  RiskAwareLatticeConfig config;
  config.maximum_search_time_ms = 1000.0;
  config.maximum_expansions = 5000U;
  config.frontier_blacklist_radius_m = 100.0;
  config.frontier_blacklist_heading_tolerance_bins = config.heading_bins / 2;
  const std::vector<LatticeFrontierBlacklistEntry> memory{
      LatticeFrontierBlacklistEntry{.failure_point = Point2{2.5, 20.5},
                                    .approach_heading_rad = 0.0,
                                    .expires_at_ns = 1'000'000'000,
                                    .soft_penalty_cost = 20.0}};

  const RiskAwareLatticeResult result = planRiskAwareMotionPrimitiveGuide(
      grid, esdf, Point2{2.5, 20.5}, 0.0, Point2{50.5, 20.5}, config, memory);

  EXPECT_TRUE(result.valid);
  EXPECT_GT(result.successor_diagnostics.soft_tabu_penalties_applied, 0U);
  EXPECT_EQ(result.successor_diagnostics.rejected_blacklisted_failure, 0U);
}

TEST(RiskAwareLattice, ParallelFrontierValidationPreservesDeterministicResult) {
  const mppi::EsdfGrid grid{80, 60, 1.0F, 0.0F, 0.0F};
  std::vector<float> esdf(static_cast<std::size_t>(grid.width * grid.height), 20.0F);
  for (int y = 10; y <= 42; ++y) {
    esdf[static_cast<std::size_t>(y) * static_cast<std::size_t>(grid.width) + 28U] =
        0.0F;
  }
  RiskAwareLatticeConfig config;
  config.maximum_expansions = 300U;
  config.maximum_search_time_ms = 1000.0;
  config.maximum_frontier_candidates = 32U;

  const RiskAwareLatticeResult serial = planRiskAwareMotionPrimitiveGuide(
      grid, esdf, Point2{20.5, 26.5}, 0.0, Point2{70.5, 26.5}, config);
  BoundedWorkerPool worker_pool{4U};
  const RiskAwareLatticeResult parallel = planRiskAwareMotionPrimitiveGuide(
      grid, esdf, Point2{20.5, 26.5}, 0.0, Point2{70.5, 26.5}, config, {}, nullptr,
      &worker_pool);

  EXPECT_EQ(parallel.status, serial.status);
  EXPECT_EQ(parallel.risk_stage, serial.risk_stage);
  EXPECT_EQ(routeFingerprint(parallel.guide), routeFingerprint(serial.guide));
  EXPECT_EQ(parallel.frontier_candidates_considered,
            serial.frontier_candidates_considered);
  EXPECT_GE(parallel.continuation_validation_ms, 0.0);
}

} // namespace
} // namespace drone_city_nav
