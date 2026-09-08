#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

#include "persistent_dstar_lite_planner_3d_test_support.hpp"

namespace drone_city_nav {
namespace {

TEST(PersistentDStarLitePlanner3DTest,
     TheFeasibilityFirstRouteClimbsOutOfAMetreItCouldOnlyCrawlAlong) {
  // A known floor two cells thick under a corridor: the unranked feasibility
  // search would fly the lowest level right above it. Ranked within its short
  // reach, the tube law prices that level by the crawl execution would have
  // to make there — half a metre of clearance over a 0.5 s response admits
  // 1 m/s against a cruise of 5 — and the search lifts the interior of the
  // route to where the tube admits cruise.
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 14, 3, 8});
  for (int x = 0; x < 14; ++x) {
    for (int y = 0; y < 3; ++y) {
      for (int z = 0; z < 2; ++z) {
        ASSERT_TRUE(occupancy->setState({x, y, z}, ObservedVoxelState::kOccupied));
      }
    }
  }
  PersistentPlannerConfig3D config = testConfig();
  config.feasibility_first_enabled = true;
  config.clearance_ranking_weight = 1.5;
  config.clearance_ranking_distance_m = 6.0;
  config.tracking_error_tube = TrackingErrorTubeConfig3D{
      .response_time_s = 0.5, .minimum_progress_speed_mps = 0.5};
  config.feasibility_clearance_ranking_distance_m = 2.0;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{1.5, 1.5, 2.5};
  const Point3 goal{12.5, 1.5, 2.5};
  PlannerUpdate3D update = planner.plan(request(start, goal, world(occupancy, 1U)));
  for (int attempt = 0; attempt < 8 && !update.publishable(); ++attempt) {
    update = planner.plan(request(start, goal, world(occupancy, 1U)));
  }
  ASSERT_TRUE(update.publishable());
  EXPECT_TRUE(update.telemetry.feasibility_route_found);
  const std::vector<Point3>& points = candidate(update).points;
  ASSERT_GE(points.size(), 3U);
  double interior_minimum_z = std::numeric_limits<double>::infinity();
  for (std::size_t index = 1U; index + 1U < points.size(); ++index) {
    interior_minimum_z = std::min(interior_minimum_z, points[index].z);
  }
  // Node centres at z = 2.5 sit half a metre above the floor's top at 2.0;
  // the ranked search climbs at least one level, where the tube admits more.
  EXPECT_GE(interior_minimum_z, 3.4) << "interior minimum z " << interior_minimum_z;
  expectRawValid(points, *occupancy, planner.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest,
     TheRankedFeasibilitySearchStaysDirectedAcrossAnOpenWorld) {
  // A ranking curve that charged open space would loosen the unranked
  // heuristic and turn the first search into a breadth-first sweep; scaled to
  // its reach it prices only the band near occupied evidence, so a far goal
  // across an open world is reached within a few thousand explored nodes.
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 120, 120, 16});
  for (int x = 0; x < 120; ++x) {
    for (int y = 0; y < 120; ++y) {
      ASSERT_TRUE(occupancy->setState({x, y, 0}, ObservedVoxelState::kOccupied));
    }
  }
  PersistentPlannerConfig3D config = testConfig();
  config.feasibility_first_enabled = true;
  config.clearance_ranking_weight = 1.5;
  config.clearance_ranking_distance_m = 6.0;
  config.feasibility_clearance_ranking_distance_m = 2.0;
  config.maximum_compute_time_ms = 2000.0;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{2.5, 2.5, 5.5};
  const Point3 goal{117.5, 117.5, 5.5};
  PlannerUpdate3D update = planner.plan(request(start, goal, world(occupancy, 1U)));
  for (int attempt = 0; attempt < 4 && !update.publishable(); ++attempt) {
    update = planner.plan(request(start, goal, world(occupancy, 1U)));
  }
  ASSERT_TRUE(update.publishable());
  EXPECT_TRUE(update.telemetry.feasibility_route_found);
  EXPECT_LT(update.telemetry.feasibility_explored_nodes, 60000U)
      << "explored " << update.telemetry.feasibility_explored_nodes;
  expectRawValid(candidate(update).points, *occupancy,
                 planner.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest,
     PreparatoryVerticalMotionPassesThroughTheOnlyLowerOpening) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 14, 10, 8});
  for (int z = 3; z < 8; ++z) {
    for (int y = 0; y < 10; ++y) {
      static_cast<void>(
          occupancy->setState(GridIndex3D{6, y, z}, ObservedVoxelState::kOccupied));
    }
  }
  PersistentPlannerConfig3D config = testConfig();
  config.feasibility_first_enabled = true;
  config.maximum_feasibility_expansions_per_update = 100000U;
  config.maximum_feasibility_compute_time_ms = 900.0;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{2.5, 5.5, 5.5};
  const Point3 goal{11.5, 5.5, 5.5};

  const PlannerUpdate3D result =
      planner.plan(request(start, goal, world(occupancy, 1U)));

  ASSERT_TRUE(result.publishable());
  EXPECT_TRUE(result.telemetry.feasibility_route_found);
  EXPECT_LT(std::ranges::min_element(candidate(result).points, {}, &Point3::z)->z,
            start.z);
  expectRawValid(candidate(result).points, *occupancy,
                 planner.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest,
     TheFeasibilitySearchKeepsItsLabelsBeyondAnEdgeAChangeBlocked) {
  // A long corridor ends at a junction with a short exit and a long one. The
  // feasibility search reaches the goal through the short exit; a change then
  // blocks that exit. The labels along the corridor and into the long exit
  // survive the change, so the search re-routes with a fraction of the
  // expansions a search that starts over on the changed world needs.
  const auto corridorWorld = [] {
    auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
        GridBounds3D{0.0, 0.0, 0.0, 1.0, 120, 10, 3});
    for (int z = 0; z < 3; ++z) {
      for (int y = 0; y < 10; ++y) {
        for (int x = 0; x < 120; ++x) {
          static_cast<void>(
              occupancy->setState(GridIndex3D{x, y, z}, ObservedVoxelState::kOccupied));
        }
      }
    }
    const auto open = [&](const int x, const int y) {
      static_cast<void>(
          occupancy->setState(GridIndex3D{x, y, 1}, ObservedVoxelState::kUnknown));
    };
    for (int x = 1; x <= 100; ++x) {
      open(x, 1);
    }
    // The short exit: north from the junction, then east to the goal.
    for (int y = 2; y <= 8; ++y) {
      open(100, y);
    }
    for (int x = 101; x <= 110; ++x) {
      open(x, 8);
    }
    // The long exit: east past the junction, north, then back west.
    for (int x = 101; x <= 118; ++x) {
      open(x, 1);
    }
    for (int y = 2; y <= 8; ++y) {
      open(118, y);
    }
    for (int x = 111; x <= 117; ++x) {
      open(x, 8);
    }
    return occupancy;
  };
  const auto blockShortExit = [](ObservedOccupancyGrid3D& occupancy) {
    for (int y = 3; y <= 5; ++y) {
      static_cast<void>(
          occupancy.setState(GridIndex3D{100, y, 1}, ObservedVoxelState::kOccupied));
    }
  };
  PersistentPlannerConfig3D config = testConfig();
  config.feasibility_first_enabled = true;
  config.maximum_feasibility_expansions_per_update = 100000U;
  config.maximum_feasibility_compute_time_ms = 900.0;
  config.maximum_expansions_per_update = 8U;
  const Point3 start{1.5, 1.5, 1.5};
  const Point3 goal{110.5, 8.5, 1.5};
  const auto planUntilPublishable =
      [&](PersistentDStarLitePlanner3D& planner,
          const std::shared_ptr<ObservedOccupancyGrid3D>& occupancy,
          const std::uint64_t revision, std::size_t& feasibility_expansions) {
        PlannerUpdate3D update;
        for (int attempt = 0; attempt < 12; ++attempt) {
          update = planner.plan(request(start, goal, world(occupancy, revision)));
          feasibility_expansions += update.telemetry.feasibility_expansions;
          if (update.publishable()) {
            break;
          }
        }
        return update;
      };

  auto occupancy = corridorWorld();
  PersistentDStarLitePlanner3D planner{config};
  std::size_t initial_expansions = 0U;
  PlannerUpdate3D update =
      planUntilPublishable(planner, occupancy, 1U, initial_expansions);
  ASSERT_TRUE(update.publishable());
  ASSERT_TRUE(update.telemetry.feasibility_route_found);
  EXPECT_LT(std::ranges::max_element(candidate(update).points, {}, &Point3::x)->x,
            115.0);
  const std::size_t explored_before_change =
      update.telemetry.feasibility_explored_nodes;

  auto blocked_occupancy = std::make_shared<ObservedOccupancyGrid3D>(*occupancy);
  blockShortExit(*blocked_occupancy);
  std::size_t continued_expansions = 0U;
  update = planUntilPublishable(planner, blocked_occupancy, 2U, continued_expansions);
  ASSERT_TRUE(update.publishable());
  EXPECT_TRUE(update.telemetry.feasibility_route_found);
  EXPECT_GT(update.telemetry.feasibility_invalidated_labels, 0U);
  EXPECT_GT(update.telemetry.feasibility_explored_nodes, explored_before_change / 2U)
      << "labels " << update.telemetry.feasibility_explored_nodes << " of "
      << explored_before_change << " survived the change";
  EXPECT_GT(std::ranges::max_element(candidate(update).points, {}, &Point3::x)->x,
            117.0);
  expectRawValid(candidate(update).points, *blocked_occupancy,
                 planner.config().physical_footprint);

  auto changed_occupancy = corridorWorld();
  blockShortExit(*changed_occupancy);
  PersistentDStarLitePlanner3D fresh_planner{config};
  std::size_t fresh_expansions = 0U;
  const PlannerUpdate3D fresh =
      planUntilPublishable(fresh_planner, changed_occupancy, 1U, fresh_expansions);
  ASSERT_TRUE(fresh.publishable());
  ASSERT_TRUE(fresh.telemetry.feasibility_route_found);
  EXPECT_LT(2U * continued_expansions, fresh_expansions)
      << "continued " << continued_expansions << " fresh " << fresh_expansions;
}

TEST(PersistentDStarLitePlanner3DTest,
     FeasibilityIncumbentDoesNotStopTheAnytimeSessionBeforeConvergence) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 18, 18, 8});
  PersistentPlannerConfig3D config = testConfig();
  config.feasibility_first_enabled = true;
  config.maximum_feasibility_expansions_per_update = 64U;
  config.maximum_expansions_per_update = 4U;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{1.5, 1.5, 1.5};
  const Point3 goal{16.5, 16.5, 6.5};

  PlannerUpdate3D update = planner.plan(request(start, goal, world(occupancy, 1U)));
  ASSERT_TRUE(update.publishable());
  ASSERT_EQ(candidate(update).source,
            SpatialRouteCandidateSource3D::kFeasibilitySearch);
  ASSERT_EQ(update.progress, SearchProgress3D::kRunning);
  double best_objective_s = candidate(update).estimated_execution_time_s;
  std::size_t refinement_expansions = update.telemetry.execution_time_search_expansions;

  for (std::size_t continuation = 0U;
       continuation < 5000U && update.progress == SearchProgress3D::kRunning;
       ++continuation) {
    update = planner.plan(request(start, goal, world(occupancy, 1U)));
    refinement_expansions += update.telemetry.execution_time_search_expansions;
    if (update.publishable()) {
      EXPECT_LT(candidate(update).estimated_execution_time_s, best_objective_s);
      best_objective_s = candidate(update).estimated_execution_time_s;
    }
  }

  EXPECT_EQ(update.progress, SearchProgress3D::kConverged);
  EXPECT_TRUE(update.telemetry.execution_time_search_complete);
  EXPECT_TRUE(update.telemetry.incumbent_available);
  EXPECT_GT(refinement_expansions, 0U);
}

TEST(PersistentDStarLitePlanner3DTest,
     FeasibilityFirstSearchUsesTheFull3DTimeObjectiveForARawValidDetour) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 30, 20, 5});
  for (int z = 0; z < 5; ++z) {
    for (int y = 3; y < 20; ++y) {
      ASSERT_TRUE(
          occupancy->setState(GridIndex3D{12, y, z}, ObservedVoxelState::kOccupied));
    }
  }
  PersistentPlannerConfig3D config = testConfig();
  config.feasibility_first_enabled = true;
  config.physical_footprint.radius_m = 0.4;
  config.physical_footprint.lower_extent_m = 0.2;
  config.physical_footprint.upper_extent_m = 0.2;
  config.physical_footprint.perimeter_samples = 8U;
  config.physical_footprint.radial_rings = 1U;
  config.physical_footprint.axial_samples = 3U;
  config.physical_footprint.sweep_step_m = 0.25;
  const Point3 start{2.5, 3.5, 2.5};
  const Point3 goal{27.5, 17.5, 2.5};
  ASSERT_FALSE(validateRawSweptFootprint(*occupancy, start, FootprintBodyAxis{}, goal,
                                         FootprintBodyAxis{}, config.physical_footprint)
                   .accepted());

  PersistentDStarLitePlanner3D planner{config};
  const PlannerUpdate3D result =
      planner.plan(request(start, goal, world(occupancy, 1U)));

  ASSERT_TRUE(result.publishable());
  EXPECT_TRUE(result.telemetry.feasibility_attempted);
  EXPECT_TRUE(result.telemetry.feasibility_route_found)
      << "feasibility_expansions=" << result.telemetry.feasibility_expansions
      << " raw_checks=" << result.telemetry.raw_edge_validation_checks
      << " graph_expansions=" << result.telemetry.expansions
      << " records=" << result.telemetry.records
      << " open=" << result.telemetry.open_entries
      << " search_ms=" << result.telemetry.search_ms;
  EXPECT_GT(result.telemetry.feasibility_expansions, 1U);
  EXPECT_GT(candidate(result).points.size(), 2U);
  expectRawValid(candidate(result).points, *occupancy,
                 planner.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest,
     ExhaustedFeasibilitySliceStillAdvancesTheResidentOptimalSearch) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 24, 16, 5});
  for (int z = 0; z < 5; ++z) {
    for (int y = 2; y < 16; ++y) {
      ASSERT_TRUE(
          occupancy->setState(GridIndex3D{11, y, z}, ObservedVoxelState::kOccupied));
    }
  }
  PersistentPlannerConfig3D config = testConfig();
  config.feasibility_first_enabled = true;
  config.maximum_feasibility_expansions_per_update = 1U;
  config.maximum_expansions_per_update = 4U;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{2.5, 3.5, 2.5};
  const Point3 goal{21.5, 13.5, 2.5};

  const PlannerUpdate3D initial =
      planner.plan(request(start, goal, world(occupancy, 1U)));
  ASSERT_EQ(initial.progress, SearchProgress3D::kRunning);
  EXPECT_TRUE(initial.telemetry.feasibility_attempted);
  EXPECT_FALSE(initial.telemetry.feasibility_route_found);
  EXPECT_EQ(initial.telemetry.feasibility_expansions, 1U);
  EXPECT_GT(initial.telemetry.expansions, 0U);

  const PlannerUpdate3D continued =
      planner.plan(request(start, goal, world(occupancy, 1U)));
  EXPECT_TRUE(continued.telemetry.search_state_reused);
  EXPECT_EQ(continued.telemetry.search_generation, initial.telemetry.search_generation);
  EXPECT_GE(continued.telemetry.records, initial.telemetry.records);
  EXPECT_GT(continued.telemetry.expansions, 0U);
}

TEST(PersistentDStarLitePlanner3DTest,
     FeasibilityFirstFrontierSurvivesBoundedContinuationSlices) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 32, 24, 5});
  for (int z = 0; z < 5; ++z) {
    for (int y = 0; y <= 20; ++y) {
      ASSERT_TRUE(
          occupancy->setState(GridIndex3D{14, y, z}, ObservedVoxelState::kOccupied));
    }
  }
  PersistentPlannerConfig3D config = testConfig();
  config.maximum_adaptive_lattice_level = 0U;
  config.feasibility_first_enabled = true;
  config.maximum_feasibility_expansions_per_update = 64U;
  config.maximum_expansions_per_update = 1U;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{2.5, 4.5, 2.5};
  const Point3 goal{29.5, 4.5, 2.5};

  PlannerUpdate3D result;
  for (std::size_t slice = 0U;
       slice < 128U && !result.telemetry.feasibility_route_found; ++slice) {
    result = planner.plan(request(start, goal, world(occupancy, 1U)));
  }

  ASSERT_TRUE(result.telemetry.feasibility_route_found)
      << "records=" << result.telemetry.records
      << " open=" << result.telemetry.open_entries;
  ASSERT_TRUE(result.publishable());
  EXPECT_EQ(candidate(result).source,
            SpatialRouteCandidateSource3D::kFeasibilitySearch);
  EXPECT_TRUE(result.telemetry.search_state_reused);
  EXPECT_GT(candidate(result).points.size(), 2U);
  expectRawValid(candidate(result).points, *occupancy,
                 planner.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest,
     FeasibilityFirstSearchReplacesAnIncumbentBlockedByNewRawEvidence) {
  auto initial_occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 22, 12, 5});
  PersistentPlannerConfig3D config = testConfig();
  config.feasibility_first_enabled = true;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{1.5, 5.5, 2.5};
  const Point3 goal{20.5, 5.5, 2.5};
  const PlannerUpdate3D initial =
      planner.plan(request(start, goal, world(initial_occupancy, 1U)));
  ASSERT_TRUE(initial.publishable());
  ASSERT_TRUE(initial.telemetry.feasibility_route_found);

  auto changed = std::make_shared<ObservedOccupancyGrid3D>(*initial_occupancy);
  const GridIndex3D obstacle{11, 5, 2};
  ASSERT_TRUE(changed->setState(obstacle, ObservedVoxelState::kOccupied));
  const PlannerUpdate3D replacement = planner.plan(
      request(start, goal,
              world(changed, 2U, {ObservedOccupancyGrid3D::chunkIndex(obstacle)})));

  ASSERT_TRUE(replacement.publishable());
  EXPECT_FALSE(replacement.telemetry.incumbent_retained);
  EXPECT_TRUE(replacement.telemetry.feasibility_attempted);
  EXPECT_TRUE(replacement.telemetry.feasibility_route_found);
  EXPECT_GT(candidate(replacement).points.size(), candidate(initial).points.size());
  expectRawValid(candidate(replacement).points, *changed,
                 planner.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest,
     TheFeasibilitySearchTakesTheUpdateBeyondThePersistentSessionReserve) {
  using std::chrono::milliseconds;
  // The session keeps its configured reserve, at most a third of the update.
  EXPECT_EQ(feasibilitySearchBudget3D(milliseconds{150}, milliseconds{140}, true),
            milliseconds{100});
  EXPECT_EQ(feasibilitySearchBudget3D(milliseconds{150}, milliseconds{140}, false),
            milliseconds{150});
  EXPECT_EQ(feasibilitySearchBudget3D(milliseconds{150}, milliseconds{40}, true),
            milliseconds{110});
  EXPECT_EQ(feasibilitySearchBudget3D(milliseconds{60}, milliseconds{50}, true),
            milliseconds{40});
  EXPECT_EQ(feasibilitySearchBudget3D(milliseconds{-5}, milliseconds{140}, true),
            milliseconds{0});
  EXPECT_EQ(feasibilitySearchBudget3D(milliseconds{150}, milliseconds{-1}, true),
            milliseconds{150});
}

TEST(PersistentDStarLitePlanner3DTest,
     AStartInContactWithObservedEvidenceStillAnchorsTheSearch) {
  // Observed evidence closed in on the vehicle's own cell. Without the
  // proprioceptive seed no departure validates and the planner reports the
  // start unavailable; with it the departure to a free anchor is contact
  // evidence and the search proceeds from the vehicle's true pose.
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 18, 18, 8});
  ASSERT_TRUE(occupancy->setState(GridIndex3D{1, 1, 1}, ObservedVoxelState::kOccupied));
  PersistentPlannerConfig3D config = testConfig();
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{1.5, 1.5, 1.5};
  const Point3 goal{16.5, 16.5, 6.5};

  const PlannerUpdate3D unseeded =
      planner.plan(request(start, goal, world(occupancy, 1U)));
  EXPECT_EQ(unseeded.input_status, PlannerInputStatus3D::kStartUnavailable);

  PersistentPlannerWorld3D seeded_world = world(occupancy, 1U);
  seeded_world.proprioceptive_free_space_seed = ProprioceptiveFreeSpaceSeed3D{
      .position = start,
      .body_axis = FootprintBodyAxis{},
      .footprint = config.physical_footprint,
      .contact_tolerance_m = 0.5,
  };
  const PlannerUpdate3D seeded =
      planner.plan(request(start, goal, std::move(seeded_world)));
  ASSERT_EQ(seeded.input_status, PlannerInputStatus3D::kAccepted);
  EXPECT_NE(seeded.progress, SearchProgress3D::kInvalidated);
  ASSERT_TRUE(seeded.improved_incumbent.has_value());
  const std::vector<Point3>& path = candidate(seeded).points;
  ASSERT_GE(path.size(), 2U);
  EXPECT_NEAR(path.front().x, start.x, 1.0e-9);
  EXPECT_NEAR(path.front().y, start.y, 1.0e-9);
  EXPECT_NEAR(path.front().z, start.z, 1.0e-9);
  EXPECT_NEAR(path.back().x, goal.x, 1.0e-9);
  EXPECT_NEAR(path.back().y, goal.y, 1.0e-9);
  EXPECT_NEAR(path.back().z, goal.z, 1.0e-9);
}

TEST(PersistentDStarLitePlanner3DTest, AStaleSeedIsReanchoredAtTheRequestStart) {
  // The request's seed was captured a continuation earlier, two metres from
  // where the vehicle now stands in contact with observed evidence. The seed
  // is the vehicle's own pose, so the planner re-anchors it at the start and
  // the departure validates as it does for the node's own validators; the
  // telemetry reports how far the copy had lagged.
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 18, 18, 8});
  ASSERT_TRUE(occupancy->setState(GridIndex3D{1, 1, 1}, ObservedVoxelState::kOccupied));
  PersistentPlannerConfig3D config = testConfig();
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{1.5, 1.5, 1.5};
  const Point3 goal{16.5, 16.5, 6.5};

  PersistentPlannerWorld3D stale_world = world(occupancy, 1U);
  stale_world.proprioceptive_free_space_seed = ProprioceptiveFreeSpaceSeed3D{
      .position = Point3{3.5, 1.5, 1.5},
      .body_axis = FootprintBodyAxis{},
      .footprint = config.physical_footprint,
      .contact_tolerance_m = 0.5,
  };
  const PlannerUpdate3D update =
      planner.plan(request(start, goal, std::move(stale_world)));

  ASSERT_EQ(update.input_status, PlannerInputStatus3D::kAccepted);
  EXPECT_NEAR(update.telemetry.departure_seed_distance_m, 2.0, 1.0e-9);
  EXPECT_NEAR(update.telemetry.departure_seed_contact_tolerance_m, 0.5, 1.0e-9);
  ASSERT_TRUE(update.improved_incumbent.has_value());
  const std::vector<Point3>& path = candidate(update).points;
  ASSERT_GE(path.size(), 2U);
  EXPECT_NEAR(path.front().x, start.x, 1.0e-9);
  EXPECT_NEAR(path.front().y, start.y, 1.0e-9);
}

TEST(PersistentDStarLitePlanner3DTest,
     ALabelDroppedNearTheAnchorReparentsTheExploredTreeInsteadOfDroppingIt) {
  // A corridor explored end to end; then occupied evidence closes its second
  // cross-section except one corner cell. Every chain beyond the closure
  // breaks near the anchor. The labels behind it are re-parented through
  // the intact corner instead of being dropped and rebuilt one by one.
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 18, 3, 3});
  PersistentPlannerConfig3D config = testConfig();
  config.feasibility_first_enabled = true;
  // A short goal connector keeps the route on lattice nodes all the way.
  config.feasibility_goal_connector_reach_m = 2.0;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{1.5, 1.5, 1.5};
  const Point3 goal{16.5, 1.5, 1.5};

  PlannerUpdate3D update = planner.plan(request(start, goal, world(occupancy, 1U)));
  for (int attempt = 0; attempt < 20 && !update.telemetry.feasibility_route_found;
       ++attempt) {
    update = planner.plan(request(start, goal, world(occupancy, 1U)));
  }
  ASSERT_TRUE(update.telemetry.feasibility_route_found);
  const std::size_t explored_before = update.telemetry.feasibility_explored_nodes;
  ASSERT_GT(explored_before, 30U);

  auto changed = std::make_shared<ObservedOccupancyGrid3D>(*occupancy);
  for (int y = 0; y < 3; ++y) {
    for (int z = 0; z < 3; ++z) {
      if (y == 0 && z == 0) {
        continue;
      }
      ASSERT_TRUE(
          changed->setState(GridIndex3D{2, y, z}, ObservedVoxelState::kOccupied));
    }
  }

  update = planner.plan(request(start, goal, world(changed, 2U)));
  for (int attempt = 0; attempt < 20 && !update.publishable(); ++attempt) {
    update = planner.plan(request(start, goal, world(changed, 2U)));
  }
  ASSERT_TRUE(update.publishable());
  EXPECT_GT(update.telemetry.feasibility_adopted_labels, 0U);
  EXPECT_LT(update.telemetry.feasibility_invalidated_labels, explored_before / 2U);
  for (const Point3& point : candidate(update).points) {
    const std::optional<GridIndex3D> cell = changed->worldToCell(point);
    ASSERT_TRUE(cell.has_value());
    EXPECT_NE(changed->state(*cell), ObservedVoxelState::kOccupied);
  }
}

TEST(PersistentDStarLitePlanner3DTest,
     EveryPlannerCallReturnsWithinItsBudgetUnderOccupiedChurnAroundTheAnchor) {
  // Occupied evidence flickers around the anchor on every call while the
  // search runs on a bounded budget: no call may hold the thread beyond a
  // small multiple of that budget, whatever the labels behind the churn do.
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 24, 12, 6});
  PersistentPlannerConfig3D config = testConfig();
  config.feasibility_first_enabled = true;
  config.feasibility_goal_connector_reach_m = 2.0;
  config.maximum_compute_time_ms = 20.0;
  config.maximum_feasibility_compute_time_ms = 8.0;
  config.maximum_expansions_per_update = 400U;
  config.maximum_feasibility_expansions_per_update = 400U;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{1.5, 5.5, 2.5};
  const Point3 goal{22.5, 5.5, 2.5};

  std::uint64_t revision{1U};
  std::uint32_t state{12345U};
  const auto next_random = [&state]() noexcept {
    state = state * 1664525U + 1013904223U;
    return state >> 8U;
  };
  double worst_call_ms{0.0};
  for (int call = 0; call < 200; ++call) {
    auto changed = std::make_shared<ObservedOccupancyGrid3D>(*occupancy);
    for (int toggle = 0; toggle < 6; ++toggle) {
      const GridIndex3D cell{2 + static_cast<int>(next_random() % 6U),
                             static_cast<int>(next_random() % 12U),
                             static_cast<int>(next_random() % 6U)};
      if (distance3D(changed->cellCenter(cell), start) < 1.0) {
        continue;
      }
      const bool occupied = changed->state(cell) == ObservedVoxelState::kOccupied;
      static_cast<void>(changed->setState(
          cell, occupied ? ObservedVoxelState::kFree : ObservedVoxelState::kOccupied));
    }
    occupancy = std::move(changed);
    const auto started = std::chrono::steady_clock::now();
    const PlannerUpdate3D update =
        planner.plan(request(start, goal, world(occupancy, ++revision)));
    const double call_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - started)
                               .count();
    worst_call_ms = std::max(worst_call_ms, call_ms);
    ASSERT_NE(update.input_status, PlannerInputStatus3D::kInvalidInput);
    ASSERT_LT(call_ms, 500.0) << "call " << call << " took " << call_ms << " ms";
  }
  EXPECT_LT(worst_call_ms, 500.0);
}

TEST(PersistentDStarLitePlanner3DTest,
     AGoalInsideOccupiedEvidenceIsReachedWithinItsTolerance) {
  // The goal's own cell is occupied — a floor the lidar has only just seen
  // under a goal set a hand's breadth above it. No node reaches the goal, but
  // the free space around it is within the goal tolerance: the search ends at
  // the nearest such point instead of reporting the goal unavailable.
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 14, 6, 6});
  const Point3 start{1.5, 2.5, 2.5};
  const Point3 goal{12.5, 2.5, 2.5};
  ASSERT_TRUE(
      occupancy->setState(GridIndex3D{12, 2, 2}, ObservedVoxelState::kOccupied));
  PersistentPlannerConfig3D config = testConfig();
  config.feasibility_first_enabled = true;
  config.goal_tolerance_m = 2.0;
  config.departure_refinement_subdivisions = 4U;
  config.maximum_departure_refinement_probes = 512U;
  PersistentDStarLitePlanner3D planner{config};
  PlannerUpdate3D update = planner.plan(request(start, goal, world(occupancy, 1U)));
  for (int attempt = 0; attempt < 64 && !update.publishable(); ++attempt) {
    update = planner.plan(request(start, goal, world(occupancy, 1U)));
  }
  ASSERT_EQ(update.input_status, PlannerInputStatus3D::kAccepted);
  EXPECT_TRUE(update.telemetry.goal_refined);
  ASSERT_TRUE(update.publishable());
  const std::vector<Point3>& points = candidate(update).points;
  ASSERT_GE(points.size(), 2U);
  const Point3& endpoint = points.back();
  EXPECT_GT(distance3D(endpoint, goal), 0.0);
  EXPECT_LE(distance3D(endpoint, goal), config.goal_tolerance_m);
  EXPECT_NEAR(distance3D(endpoint, update.telemetry.search_goal), 0.0, 1.0e-9);
  expectRawValid(points, *occupancy, planner.config().physical_footprint);

  // A goal in free space is reached exactly, without refinement.
  PersistentDStarLitePlanner3D exact_planner{config};
  const Point3 free_goal{12.5, 4.5, 2.5};
  PlannerUpdate3D exact =
      exact_planner.plan(request(start, free_goal, world(occupancy, 1U)));
  for (int attempt = 0; attempt < 64 && !exact.publishable(); ++attempt) {
    exact = exact_planner.plan(request(start, free_goal, world(occupancy, 1U)));
  }
  ASSERT_TRUE(exact.publishable());
  EXPECT_FALSE(exact.telemetry.goal_refined);
  EXPECT_NEAR(distance3D(candidate(exact).points.back(), free_goal), 0.0, 1.0e-9);
}

} // namespace
} // namespace drone_city_nav
