#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

#include "persistent_dstar_lite_planner_3d_internal.hpp"
#include "persistent_dstar_lite_planner_3d_test_support.hpp"

// Edge refinement, departure waypoints, the start anchor and the escape
// search: how the planner reaches a route where no straight lattice segment
// admits the body.

namespace drone_city_nav {
namespace {

// A corridor crossed by a thin wall whose doorway is offset from the lattice
// rows: on a 2 m lattice over a 0.25 m map with a 0.5 m body, the rows lie at
// y = 1, 3 and 5, the wall stands at x in [8, 8.25) between the nodes at x = 7
// and x = 9, and the door spans y in [2.75, 4.25). The straight row segment at
// y = 3 clips the lower jamb, the diagonals clip a jamb too, and a lateral
// step to y = 3.5 at the wall clears it.
struct RefinedDoorwayFixture {
  static constexpr double kResolution = 0.25;
  static constexpr int kWidth = 64;  // 16 m
  static constexpr int kHeight = 24; // 6 m
  static constexpr int kDepth = 16;  // 4 m
  static constexpr int kWallX = 32;  // x in [8, 8.25)
  std::shared_ptr<ObservedOccupancyGrid3D> occupancy{
      std::make_shared<ObservedOccupancyGrid3D>(
          GridBounds3D{0.0, 0.0, 0.0, kResolution, kWidth, kHeight, kDepth})};
  PersistentPlannerConfig3D config{testConfig()};
  Point3 start{3.0, 3.0, 3.0};
  Point3 goal{13.0, 3.0, 3.0};
  // The door edge on the upper lattice level (z = 3).
  detail::PersistentPlannerNode3D door_west{3, 1, 1};
  detail::PersistentPlannerNode3D door_east{4, 1, 1};

  RefinedDoorwayFixture() {
    for (int y = 0; y < kHeight; ++y) {
      const double y_m = static_cast<double>(y) * kResolution;
      if (y_m >= 2.75 && y_m < 4.25) {
        continue;
      }
      for (int z = 0; z < kDepth; ++z) {
        if (!occupancy->setState({kWallX, y, z}, ObservedVoxelState::kOccupied)) {
          throw std::logic_error{"fixture cell outside the grid"};
        }
      }
    }
    config.minimum_horizontal_step_m = 2.0;
    config.minimum_vertical_step_m = 2.0;
    config.physical_footprint.radius_m = 0.5;
    config.physical_footprint.perimeter_samples = 8U;
    config.physical_footprint.radial_rings = 1U;
    config.physical_footprint.axial_samples = 1U;
    config.physical_footprint.sweep_step_m = 0.1;
    config.edge_refinement_offsets = 3U;
    config.maximum_edge_refinement_probes = 256U;
  }

  // Cells above the upper level's waypoint at (8, 3.5, 3): x in [7.75, 8.25),
  // y in [3.75, 4.25), z in [2.75, 3.25). They touch the body at the waypoint
  // and on both legs, and their centres lie at least 0.875 m from the straight
  // row segment at y = 3 against a body margin of 0.5 m plus a half voxel
  // diagonal. The lower level's door at z = 1 is untouched.
  [[nodiscard]] static std::vector<GridIndex3D> cellsAboveTheUpperWaypoint() {
    std::vector<GridIndex3D> cells;
    for (const int x : {31, 32}) {
      for (const int y : {15, 16}) {
        for (const int z : {11, 12}) {
          cells.push_back({x, y, z});
        }
      }
    }
    return cells;
  }

  [[nodiscard]] static std::shared_ptr<ObservedOccupancyGrid3D>
  withCells(const ObservedOccupancyGrid3D& base,
            const std::vector<GridIndex3D>& cells) {
    auto changed = std::make_shared<ObservedOccupancyGrid3D>(base);
    for (const GridIndex3D cell : cells) {
      if (!changed->setState(cell, ObservedVoxelState::kOccupied)) {
        throw std::logic_error{"changed cell outside the grid"};
      }
    }
    return changed;
  }

  [[nodiscard]] static std::vector<OccupancyChunkIndex3D>
  chunksOf(const std::vector<GridIndex3D>& cells) {
    std::vector<OccupancyChunkIndex3D> chunks;
    for (const GridIndex3D cell : cells) {
      chunks.push_back(ObservedOccupancyGrid3D::chunkIndex(cell));
    }
    return chunks;
  }

  [[nodiscard]] static PlannerUpdate3D
  planUntilPublishable(PersistentDStarLitePlanner3D& planner,
                       const PersistentPlannerRequest3D& req) {
    PlannerUpdate3D update = planner.plan(req);
    for (int attempt = 0; attempt < 8 && !update.publishable(); ++attempt) {
      update = planner.plan(req);
    }
    return update;
  }
};

TEST(PersistentDStarLitePlanner3DTest, ARouteThroughARefinedEdgeCarriesItsWaypoint) {
  RefinedDoorwayFixture fixture;

  PersistentPlannerConfig3D unrefined_config = fixture.config;
  unrefined_config.edge_refinement_offsets = 0U;
  PersistentDStarLitePlanner3D unrefined{unrefined_config};
  const PlannerUpdate3D stuck = fixture.planUntilPublishable(
      unrefined, request(fixture.start, fixture.goal, world(fixture.occupancy, 1U)));
  ASSERT_FALSE(stuck.publishable()) << "the fixture no longer needs a refined edge";

  PersistentDStarLitePlanner3D planner{fixture.config};
  const PlannerUpdate3D update = fixture.planUntilPublishable(
      planner, request(fixture.start, fixture.goal, world(fixture.occupancy, 1U)));
  ASSERT_TRUE(update.publishable())
      << "edge refinement did not open the doorway: input="
      << static_cast<int>(update.input_status)
      << " expansions=" << update.telemetry.expansions
      << " time_expansions=" << update.telemetry.execution_time_search_expansions;
  const std::vector<Point3>& points = candidate(update).points;
  // The waypoint is off the lattice row and inside the door.
  const bool carries_waypoint = std::ranges::any_of(points, [](const Point3& point) {
    return point.x > 7.5 && point.x < 8.5 && point.y > 3.1 && point.y < 4.0;
  });
  EXPECT_TRUE(carries_waypoint);
  expectRawValid(points, *fixture.occupancy, planner.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest, ARejectedSegmentNamesTheEdgeThatAdmittedIt) {
  // The raw sweep is the authority on a candidate, and a leg it rejects has
  // to reach the cache entry that admitted it: a straight edge, or a refined
  // edge through its waypoint. Rejecting the edge forgets it and records it
  // for the persistent session's repair.
  RefinedDoorwayFixture fixture;
  detail::PlannerLattice3D lattice{fixture.config};
  const PersistentPlannerWorld3D initial = world(fixture.occupancy, 1U);
  lattice.configureGridGeometry(*initial.bounds());
  lattice.installWorld(initial);
  ASSERT_TRUE(
      std::isfinite(lattice.rankedEdgeCost(fixture.door_west, fixture.door_east)));
  const std::optional<Point3> waypoint =
      lattice.edgeWaypoint(fixture.door_west, fixture.door_east);
  ASSERT_TRUE(waypoint.has_value()) << "the doorway edge was not refined";
  EXPECT_NEAR(waypoint->x, 8.0, 1.0e-9);
  EXPECT_NEAR(waypoint->y, 3.5, 1.0e-9);
  const Point3 west = lattice.pointFor(fixture.door_west);
  const Point3 east = lattice.pointFor(fixture.door_east);

  const std::vector<Point3> through_waypoint{fixture.start, west, *waypoint, east};
  for (const std::size_t leg : {2U, 3U}) {
    const std::optional<detail::PlannerLattice3D::PathSegmentEdge3D> priced =
        lattice.pricedEdgeForSegment(through_waypoint, leg);
    ASSERT_TRUE(priced.has_value()) << "leg " << leg;
    EXPECT_EQ(priced->from, fixture.door_west);
    EXPECT_EQ(priced->to, fixture.door_east);
  }
  // The departure from the exact start is not a lattice edge.
  EXPECT_FALSE(
      lattice.pricedEdgeForSegment({Point3{6.4, 3.2, 3.0}, west}, 1U).has_value());
  // A straight edge names itself.
  const std::optional<detail::PlannerLattice3D::PathSegmentEdge3D> straight =
      lattice.pricedEdgeForSegment({west, lattice.pointFor({2, 1, 1})}, 1U);
  ASSERT_TRUE(straight.has_value());
  EXPECT_EQ(straight->from, fixture.door_west);
  EXPECT_EQ(straight->to, (detail::PersistentPlannerNode3D{2, 1, 1}));

  const detail::PersistentPlannerEdge3D edge =
      detail::canonicalEdge(fixture.door_west, fixture.door_east);
  EXPECT_TRUE(lattice.rejectEdgeBySweep(edge));
  EXPECT_FALSE(lattice.edgeWaypoint(fixture.door_west, fixture.door_east).has_value());
  EXPECT_FALSE(lattice.hasEdgeCost(edge));
  EXPECT_FALSE(lattice.rejectEdgeBySweep(edge)) << "nothing was cached to reject";
  const std::vector<detail::PersistentPlannerEdge3D> recorded =
      lattice.takeSweepRejectedEdges();
  ASSERT_EQ(recorded.size(), 1U);
  EXPECT_EQ(recorded.front(), edge);
  EXPECT_TRUE(lattice.takeSweepRejectedEdges().empty());
}

TEST(PersistentDStarLitePlanner3DTest,
     AChangeBesideARefinedEdgesWaypointForgetsTheEdge) {
  // The change scheduling tests the geometry the vehicle flies. A cell that
  // touches a refined edge's leg through its waypoint, but lies farther from
  // the straight node-to-node segment than the body margin, has to forget the
  // edge all the same; otherwise the cache keeps admitting a leg the sweep
  // rejects, and every search on the lattice rediscovers the same blocked
  // route for as long as the cell stays.
  RefinedDoorwayFixture fixture;
  detail::PlannerLattice3D lattice{fixture.config};
  const PersistentPlannerWorld3D initial = world(fixture.occupancy, 1U);
  lattice.configureGridGeometry(*initial.bounds());
  lattice.installWorld(initial);
  ASSERT_TRUE(
      std::isfinite(lattice.rankedEdgeCost(fixture.door_west, fixture.door_east)));
  const std::optional<Point3> waypoint =
      lattice.edgeWaypoint(fixture.door_west, fixture.door_east);
  ASSERT_TRUE(waypoint.has_value()) << "the doorway edge was not refined";

  const std::vector<GridIndex3D> cells =
      RefinedDoorwayFixture::cellsAboveTheUpperWaypoint();
  const PersistentPlannerWorld3D after =
      world(RefinedDoorwayFixture::withCells(*fixture.occupancy, cells), 2U,
            RefinedDoorwayFixture::chunksOf(cells));
  lattice.installWorld(after);
  const Point3 west = lattice.pointFor(fixture.door_west);
  const Point3 east = lattice.pointFor(fixture.door_east);
  ASSERT_FALSE(lattice.pathTraversable({west, *waypoint, east}))
      << "the cells do not block the refined legs";
  ASSERT_TRUE(lattice.edgeWaypoint(fixture.door_west, fixture.door_east).has_value())
      << "installing the world must not forget edges by itself";

  detail::DStarLiteSession3D session{fixture.config, lattice};
  std::size_t affected_states = 0U;
  session.scheduleAffectedVertices(after, cells, affected_states);
  EXPECT_FALSE(lattice.edgeWaypoint(fixture.door_west, fixture.door_east).has_value());
  EXPECT_FALSE(
      lattice.hasEdgeCost(detail::canonicalEdge(fixture.door_west, fixture.door_east)));
}

TEST(PersistentDStarLitePlanner3DTest,
     AChangeBesideTheWaypointReroutesWithoutRestartingTheFeasibilitySearch) {
  // The doorway is refined on both lattice levels. Cells that block the upper
  // level's waypoint leave the lower one: the searches have to re-route
  // through it from their labels, not restart from nothing and rediscover
  // the blocked edge from the cache on every update.
  RefinedDoorwayFixture fixture;
  fixture.config.feasibility_first_enabled = true;
  PersistentDStarLitePlanner3D planner{fixture.config};
  PlannerUpdate3D update = fixture.planUntilPublishable(
      planner, request(fixture.start, fixture.goal, world(fixture.occupancy, 1U)));
  ASSERT_TRUE(update.publishable());

  const std::vector<GridIndex3D> cells =
      RefinedDoorwayFixture::cellsAboveTheUpperWaypoint();
  const std::shared_ptr<ObservedOccupancyGrid3D> changed =
      RefinedDoorwayFixture::withCells(*fixture.occupancy, cells);
  PersistentPlannerRequest3D blocked =
      request(fixture.start, fixture.goal,
              world(changed, 2U, RefinedDoorwayFixture::chunksOf(cells)));
  // The consumer could not enter the route it held, so the feasibility
  // search runs.
  blocked.incumbent_rejection_sequence = 1U;
  std::size_t restarts = 0U;
  for (int attempt = 0; attempt < 12; ++attempt) {
    update = planner.plan(blocked);
    restarts = std::max(restarts, update.telemetry.feasibility_restarts);
    if (update.publishable()) {
      break;
    }
  }
  ASSERT_TRUE(update.publishable())
      << "no route below the blocked waypoint: restarts=" << restarts
      << " closest_goal_m=" << update.telemetry.feasibility_closest_goal_distance_m
      << " exhausted=" << update.telemetry.feasibility_frontier_exhausted;
  EXPECT_EQ(restarts, 0U) << "the search restarted instead of forgetting the edge";
  const std::vector<Point3>& points = candidate(update).points;
  expectRawValid(points, *changed, planner.config().physical_footprint);
  EXPECT_TRUE(std::ranges::any_of(points, [](const Point3& point) {
    return point.x > 7.5 && point.x < 8.5 && point.z < 2.6;
  })) << "the route does not pass the door below the blocked cells";
}

TEST(PersistentDStarLitePlanner3DTest,
     RepairRunsWhileTheFeasibilitySearchLooksForAFirstRoute) {
  // No route is held and the world just changed: the feasibility search needs
  // most of the update, and the persistent session's repair queue is what
  // makes its labels true again. The repair has to get a share of the update
  // before the feasibility search takes the rest, or the session reports a
  // shortest path on labels the world has moved for as long as no route exists.
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 40, 40, 3});
  PersistentPlannerConfig3D config = testConfig();
  config.feasibility_first_enabled = true;
  config.maximum_compute_time_ms = 20.0;
  config.maximum_feasibility_compute_time_ms = 19.0;
  config.maximum_feasibility_expansions_per_update = 1U << 20U;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{1.5, 20.5, 1.5};
  const Point3 goal{38.5, 20.5, 1.5};
  PlannerUpdate3D update = planner.plan(request(start, goal, world(occupancy, 1U)));
  for (int attempt = 0; attempt < 8 && !update.publishable(); ++attempt) {
    update = planner.plan(request(start, goal, world(occupancy, 1U)));
  }
  ASSERT_TRUE(update.publishable())
      << "input=" << static_cast<int>(update.input_status)
      << " feasibility_expansions=" << update.telemetry.feasibility_expansions
      << " expansions=" << update.telemetry.expansions
      << " closest_goal_m=" << update.telemetry.feasibility_closest_goal_distance_m;

  // Enclose the goal: the labels around it move, and no route exists, so the
  // feasibility search runs to its deadline on every update.
  auto enclosed = std::make_shared<ObservedOccupancyGrid3D>(*occupancy);
  std::vector<OccupancyChunkIndex3D> dirty;
  for (int x = 35; x < 40; ++x) {
    for (int y = 17; y < 24; ++y) {
      if (x == 35 || y == 17 || y == 23) {
        for (int z = 0; z < 3; ++z) {
          const GridIndex3D cell{x, y, z};
          ASSERT_TRUE(enclosed->setState(cell, ObservedVoxelState::kOccupied));
          dirty.push_back(ObservedOccupancyGrid3D::chunkIndex(cell));
        }
      }
    }
  }
  PersistentPlannerRequest3D blocked =
      request(start, goal, world(enclosed, 2U, std::move(dirty)));
  blocked.incumbent_rejection_sequence = 1U;
  update = planner.plan(blocked);
  ASSERT_TRUE(update.telemetry.feasibility_attempted);
  EXPECT_GT(update.telemetry.repair_lattice_states_processed, 0U)
      << "pending=" << update.telemetry.repair_lattice_states_pending
      << " feasibility_ms=" << update.telemetry.feasibility_ms
      << " repair_ms=" << update.telemetry.repair_ms;
}

TEST(PersistentDStarLitePlanner3DTest, TheStartAnchorIsKeptWhileItStaysAdmissible) {
  // A vehicle drifting between two nodes keeps the anchor its searches were
  // seeded from; re-anchoring on every flip of the nearest node would restart
  // them for nothing. The walk over the anchors on exhaustion still wins.
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 12, 5, 3});
  PersistentPlannerConfig3D config = testConfig();
  config.minimum_horizontal_step_m = 2.0;
  detail::PlannerLattice3D lattice{config};
  const PersistentPlannerWorld3D open = world(occupancy, 1U);
  lattice.configureGridGeometry(*open.bounds());
  lattice.installWorld(open);
  const detail::PersistentPlannerNode3D first{1, 1, 1};
  const detail::PersistentPlannerNode3D next{2, 1, 1};
  ASSERT_NEAR(lattice.pointFor(first).x, 3.0, 1.0e-9);
  ASSERT_NEAR(lattice.pointFor(next).x, 5.0, 1.0e-9);

  const Point3 nearer_the_next{4.2, 3.0, 1.5};
  const detail::PlannerLattice3D::DepartureConnection3D nearest =
      lattice.selectDepartureConnection(nearer_the_next);
  ASSERT_TRUE(nearest.available());
  EXPECT_EQ(*nearest.anchor, next);

  const detail::PlannerLattice3D::DepartureConnection3D kept =
      lattice.selectDepartureConnection(nearer_the_next, 0U, first);
  ASSERT_TRUE(kept.available());
  EXPECT_EQ(*kept.anchor, first) << "the anchor flipped to the nearest node";

  const detail::PlannerLattice3D::DepartureConnection3D walked =
      lattice.selectDepartureConnection(nearer_the_next, 1U, first);
  const detail::PlannerLattice3D::DepartureConnection3D walked_unpreferred =
      lattice.selectDepartureConnection(nearer_the_next, 1U);
  ASSERT_TRUE(walked.available());
  ASSERT_TRUE(walked_unpreferred.available());
  EXPECT_EQ(*walked.anchor, *walked_unpreferred.anchor)
      << "the preference must not alter the walk on exhaustion";

  // A preferred anchor the body no longer reaches is not kept.
  auto walled = std::make_shared<ObservedOccupancyGrid3D>(*occupancy);
  for (int y = 0; y < 5; ++y) {
    for (int z = 0; z < 3; ++z) {
      ASSERT_TRUE(walled->setState({3, y, z}, ObservedVoxelState::kOccupied));
    }
  }
  lattice.installWorld(world(walled, 2U));
  const detail::PlannerLattice3D::DepartureConnection3D released =
      lattice.selectDepartureConnection(nearer_the_next, 0U, first);
  ASSERT_TRUE(released.available());
  EXPECT_EQ(*released.anchor, next);
}

// Two rooms joined by a corridor 1.5 m wide whose centreline lies half a
// lattice step off the lattice rows: on a 2 m lattice over a 0.25 m map with a
// 0.5 m body, no node inside the corridor is valid and no straight lattice
// edge crosses it, so the graph has no path from one room to the other. The
// corridor is wide enough for the body, and a vehicle in the first room has a
// way out at the body's own scale.
struct NodelessCorridorFixture {
  static constexpr double kResolution = 0.25;
  static constexpr int kWidth = 64;  // 16 m
  static constexpr int kHeight = 24; // 6 m
  static constexpr int kDepth = 8;   // 2 m
  std::shared_ptr<ObservedOccupancyGrid3D> occupancy{
      std::make_shared<ObservedOccupancyGrid3D>(
          GridBounds3D{0.0, 0.0, 0.0, kResolution, kWidth, kHeight, kDepth})};
  PersistentPlannerConfig3D config{testConfig()};
  Point3 start{3.0, 3.5, 1.0};
  Point3 goal{13.0, 3.0, 1.0};

  NodelessCorridorFixture() {
    // Walls between the rooms, x in [6, 10), except the corridor y in
    // [2.75, 4.25).
    for (int x = 24; x < 40; ++x) {
      for (int y = 0; y < kHeight; ++y) {
        const double y_m = static_cast<double>(y) * kResolution;
        if (y_m >= 2.75 && y_m < 4.25) {
          continue;
        }
        for (int z = 0; z < kDepth; ++z) {
          if (!occupancy->setState({x, y, z}, ObservedVoxelState::kOccupied)) {
            throw std::logic_error{"fixture cell outside the grid"};
          }
        }
      }
    }
    config.minimum_horizontal_step_m = 2.0;
    config.minimum_vertical_step_m = 2.0;
    config.physical_footprint.radius_m = 0.5;
    config.physical_footprint.perimeter_samples = 8U;
    config.physical_footprint.radial_rings = 1U;
    config.physical_footprint.axial_samples = 1U;
    config.physical_footprint.sweep_step_m = 0.1;
    config.flight_envelope.maximum_target_z_m = 2.0;
    config.feasibility_first_enabled = true;
    config.maximum_compute_time_ms = 200.0;
    config.maximum_feasibility_compute_time_ms = 100.0;
    config.departure_refinement_subdivisions = 4U;
    config.escape_search_radius_cells = 4U;
    config.escape_search_maximum_probes_per_update = 1U << 16U;
  }
};

TEST(PersistentDStarLitePlanner3DTest,
     AnExhaustedFrontierIsReportedAndWalksTheAnchors) {
  // Without an escape search a closed component is terminal, and the planner
  // has to say so: the exhaustion survives the search's own restart, and the
  // walk over the start's anchors advances on it.
  NodelessCorridorFixture fixture;
  fixture.config.escape_search_radius_cells = 0U;
  PersistentDStarLitePlanner3D planner{fixture.config};
  bool exhausted = false;
  std::size_t skip = 0U;
  PlannerUpdate3D update;
  for (int attempt = 0; attempt < 12; ++attempt) {
    update = planner.plan(
        request(fixture.start, fixture.goal, world(fixture.occupancy, 1U)));
    exhausted = exhausted || update.telemetry.feasibility_frontier_exhausted;
    skip = std::max(skip, update.telemetry.departure_anchor_skip);
  }
  EXPECT_FALSE(update.publishable());
  EXPECT_TRUE(exhausted) << "the exhaustion was hidden by the restart";
  EXPECT_GT(skip, 0U) << "the anchor walk never advanced";
  EXPECT_FALSE(update.telemetry.escape_search_attempted);
}

TEST(PersistentDStarLitePlanner3DTest,
     TheEscapeSearchLeavesAClosedComponentThroughTheCorridor) {
  NodelessCorridorFixture fixture;
  PersistentDStarLitePlanner3D planner{fixture.config};
  bool exhausted = false;
  bool escape_found = false;
  PlannerUpdate3D update;
  for (int attempt = 0; attempt < 40; ++attempt) {
    update = planner.plan(
        request(fixture.start, fixture.goal, world(fixture.occupancy, 1U)));
    exhausted = exhausted || update.telemetry.feasibility_frontier_exhausted;
    escape_found = escape_found || update.telemetry.escape_search_found;
    if (update.publishable()) {
      break;
    }
  }
  EXPECT_TRUE(exhausted);
  EXPECT_TRUE(escape_found) << "explored="
                            << update.telemetry.escape_search_explored_cells
                            << " exhausted="
                            << update.telemetry.escape_search_exhausted;
  ASSERT_TRUE(update.publishable()) << "no route through the corridor";
  EXPECT_TRUE(update.telemetry.escape_connection_active);
  EXPECT_TRUE(update.telemetry.departure_waypoint_used);
  EXPECT_GT(update.telemetry.departure_waypoint_count, 1U);
  const std::vector<Point3>& points = candidate(update).points;
  ASSERT_GE(points.size(), 2U);
  EXPECT_NEAR(points.front().x, fixture.start.x, 1.0e-9);
  EXPECT_NEAR(points.back().x, fixture.goal.x, 1.0e-9);
  EXPECT_TRUE(std::ranges::any_of(points, [](const Point3& point) {
    return point.x > 6.0 && point.x < 10.0 && point.y > 2.9 && point.y < 4.1;
  })) << "the route does not pass the corridor";
  expectRawValid(points, *fixture.occupancy, planner.config().physical_footprint);
}

} // namespace
} // namespace drone_city_nav
