#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <vector>

#include "persistent_dstar_lite_planner_3d_internal.hpp"
#include "persistent_dstar_lite_planner_3d_test_support.hpp"

namespace drone_city_nav {
namespace {

TEST(PersistentDStarLitePlanner3DTest, ClearanceCenteringSlidesVerticesOffTheWall) {
  // A straight corridor whose free lane runs from y = 4 to y = 6: its middle
  // is y = 5. The lattice put the route's interior vertices against the
  // southern wall at y = 4.2, which is where a node row happens to lie.
  const auto clearance = [](const Point3& point) {
    return std::min(point.y - 4.0, 6.0 - point.y);
  };
  const auto segment_valid = [](const Point3& first, const Point3& second, bool) {
    const auto inside = [](const Point3& point) {
      return point.y > 4.05 && point.y < 5.95;
    };
    return inside(first) && inside(second);
  };
  const std::vector<Point3> path{
      {0.0, 4.2, 2.0}, {2.0, 4.2, 2.0}, {4.0, 4.2, 2.0}, {6.0, 4.2, 2.0}};
  detail::PathPostprocessor3D postprocessor;
  std::size_t queries{0U};
  std::size_t moved{0U};
  const std::vector<Point3> centred =
      postprocessor.centerOnClearance(path,
                                      detail::PathClearanceCenteringContext3D{
                                          .segment_valid = segment_valid,
                                          .clearance = clearance,
                                          .target_clearance_m = 1.0,
                                          .probe_step_m = 0.1,
                                          .maximum_passes = 12U,
                                          .maximum_clearance_queries = 4'096U,
                                      },
                                      queries, moved);

  ASSERT_EQ(centred.size(), path.size());
  EXPECT_GT(queries, 0U);
  EXPECT_GT(moved, 0U);
  // The endpoints are the vehicle's own position and the goal; only the
  // interior slides, and it stops at the middle of the passage.
  EXPECT_DOUBLE_EQ(centred.front().y, path.front().y);
  EXPECT_DOUBLE_EQ(centred.back().y, path.back().y);
  for (std::size_t index = 1U; index + 1U < centred.size(); ++index) {
    EXPECT_NEAR(centred[index].y, 5.0, 0.15) << "vertex " << index;
    // The move is across the local route direction, so a vertex slides along
    // the corridor only as far as its neighbours' own moves tilt that
    // direction.
    EXPECT_NEAR(centred[index].x, path[index].x, 0.3) << "vertex " << index;
    EXPECT_DOUBLE_EQ(centred[index].z, path[index].z);
  }
}

TEST(PersistentDStarLitePlanner3DTest,
     ClearanceCenteringKeepsAMoveOnlyWhenItValidates) {
  // The same corridor, but every candidate position is refused: the pass may
  // not turn a valid route into one raw evidence rejects, so the path comes
  // back untouched.
  const auto clearance = [](const Point3& point) {
    return std::min(point.y - 4.0, 6.0 - point.y);
  };
  const std::vector<Point3> path{{0.0, 4.2, 2.0}, {2.0, 4.2, 2.0}, {4.0, 4.2, 2.0}};
  detail::PathPostprocessor3D postprocessor;
  std::size_t queries{0U};
  std::size_t moved{0U};
  const std::vector<Point3> unchanged = postprocessor.centerOnClearance(
      path,
      detail::PathClearanceCenteringContext3D{
          .segment_valid = [](const Point3&, const Point3&, bool) { return false; },
          .clearance = clearance,
          .target_clearance_m = 1.0,
          .probe_step_m = 0.1,
          .maximum_passes = 4U,
          .maximum_clearance_queries = 4'096U,
      },
      queries, moved);

  EXPECT_EQ(moved, 0U);
  expectSamePath(unchanged, path);
}

TEST(PersistentDStarLitePlanner3DTest,
     ClearanceCenteringNeverMovesAVertexIntoUnobservedSpace) {
  // The same corridor, but only its southern half has been scanned: north of
  // y = 5.0 nothing is observed yet. The clearance runs to observed evidence
  // alone, so its gradient still points north across the whole lane; the
  // vertex may climb toward the observed boundary and no further, because a
  // probe or a candidate in unobserved space carries no information about
  // the jamb the next scan may reveal there.
  const auto clearance = [](const Point3& point) {
    return std::min(point.y - 4.0, 6.0 - point.y);
  };
  const auto observed = [](const Point3& point) { return point.y < 5.0; };
  const auto segment_valid = [](const Point3& first, const Point3& second, bool) {
    const auto inside = [](const Point3& point) {
      return point.y > 4.05 && point.y < 5.95;
    };
    return inside(first) && inside(second);
  };
  const std::vector<Point3> path{
      {0.0, 4.2, 2.0}, {2.0, 4.2, 2.0}, {4.0, 4.2, 2.0}, {6.0, 4.2, 2.0}};
  detail::PathPostprocessor3D postprocessor;
  std::size_t queries{0U};
  std::size_t moved{0U};
  const std::vector<Point3> centred =
      postprocessor.centerOnClearance(path,
                                      detail::PathClearanceCenteringContext3D{
                                          .segment_valid = segment_valid,
                                          .clearance = clearance,
                                          .observed = observed,
                                          .target_clearance_m = 1.0,
                                          .probe_step_m = 0.1,
                                          .maximum_passes = 12U,
                                          .maximum_clearance_queries = 4'096U,
                                      },
                                      queries, moved);

  ASSERT_EQ(centred.size(), path.size());
  EXPECT_GT(moved, 0U);
  for (std::size_t index = 1U; index + 1U < centred.size(); ++index) {
    EXPECT_GT(centred[index].y, path[index].y) << "vertex " << index;
    EXPECT_LT(centred[index].y, 5.0) << "vertex " << index;
  }

  // Nothing observed at all: the pass moves nothing.
  queries = 0U;
  moved = 0U;
  const std::vector<Point3> unchanged = postprocessor.centerOnClearance(
      path,
      detail::PathClearanceCenteringContext3D{
          .segment_valid = segment_valid,
          .clearance = clearance,
          .observed = [](const Point3&) { return false; },
          .target_clearance_m = 1.0,
          .probe_step_m = 0.1,
          .maximum_passes = 4U,
          .maximum_clearance_queries = 4'096U,
      },
      queries, moved);
  EXPECT_EQ(moved, 0U);
  expectSamePath(unchanged, path);
}

TEST(PersistentDStarLitePlanner3DTest,
     APublishedRouteIsCentredOnlyWithinTheObservedLane) {
  // A hall whose southern wall at y = 2 and floor are observed, whose lane
  // y = 3..8 is known free, and whose northern part (y >= 9) has never been
  // scanned. Measured to observed evidence alone, the clearance keeps growing
  // northward all the way to the centering target, so an unbounded pass would
  // carry every interior vertex into the unknown; the observed oracle keeps
  // them inside the scanned lane.
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 14, 12, 10});
  for (int x = 0; x < 14; ++x) {
    for (int y = 0; y < 9; ++y) {
      ASSERT_TRUE(occupancy->setState({x, y, 0}, ObservedVoxelState::kOccupied));
    }
    for (int z = 1; z < 10; ++z) {
      ASSERT_TRUE(occupancy->setState({x, 2, z}, ObservedVoxelState::kOccupied));
    }
    for (int y = 3; y < 9; ++y) {
      for (int z = 1; z < 10; ++z) {
        ASSERT_TRUE(occupancy->setState({x, y, z}, ObservedVoxelState::kFree));
      }
    }
  }
  // A pillar in the southern cell of the lane: the straight line between the
  // endpoints crosses it, so the route bends around it and keeps interior
  // vertices for the centering pass to move.
  for (int z = 1; z < 10; ++z) {
    ASSERT_TRUE(occupancy->setState({6, 3, z}, ObservedVoxelState::kOccupied));
  }
  PersistentPlannerConfig3D config = testConfig();
  // No clearance ranking: the search takes the shortest lane route and only
  // the centering pass decides where its vertices settle. The pass budget is
  // generous, so nothing but the oracle bounds the climb.
  config.clearance_ranking_weight = 0.0;
  config.clearance_ranking_distance_m = 6.0;
  config.clearance_centering_passes = 40U;
  config.maximum_clearance_centering_queries = 65536U;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{1.5, 3.5, 4.5};
  const Point3 goal{12.5, 3.5, 4.5};
  PlannerUpdate3D update = planner.plan(request(start, goal, world(occupancy, 1U)));
  for (int attempt = 0; attempt < 8 && !update.publishable(); ++attempt) {
    update = planner.plan(request(start, goal, world(occupancy, 1U)));
  }
  ASSERT_TRUE(update.publishable());
  const std::vector<Point3>& points = candidate(update).points;
  ASSERT_GE(points.size(), 3U);
  EXPECT_GT(update.telemetry.clearance_centering_moves, 0U);
  for (std::size_t index = 1U; index + 1U < points.size(); ++index) {
    EXPECT_GT(points[index].y, 3.5) << "vertex " << index << " was not centred";
    EXPECT_LT(points[index].y, 9.0) << "vertex " << index << " left the observed lane";
    EXPECT_LT(points[index].z, 10.0) << "vertex " << index << " left the observed lane";
  }
  expectRawValid(points, *occupancy, planner.config().physical_footprint);
}

} // namespace
} // namespace drone_city_nav
