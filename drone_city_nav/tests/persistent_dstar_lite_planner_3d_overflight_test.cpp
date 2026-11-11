#include "persistent_dstar_lite_planner_3d_test_support.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] double highestPointZ(const std::vector<Point3>& path) {
  double highest = -std::numeric_limits<double>::infinity();
  for (const Point3& point : path) {
    highest = std::max(highest, point.z);
  }
  return highest;
}

// The open sky's leg (specification K76): over everything the memory holds,
// found without a search, and taken again when something taller is seen.
TEST(PersistentDStarLitePlanner3DTest, TheOpenSkysLegGoesOverEverythingTheMemoryHolds) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 60, 12, 24});
  for (int x = 0; x < 60; ++x) {
    for (int y = 0; y < 12; ++y) {
      ASSERT_TRUE(occupancy->setState({x, y, 0}, ObservedVoxelState::kOccupied));
    }
  }
  // A wall across the leg, six metres tall.
  for (int y = 0; y < 12; ++y) {
    for (int z = 1; z < 6; ++z) {
      ASSERT_TRUE(occupancy->setState({20, y, z}, ObservedVoxelState::kOccupied));
    }
  }
  PersistentPlannerConfig3D config = testConfig();
  config.overflight_margin_m = 3.0;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{2.5, 5.5, 2.5};
  const Point3 goal{56.5, 5.5, 2.5};

  const PlannerUpdate3D update =
      planner.plan(request(start, goal, world(occupancy, 1U)));

  ASSERT_TRUE(update.publishable());
  const SpatialRouteCandidate3D& leg = candidate(update);
  EXPECT_EQ(leg.source, SpatialRouteCandidateSource3D::kOverflight);
  EXPECT_STREQ(spatialRouteCandidateSource3DName(leg.source), "overflight");
  // The wall's top at 6 m and the margin: the climb and the descent slant at
  // two metres a metre, 13 m each for the 6.5 m.
  ASSERT_EQ(leg.points.size(), 4U);
  EXPECT_DOUBLE_EQ(highestPointZ(leg.points), 9.0);
  EXPECT_NEAR(leg.points[1].x, 15.5, 1.0e-9);
  EXPECT_NEAR(leg.points[2].x, 43.5, 1.0e-9);
  expectRawValid(leg.points, *occupancy, planner.config().physical_footprint);
  // Nothing was searched, and nothing is left to search.
  EXPECT_EQ(update.telemetry.expansions, 0U);
  EXPECT_EQ(update.progress, SearchProgress3D::kConverged);

  // A mast twelve metres tall seen on the leg: the leg is taken again over it.
  auto taller = std::make_shared<ObservedOccupancyGrid3D>(*occupancy);
  for (int z = 1; z < 12; ++z) {
    ASSERT_TRUE(taller->setState({40, 5, z}, ObservedVoxelState::kOccupied));
  }
  const PlannerUpdate3D again = planner.plan(
      request(start, goal, world(taller, 2U, {OccupancyChunkIndex3D{2, 0, 0}})));
  ASSERT_TRUE(again.publishable());
  EXPECT_EQ(candidate(again).source, SpatialRouteCandidateSource3D::kOverflight);
  EXPECT_DOUBLE_EQ(highestPointZ(candidate(again).points), 15.0);
  expectRawValid(candidate(again).points, *taller, planner.config().physical_footprint);
}

// On the way down already, the rest of the descent is taken from where the
// vehicle is: straight while that clears, level at its height and then steeper
// where it does not, and never back up to the leg's level first (r3261).
TEST(PersistentDStarLitePlanner3DTest, ADescentIsTakenAgainFromWhereTheVehicleIs) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 60, 12, 24});
  for (int x = 0; x < 60; ++x) {
    for (int y = 0; y < 12; ++y) {
      ASSERT_TRUE(occupancy->setState({x, y, 0}, ObservedVoxelState::kOccupied));
    }
  }
  // A mast off the leg sets the leg's level at 19 m.
  for (int z = 1; z < 16; ++z) {
    ASSERT_TRUE(occupancy->setState({10, 1, z}, ObservedVoxelState::kOccupied));
  }
  PersistentPlannerConfig3D config = testConfig();
  config.overflight_margin_m = 3.0;
  const Point3 start{40.5, 5.5, 12.0};
  const Point3 goal{56.5, 5.5, 2.5};
  {
    PersistentDStarLitePlanner3D planner{config};
    const PlannerUpdate3D update =
        planner.plan(request(start, goal, world(occupancy, 1U)));
    ASSERT_TRUE(update.publishable());
    EXPECT_EQ(candidate(update).source, SpatialRouteCandidateSource3D::kOverflight);
    // 16 m out and 9.5 m up: inside the two-to-one cone, the straight line.
    ASSERT_EQ(candidate(update).points.size(), 2U);
  }
  // A crown on that line: level at 12 m to 9.5 m short of the goal, then down
  // at one to one, over the crown.
  auto crown = std::make_shared<ObservedOccupancyGrid3D>(*occupancy);
  ASSERT_TRUE(crown->setState({50, 5, 6}, ObservedVoxelState::kOccupied));
  PersistentDStarLitePlanner3D planner{config};
  const PlannerUpdate3D update = planner.plan(request(start, goal, world(crown, 1U)));
  ASSERT_TRUE(update.publishable());
  const SpatialRouteCandidate3D& descent = candidate(update);
  EXPECT_EQ(descent.source, SpatialRouteCandidateSource3D::kOverflight);
  ASSERT_EQ(descent.points.size(), 3U);
  EXPECT_NEAR(descent.points[1].x, 47.0, 1.0e-9);
  EXPECT_DOUBLE_EQ(highestPointZ(descent.points), 12.0);
  expectRawValid(descent.points, *crown, planner.config().physical_footprint);
}

// A vehicle on the leg's level joins the leg where it is, not at the nearest
// of its vertices: that lay 88 m behind it, and the route rebased there flew
// back to it and along the same line forward again (r3280, r3287, r3288).
TEST(PersistentDStarLitePlanner3DTest, AVehicleOnTheLegJoinsItWhereItIs) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 60, 12, 24});
  for (int x = 0; x < 60; ++x) {
    for (int y = 0; y < 12; ++y) {
      ASSERT_TRUE(occupancy->setState({x, y, 0}, ObservedVoxelState::kOccupied));
    }
  }
  for (int y = 0; y < 12; ++y) {
    for (int z = 1; z < 6; ++z) {
      ASSERT_TRUE(occupancy->setState({20, y, z}, ObservedVoxelState::kOccupied));
    }
  }
  PersistentPlannerConfig3D config = testConfig();
  config.overflight_margin_m = 3.0;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 goal{56.5, 5.5, 2.5};
  PersistentPlannerRequest3D first =
      request(Point3{2.5, 5.5, 2.5}, goal, world(occupancy, 1U));
  first.session_id = 1U;
  ASSERT_TRUE(planner.plan(first).publishable());

  // Over the wall at the level, nine metres past the climb's top and nineteen
  // short of the descent's; a replan opens a new session, which receives the
  // rebased leg (r3288's route generations 2 and 3).
  const Point3 on_leg{24.5, 5.5, 9.0};
  PersistentPlannerRequest3D moved_request =
      request(on_leg, goal, world(occupancy, 1U));
  moved_request.session_id = 2U;
  const PlannerUpdate3D moved = planner.plan(moved_request);

  ASSERT_TRUE(moved.publishable());
  EXPECT_TRUE(moved.telemetry.incumbent_retained);
  const SpatialRouteCandidate3D& leg = candidate(moved);
  EXPECT_EQ(leg.source, SpatialRouteCandidateSource3D::kOverflight);
  ASSERT_EQ(leg.points.size(), 3U);
  EXPECT_NEAR(leg.points[1].x, 43.5, 1.0e-9);
  for (const Point3& point : leg.points) {
    EXPECT_GE(point.x, on_leg.x - 1.0e-9);
  }
}

TEST(PersistentDStarLitePlanner3DTest, TheLegStaysUnderTheEnvelopesCeiling) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 60, 12, 24});
  // A mast on the leg, taller than the ceiling leaves a margin over.
  for (int z = 0; z < 18; ++z) {
    ASSERT_TRUE(occupancy->setState({30, 8, z}, ObservedVoxelState::kOccupied));
  }
  PersistentPlannerConfig3D config = testConfig();
  config.overflight_margin_m = 3.0;
  PersistentDStarLitePlanner3D planner{config};

  const PlannerUpdate3D update = planner.plan(
      request(Point3{2.5, 8.5, 2.5}, Point3{56.5, 8.5, 2.5}, world(occupancy, 1U)));

  ASSERT_TRUE(update.publishable());
  EXPECT_EQ(candidate(update).source, SpatialRouteCandidateSource3D::kOverflight);
  // The ceiling of 20 m less one vertical step, not 18 m and the margin.
  EXPECT_DOUBLE_EQ(highestPointZ(candidate(update).points), 19.0);
}

// What stands aside of the leg's track, beyond the planner's clearance reach,
// does not raise it (r3273).
TEST(PersistentDStarLitePlanner3DTest, AMastAsideOfTheLegDoesNotRaiseIt) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 60, 24, 24});
  for (int z = 0; z < 15; ++z) {
    ASSERT_TRUE(occupancy->setState({30, 20, z}, ObservedVoxelState::kOccupied));
  }
  for (int z = 0; z < 4; ++z) {
    ASSERT_TRUE(occupancy->setState({30, 6, z}, ObservedVoxelState::kOccupied));
  }
  PersistentPlannerConfig3D config = testConfig();
  config.overflight_margin_m = 3.0;
  PersistentDStarLitePlanner3D planner{config};

  const PlannerUpdate3D update = planner.plan(
      request(Point3{2.5, 5.5, 2.5}, Point3{56.5, 5.5, 2.5}, world(occupancy, 1U)));

  ASSERT_TRUE(update.publishable());
  EXPECT_EQ(candidate(update).source, SpatialRouteCandidateSource3D::kOverflight);
  // The post on the track (4 m) and the margin; the mast 14.5 m aside is not read.
  EXPECT_DOUBLE_EQ(highestPointZ(candidate(update).points), 7.0);
}

TEST(PersistentDStarLitePlanner3DTest, ACaveProfilePlansNoSuchLeg) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 16, 6, 8});
  for (int x = 0; x < 16; ++x) {
    for (int y = 0; y < 6; ++y) {
      ASSERT_TRUE(occupancy->setState({x, y, 0}, ObservedVoxelState::kOccupied));
    }
  }
  PersistentDStarLitePlanner3D planner{testConfig()};
  const Point3 start{1.5, 2.5, 3.5};
  const Point3 goal{14.5, 2.5, 3.5};

  PlannerUpdate3D update = planner.plan(request(start, goal, world(occupancy, 1U)));
  for (int attempt = 0; attempt < 8 && !update.publishable(); ++attempt) {
    update = planner.plan(request(start, goal, world(occupancy, 1U)));
  }

  ASSERT_TRUE(update.publishable());
  EXPECT_NE(candidate(update).source, SpatialRouteCandidateSource3D::kOverflight);
}

} // namespace
} // namespace drone_city_nav
