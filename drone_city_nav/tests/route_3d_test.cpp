#include "drone_city_nav/bounded_worker_pool.hpp"
#include "drone_city_nav/distance_field_3d.hpp"
#include "drone_city_nav/risk_aware_lattice_3d.hpp"
#include "drone_city_nav/route_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <vector>

#include "risk_aware_lattice_3d_geometry.hpp"

namespace drone_city_nav {
namespace {

TEST(Route3DTest, SamplesContinuousAltitudeProfile) {
  const std::vector<Point3> points{{0.0, 0.0, 2.0}, {4.0, 0.0, 4.0}, {4.0, 4.0, 4.0}};
  const std::vector<RouteSample3D> route = sampleRoute3D(points, 1.0, 10.0);

  ASSERT_GT(route.size(), 4U);
  EXPECT_EQ(route.front().position.z, 2.0);
  EXPECT_EQ(route.back().position.z, 4.0);
  EXPECT_GT(route.back().station_m, 8.0);
  EXPECT_EQ(route.back().reference_speed_mps, 10.0);
}

TEST(Route3DTest, RouteFingerprintIsStableAndGeometrySensitive) {
  const std::vector<RouteSample3D> first =
      sampleRoute3D(std::vector<Point3>{{0.0, 0.0, 2.0}, {4.0, 0.0, 2.0}}, 1.0, 10.0);
  std::vector<RouteSample3D> changed = first;
  changed.back().position.y = 0.01;

  EXPECT_EQ(routeFingerprint(first), routeFingerprint(first));
  EXPECT_NE(routeFingerprint(first), routeFingerprint(changed));
}

TEST(Route3DTest, ProjectsProgressUsingThreeDimensionalStation) {
  const std::vector<RouteSample3D> route = sampleRoute3D(
      std::vector<Point3>{{0.0, 0.0, 10.0}, {0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}}, 1.0,
      10.0);

  const RouteProjection3D projection = projectOntoRoute3D(route, Point3{5.0, 0.0, 0.0});

  ASSERT_TRUE(projection.valid);
  EXPECT_NEAR(projection.station_m, 15.0, 1.0e-6);
  EXPECT_NEAR(projection.remaining_m, 5.0, 1.0e-6);
}

TEST(Route3DTest, CoordinatesVerticalAlignmentBeforeChannelEntry) {
  const std::vector<RouteSample3D> route = sampleRoute3D(
      std::vector<Point3>{{0.0, 0.0, 18.0}, {80.0, 0.0, 5.0}, {100.0, 0.0, 5.0}}, 1.0,
      20.0);
  const double entry_station = route[route.size() - 21U].station_m;
  const std::vector<ConstrainedRouteSpan> spans{ConstrainedRouteSpan{
      .channel_id = "channel",
      .route_generation = 4U,
      .direction_sign = 1,
      .begin_station_m = entry_station,
      .end_station_m = route.back().station_m,
      .envelope = {RouteEnvelopeSample{.station_m = entry_station,
                                       .min_z_m = 1.5,
                                       .max_z_m = 8.5,
                                       .reference_z_m = 5.0,
                                       .reference_speed_mps = 10.0}},
  }};
  ConstrainedRouteCoordinator coordinator;
  const ConstrainedRouteObservation approach = observeConstrainedRoute(
      route, spans, 4U, entry_station - 40.0, Point3{40.0, 0.0, 18.0},
      Vec3{20.0, 0.0, 0.0}, RouteEnvelopeConfig{}, 180.0);
  const ConstrainedRouteControl control =
      coordinator.update(approach, 20.0, ConstrainedRouteControlConfig{});

  EXPECT_TRUE(control.active);
  EXPECT_FALSE(control.vertical_ready);
  EXPECT_FALSE(control.hold_xy);
  EXPECT_LT(control.speed_limit_mps, 20.0);
  EXPECT_DOUBLE_EQ(control.reference_z_m, 5.0);

  ConstrainedRouteObservation at_entry = approach;
  at_entry.distance_to_entry_m = 1.0;
  EXPECT_TRUE(
      coordinator.update(at_entry, 20.0, ConstrainedRouteControlConfig{}).hold_xy);
  at_entry.actual_z_m = 5.0;
  at_entry.actual_vertical_speed_mps = 0.1;
  const ConstrainedRouteControl ready =
      coordinator.update(at_entry, 20.0, ConstrainedRouteControlConfig{});
  EXPECT_TRUE(ready.vertical_ready);
  EXPECT_FALSE(ready.hold_xy);
}

TEST(Route3DTest, ObservesConstrainedSpanLifecycleAndMotionMetrics) {
  const std::vector<Point3> route_points{{0.0, 0.0, 0.0}, {100.0, 0.0, 10.0}};
  const std::vector<RouteSample3D> route = sampleRoute3D(route_points, 5.0, 20.0);
  const std::vector<ConstrainedRouteSpan> spans{
      ConstrainedRouteSpan{
          .channel_id = "test_channel",
          .route_generation = 7U,
          .direction_sign = -1,
          .begin_station_m = 40.0,
          .end_station_m = 50.0,
          .envelope =
              {
                  RouteEnvelopeSample{.station_m = 40.0,
                                      .lateral_free_left_m = 3.0,
                                      .lateral_free_right_m = 3.0,
                                      .min_z_m = 2.0,
                                      .max_z_m = 6.0,
                                      .reference_z_m = 4.0,
                                      .reference_speed_mps = 10.0},
                  RouteEnvelopeSample{.station_m = 50.0,
                                      .lateral_free_left_m = 3.0,
                                      .lateral_free_right_m = 3.0,
                                      .min_z_m = 3.0,
                                      .max_z_m = 7.0,
                                      .reference_z_m = 5.0,
                                      .reference_speed_mps = 10.0},
              },
      },
  };
  const auto observe = [&route, &spans](const double station_m) {
    const double route_x = station_m / std::sqrt(1.01);
    return observeConstrainedRoute(route, spans, 7U, station_m,
                                   Point3{route_x, 1.0, 5.0}, Vec3{8.0, 0.0, -0.5},
                                   RouteEnvelopeConfig{}, 30.0);
  };

  EXPECT_EQ(observe(5.0).phase, ConstrainedRoutePhase::kUnconstrained);
  EXPECT_EQ(observe(15.0).phase, ConstrainedRoutePhase::kApproach);
  const ConstrainedRouteObservation traversal = observe(45.0);
  EXPECT_EQ(traversal.phase, ConstrainedRoutePhase::kTraversal);
  EXPECT_TRUE(traversal.span_available);
  EXPECT_EQ(traversal.route_generation, 7U);
  EXPECT_EQ(traversal.span_index, 0U);
  EXPECT_EQ(traversal.direction_sign, -1);
  EXPECT_EQ(traversal.span_count, 1U);
  EXPECT_DOUBLE_EQ(traversal.distance_to_entry_m, -5.0);
  EXPECT_DOUBLE_EQ(traversal.distance_to_exit_m, 5.0);
  EXPECT_NEAR(traversal.entry_position.x, 39.8015, 1.0e-3);
  EXPECT_NEAR(traversal.exit_position.x, 49.7519, 1.0e-3);
  EXPECT_DOUBLE_EQ(traversal.reference_z_m, 4.0);
  EXPECT_DOUBLE_EQ(traversal.vertical_error_m, 1.0);
  EXPECT_DOUBLE_EQ(traversal.cross_track_error_m, 1.0);
  EXPECT_DOUBLE_EQ(traversal.actual_horizontal_speed_mps, 8.0);
  EXPECT_DOUBLE_EQ(traversal.actual_vertical_speed_mps, -0.5);
  EXPECT_TRUE(traversal.within_vertical_window);
  EXPECT_DOUBLE_EQ(traversal.lateral_width_m, 6.0);
  EXPECT_DOUBLE_EQ(traversal.vertical_height_m, 4.0);
  EXPECT_TRUE(traversal.lateral_constrained);
  EXPECT_TRUE(traversal.vertical_constrained);
  EXPECT_EQ(observe(55.0).phase, ConstrainedRoutePhase::kDeparture);
  EXPECT_EQ(observe(90.0).phase, ConstrainedRoutePhase::kUnconstrained);
}

TEST(Route3DTest, ReportsUnavailableWithoutRoute) {
  const ConstrainedRouteObservation observation = observeConstrainedRoute(
      {}, {}, 0U, 0.0, Point3{}, Vec3{}, RouteEnvelopeConfig{}, 30.0);

  EXPECT_EQ(observation.phase, ConstrainedRoutePhase::kUnavailable);
  EXPECT_FALSE(observation.span_available);
  EXPECT_EQ(constrainedRoutePhaseName(observation.phase), "unavailable");
}

TEST(Route3DTest, AssignsRequiredRiskTierFromRawEsdfClearance) {
  const mppi::EsdfGrid grid{3, 1, 1.0F, 0.0F, 0.0F, 1, 0.0F};
  const std::vector<float> esdf{std::numeric_limits<float>::infinity(), 4.0F, 1.5F};
  std::vector<RouteSample3D> route{
      RouteSample3D{.position = Point3{0.5, 0.5, 0.5}},
      RouteSample3D{.position = Point3{1.5, 0.5, 0.5}},
      RouteSample3D{.position = Point3{2.5, 0.5, 0.5}},
  };

  ASSERT_TRUE(assignRouteRiskTiers(route, grid, esdf, 1.0, 6.0));
  EXPECT_EQ(route[0].required_risk_tier, mppi::RiskTier::kPreferred);
  EXPECT_EQ(route[1].required_risk_tier, mppi::RiskTier::kPlanning);
  EXPECT_EQ(route[2].required_risk_tier, mppi::RiskTier::kCritical);
}

TEST(Route3DTest, LatticeUsesVerticalFreeOpeningWithoutYawConstraint) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 12, 12, 8}};
  for (int y = 0; y < 12; ++y) {
    for (int z = 0; z < 8; ++z) {
      if (z < 3 || z > 5) {
        occupancy.setOccupied(GridIndex3D{5, y, z});
      }
    }
  }
  const DistanceField3D field = DistanceField3D::build(occupancy, 20.0);
  const GridBounds3D& bounds = field.bounds();
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 1.0;
  config.vertical_step_m = 1.0;
  config.planning_goal_distance_m = 10.0;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.maximum_search_time_ms = 1000.0;
  const RiskAwareLattice3DResult result =
      planRiskAwareLattice3D(grid, field.distancesM(), Point3{2.5, 5.5, 2.5},
                             Vec3{-1.0, 0.0, 0.0}, Point3{9.5, 5.5, 4.5}, {}, config);

  ASSERT_EQ(result.status, Lattice3DStatus::kReachedPlanningGoal);
  EXPECT_EQ(result.termination, Lattice3DSearchTermination::kPlanningGoalReached);
  EXPECT_GT(result.records_peak, 0U);
  EXPECT_GT(result.successor_diagnostics.lattice_generated, 0U);
  EXPECT_GT(result.successor_profiling.search.collection_calls, 0U);
  EXPECT_GT(result.successor_profiling.search.candidates, 0U);
  EXPECT_GT(result.successor_profiling.search.maximum_candidates, 0U);
  EXPECT_GE(result.successor_profiling.search.worker_ms, 0.0);
  ASSERT_FALSE(result.points.empty());
  EXPECT_NEAR(result.points.back().x, 9.5, 1.0e-6);
  EXPECT_TRUE(result.reached_mission_goal);
}

TEST(Route3DTest, ReportsExpansionBudgetWithoutCallingItGraphExhaustion) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 12, 12, 4}};
  const DistanceField3D field = DistanceField3D::build(occupancy, 20.0);
  const GridBounds3D& bounds = field.bounds();
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  RiskAwareLattice3DConfig config;
  config.maximum_expansions = 0U;
  config.maximum_search_time_ms = 1000.0;

  const RiskAwareLattice3DResult result =
      planRiskAwareLattice3D(grid, field.distancesM(), Point3{1.5, 1.5, 1.5},
                             Vec3{1.0, 0.0, 0.0}, Point3{10.5, 10.5, 1.5}, {}, config);

  EXPECT_EQ(result.status, Lattice3DStatus::kSearchIncomplete);
  EXPECT_EQ(result.termination, Lattice3DSearchTermination::kExpansionBudgetExhausted);
  EXPECT_STREQ(lattice3DSearchTerminationName(result.termination),
               "expansion_budget_exhausted");
}

TEST(Route3DTest, ParallelSuccessorEvaluationPreservesDeterministicRoute) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 20, 20, 8}};
  for (int z = 0; z < 8; ++z) {
    for (int y = 5; y <= 14; ++y) {
      occupancy.setOccupied(GridIndex3D{10, y, z});
    }
  }
  const DistanceField3D field = DistanceField3D::build(occupancy, 30.0);
  const GridBounds3D& bounds = field.bounds();
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 1.0;
  config.vertical_step_m = 1.0;
  config.planning_goal_distance_m = 30.0;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.maximum_search_time_ms = 3000.0;
  const Point3 start{2.5, 9.5, 3.5};
  const Point3 goal{17.5, 9.5, 3.5};

  const RiskAwareLattice3DResult serial = planRiskAwareLattice3D(
      grid, field.distancesM(), start, Vec3{1.0, 0.0, 0.0}, goal, {}, config);
  BoundedWorkerPool worker_pool{4U};
  const RiskAwareLattice3DResult parallel =
      planRiskAwareLattice3D(grid, field.distancesM(), start, Vec3{1.0, 0.0, 0.0}, goal,
                             {}, config, &worker_pool);

  EXPECT_EQ(parallel.status, serial.status);
  EXPECT_EQ(parallel.risk_stage, serial.risk_stage);
  EXPECT_EQ(parallel.route_fingerprint, serial.route_fingerprint);
  EXPECT_EQ(parallel.successor_diagnostics.lattice_generated,
            serial.successor_diagnostics.lattice_generated);
  EXPECT_GT(parallel.successor_profiling.search.parallel_collection_calls, 0U);
  EXPECT_GT(parallel.successor_profiling.search.parallel_candidates, 0U);
}

TEST(Route3DTest, ReportsRawCollisionSuccessorRejectionsWhenGraphIsExhausted) {
  const mppi::EsdfGrid grid{6, 6, 1.0F, 0.0F, 0.0F, 4, 0.0F};
  const std::vector<float> occupied(
      static_cast<std::size_t>(grid.width * grid.height * grid.depth), 0.0F);
  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 1.0;
  config.vertical_step_m = 1.0;
  config.maximum_search_time_ms = 1000.0;

  const RiskAwareLattice3DResult result =
      planRiskAwareLattice3D(grid, occupied, Point3{2.5, 2.5, 1.5}, Vec3{1.0, 0.0, 0.0},
                             Point3{4.5, 4.5, 1.5}, {}, config);

  EXPECT_EQ(result.status, Lattice3DStatus::kMotionGraphExhausted);
  EXPECT_EQ(result.termination, Lattice3DSearchTermination::kOpenSetExhausted);
  EXPECT_GT(result.successor_diagnostics.lattice_rejected_raw_collision, 0U);
  EXPECT_EQ(lattice3DRiskStageName(result.risk_stage), std::string_view{"critical"});
}

TEST(Route3DTest, LatticeTraversesLShapedChannel) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 20, 20, 4}};
  for (int y = 0; y < 12; ++y) {
    for (int z = 0; z < 4; ++z) {
      occupancy.setOccupied(GridIndex3D{8, y, z});
      occupancy.setOccupied(GridIndex3D{9, y, z});
    }
  }
  const DistanceField3D field = DistanceField3D::build(occupancy, 20.0);
  const GridBounds3D& bounds = field.bounds();
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 1.0;
  config.vertical_step_m = 1.0;
  config.planning_goal_distance_m = 30.0;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.maximum_search_time_ms = 1000.0;
  const RiskAwareLattice3DResult result =
      planRiskAwareLattice3D(grid, field.distancesM(), Point3{3.5, 3.5, 1.5},
                             Vec3{1.0, 0.0, 0.0}, Point3{15.5, 15.5, 1.5}, {}, config);

  ASSERT_EQ(result.status, Lattice3DStatus::kReachedPlanningGoal);
  ASSERT_FALSE(result.points.empty());
  EXPECT_TRUE(std::any_of(result.points.begin(), result.points.end(),
                          [](const Point3& p) { return p.y > 11.0 && p.x < 10.0; }));
  EXPECT_NEAR(result.points.back().x, 15.5, 1.0e-6);
  EXPECT_NEAR(result.points.back().y, 15.5, 1.0e-6);
}

TEST(Route3DTest, FullPlanningRouteBeatsPreferredFrontier) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 20, 13, 4}};
  for (int y = 0; y < 13; ++y) {
    if (y >= 5 && y <= 7) {
      continue;
    }
    for (int z = 0; z < 4; ++z) {
      occupancy.setOccupied(GridIndex3D{10, y, z});
    }
  }
  const DistanceField3D field = DistanceField3D::build(occupancy, 20.0);
  const GridBounds3D& bounds = field.bounds();
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 1.0;
  config.vertical_step_m = 1.0;
  config.planning_goal_distance_m = 30.0;
  config.preferred_distance_m = 2.0;
  config.critical_distance_m = 0.1;
  config.maximum_search_time_ms = 1000.0;

  const RiskAwareLattice3DResult result =
      planRiskAwareLattice3D(grid, field.distancesM(), Point3{2.5, 6.5, 1.5},
                             Vec3{1.0, 0.0, 0.0}, Point3{17.5, 6.5, 1.5}, {}, config);

  EXPECT_EQ(result.status, Lattice3DStatus::kReachedPlanningGoal);
  EXPECT_EQ(result.risk_stage, Lattice3DRiskStage::kPlanningAllowed);
  EXPECT_TRUE(result.reached_mission_goal);
}

TEST(Route3DTest, FartherPlanningFrontierBeatsBlockedPreferredFrontier) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 20, 13, 4}};
  for (int y = 0; y < 13; ++y) {
    if (y >= 5 && y <= 7) {
      continue;
    }
    for (int z = 0; z < 4; ++z) {
      occupancy.setOccupied(GridIndex3D{10, y, z});
    }
  }
  const DistanceField3D field = DistanceField3D::build(occupancy, 40.0);
  const GridBounds3D& bounds = field.bounds();
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 1.0;
  config.vertical_step_m = 1.0;
  config.planning_goal_distance_m = 30.0;
  config.preferred_distance_m = 2.0;
  config.critical_distance_m = 0.1;
  config.maximum_search_time_ms = 1000.0;

  const RiskAwareLattice3DResult result =
      planRiskAwareLattice3D(grid, field.distancesM(), Point3{2.5, 6.5, 1.5},
                             Vec3{1.0, 0.0, 0.0}, Point3{30.5, 6.5, 1.5}, {}, config);

  ASSERT_EQ(result.status, Lattice3DStatus::kViableFrontier);
  EXPECT_EQ(result.risk_stage, Lattice3DRiskStage::kPlanningAllowed);
  ASSERT_FALSE(result.points.empty());
  EXPECT_GT(result.points.back().x, 10.0);
  EXPECT_GT(result.successor_profiling.continuation.collection_calls, 0U);
  EXPECT_GT(result.successor_profiling.continuation.candidates, 0U);
  EXPECT_GT(result.successor_profiling.continuation.maximum_candidates, 0U);
  EXPECT_GE(result.successor_profiling.continuation.worker_ms, 0.0);
}

TEST(Route3DTest, BuildsTypedSpanFromSelectedChannelTraversal) {
  const std::vector<RouteSample3D> route = sampleRoute3D(
      std::vector<Point3>{{0.0, 0.0, 5.0}, {5.0, 0.0, 5.0}, {10.0, 0.0, 5.0}}, 0.5,
      20.0);
  const std::vector<SelectedChannelTraversal> traversals{
      SelectedChannelTraversal{.channel_id = "test_channel",
                               .direction_sign = -1,
                               .begin_station_m = 2.0,
                               .end_station_m = 8.0,
                               .min_z_m = 1.5,
                               .max_z_m = 8.5,
                               .width_m = 24.0,
                               .height_m = 7.0,
                               .minimum_clearance_m = 3.5,
                               .speed_limit_mps = 10.0}};

  const std::vector<ConstrainedRouteSpan> spans =
      makeConstrainedRouteSpans(route, traversals, 12U, RouteEnvelopeConfig{});

  ASSERT_EQ(spans.size(), 1U);
  EXPECT_EQ(spans.front().channel_id, "test_channel");
  EXPECT_EQ(spans.front().route_generation, 12U);
  EXPECT_EQ(spans.front().direction_sign, -1);
  ASSERT_FALSE(spans.front().envelope.empty());
  EXPECT_DOUBLE_EQ(spans.front().envelope.front().min_z_m, 1.5);
  EXPECT_DOUBLE_EQ(spans.front().envelope.front().max_z_m, 8.5);
  EXPECT_DOUBLE_EQ(spans.front().envelope.front().reference_speed_mps, 10.0);
}

TEST(Route3DTest, ComparesReachedRoutesAcrossAllRiskStages) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 24, 20, 4}};
  for (int y = 2; y <= 16; ++y) {
    if (y >= 8 && y <= 10) {
      continue;
    }
    for (int z = 0; z < 4; ++z) {
      occupancy.setOccupied(GridIndex3D{11, y, z});
    }
  }
  const DistanceField3D field = DistanceField3D::build(occupancy, 30.0);
  const GridBounds3D& bounds = field.bounds();
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 1.0;
  config.vertical_step_m = 1.0;
  config.planning_goal_distance_m = 30.0;
  config.preferred_distance_m = 2.0;
  config.critical_distance_m = 0.1;
  config.maximum_search_time_ms = 3000.0;
  config.maximum_expansions = 500000U;
  config.heading_bias_cost_per_rad = 0.0;

  const RiskAwareLattice3DResult result =
      planRiskAwareLattice3D(grid, field.distancesM(), Point3{2.5, 9.5, 1.5},
                             Vec3{1.0, 0.0, 0.0}, Point3{21.5, 9.5, 1.5}, {}, config);

  ASSERT_EQ(result.status, Lattice3DStatus::kReachedPlanningGoal);
  EXPECT_EQ(result.topology_candidates.size(), 3U);
  EXPECT_TRUE(
      std::ranges::any_of(result.topology_candidates,
                          [](const auto& candidate) { return candidate.selected; }));
  for (std::size_t rank = 0U; rank < result.topology_candidates.size(); ++rank) {
    EXPECT_EQ(result.topology_candidates[rank].candidate_rank, rank);
  }
  EXPECT_NE(result.route_fingerprint, 0U);
  EXPECT_GE(result.search_ms, 0.0);
  EXPECT_NE(result.risk_stage, Lattice3DRiskStage::kPreferredOnly);
}

TEST(Route3DTest, SelectsEmbeddedChannelEdgeWhenItsObjectiveCostIsLower) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 20, 20, 10}};
  const DistanceField3D field = DistanceField3D::build(occupancy, 30.0);
  const GridBounds3D& bounds = field.bounds();
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  const std::vector<ConstrainedFreeSpaceEdge> channels{ConstrainedFreeSpaceEdge{
      .id = "direct_channel",
      .centerline = sampleRoute3D(
          std::vector<Point3>{{0.5, 0.5, 5.5}, {12.5, 8.5, 5.5}}, 0.5, 10.0),
      .entry = Point3{0.5, 0.5, 5.5},
      .exit = Point3{12.5, 8.5, 5.5},
      .min_z_m = 1.5,
      .max_z_m = 8.5,
      .width_m = 24.0,
      .height_m = 7.0,
      .minimum_clearance_m = 3.5,
      .speed_limit_mps = 10.0}};
  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 4.0;
  config.vertical_step_m = 1.0;
  config.planning_goal_distance_m = 30.0;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.heading_bias_cost_per_rad = 0.0;
  config.route_shape_turn_cost_per_rad = 0.0;
  config.channel_connection_distance_m = 1.0;
  config.maximum_search_time_ms = 1000.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_samples = 0U;

  const RiskAwareLattice3DResult result = planRiskAwareLattice3D(
      grid, field.distancesM(), Point3{0.5, 0.5, 5.5}, Vec3{1.0, 0.0, 0.0},
      Point3{12.5, 8.5, 5.5}, channels, config);

  ASSERT_EQ(result.status, Lattice3DStatus::kReachedPlanningGoal);
  ASSERT_EQ(result.selected_channels.size(), 1U);
  EXPECT_EQ(result.selected_channels.front().channel_id, "direct_channel");
  EXPECT_GT(result.selected_channels.front().end_station_m,
            result.selected_channels.front().begin_station_m);
}

TEST(Route3DTest, SeedsCollisionValidatedChannelBeyondLocalConnectionRadius) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 24, 8, 10}};
  const DistanceField3D field = DistanceField3D::build(occupancy, 30.0);
  const GridBounds3D& bounds = field.bounds();
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  const std::vector<ConstrainedFreeSpaceEdge> channels{ConstrainedFreeSpaceEdge{
      .id = "far_channel",
      .centerline = sampleRoute3D(
          std::vector<Point3>{{8.5, 2.5, 5.5}, {18.5, 2.5, 5.5}}, 0.5, 10.0),
      .entry = Point3{8.5, 2.5, 5.5},
      .exit = Point3{18.5, 2.5, 5.5},
      .min_z_m = 1.5,
      .max_z_m = 8.5,
      .width_m = 24.0,
      .height_m = 7.0,
      .minimum_clearance_m = 3.5,
      .speed_limit_mps = 10.0}};
  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 4.0;
  config.vertical_step_m = 1.0;
  config.planning_goal_distance_m = 30.0;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.heading_bias_cost_per_rad = 0.0;
  config.route_shape_turn_cost_per_rad = 0.0;
  config.channel_connection_distance_m = 1.0;
  config.maximum_search_time_ms = 1000.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_samples = 0U;

  const RiskAwareLattice3DResult result = planRiskAwareLattice3D(
      grid, field.distancesM(), Point3{0.5, 2.5, 8.5}, Vec3{1.0, 0.0, 0.0},
      Point3{22.5, 2.5, 5.5}, channels, config);

  ASSERT_EQ(result.status, Lattice3DStatus::kReachedPlanningGoal);
  ASSERT_EQ(result.selected_channels.size(), 1U);
  EXPECT_EQ(result.selected_channels.front().channel_id, "far_channel");
}

TEST(Route3DTest, MaterializesRequiredChannelWhenUnconstrainedSlicePrefersFrontier) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 80, 12, 10}};
  const DistanceField3D field = DistanceField3D::build(occupancy, 100.0);
  const GridBounds3D& bounds = field.bounds();
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  const std::vector<ConstrainedFreeSpaceEdge> channels{ConstrainedFreeSpaceEdge{
      .id = "required_channel",
      .centerline = sampleRoute3D(
          std::vector<Point3>{{30.5, 5.5, 5.5}, {60.5, 5.5, 5.5}}, 0.5, 10.0),
      .entry = Point3{30.5, 5.5, 5.5},
      .exit = Point3{60.5, 5.5, 5.5},
      .min_z_m = 1.5,
      .max_z_m = 8.5,
      .width_m = 24.0,
      .height_m = 7.0,
      .minimum_clearance_m = 3.5,
      .speed_limit_mps = 10.0}};
  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 2.0;
  config.vertical_step_m = 1.0;
  config.planning_goal_distance_m = 100.0;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.heading_bias_cost_per_rad = 0.0;
  config.route_shape_turn_cost_per_rad = 0.0;
  config.channel_topology_transition_cost = 100.0;
  config.maximum_topology_search_groups = 1U;
  config.maximum_expansions = 8U;
  config.maximum_search_time_ms = 1000.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_samples = 0U;

  const RiskAwareLattice3DResult result = planRiskAwareLattice3D(
      grid, field.distancesM(), Point3{2.5, 5.5, 5.5}, Vec3{1.0, 0.0, 0.0},
      Point3{74.5, 5.5, 5.5}, channels, config);

  EXPECT_EQ(result.topology_searches, 2U);
  ASSERT_EQ(result.selected_channels.size(), 1U);
  EXPECT_EQ(result.selected_channels.front().channel_id, "required_channel");
  EXPECT_GT(result.achieved_progress_m, 50.0);
}

TEST(Route3DTest, ParallelTopologyGroupsPreserveBestCompleteRoute) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 24, 20, 10}};
  const DistanceField3D field = DistanceField3D::build(occupancy, 30.0);
  const GridBounds3D& bounds = field.bounds();
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  const auto channel = [](const std::string& id, const std::vector<Point3>& points) {
    return ConstrainedFreeSpaceEdge{.id = id,
                                    .centerline = sampleRoute3D(points, 0.5, 10.0),
                                    .entry = points.front(),
                                    .exit = points.back(),
                                    .min_z_m = 1.5,
                                    .max_z_m = 8.5,
                                    .width_m = 24.0,
                                    .height_m = 7.0,
                                    .minimum_clearance_m = 3.5,
                                    .speed_limit_mps = 10.0};
  };
  const std::vector<ConstrainedFreeSpaceEdge> channels{
      channel("direct", {{0.5, 2.5, 5.5}, {18.5, 2.5, 5.5}}),
      channel("detour", {{0.5, 2.5, 5.5}, {9.5, 15.5, 5.5}, {18.5, 2.5, 5.5}})};
  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 2.0;
  config.vertical_step_m = 1.0;
  config.planning_goal_distance_m = 30.0;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.heading_bias_cost_per_rad = 0.0;
  config.route_shape_turn_cost_per_rad = 0.0;
  config.maximum_search_time_ms = 1000.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_samples = 0U;
  const Point3 start{0.5, 2.5, 5.5};
  const Point3 goal{20.5, 2.5, 5.5};

  const RiskAwareLattice3DResult serial = planRiskAwareLattice3D(
      grid, field.distancesM(), start, Vec3{1.0, 0.0, 0.0}, goal, channels, config);
  BoundedWorkerPool two_worker_pool{2U};
  const RiskAwareLattice3DResult two_worker =
      planRiskAwareLattice3D(grid, field.distancesM(), start, Vec3{1.0, 0.0, 0.0}, goal,
                             channels, config, &two_worker_pool);
  BoundedWorkerPool worker_pool{4U};
  const RiskAwareLattice3DResult parallel =
      planRiskAwareLattice3D(grid, field.distancesM(), start, Vec3{1.0, 0.0, 0.0}, goal,
                             channels, config, &worker_pool);

  ASSERT_EQ(serial.status, Lattice3DStatus::kReachedPlanningGoal);
  EXPECT_EQ(serial.topology_searches, 3U);
  EXPECT_EQ(serial.parallel_topology_searches, 0U);
  ASSERT_EQ(two_worker.status, serial.status);
  EXPECT_EQ(two_worker.route_fingerprint, serial.route_fingerprint);
  EXPECT_EQ(two_worker.topology_searches, 3U);
  EXPECT_EQ(two_worker.parallel_topology_searches, two_worker.topology_searches);
  ASSERT_EQ(parallel.status, serial.status);
  EXPECT_EQ(parallel.route_fingerprint, serial.route_fingerprint);
  EXPECT_EQ(parallel.topology_searches, 3U);
  EXPECT_EQ(parallel.parallel_topology_searches, parallel.topology_searches);
  EXPECT_GT(parallel.topology_search_worker_ms, 0.0);
}

TEST(Route3DTest, MaterializesValidatedContinuationWhenSearchSliceExpires) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 40, 20, 12}};
  const DistanceField3D field = DistanceField3D::build(occupancy, 40.0);
  const GridBounds3D& bounds = field.bounds();
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 2.0;
  config.vertical_step_m = 1.0;
  config.planning_goal_distance_m = 30.0;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.maximum_search_time_ms = 0.001;
  config.frontier_minimum_reachable_depth_m = 8.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_samples = 0U;
  const Point3 start{2.5, 10.5, 5.5};
  const Point3 goal{30.5, 10.5, 5.5};

  const RiskAwareLattice3DResult result = planRiskAwareLattice3D(
      grid, field.distancesM(), start, Vec3{1.0, 0.0, 0.0}, goal, {}, config);

  ASSERT_EQ(result.status, Lattice3DStatus::kViableFrontier);
  EXPECT_EQ(result.termination, Lattice3DSearchTermination::kDeadlineReached);
  EXPECT_GE(result.continuation_reachable_depth_m,
            config.frontier_minimum_reachable_depth_m);
  EXPECT_GE(result.route_length_m, config.frontier_minimum_reachable_depth_m);
  ASSERT_GE(result.points.size(), 2U);
  EXPECT_LT(distance3D(result.points.back(), goal), distance3D(start, goal));
  EXPECT_NEAR(result.points.back().y, start.y, 1.0e-9);
  EXPECT_NEAR(result.points.back().z, start.z, 1.0e-9);
}

TEST(Route3DTest, MaterializedContinuationCommitsToGoalDirectedWallDetour) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 40, 30, 12}};
  for (int y = 11; y <= 19; ++y) {
    for (int z = 0; z < 8; ++z) {
      occupancy.setOccupied(GridIndex3D{8, y, z});
    }
  }
  const DistanceField3D field = DistanceField3D::build(occupancy, 40.0);
  const GridBounds3D& bounds = field.bounds();
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 2.0;
  config.vertical_step_m = 1.0;
  config.planning_goal_distance_m = 30.0;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.maximum_search_time_ms = 0.001;
  config.frontier_minimum_reachable_depth_m = 8.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_samples = 0U;
  const Point3 start{2.5, 15.5, 5.5};
  const Point3 goal{30.5, 15.5, 5.5};

  const RiskAwareLattice3DResult result = planRiskAwareLattice3D(
      grid, field.distancesM(), start, Vec3{1.0, 0.0, 0.0}, goal, {}, config);

  ASSERT_EQ(result.status, Lattice3DStatus::kViableFrontier);
  EXPECT_EQ(result.termination, Lattice3DSearchTermination::kDeadlineReached);
  ASSERT_GE(result.points.size(), 2U);
  EXPECT_LT(distance3D(result.points.back(), goal), distance3D(start, goal));
  EXPECT_GT(std::abs(result.points.back().y - start.y), 1.0);
  EXPECT_NEAR(result.points.back().z, start.z, 1.0e-9);
}

TEST(Route3DTest, EqualCostSearchKeepsLevelAltitudeBeforeVerticalAlternatives) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 30, 10, 10}};
  const DistanceField3D field = DistanceField3D::build(occupancy, 30.0);
  const GridBounds3D& bounds = field.bounds();
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 2.0;
  config.vertical_step_m = 1.0;
  config.planning_goal_distance_m = 30.0;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.heading_bias_cost_per_rad = 0.0;
  config.route_shape_turn_cost_per_rad = 0.0;
  config.maximum_search_time_ms = 1000.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_samples = 0U;

  const RiskAwareLattice3DResult result =
      planRiskAwareLattice3D(grid, field.distancesM(), Point3{1.5, 4.5, 5.5},
                             Vec3{1.0, 0.0, 0.0}, Point3{20.5, 4.5, 5.5}, {}, config);

  ASSERT_EQ(result.status, Lattice3DStatus::kReachedPlanningGoal);
  ASSERT_FALSE(result.route.empty());
  for (const RouteSample3D& sample : result.route) {
    EXPECT_DOUBLE_EQ(sample.position.z, 5.5);
    EXPECT_GE(sample.position.z, config.flight_envelope.minimum_target_z_m);
    EXPECT_LT(sample.position.z, config.flight_envelope.maximum_target_z_m);
  }
}

TEST(Route3DTest, EdgeFootprintUsesSafetySweepStepIndependentOfRouteSampling) {
  constexpr int width{30};
  constexpr int height{5};
  constexpr int depth{30};
  const mppi::EsdfGrid grid{width, height, 0.1F, 0.0F, 0.0F, depth, 0.0F};
  std::vector<float> esdf(static_cast<std::size_t>(width * height * depth),
                          std::numeric_limits<float>::infinity());
  const std::size_t occupied_index =
      (std::size_t{22U} * static_cast<std::size_t>(height) + std::size_t{2U}) *
          static_cast<std::size_t>(width) +
      std::size_t{12U};
  esdf[occupied_index] = 0.0F;

  RiskAwareLattice3DConfig coarse;
  coarse.sample_step_m = 0.5;
  coarse.physical_footprint_radius_m = 0.0;
  coarse.physical_footprint_samples = 0U;
  coarse.physical_footprint_sweep_step_m = 0.5;
  coarse.preferred_distance_m = 0.0;
  coarse.critical_distance_m = 0.0;
  RiskAwareLattice3DConfig safety = coarse;
  safety.physical_footprint_sweep_step_m = 0.1;
  const Point3 first{0.05, 0.25, 2.25};
  const Point3 second{2.05, 0.25, 2.25};

  EXPECT_EQ(detail::evaluateLattice3DEdge(grid, esdf, first, second,
                                          Lattice3DRiskStage::kPreferredOnly, coarse)
                .status,
            detail::Lattice3DEdgeEvaluationStatus::kValid);
  EXPECT_EQ(detail::evaluateLattice3DEdge(grid, esdf, first, second,
                                          Lattice3DRiskStage::kPreferredOnly, safety)
                .status,
            detail::Lattice3DEdgeEvaluationStatus::kRawCollision);
}

} // namespace
} // namespace drone_city_nav
