#include "drone_city_nav/distance_field_3d.hpp"
#include "drone_city_nav/risk_aware_lattice_3d.hpp"
#include "drone_city_nav/route_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

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

TEST(Route3DTest, ObservesConstrainedSpanLifecycleAndMotionMetrics) {
  const std::vector<Point3> route_points{{0.0, 0.0, 0.0}, {100.0, 0.0, 10.0}};
  const std::vector<RouteSample3D> route = sampleRoute3D(route_points, 5.0, 20.0);
  const std::vector<ConstrainedRouteSpan> spans{
      ConstrainedRouteSpan{
          .route_generation = 7U,
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
                             Vec3{-1.0, 0.0, 0.0}, Point3{9.5, 5.5, 4.5}, config);

  ASSERT_EQ(result.status, Lattice3DStatus::kReachedPlanningGoal);
  ASSERT_FALSE(result.points.empty());
  EXPECT_NEAR(result.points.back().x, 9.5, 1.0e-6);
  EXPECT_TRUE(result.reached_mission_goal);
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
                             Vec3{1.0, 0.0, 0.0}, Point3{15.5, 15.5, 1.5}, config);

  ASSERT_EQ(result.status, Lattice3DStatus::kReachedPlanningGoal);
  ASSERT_FALSE(result.points.empty());
  EXPECT_TRUE(std::any_of(result.points.begin(), result.points.end(),
                          [](const Point3& p) { return p.y > 11.0 && p.x < 10.0; }));
  EXPECT_NEAR(result.points.back().x, 15.5, 1.0e-6);
  EXPECT_NEAR(result.points.back().y, 15.5, 1.0e-6);
}

} // namespace
} // namespace drone_city_nav
