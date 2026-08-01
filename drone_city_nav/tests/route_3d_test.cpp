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
