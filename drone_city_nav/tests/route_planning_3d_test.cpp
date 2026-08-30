#include "drone_city_nav/route_planning_3d.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace drone_city_nav {
namespace {

TEST(RoutePlanning3D, NetCoordinateProgressUsesTheFullThreeDimensionalMission) {
  const Point3 start{0.0, 0.0, 10.0};
  const Point3 horizontal_goal{100.0, 0.0, 10.0};

  EXPECT_DOUBLE_EQ(
      routeNetCoordinateProgress3D(start, Point3{10.0, 0.0, 10.0}, horizontal_goal),
      20.0);
  EXPECT_GT(
      routeNetCoordinateProgress3D(start, Point3{0.0, 10.0, 10.0}, horizontal_goal),
      0.0);
  EXPECT_DOUBLE_EQ(
      routeNetCoordinateProgress3D(start, Point3{-10.0, 0.0, 10.0}, horizontal_goal),
      0.0);
  EXPECT_GT(
      routeNetCoordinateProgress3D(start, Point3{0.0, 0.0, 20.0}, horizontal_goal),
      0.0);
  EXPECT_DOUBLE_EQ(routeNetCoordinateProgress3D(start, Point3{0.0, 0.0, 15.0},
                                                Point3{0.0, 0.0, 20.0}),
                   10.0);
}

[[nodiscard]] SegmentEvidenceWorld3D world(const mppi::EsdfGrid& grid,
                                           const std::vector<float>& esdf) {
  return SegmentEvidenceWorld3D{
      .grid = &grid,
      .esdf_m = esdf,
      .collision =
          OccupiedCollisionWorld3D{
              .observed_occupancy = nullptr,
              .static_occupancy = nullptr,
              .planar_occupancy = nullptr,
              .raw_point_cloud = {},
              .launch_support_contact = nullptr,
              .footprint = {.radius_m = 0.0, .sweep_step_m = 0.25},
              .flight_envelope = FlightEnvelopeConfig{.minimum_target_z_m = -10.0,
                                                      .maximum_target_z_m = 10.0},
          },
      .validated_through_revision = 12U,
  };
}

TEST(RoutePlanning3DTest, PartialRouteDoesNotCompleteMissionIntent) {
  mppi::EsdfGrid grid{.width = 4,
                      .height = 2,
                      .resolution_m = 1.0F,
                      .origin_x_m = 0.0F,
                      .origin_y_m = 0.0F,
                      .depth = 2,
                      .origin_z_m = 0.0F,
                      .outside_is_unknown = true};
  std::vector<float> esdf(16U, 5.0F);
  RouteIntent3D intent{.id = 1U,
                       .planned_on_revision = 10U,
                       .mission_target = {20.0, 0.0, 0.0},
                       .valid = true};
  const std::vector<RouteSample3D> route{{.position = {0.5, 0.5, 0.5}},
                                         {.position = {2.0, 0.5, 0.5}}};

  const SegmentEvidence3D evidence = evaluateSegmentEvidence3D(
      intent, route, route.front().position, true, false, 1.0, world(grid, esdf));

  EXPECT_TRUE(evidence.physical_executable);
  EXPECT_FALSE(evidence.reaches_mission_target);
}

TEST(RoutePlanning3DTest, UnknownIsTraversableAndDoesNotInventKnownClearance) {
  mppi::EsdfGrid grid{.width = 4,
                      .height = 2,
                      .resolution_m = 1.0F,
                      .origin_x_m = 0.0F,
                      .origin_y_m = 0.0F,
                      .depth = 2,
                      .origin_z_m = 0.0F,
                      .outside_is_unknown = true};
  std::vector<float> esdf(16U, mppi::kUnknownEsdfDistanceM);
  RouteIntent3D intent{.id = 2U,
                       .planned_on_revision = 11U,
                       .mission_target = {3.0, 0.5, 0.5},
                       .valid = true};
  const std::vector<RouteSample3D> route{{.position = {0.5, 0.5, 0.5}},
                                         {.position = {2.5, 0.5, 0.5}}};

  const SegmentEvidence3D evidence = evaluateSegmentEvidence3D(
      intent, route, route.front().position, true, false, 1.0, world(grid, esdf));

  EXPECT_TRUE(evidence.physical_executable);
  EXPECT_TRUE(evidence.unknown_exposure);
  EXPECT_FALSE(evidence.known_clearance_observed);
  EXPECT_TRUE(std::isinf(evidence.minimum_known_clearance_m));
}

TEST(RoutePlanning3DTest, ZeroDerivedDistanceCannotManufactureRawCollision) {
  mppi::EsdfGrid grid{.width = 4,
                      .height = 2,
                      .resolution_m = 1.0F,
                      .origin_x_m = 0.0F,
                      .origin_y_m = 0.0F,
                      .depth = 2,
                      .origin_z_m = 0.0F,
                      .outside_is_unknown = true};
  std::vector<float> esdf(16U, mppi::kUnknownEsdfDistanceM);
  esdf[2U] = 0.0F;
  RouteIntent3D intent{.id = 3U,
                       .planned_on_revision = 11U,
                       .mission_target = {3.0, 0.5, 0.5},
                       .valid = true};
  const std::vector<RouteSample3D> route{{.position = {0.5, 0.5, 0.5}},
                                         {.position = {2.5, 0.5, 0.5}}};

  const SegmentEvidence3D evidence = evaluateSegmentEvidence3D(
      intent, route, route.front().position, true, false, 1.0, world(grid, esdf));

  EXPECT_TRUE(evidence.physical_executable);
  EXPECT_TRUE(evidence.unknown_exposure);
  EXPECT_EQ(evidence.status, SegmentEvidenceStatus3D::kValid);
  EXPECT_TRUE(evidence.known_clearance_observed);
  EXPECT_DOUBLE_EQ(evidence.minimum_known_clearance_m, 0.0);
}

TEST(RoutePlanning3DTest, LatestRawCollisionRejectsAStaleUnknownEsdfRoute) {
  mppi::EsdfGrid grid{.width = 4,
                      .height = 2,
                      .resolution_m = 1.0F,
                      .origin_x_m = 0.0F,
                      .origin_y_m = 0.0F,
                      .depth = 2,
                      .origin_z_m = 0.0F,
                      .outside_is_unknown = true};
  const std::vector<float> esdf(16U, mppi::kUnknownEsdfDistanceM);
  ObservedOccupancyGrid3D latest_raw{GridBounds3D{0.0, 0.0, 0.0, 1.0, 4, 2, 2}};
  ASSERT_TRUE(latest_raw.setState(GridIndex3D{1, 0, 0}, ObservedVoxelState::kOccupied));
  const RouteIntent3D intent{.id = 4U,
                             .planned_on_revision = 11U,
                             .mission_target = {3.0, 0.5, 0.5},
                             .valid = true};
  const std::vector<RouteSample3D> route{{.position = {0.5, 0.5, 0.5}},
                                         {.position = {2.5, 0.5, 0.5}}};
  SegmentEvidenceWorld3D evidence_world = world(grid, esdf);
  evidence_world.collision.observed_occupancy = &latest_raw;

  const SegmentEvidence3D evidence = evaluateSegmentEvidence3D(
      intent, route, route.front().position, true, false, 1.0, evidence_world);

  EXPECT_FALSE(evidence.physical_executable);
  EXPECT_FALSE(evidence.unknown_exposure);
  EXPECT_EQ(evidence.status, SegmentEvidenceStatus3D::kRawCollision);
}

TEST(RoutePlanning3DTest, RawAuthorityMakesInvalidDerivedDistanceNonBlocking) {
  mppi::EsdfGrid grid{.width = 4,
                      .height = 2,
                      .resolution_m = 1.0F,
                      .origin_x_m = 0.0F,
                      .origin_y_m = 0.0F,
                      .depth = 2,
                      .origin_z_m = 0.0F,
                      .outside_is_unknown = true};
  const std::vector<float> esdf(16U, std::numeric_limits<float>::quiet_NaN());
  ObservedOccupancyGrid3D latest_raw{GridBounds3D{0.0, 0.0, 0.0, 1.0, 4, 2, 2}};
  for (int x = 0; x < 4; ++x) {
    ASSERT_TRUE(latest_raw.setState(GridIndex3D{x, 0, 0}, ObservedVoxelState::kFree));
  }
  const RouteIntent3D intent{.id = 5U,
                             .planned_on_revision = 12U,
                             .mission_target = {3.0, 0.5, 0.5},
                             .valid = true};
  const std::vector<RouteSample3D> route{{.position = {0.5, 0.5, 0.5}},
                                         {.position = {2.5, 0.5, 0.5}}};
  SegmentEvidenceWorld3D permissive_world = world(grid, esdf);
  permissive_world.collision.observed_occupancy = &latest_raw;

  const SegmentEvidence3D permissive = evaluateSegmentEvidence3D(
      intent, route, route.front().position, true, false, 1.0, permissive_world);
  EXPECT_TRUE(permissive.physical_executable);
  EXPECT_TRUE(permissive.invalid_esdf_exposure);
  EXPECT_EQ(permissive.status, SegmentEvidenceStatus3D::kValid);
}

TEST(RoutePlanning3DTest, RouteBeyondLocalDistanceCacheUsesRawAuthority) {
  const mppi::EsdfGrid grid{.width = 2,
                            .height = 2,
                            .resolution_m = 1.0F,
                            .origin_x_m = 0.0F,
                            .origin_y_m = 0.0F,
                            .depth = 2,
                            .origin_z_m = 0.0F,
                            .outside_is_unknown = true};
  const std::vector<float> esdf(8U, std::numeric_limits<float>::infinity());
  ObservedOccupancyGrid3D raw{GridBounds3D{0.0, 0.0, 0.0, 1.0, 8, 2, 2}};
  const RouteIntent3D intent{.id = 6U,
                             .planned_on_revision = 13U,
                             .mission_target = {6.5, 0.5, 0.5},
                             .valid = true};
  const std::vector<RouteSample3D> route{{.position = {0.5, 0.5, 0.5}},
                                         {.position = {6.5, 0.5, 0.5}}};
  SegmentEvidenceWorld3D evidence_world = world(grid, esdf);
  evidence_world.collision.observed_occupancy = &raw;

  const SegmentEvidence3D evidence = evaluateSegmentEvidence3D(
      intent, route, route.front().position, true, true, 6.0, evidence_world);

  EXPECT_TRUE(evidence.physical_executable);
  EXPECT_EQ(evidence.status, SegmentEvidenceStatus3D::kValid);
  EXPECT_TRUE(evidence.outside_grid_exposure);
}

} // namespace
} // namespace drone_city_nav
