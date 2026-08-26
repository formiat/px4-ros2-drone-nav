#include "drone_city_nav/bounded_worker_pool.hpp"
#include "drone_city_nav/distance_field_3d.hpp"
#include "drone_city_nav/esdf_query.hpp"
#include "drone_city_nav/risk_aware_lattice_3d.hpp"
#include "drone_city_nav/route_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "risk_aware_lattice_3d_continuation.hpp"
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

TEST(Route3DTest, InterpolatedSamplesRetainUnitTangentsAndExactTransitions) {
  const std::vector<RouteSample3D> route{
      RouteSample3D{
          .position = Point3{0.0, 0.0, 5.0},
          .tangent = Vec3{1.0, 0.0, 0.0},
          .station_m = 0.0,
      },
      RouteSample3D{
          .position = Point3{2.0, 0.0, 5.0},
          .tangent = Vec3{0.0, 1.0, 0.0},
          .station_m = 2.0,
          .transition = RouteKinematicTransition3D::kStopAndTurn,
      },
  };

  const RouteSample3D interpolated = sampleRoute3DAtStation(route, 1.0);
  const RouteSample3D endpoint = sampleRoute3DAtStation(route, 2.0);

  EXPECT_NEAR(std::hypot(std::hypot(interpolated.tangent.x, interpolated.tangent.y),
                         interpolated.tangent.z),
              1.0, 1.0e-9);
  EXPECT_EQ(interpolated.transition, RouteKinematicTransition3D::kContinuous);
  EXPECT_EQ(endpoint.transition, RouteKinematicTransition3D::kStopAndTurn);
  EXPECT_DOUBLE_EQ(endpoint.tangent.x, 0.0);
  EXPECT_DOUBLE_EQ(endpoint.tangent.y, 1.0);
}

TEST(Route3DTest, MaterializesAnExactActivePrefixBeforeSuccessorContinuation) {
  const std::vector<RouteSample3D> active = sampleRoute3D(
      std::vector<Point3>{{0.0, 0.0, 5.0}, {10.0, 0.0, 5.0}, {20.0, 0.0, 5.0}}, 1.0,
      4.0);
  const std::vector<RouteSample3D> successor = sampleRoute3D(
      std::vector<Point3>{{0.0, 0.0, 5.0}, {10.0, 0.0, 5.0}, {20.0, 8.0, 5.0}}, 1.0,
      4.0);

  const std::optional<FrozenRoutePrefix3D> frozen =
      materializeFrozenRoutePrefix3D(active, successor, Point3{2.0, 0.0, 5.0}, 6.0);

  ASSERT_TRUE(frozen.has_value());
  if (!frozen.has_value()) {
    return;
  }
  const FrozenRoutePrefix3D& frozen_prefix = frozen.value();
  ASSERT_TRUE(frozen_prefix.valid());
  EXPECT_NEAR(frozen_prefix.route.front().position.x, 2.0, 1.0e-6);
  EXPECT_GT(frozen_prefix.route.back().station_m, 18.0);
  const RouteSample3D stitch = sampleRoute3DAtStation(frozen_prefix.route, 6.0);
  EXPECT_NEAR(stitch.position.x, 8.0, 1.0e-6);
  EXPECT_NEAR(stitch.position.y, 0.0, 1.0e-6);
  for (std::size_t index = 1U; index < frozen_prefix.route.size(); ++index) {
    EXPECT_GT(frozen_prefix.route[index].station_m,
              frozen_prefix.route[index - 1U].station_m);
  }
}

TEST(Route3DTest, MaterializesSuccessorPlannedFromFutureStitchStation) {
  const std::vector<RouteSample3D> active = sampleRoute3D(
      std::vector<Point3>{{0.0, 0.0, 5.0}, {10.0, 0.0, 5.0}, {20.0, 0.0, 5.0}}, 1.0,
      4.0);
  const std::vector<RouteSample3D> successor = sampleRoute3D(
      std::vector<Point3>{{8.0, 0.0, 5.0}, {14.0, 0.0, 5.0}, {20.0, 6.0, 5.0}}, 1.0,
      4.0);

  const std::optional<FrozenRoutePrefix3D> frozen =
      materializeFrozenRoutePrefix3D(active, successor, Point3{2.0, 0.0, 5.0}, 6.0);

  ASSERT_TRUE(frozen.has_value());
  if (!frozen.has_value()) {
    return;
  }
  const FrozenRoutePrefix3D& frozen_prefix = frozen.value();
  EXPECT_NEAR(frozen_prefix.stitch_station_m, 8.0, 1.0e-6);
  EXPECT_NEAR(frozen_prefix.successor_begin_station_m, 0.0, 1.0e-6);
  EXPECT_NEAR(frozen_prefix.successor_stitch_station_m, 0.0, 1.0e-6);
  EXPECT_NEAR(sampleRoute3DAtStation(frozen_prefix.route, 6.0).position.x, 8.0, 1.0e-6);
}

TEST(Route3DTest, PreservesSearchStitchStationAcrossVehicleProgress) {
  const std::vector<RouteSample3D> active = sampleRoute3D(
      std::vector<Point3>{{0.0, 0.0, 5.0}, {10.0, 0.0, 5.0}, {20.0, 0.0, 5.0}}, 1.0,
      4.0);
  const std::vector<RouteSample3D> successor = sampleRoute3D(
      std::vector<Point3>{{8.0, 0.0, 5.0}, {14.0, 0.0, 5.0}, {20.0, 6.0, 5.0}}, 1.0,
      4.0);

  EXPECT_FALSE(
      materializeFrozenRoutePrefix3D(active, successor, Point3{4.0, 0.0, 5.0}, 6.0)
          .has_value());
  const std::optional<FrozenRoutePrefix3D> frozen =
      materializeFrozenRoutePrefixAtStation3D(active, successor, Point3{4.0, 0.0, 5.0},
                                              8.0);

  ASSERT_TRUE(frozen.has_value());
  if (!frozen.has_value()) {
    return;
  }
  const FrozenRoutePrefix3D& frozen_prefix = frozen.value();
  EXPECT_NEAR(frozen_prefix.active_begin_station_m, 4.0, 1.0e-6);
  EXPECT_NEAR(frozen_prefix.stitch_station_m, 8.0, 1.0e-6);
  EXPECT_NEAR(frozen_prefix.route.front().position.x, 4.0, 1.0e-6);
  EXPECT_NEAR(sampleRoute3DAtStation(frozen_prefix.route, 4.0).position.x, 8.0, 1.0e-6);
}

TEST(Route3DTest, RejectsSearchStitchAlreadyPassedByVehicle) {
  const std::vector<RouteSample3D> active =
      sampleRoute3D(std::vector<Point3>{{0.0, 0.0, 5.0}, {20.0, 0.0, 5.0}}, 1.0, 4.0);
  const std::vector<RouteSample3D> successor =
      sampleRoute3D(std::vector<Point3>{{8.0, 0.0, 5.0}, {20.0, 0.0, 5.0}}, 1.0, 4.0);

  EXPECT_FALSE(materializeFrozenRoutePrefixAtStation3D(active, successor,
                                                       Point3{9.0, 0.0, 5.0}, 8.0)
                   .has_value());
}

TEST(Route3DTest, RemapsPassageContractsOntoCanonicalFrozenRoute) {
  const std::vector<RouteSample3D> source = sampleRoute3D(
      std::vector<Point3>{{0.0, 0.0, 5.0}, {10.0, 0.0, 5.0}, {20.0, 0.0, 5.0}}, 0.5,
      4.0);
  const std::vector<SelectedPassageTraversal> traversals{
      SelectedPassageTraversal{.passage_traversal_id = "opening",
                               .direction_sign = 1,
                               .begin_station_m = 3.0,
                               .end_station_m = 9.0,
                               .min_z_m = 2.0,
                               .max_z_m = 8.0,
                               .width_m = 8.0,
                               .height_m = 6.0,
                               .minimum_clearance_m = 2.0,
                               .speed_limit_mps = 3.0,
                               .segment_spans = {}}};
  const std::vector<ConstrainedRouteSpan> source_spans =
      makeConstrainedRouteSpans(source, traversals, 7U, RouteEnvelopeConfig{});
  const std::vector<RouteSample3D> canonical = sampleRoute3D(
      std::vector<Point3>{{2.0, 0.0, 5.0}, {10.0, 0.0, 5.0}, {20.0, 0.0, 5.0}}, 0.5,
      4.0);

  const std::vector<ConstrainedRouteSpan> remapped = remapConstrainedRouteSpans(
      source, source_spans, canonical, RouteEnvelopeConfig{});

  ASSERT_EQ(remapped.size(), 1U);
  EXPECT_EQ(remapped.front().passage_traversal_id, "opening");
  EXPECT_EQ(remapped.front().route_generation, 7U);
  EXPECT_NEAR(remapped.front().begin_station_m, 1.0, 1.0e-6);
  EXPECT_NEAR(remapped.front().end_station_m, 7.0, 1.0e-6);
  ASSERT_FALSE(remapped.front().envelope.empty());
  EXPECT_NEAR(remapped.front().envelope.front().reference_z_m, 5.0, 1.0e-6);
}

TEST(Route3DTest, BoundedProjectionStaysLocalAtSelfCrossing) {
  const std::vector<RouteSample3D> route = sampleRoute3D(
      std::vector<Point3>{
          {0.0, 0.0, 0.0}, {17.0, 17.0, 0.0}, {0.0, 17.0, 0.0}, {17.0, 0.0, 0.0}},
      100.0, 10.0);
  const Point3 position{8.6, 8.4, 0.0};

  const RouteProjection3D global = projectOntoRoute3D(route, position);
  const RouteProjection3D bounded =
      projectOntoRoute3DWithinStationWindow(route, position, 11.0, 13.0);

  ASSERT_TRUE(global.valid);
  ASSERT_TRUE(bounded.valid);
  EXPECT_NEAR(global.point.x, position.x, 1.0e-6);
  EXPECT_NEAR(global.point.y, position.y, 1.0e-6);
  EXPECT_GT(global.station_m - bounded.station_m, 40.0);
  EXPECT_NEAR(bounded.station_m, route[1U].station_m * 0.5, 1.0e-6);
  EXPECT_NEAR(bounded.point.x, 8.5, 1.0e-6);
  EXPECT_NEAR(bounded.point.y, 8.5, 1.0e-6);
}

TEST(Route3DTest, RejectsInvalidBoundedProjectionWindows) {
  const std::vector<RouteSample3D> route = sampleRoute3D(
      std::vector<Point3>{{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}}, 100.0, 10.0);
  const Point3 position{5.0, 0.0, 0.0};
  const double infinity = std::numeric_limits<double>::infinity();
  const double not_a_number = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(projectOntoRoute3DWithinStationWindow({}, position, 0.0, 10.0).valid);
  EXPECT_FALSE(projectOntoRoute3DWithinStationWindow(route, position, 6.0, 4.0).valid);
  EXPECT_FALSE(
      projectOntoRoute3DWithinStationWindow(route, position, not_a_number, 10.0).valid);
  EXPECT_FALSE(
      projectOntoRoute3DWithinStationWindow(route, position, 0.0, not_a_number).valid);
  EXPECT_FALSE(
      projectOntoRoute3DWithinStationWindow(route, position, -infinity, 10.0).valid);
  EXPECT_FALSE(
      projectOntoRoute3DWithinStationWindow(route, position, 0.0, infinity).valid);
  EXPECT_FALSE(
      projectOntoRoute3DWithinStationWindow(route, position, -2.0, -1.0).valid);
  EXPECT_FALSE(
      projectOntoRoute3DWithinStationWindow(route, position, 11.0, 12.0).valid);
}

TEST(Route3DTest, ClipsProjectionToAllowedPartialSegmentEndpoints) {
  const std::vector<RouteSample3D> route = sampleRoute3D(
      std::vector<Point3>{{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}}, 100.0, 10.0);
  ASSERT_EQ(route.size(), 2U);

  const RouteProjection3D before =
      projectOntoRoute3DWithinStationWindow(route, Point3{1.0, 0.0, 0.0}, 3.0, 7.0);
  const RouteProjection3D after =
      projectOntoRoute3DWithinStationWindow(route, Point3{9.0, 0.0, 0.0}, 3.0, 7.0);

  ASSERT_TRUE(before.valid);
  EXPECT_DOUBLE_EQ(before.station_m, 3.0);
  EXPECT_DOUBLE_EQ(before.point.x, 3.0);
  EXPECT_DOUBLE_EQ(before.distance_m, 2.0);
  EXPECT_DOUBLE_EQ(before.remaining_m, 7.0);
  ASSERT_TRUE(after.valid);
  EXPECT_DOUBLE_EQ(after.station_m, 7.0);
  EXPECT_DOUBLE_EQ(after.point.x, 7.0);
  EXPECT_DOUBLE_EQ(after.distance_m, 2.0);
  EXPECT_DOUBLE_EQ(after.remaining_m, 3.0);
}

TEST(Route3DTest, CoordinatesVerticalAlignmentBeforePassageEntry) {
  const std::vector<RouteSample3D> route = sampleRoute3D(
      std::vector<Point3>{{0.0, 0.0, 18.0}, {80.0, 0.0, 5.0}, {100.0, 0.0, 5.0}}, 1.0,
      20.0);
  const double entry_station = route[route.size() - 21U].station_m;
  const std::vector<ConstrainedRouteSpan> spans{ConstrainedRouteSpan{
      .passage_traversal_id = "passage",
      .route_generation = 4U,
      .direction_sign = 1,
      .begin_station_m = entry_station,
      .end_station_m = route.back().station_m,
      .envelope = {RouteEnvelopeSample{.station_m = entry_station,
                                       .min_z_m = 1.5,
                                       .max_z_m = 8.5,
                                       .reference_z_m = 5.0,
                                       .reference_speed_mps = 10.0}},
      .segment_spans = {},
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

TEST(Route3DTest, CoordinatesVerticalClimbBeforePassageEntry) {
  const std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{0.0, 0.0, 1.0}, {100.0, 0.0, 5.0}}, 1.0, 20.0);
  const std::vector<ConstrainedRouteSpan> spans{ConstrainedRouteSpan{
      .passage_traversal_id = "passage",
      .route_generation = 5U,
      .direction_sign = 1,
      .begin_station_m = 50.0,
      .end_station_m = route.back().station_m,
      .envelope = {RouteEnvelopeSample{.station_m = 50.0,
                                       .min_z_m = 1.5,
                                       .max_z_m = 8.5,
                                       .reference_z_m = 5.0,
                                       .reference_speed_mps = 10.0}},
      .segment_spans = {},
  }};
  const ConstrainedRouteObservation approach =
      observeConstrainedRoute(route, spans, 5U, 10.0, Point3{10.0, 0.0, 1.0},
                              Vec3{20.0, 0.0, 0.0}, RouteEnvelopeConfig{}, 180.0);

  ConstrainedRouteCoordinator coordinator;
  const ConstrainedRouteControl control =
      coordinator.update(approach, 20.0, ConstrainedRouteControlConfig{});

  EXPECT_TRUE(control.active);
  EXPECT_FALSE(control.vertical_ready);
  EXPECT_FALSE(control.hold_xy);
  EXPECT_GT(control.required_alignment_time_s, 0.0);
  EXPECT_GT(control.alignment_start_distance_m, 40.0);
  EXPECT_LT(control.speed_limit_mps, 20.0);
}

TEST(Route3DTest, ObservesConstrainedSpanLifecycleAndMotionMetrics) {
  const std::vector<Point3> route_points{{0.0, 0.0, 0.0}, {100.0, 0.0, 10.0}};
  const std::vector<RouteSample3D> route = sampleRoute3D(route_points, 5.0, 20.0);
  const std::vector<ConstrainedRouteSpan> spans{
      ConstrainedRouteSpan{
          .passage_traversal_id = "test_passage",
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
          .segment_spans = {},
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

  ASSERT_TRUE(assignRouteRiskTiers(route, grid, esdf, 1.0, 6.0).accepted());
  EXPECT_EQ(route[0].required_risk_tier, mppi::RiskTier::kPreferred);
  EXPECT_EQ(route[1].required_risk_tier, mppi::RiskTier::kPlanning);
  EXPECT_EQ(route[2].required_risk_tier, mppi::RiskTier::kCritical);
}

TEST(Route3DTest, RiskTierAssignmentAllowsUnknownSpaceOutsideStrictMode) {
  const mppi::EsdfGrid grid{2, 1, 1.0F, 0.0F, 0.0F, 1, 0.0F};
  const std::vector<float> esdf{mppi::kUnknownEsdfDistanceM,
                                std::numeric_limits<float>::infinity()};
  std::vector<RouteSample3D> route{
      RouteSample3D{.position = Point3{0.5, 0.5, 0.5}},
      RouteSample3D{.position = Point3{1.5, 0.5, 0.5}},
  };

  EXPECT_TRUE(assignRouteRiskTiers(route, grid, esdf, 1.0, 6.0, false).accepted());
  const RouteRiskTierAssignmentResult strict =
      assignRouteRiskTiers(route, grid, esdf, 1.0, 6.0, true);
  EXPECT_EQ(strict.status, RouteRiskTierAssignmentStatus::kUnknownSpace);
  EXPECT_EQ(strict.failure_sample_index, 0U);
  EXPECT_DOUBLE_EQ(strict.failure_point.x, 0.5);
}

TEST(Route3DTest, RiskTierAssignmentReportsRawCollision) {
  const mppi::EsdfGrid grid{2, 1, 1.0F, 0.0F, 0.0F, 1, 0.0F};
  const std::vector<float> esdf{std::numeric_limits<float>::infinity(), 0.0F};
  std::vector<RouteSample3D> route{
      RouteSample3D{.position = Point3{0.5, 0.5, 0.5}},
      RouteSample3D{.position = Point3{1.5, 0.5, 0.5}},
  };

  const RouteRiskTierAssignmentResult result =
      assignRouteRiskTiers(route, grid, esdf, 1.0, 6.0, false);
  EXPECT_EQ(result.status, RouteRiskTierAssignmentStatus::kRawCollision);
  EXPECT_EQ(result.failure_sample_index, 1U);
  EXPECT_DOUBLE_EQ(result.failure_point.x, 1.5);
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

TEST(Route3DTest, DistinguishesUnknownSpaceFromLocalRoiBoundary) {
  const mppi::EsdfGrid grid{6, 6, 1.0F, 0.0F, 0.0F, 4, 0.0F, true};
  std::vector<float> esdf(
      static_cast<std::size_t>(grid.width * grid.height * grid.depth), 20.0F);
  const std::size_t unknown_index =
      (std::size_t{1U} * static_cast<std::size_t>(grid.height) + std::size_t{2U}) *
          static_cast<std::size_t>(grid.width) +
      std::size_t{2U};
  esdf.at(unknown_index) = mppi::kUnknownEsdfDistanceM;
  RiskAwareLattice3DConfig config;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_lower_extent_m = 0.0;
  config.physical_footprint_upper_extent_m = 0.0;
  config.physical_footprint_samples = 0U;
  config.require_known_free_space = true;
  EXPECT_EQ(detail::evaluateLattice3DEdge(grid, esdf, Point3{1.5, 2.5, 1.5},
                                          Point3{2.5, 2.5, 1.5},
                                          Lattice3DRiskStage::kCriticalAllowed, config)
                .status,
            detail::Lattice3DEdgeEvaluationStatus::kUnknownSpace);
  config.require_known_free_space = false;
  const detail::Lattice3DEdgeEvaluation optimistic_unknown =
      detail::evaluateLattice3DEdge(grid, esdf, Point3{1.5, 2.5, 1.5},
                                    Point3{2.5, 2.5, 1.5},
                                    Lattice3DRiskStage::kCriticalAllowed, config);
  EXPECT_EQ(optimistic_unknown.status, detail::Lattice3DEdgeEvaluationStatus::kValid);
  EXPECT_TRUE(optimistic_unknown.evidence.unknown_exposure);
  EXPECT_FALSE(optimistic_unknown.evidence.raw_collision);
  EXPECT_EQ(detail::evaluateLattice3DEdge(grid, esdf, Point3{4.5, 2.5, 1.5},
                                          Point3{6.5, 2.5, 1.5},
                                          Lattice3DRiskStage::kCriticalAllowed, config)
                .status,
            detail::Lattice3DEdgeEvaluationStatus::kOutsideGrid);
}

TEST(Route3DTest, LatticeTraversesLShapedPassage) {
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
  config.critical_distance_m = 0.0;
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
  config.critical_distance_m = 0.0;
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

TEST(Route3DTest, BuildsTypedSpanFromSelectedPassageTraversal) {
  const std::vector<RouteSample3D> route = sampleRoute3D(
      std::vector<Point3>{{0.0, 0.0, 5.0}, {5.0, 0.0, 5.0}, {10.0, 0.0, 5.0}}, 0.5,
      20.0);
  const std::vector<SelectedPassageTraversal> traversals{
      SelectedPassageTraversal{.passage_traversal_id = "test_passage",
                               .direction_sign = -1,
                               .begin_station_m = 2.0,
                               .end_station_m = 8.0,
                               .min_z_m = 1.5,
                               .max_z_m = 8.5,
                               .width_m = 24.0,
                               .height_m = 7.0,
                               .minimum_clearance_m = 3.5,
                               .speed_limit_mps = 10.0,
                               .segment_spans = {}}};

  const std::vector<ConstrainedRouteSpan> spans =
      makeConstrainedRouteSpans(route, traversals, 12U, RouteEnvelopeConfig{});

  ASSERT_EQ(spans.size(), 1U);
  EXPECT_EQ(spans.front().passage_traversal_id, "test_passage");
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
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_lower_extent_m = 0.0;
  config.physical_footprint_upper_extent_m = 0.0;
  config.physical_footprint_samples = 0U;
  config.maximum_search_time_ms = 3000.0;
  config.maximum_expansions = 500000U;
  config.heading_bias_cost_per_rad = 0.0;

  const RiskAwareLattice3DResult result =
      planRiskAwareLattice3D(grid, field.distancesM(), Point3{2.5, 9.5, 1.5},
                             Vec3{1.0, 0.0, 0.0}, Point3{21.5, 9.5, 1.5}, {}, config);

  ASSERT_EQ(result.status, Lattice3DStatus::kReachedPlanningGoal)
      << "termination=" << static_cast<int>(result.termination)
      << " expansions=" << result.expansions << " records_peak=" << result.records_peak
      << " topology_candidates=" << result.topology_candidates.size();
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

TEST(Route3DTest, SelectsEmbeddedPassageEdgeWhenItsObjectiveCostIsLower) {
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
  const std::vector<PassageTraversalEdge> passages{PassageTraversalEdge{
      .id = "direct_passage",
      .region_id = "direct_region",
      .entry_portal_id = "direct_entry",
      .exit_portal_id = "direct_exit",
      .centerline = sampleRoute3D(
          std::vector<Point3>{{0.5, 0.5, 5.5}, {12.5, 8.5, 5.5}}, 0.5, 10.0),
      .entry = Point3{0.5, 0.5, 5.5},
      .exit = Point3{12.5, 8.5, 5.5},
      .min_z_m = 1.5,
      .max_z_m = 8.5,
      .width_m = 24.0,
      .height_m = 7.0,
      .minimum_clearance_m = 3.5,
      .speed_limit_mps = 10.0,
      .segment_spans = {}}};
  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 4.0;
  config.vertical_step_m = 1.0;
  config.planning_goal_distance_m = 30.0;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.heading_bias_cost_per_rad = 0.0;
  config.route_shape_turn_cost_per_rad = 0.0;
  config.passage_connection_distance_m = 1.0;
  config.maximum_search_time_ms = 1000.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_samples = 0U;

  const RiskAwareLattice3DResult result = planRiskAwareLattice3D(
      grid, field.distancesM(), Point3{0.5, 0.5, 5.5}, Vec3{1.0, 0.0, 0.0},
      Point3{12.5, 8.5, 5.5}, passages, config);

  ASSERT_EQ(result.status, Lattice3DStatus::kReachedPlanningGoal);
  ASSERT_EQ(result.selected_passage_traversals.size(), 1U);
  EXPECT_EQ(result.selected_passage_traversals.front().passage_traversal_id,
            "direct_passage");
  EXPECT_GT(result.selected_passage_traversals.front().end_station_m,
            result.selected_passage_traversals.front().begin_station_m);
}

TEST(Route3DTest, SeedsCollisionValidatedPassageBeyondLocalConnectionRadius) {
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
  const std::vector<PassageTraversalEdge> passages{PassageTraversalEdge{
      .id = "far_passage",
      .region_id = "far_region",
      .entry_portal_id = "far_entry",
      .exit_portal_id = "far_exit",
      .centerline = sampleRoute3D(
          std::vector<Point3>{{8.5, 2.5, 5.5}, {18.5, 2.5, 5.5}}, 0.5, 10.0),
      .entry = Point3{8.5, 2.5, 5.5},
      .exit = Point3{18.5, 2.5, 5.5},
      .min_z_m = 1.5,
      .max_z_m = 8.5,
      .width_m = 24.0,
      .height_m = 7.0,
      .minimum_clearance_m = 3.5,
      .speed_limit_mps = 10.0,
      .segment_spans = {}}};
  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 4.0;
  config.vertical_step_m = 1.0;
  config.planning_goal_distance_m = 30.0;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.heading_bias_cost_per_rad = 0.0;
  config.route_shape_turn_cost_per_rad = 0.0;
  config.passage_connection_distance_m = 1.0;
  config.maximum_search_time_ms = 1000.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_samples = 0U;

  const RiskAwareLattice3DResult result = planRiskAwareLattice3D(
      grid, field.distancesM(), Point3{0.5, 2.5, 8.5}, Vec3{1.0, 0.0, 0.0},
      Point3{22.5, 2.5, 5.5}, passages, config);

  ASSERT_EQ(result.status, Lattice3DStatus::kReachedPlanningGoal);
  ASSERT_EQ(result.selected_passage_traversals.size(), 1U);
  EXPECT_EQ(result.selected_passage_traversals.front().passage_traversal_id,
            "far_passage");
}

TEST(Route3DTest, MaterializesRequiredPassageWhenUnconstrainedSlicePrefersFrontier) {
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
  const std::vector<PassageTraversalEdge> passages{PassageTraversalEdge{
      .id = "required_passage",
      .region_id = "required_region",
      .entry_portal_id = "required_entry",
      .exit_portal_id = "required_exit",
      .centerline = sampleRoute3D(
          std::vector<Point3>{{30.5, 5.5, 5.5}, {60.5, 5.5, 5.5}}, 0.5, 10.0),
      .entry = Point3{30.5, 5.5, 5.5},
      .exit = Point3{60.5, 5.5, 5.5},
      .min_z_m = 1.5,
      .max_z_m = 8.5,
      .width_m = 24.0,
      .height_m = 7.0,
      .minimum_clearance_m = 3.5,
      .speed_limit_mps = 10.0,
      .segment_spans = {}}};
  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 2.0;
  config.vertical_step_m = 1.0;
  config.planning_goal_distance_m = 100.0;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.heading_bias_cost_per_rad = 0.0;
  config.route_shape_turn_cost_per_rad = 0.0;
  config.passage_topology_transition_cost = 100.0;
  config.maximum_topology_search_groups = 1U;
  config.maximum_expansions = 8U;
  config.maximum_search_time_ms = 1000.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_samples = 0U;

  const RiskAwareLattice3DResult result = planRiskAwareLattice3D(
      grid, field.distancesM(), Point3{2.5, 5.5, 5.5}, Vec3{1.0, 0.0, 0.0},
      Point3{74.5, 5.5, 5.5}, passages, config);

  EXPECT_EQ(result.topology_searches, 2U);
  ASSERT_EQ(result.selected_passage_traversals.size(), 1U);
  EXPECT_EQ(result.selected_passage_traversals.front().passage_traversal_id,
            "required_passage");
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
  const auto passage = [](const std::string& id, const std::vector<Point3>& points) {
    return PassageTraversalEdge{.id = id,
                                .region_id = id + "_region",
                                .entry_portal_id = id + "_entry",
                                .exit_portal_id = id + "_exit",
                                .centerline = sampleRoute3D(points, 0.5, 10.0),
                                .entry = points.front(),
                                .exit = points.back(),
                                .min_z_m = 1.5,
                                .max_z_m = 8.5,
                                .width_m = 24.0,
                                .height_m = 7.0,
                                .minimum_clearance_m = 3.5,
                                .speed_limit_mps = 10.0,
                                .segment_spans = {}};
  };
  const std::vector<PassageTraversalEdge> passages{
      passage("direct", {{0.5, 2.5, 5.5}, {18.5, 2.5, 5.5}}),
      passage("detour", {{0.5, 2.5, 5.5}, {9.5, 15.5, 5.5}, {18.5, 2.5, 5.5}})};
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
      grid, field.distancesM(), start, Vec3{1.0, 0.0, 0.0}, goal, passages, config);
  BoundedWorkerPool two_worker_pool{2U};
  const RiskAwareLattice3DResult two_worker =
      planRiskAwareLattice3D(grid, field.distancesM(), start, Vec3{1.0, 0.0, 0.0}, goal,
                             passages, config, &two_worker_pool);
  BoundedWorkerPool worker_pool{4U};
  const RiskAwareLattice3DResult parallel =
      planRiskAwareLattice3D(grid, field.distancesM(), start, Vec3{1.0, 0.0, 0.0}, goal,
                             passages, config, &worker_pool);

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

TEST(Route3DTest, SpatiallyIndexesPassageEntriesDuringLatticeExpansion) {
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
  std::vector<PassageTraversalEdge> passages;
  for (std::size_t index = 0U; index < 40U; ++index) {
    const double y = 100.5 + 4.0 * static_cast<double>(index);
    const std::string id = "remote_" + std::to_string(index);
    passages.push_back(PassageTraversalEdge{
        .id = id,
        .region_id = id + "_region",
        .entry_portal_id = id + "_entry",
        .exit_portal_id = id + "_exit",
        .centerline = sampleRoute3D(std::vector<Point3>{{4.5, y, 5.5}, {12.5, y, 5.5}},
                                    0.5, 10.0),
        .entry = Point3{4.5, y, 5.5},
        .exit = Point3{12.5, y, 5.5},
        .min_z_m = 1.5,
        .max_z_m = 8.5,
        .width_m = 8.0,
        .height_m = 7.0,
        .minimum_clearance_m = 3.5,
        .speed_limit_mps = 10.0,
        .segment_spans = {},
    });
  }
  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 2.0;
  config.vertical_step_m = 1.0;
  config.planning_goal_distance_m = 30.0;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.maximum_topology_search_groups = 0U;
  config.maximum_search_time_ms = 1000.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_samples = 0U;

  const RiskAwareLattice3DResult result = planRiskAwareLattice3D(
      grid, field.distancesM(), Point3{2.5, 2.5, 5.5}, Vec3{1.0, 0.0, 0.0},
      Point3{20.5, 2.5, 5.5}, passages, config);

  ASSERT_EQ(result.status, Lattice3DStatus::kReachedPlanningGoal);
  EXPECT_EQ(result.successor_profiling.search.maximum_candidates, passages.size() * 2U);
  EXPECT_TRUE(result.selected_passage_traversals.empty());
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
  EXPECT_NEAR(result.achieved_progress_m,
              distance3D(start, goal) - distance3D(result.points.back(), goal), 1.0e-9);
  EXPECT_LT(distance3D(result.points.back(), goal), distance3D(start, goal));
  EXPECT_NEAR(result.points.back().y, start.y, 1.0e-9);
  EXPECT_NEAR(result.points.back().z, start.z, 1.0e-9);
}

TEST(Route3DTest, ContinuationSelectsGoalProgressThatExtendsTheRouteFrontier) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 30, 30, 12}};
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
  config.frontier_minimum_reachable_depth_m = 8.0;
  config.frontier_validation_maximum_states = 256U;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_samples = 0U;
  const Point3 route_origin{2.5, 15.5, 5.5};
  const Point3 terminal{10.5, 15.5, 5.5};

  const detail::Lattice3DContinuationMetrics continuation =
      detail::evaluateLattice3DContinuation(
          grid, field.distancesM(), route_origin, terminal, Vec3{1.0, 0.0, 0.0},
          Point3{25.5, 15.5, 5.5}, Lattice3DRiskStage::kPreferredOnly, config, nullptr);

  ASSERT_GE(continuation.path.size(), 2U);
  EXPECT_GT(distance3D(route_origin, continuation.path.back()),
            distance3D(route_origin, terminal));
  EXPECT_LT(distance3D(continuation.path.back(), Point3{25.5, 15.5, 5.5}),
            distance3D(terminal, Point3{25.5, 15.5, 5.5}));
}

TEST(Route3DTest, ContinuationRejectsReachableSpaceThatOnlyLoopsBack) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 32, 5, 3}};
  for (int x = 0; x < 32; ++x) {
    for (int y = 0; y < 5; ++y) {
      for (int z = 0; z < 3; ++z) {
        occupancy.setOccupied(GridIndex3D{x, y, z});
      }
    }
  }
  for (int x = 2; x <= 28; ++x) {
    occupancy.clearOccupied(GridIndex3D{x, 2, 1});
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
  config.frontier_minimum_reachable_depth_m = 8.0;
  config.frontier_validation_maximum_states = 256U;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_samples = 0U;
  const Point3 route_origin{2.5, 2.5, 1.5};
  const Point3 terminal{28.5, 2.5, 1.5};

  const detail::Lattice3DContinuationMetrics continuation =
      detail::evaluateLattice3DContinuation(
          grid, field.distancesM(), route_origin, terminal, Vec3{0.0, 1.0, 0.0},
          Point3{40.5, 2.5, 1.5}, Lattice3DRiskStage::kCriticalAllowed, config,
          nullptr);

  EXPECT_GT(continuation.immediate_successors, 0U);
  EXPECT_GE(continuation.reachable_depth_m, config.frontier_minimum_reachable_depth_m);
  EXPECT_TRUE(continuation.path.empty());
}

TEST(Route3DTest, ContinuationValidationHonorsItsTimeBudget) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 30, 30, 12}};
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
  config.frontier_validation_maximum_states = 100000U;
  config.frontier_validation_maximum_time_ms = 1.0e-9;
  config.frontier_minimum_reachable_depth_m = 100.0;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_samples = 0U;

  const detail::Lattice3DContinuationMetrics continuation =
      detail::evaluateLattice3DContinuation(
          grid, field.distancesM(), Point3{2.5, 15.5, 5.5}, Point3{10.5, 15.5, 5.5},
          Vec3{1.0, 0.0, 0.0}, Point3{25.5, 15.5, 5.5},
          Lattice3DRiskStage::kPreferredOnly, config, nullptr);

  EXPECT_TRUE(continuation.time_budget_exhausted);
  EXPECT_TRUE(continuation.path.empty());
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
  EXPECT_NEAR(result.achieved_progress_m,
              distance3D(start, goal) - distance3D(result.points.back(), goal), 1.0e-9);
  EXPECT_LT(distance3D(result.points.back(), goal), distance3D(start, goal));
  EXPECT_GT(std::abs(result.points.back().y - start.y), 1.0);
  EXPECT_NEAR(result.points.back().z, start.z, 1.0e-9);
}

TEST(Route3DTest, ClipsSpanWithInterpolatedBoundaryEnvelopeSamples) {
  ConstrainedRouteSpan span{
      .passage_traversal_id = PassageTraversalId{"passage"},
      .route_generation = 7U,
      .direction_sign = 1,
      .begin_station_m = 1.0,
      .end_station_m = 5.0,
      .envelope =
          {
              RouteEnvelopeSample{
                  .station_m = 1.0, .reference_z_m = 2.0, .reference_speed_mps = 3.0},
              RouteEnvelopeSample{
                  .station_m = 5.0, .reference_z_m = 6.0, .reference_speed_mps = 7.0},
          },
      .segment_spans = {},
  };

  const std::vector<ConstrainedRouteSpan> clipped =
      clipConstrainedRouteSpans({&span, 1U}, 2.0, 4.0);

  ASSERT_EQ(clipped.size(), 1U);
  ASSERT_EQ(clipped.front().envelope.size(), 2U);
  EXPECT_DOUBLE_EQ(clipped.front().begin_station_m, 2.0);
  EXPECT_DOUBLE_EQ(clipped.front().end_station_m, 4.0);
  EXPECT_DOUBLE_EQ(clipped.front().envelope.front().station_m, 2.0);
  EXPECT_DOUBLE_EQ(clipped.front().envelope.front().reference_z_m, 3.0);
  EXPECT_DOUBLE_EQ(clipped.front().envelope.back().station_m, 4.0);
  EXPECT_DOUBLE_EQ(clipped.front().envelope.back().reference_speed_mps, 6.0);
}

} // namespace
} // namespace drone_city_nav
