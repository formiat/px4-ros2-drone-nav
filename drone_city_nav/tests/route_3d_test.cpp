#include "drone_city_nav/esdf_query.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/route_risk_annotation_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
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
  EXPECT_FALSE(
      materializeRouteHandoffAtStation3D(active, successor, Point3{9.0, 0.0, 5.0}, 8.0)
          .has_value());
}

TEST(Route3DTest, MaterializesDiscontinuousFutureStitchAsStopTurnHandoff) {
  const std::vector<RouteSample3D> active =
      sampleRoute3D(std::vector<Point3>{{0.0, 0.0, 5.0}, {20.0, 0.0, 5.0}}, 1.0, 4.0);
  const std::vector<RouteSample3D> successor =
      sampleRoute3D(std::vector<Point3>{{8.0, 0.0, 5.0}, {8.0, 8.0, 5.0}}, 1.0, 4.0);

  EXPECT_FALSE(materializeFrozenRoutePrefixAtStation3D(active, successor,
                                                       Point3{4.0, 0.0, 5.0}, 8.0)
                   .has_value());
  const std::optional<FrozenRoutePrefix3D> handoff =
      materializeRouteHandoffAtStation3D(active, successor, Point3{4.0, 0.0, 5.0}, 8.0);

  ASSERT_TRUE(handoff.has_value());
  if (!handoff.has_value()) {
    return;
  }
  std::vector<RouteSample3D> canonical = handoff.value().route;
  std::size_t stop_turn_count{0U};
  ASSERT_TRUE(
      canonicalizeRouteKinematics3D(canonical, 0.7071067811865476, &stop_turn_count));
  EXPECT_EQ(stop_turn_count, 1U);
  const auto stitch = std::ranges::find_if(canonical, [](const RouteSample3D& sample) {
    return distance3D(sample.position, Point3{8.0, 0.0, 5.0}) < 1.0e-9;
  });
  ASSERT_NE(stitch, canonical.end());
  EXPECT_EQ(stitch->transition, RouteKinematicTransition3D::kStopAndTurn);
  EXPECT_DOUBLE_EQ(stitch->reference_speed_mps, 0.0);
}

TEST(Route3DTest, ConnectsDiscontinuousFutureStitchWithoutStopTurn) {
  const std::vector<RouteSample3D> active =
      sampleRoute3D(std::vector<Point3>{{0.0, 0.0, 5.0}, {20.0, 0.0, 5.0}}, 1.0, 4.0);
  const std::vector<RouteSample3D> successor =
      sampleRoute3D(std::vector<Point3>{{8.0, 0.0, 5.0}, {8.0, 8.0, 5.0}}, 1.0, 4.0);

  const std::optional<FrozenRoutePrefix3D> connected =
      materializeTangentContinuousRoutePrefixAtStation3D(
          active, successor, Point3{4.0, 0.0, 5.0}, 8.0,
          FutureRouteConnectorConfig3D{});

  ASSERT_TRUE(connected.has_value());
  if (!connected.has_value()) {
    return;
  }
  const FrozenRoutePrefix3D& connected_prefix = connected.value();
  EXPECT_NEAR(connected_prefix.successor_stitch_station_m, 2.0, 1.0e-9);
  std::vector<RouteSample3D> canonical = connected_prefix.route;
  std::size_t stop_turn_count{0U};
  ASSERT_TRUE(
      canonicalizeRouteKinematics3D(canonical, 0.7071067811865476, &stop_turn_count));
  EXPECT_EQ(stop_turn_count, 0U);
  const RouteSample3D stitch = sampleRoute3DAtStation(canonical, 4.0);
  EXPECT_NEAR(stitch.position.x, 8.0, 1.0e-6);
  EXPECT_NEAR(stitch.position.y, 0.0, 1.0e-6);
  EXPECT_GT(stitch.tangent.x, 0.995);
}

TEST(Route3DTest, RejectsSpatialGapForFutureStitchHandoff) {
  const std::vector<RouteSample3D> active =
      sampleRoute3D(std::vector<Point3>{{0.0, 0.0, 5.0}, {20.0, 0.0, 5.0}}, 1.0, 4.0);
  const std::vector<RouteSample3D> successor =
      sampleRoute3D(std::vector<Point3>{{8.1, 0.0, 5.0}, {8.1, 8.0, 5.0}}, 1.0, 4.0);

  EXPECT_FALSE(
      materializeRouteHandoffAtStation3D(active, successor, Point3{4.0, 0.0, 5.0}, 8.0)
          .has_value());
  EXPECT_FALSE(
      materializeTangentContinuousRoutePrefixAtStation3D(
          active, successor, Point3{4.0, 0.0, 5.0}, 8.0, FutureRouteConnectorConfig3D{})
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

TEST(Route3DTest, AnnotatesRequiredRiskTierFromDerivedEsdfClearance) {
  const mppi::EsdfGrid grid{3, 1, 1.0F, 0.0F, 0.0F, 1, 0.0F};
  const std::vector<float> esdf{std::numeric_limits<float>::infinity(), 4.0F, 1.5F};
  std::vector<RouteSample3D> route{
      RouteSample3D{.position = Point3{0.5, 0.5, 0.5}},
      RouteSample3D{.position = Point3{1.5, 0.5, 0.5}},
      RouteSample3D{.position = Point3{2.5, 0.5, 0.5}},
  };

  ASSERT_TRUE(
      annotateRouteRiskTiersFromDerivedEsdf3D(route, grid, esdf, 1.0, 6.0).accepted());
  EXPECT_EQ(route[0].required_risk_tier, RouteRiskTier3D::kPreferred);
  EXPECT_EQ(route[1].required_risk_tier, RouteRiskTier3D::kPlanning);
  EXPECT_EQ(route[2].required_risk_tier, RouteRiskTier3D::kCritical);
}

TEST(Route3DTest, RiskTierAssignmentTreatsUnknownAsPreferredWithoutAStrictMode) {
  const mppi::EsdfGrid grid{2, 1, 1.0F, 0.0F, 0.0F, 1, 0.0F};
  const std::vector<float> esdf{mppi::kUnknownEsdfDistanceM,
                                std::numeric_limits<float>::infinity()};
  std::vector<RouteSample3D> route{
      RouteSample3D{.position = Point3{0.5, 0.5, 0.5}},
      RouteSample3D{.position = Point3{1.5, 0.5, 0.5}},
  };

  EXPECT_TRUE(
      annotateRouteRiskTiersFromDerivedEsdf3D(route, grid, esdf, 1.0, 6.0).accepted());
  EXPECT_EQ(route.front().required_risk_tier, RouteRiskTier3D::kPreferred);
}

TEST(Route3DTest, ZeroDerivedClearanceIsOnlyASoftCriticalRiskAnnotation) {
  const mppi::EsdfGrid grid{2, 1, 1.0F, 0.0F, 0.0F, 1, 0.0F};
  const std::vector<float> esdf{std::numeric_limits<float>::infinity(), 0.0F};
  std::vector<RouteSample3D> route{
      RouteSample3D{.position = Point3{0.5, 0.5, 0.5}},
      RouteSample3D{.position = Point3{1.5, 0.5, 0.5}},
  };

  const RouteRiskAnnotationResult3D result =
      annotateRouteRiskTiersFromDerivedEsdf3D(route, grid, esdf, 1.0, 6.0);
  EXPECT_TRUE(result.accepted());
  EXPECT_EQ(route.back().required_risk_tier, RouteRiskTier3D::kCritical);
}

} // namespace
} // namespace drone_city_nav
