#include "drone_city_nav/semantic_portal_route.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] PassageOpening opening(const double center_x = 10.0,
                                     const double center_y = 0.0) {
  return PassageOpening{
      .id = "portal",
      .structure_id = "connector",
      .center = Point3{center_x, center_y, 5.0},
      .normal_xy = Point2{1.0, 0.0},
      .width_m = 6.0,
      .height_m = 7.0,
      .depth_m = 2.0,
      .min_z_m = 1.5,
      .max_z_m = 8.5,
      .approach_distance_m = 5.0,
      .exit_distance_m = 4.0,
  };
}

[[nodiscard]] KnownPassageMap passageMap(PassageOpening passage) {
  return KnownPassageMap{
      .frame_id = "map",
      .structures =
          {
              PassageStructure{
                  .id = "connector",
                  .center = Point2{10.0, 0.0},
                  .size_x_m = 2.0,
                  .size_y_m = 10.0,
                  .z_min_m = 0.0,
                  .z_max_m = 10.0,
                  .openings = {std::move(passage)},
              },
          },
  };
}

[[nodiscard]] std::shared_ptr<const std::vector<Point2>>
route(std::initializer_list<Point2> points) {
  return std::make_shared<const std::vector<Point2>>(points);
}

TEST(SemanticPortalRouteTest, CreatesTypedPortalEventFromRouteCrossing) {
  const SemanticPortalRouteBuildResult result = buildSemanticPortalRoute(
      route({Point2{0.0, 0.0}, Point2{20.0, 0.0}}), 7U, passageMap(opening()), 10.0);

  ASSERT_TRUE(result.route);
  EXPECT_EQ(result.route->generation, 7U);
  ASSERT_EQ(result.route->passage_events.size(), 1U);
  const RoutePassageEvent& event = result.route->passage_events.front();
  EXPECT_EQ(event.portal.id, "portal");
  EXPECT_EQ(event.traversal_direction, 1);
  EXPECT_DOUBLE_EQ(event.approach_station_m, 4.0);
  EXPECT_DOUBLE_EQ(event.entry_station_m, 9.0);
  EXPECT_DOUBLE_EQ(event.exit_station_m, 11.0);
  EXPECT_DOUBLE_EQ(event.departure_station_m, 15.0);
  EXPECT_DOUBLE_EQ(event.speed_limit_mps, 10.0);
  ASSERT_EQ(result.route->segments.size(), 5U);
  EXPECT_EQ(result.route->segments[1].type, SemanticRouteSegmentType::kPortalApproach);
  EXPECT_EQ(result.route->segments[2].type, SemanticRouteSegmentType::kPortalTraversal);
  EXPECT_EQ(result.route->segments[3].type, SemanticRouteSegmentType::kPortalExit);
}

TEST(SemanticPortalRouteTest, DoesNotCreateEventForPortalBesideRoute) {
  const SemanticPortalRouteBuildResult result = buildSemanticPortalRoute(
      route({Point2{0.0, 10.0}, Point2{20.0, 10.0}}), 1U, passageMap(opening()), 5.0);

  ASSERT_TRUE(result.route);
  EXPECT_TRUE(result.route->passage_events.empty());
  EXPECT_EQ(result.rejected_route_miss, 1U);
}

TEST(SemanticPortalRouteTest, RequiresEntryAndExitPlaneCrossings) {
  const SemanticPortalRouteBuildResult result = buildSemanticPortalRoute(
      route({Point2{0.0, 0.0}, Point2{10.0, 0.0}, Point2{10.0, 8.0}}), 1U,
      passageMap(opening()), 5.0);

  ASSERT_TRUE(result.route);
  EXPECT_TRUE(result.route->passage_events.empty());
  EXPECT_EQ(result.rejected_route_miss, 1U);
}

TEST(SemanticPortalRouteTest, AcceptsTraversalAcrossMultipleRouteSegments) {
  const SemanticPortalRouteBuildResult result =
      buildSemanticPortalRoute(route({Point2{0.0, 0.0}, Point2{9.5, 0.0},
                                      Point2{10.5, 0.25}, Point2{20.0, 0.25}}),
                               1U, passageMap(opening()), 5.0);

  ASSERT_TRUE(result.route);
  ASSERT_EQ(result.route->passage_events.size(), 1U);
  EXPECT_LT(result.route->passage_events.front().entry_station_m,
            result.route->passage_events.front().exit_station_m);
}

TEST(SemanticPortalRouteTest, EncodesReverseTraversalInPlanesAndEvent) {
  const SemanticPortalRouteBuildResult result = buildSemanticPortalRoute(
      route({Point2{20.0, 0.0}, Point2{0.0, 0.0}}), 2U, passageMap(opening()), 5.0);

  ASSERT_TRUE(result.route);
  ASSERT_EQ(result.route->passage_events.size(), 1U);
  const RoutePassageEvent& event = result.route->passage_events.front();
  EXPECT_EQ(event.traversal_direction, -1);
  EXPECT_DOUBLE_EQ(event.portal.normal_xy.x, -1.0);
  EXPECT_DOUBLE_EQ(event.portal.entry_plane.point.x, 11.0);
  EXPECT_DOUBLE_EQ(event.portal.exit_plane.point.x, 9.0);
}

TEST(SemanticPortalRouteTest, CreatesPartialEventWhenRouteStartsInsidePortal) {
  const SemanticPortalRouteBuildResult result = buildSemanticPortalRoute(
      route({Point2{10.5, 0.0}, Point2{20.0, 0.0}}), 5U, passageMap(opening()), 5.0);

  ASSERT_TRUE(result.route);
  ASSERT_EQ(result.route->passage_events.size(), 1U);
  const RoutePassageEvent& event = result.route->passage_events.front();
  EXPECT_DOUBLE_EQ(event.entry_station_m, 0.0);
  EXPECT_NEAR(event.exit_station_m, 0.5, 1.0e-9);
}

TEST(SemanticPortalRouteTest, ProvidesContinuousStationBasedAltitudeProfile) {
  const SemanticPortalRouteBuildResult result = buildSemanticPortalRoute(
      route({Point2{0.0, 0.0}, Point2{20.0, 0.0}}), 3U, passageMap(opening()), 5.0);
  ASSERT_TRUE(result.route);

  EXPECT_DOUBLE_EQ(semanticRouteZReference(*result.route, 0.0, 18.0), 18.0);
  EXPECT_DOUBLE_EQ(semanticRouteZReference(*result.route, 4.0, 18.0), 18.0);
  EXPECT_DOUBLE_EQ(semanticRouteZReference(*result.route, 9.0, 18.0), 5.0);
  EXPECT_DOUBLE_EQ(semanticRouteZReference(*result.route, 10.0, 18.0), 5.0);
  EXPECT_DOUBLE_EQ(semanticRouteZReference(*result.route, 15.0, 18.0), 18.0);
}

TEST(SemanticPortalRouteTest, RejectsOverlappingPortalEventsDeterministically) {
  KnownPassageMap map = passageMap(opening(10.0));
  PassageOpening second = opening(12.0);
  second.id = "portal_2";
  map.structures.front().openings.push_back(second);

  const SemanticPortalRouteBuildResult result = buildSemanticPortalRoute(
      route({Point2{0.0, 0.0}, Point2{30.0, 0.0}}), 4U, map, 5.0);

  ASSERT_TRUE(result.route);
  ASSERT_EQ(result.route->passage_events.size(), 1U);
  EXPECT_EQ(result.route->passage_events.front().portal.id, "portal");
  EXPECT_EQ(result.rejected_overlap, 1U);
}

} // namespace
} // namespace drone_city_nav
