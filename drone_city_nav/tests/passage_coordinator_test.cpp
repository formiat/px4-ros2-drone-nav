#include "drone_city_nav/passage_coordinator.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::shared_ptr<const SemanticPortalRoute>
testRoute(const std::uint64_t generation = 1U) {
  auto polyline = std::make_shared<const std::vector<Point2>>(
      std::initializer_list<Point2>{Point2{0.0, 0.0}, Point2{20.0, 0.0}});
  auto route = std::make_shared<SemanticPortalRoute>();
  route->generation = generation;
  route->polyline = std::move(polyline);
  route->point_stations_m = {0.0, 20.0};
  route->total_length_m = 20.0;
  route->passage_events = {
      RoutePassageEvent{
          .portal =
              Portal{
                  .id = "portal",
                  .structure_id = "connector",
                  .entry_plane = PortalPlane{Point2{9.0, 0.0}, Point2{1.0, 0.0}},
                  .exit_plane = PortalPlane{Point2{11.0, 0.0}, Point2{1.0, 0.0}},
                  .center = Point3{10.0, 0.0, 5.0},
                  .normal_xy = Point2{1.0, 0.0},
                  .width_m = 6.0,
                  .depth_m = 2.0,
                  .min_z_m = 1.5,
                  .max_z_m = 8.5,
              },
          .approach_station_m = 4.0,
          .entry_station_m = 9.0,
          .exit_station_m = 11.0,
          .departure_station_m = 15.0,
          .traversal_direction = 1,
          .preferred_z_m = 5.0,
          .speed_limit_mps = 5.0,
      },
  };
  return route;
}

[[nodiscard]] PassageCoordinatorInput
inputAt(std::shared_ptr<const SemanticPortalRoute> route, const float x, const float z,
        const float vz = 0.0F) {
  return PassageCoordinatorInput{
      .state =
          mppi::State{
              .x = x,
              .y = 0.0F,
              .z = z,
              .vx = 5.0F,
              .vy = 0.0F,
              .vz = vz,
          },
      .route = std::move(route),
      .route_station_m = x,
      .normal_flight_z_m = 18.0,
      .approach_speed_mps = 5.0,
  };
}

TEST(PassageCoordinatorTest, RemainsInactiveWithoutSemanticRoute) {
  PassageCoordinator coordinator;

  const PassageCoordinatorResult result =
      coordinator.update(inputAt(nullptr, 0.0F, 18.0F));

  EXPECT_EQ(result.phase, PassageCoordinatorPhase::kInactive);
  EXPECT_FALSE(result.active);
  EXPECT_FALSE(result.constraint.has_value());
}

TEST(PassageCoordinatorTest, ReportsUpcomingEventWithoutLimitingSpeed) {
  PassageCoordinator coordinator;

  const PassageCoordinatorResult result =
      coordinator.update(inputAt(testRoute(), 0.0F, 18.0F));

  EXPECT_EQ(result.phase, PassageCoordinatorPhase::kUpcoming);
  EXPECT_TRUE(result.active);
  EXPECT_FALSE(result.hold_xy);
  EXPECT_FALSE(result.speed_limit_active);
  EXPECT_EQ(result.portal_id, "portal");
  ASSERT_TRUE(result.constraint.has_value());
  const mppi::PassageConstraint upcoming_constraint =
      result.constraint.value_or(mppi::PassageConstraint{});
  EXPECT_EQ(upcoming_constraint.phase, mppi::PassagePhase::kUpcoming);
}

TEST(PassageCoordinatorTest, HoldsXYWhenAltitudeCannotBeCapturedBeforeEntry) {
  PassageCoordinator coordinator;

  const PassageCoordinatorResult result =
      coordinator.update(inputAt(testRoute(), 5.0F, 18.0F));

  EXPECT_EQ(result.phase, PassageCoordinatorPhase::kVerticalAlignment);
  EXPECT_TRUE(result.hold_xy);
  EXPECT_TRUE(result.speed_limit_active);
  EXPECT_GT(result.required_alignment_distance_m, result.distance_to_entry_m);
  ASSERT_TRUE(result.constraint.has_value());
  const mppi::PassageConstraint alignment_constraint =
      result.constraint.value_or(mppi::PassageConstraint{});
  EXPECT_EQ(alignment_constraint.phase, mppi::PassagePhase::kVerticalAlignment);
}

TEST(PassageCoordinatorTest, ReleasesHoldAfterStableVerticalCapture) {
  PassageCoordinator coordinator;
  const auto route = testRoute();
  static_cast<void>(coordinator.update(inputAt(route, 5.0F, 18.0F)));

  PassageCoordinatorResult result;
  for (std::size_t cycle = 0U; cycle < 3U; ++cycle) {
    result = coordinator.update(inputAt(route, 5.0F, 5.0F, 0.1F));
  }

  EXPECT_EQ(result.phase, PassageCoordinatorPhase::kReady);
  EXPECT_FALSE(result.hold_xy);
  EXPECT_TRUE(result.vertical_ready);
  EXPECT_DOUBLE_EQ(result.z_reference_m, 5.0);
}

TEST(PassageCoordinatorTest, TraversesOnlyTheEventFromCurrentRouteGeneration) {
  PassageCoordinator coordinator;
  const auto first_route = testRoute(10U);
  static_cast<void>(coordinator.update(inputAt(first_route, 5.0F, 18.0F)));
  for (std::size_t cycle = 0U; cycle < 3U; ++cycle) {
    static_cast<void>(coordinator.update(inputAt(first_route, 5.0F, 5.0F)));
  }
  const PassageCoordinatorResult traversal =
      coordinator.update(inputAt(first_route, 10.0F, 5.0F));

  const auto replacement_route = testRoute(11U);
  const PassageCoordinatorResult replacement =
      coordinator.update(inputAt(replacement_route, 0.0F, 18.0F));

  EXPECT_EQ(traversal.phase, PassageCoordinatorPhase::kTraversal);
  EXPECT_EQ(traversal.route_generation, 10U);
  EXPECT_EQ(replacement.phase, PassageCoordinatorPhase::kUpcoming);
  EXPECT_EQ(replacement.route_generation, 11U);
  EXPECT_FALSE(replacement.vertical_ready);
}

TEST(PassageCoordinatorTest, PreservesValidAltitudeWhenRouteStartsInsidePortal) {
  PassageCoordinator coordinator;

  const PassageCoordinatorResult result =
      coordinator.update(inputAt(testRoute(), 10.0F, 6.0F));

  EXPECT_EQ(result.phase, PassageCoordinatorPhase::kTraversal);
  EXPECT_TRUE(result.vertical_ready);
  EXPECT_DOUBLE_EQ(result.preferred_z_m, 6.0);
}

TEST(PassageCoordinatorTest, CompletesEventAfterCrossingExitPlane) {
  PassageCoordinator coordinator;
  const auto route = testRoute();
  static_cast<void>(coordinator.update(inputAt(route, 10.0F, 5.0F)));

  const PassageCoordinatorResult cleared =
      coordinator.update(inputAt(route, 12.0F, 5.0F));
  const PassageCoordinatorResult inactive =
      coordinator.update(inputAt(route, 12.0F, 5.0F));

  EXPECT_EQ(cleared.phase, PassageCoordinatorPhase::kCleared);
  EXPECT_EQ(cleared.portal_id, "portal");
  EXPECT_EQ(inactive.phase, PassageCoordinatorPhase::kInactive);
}

TEST(PassageCoordinatorTest, DoesNotCompleteFromRouteStationWithoutPlaneCrossing) {
  PassageCoordinator coordinator;
  const auto route = testRoute();
  static_cast<void>(coordinator.update(inputAt(route, 10.0F, 5.0F)));

  PassageCoordinatorInput projected_past_exit = inputAt(route, 10.0F, 5.0F);
  projected_past_exit.route_station_m = 12.0;
  const PassageCoordinatorResult result = coordinator.update(projected_past_exit);

  EXPECT_EQ(result.phase, PassageCoordinatorPhase::kTraversal);
  EXPECT_TRUE(result.entry_plane_crossed);
  EXPECT_FALSE(result.exit_plane_crossed);
}

TEST(PassageCoordinatorTest, RetainsReadyStateAcrossBriefAltitudeViolation) {
  PassageCoordinator coordinator;
  const auto route = testRoute();
  static_cast<void>(coordinator.update(inputAt(route, 5.0F, 18.0F)));
  for (std::size_t cycle = 0U; cycle < 3U; ++cycle) {
    static_cast<void>(coordinator.update(inputAt(route, 5.0F, 5.0F)));
  }

  const PassageCoordinatorResult first = coordinator.update(inputAt(route, 5.0F, 8.0F));
  const PassageCoordinatorResult second =
      coordinator.update(inputAt(route, 5.0F, 8.0F));
  const PassageCoordinatorResult lost = coordinator.update(inputAt(route, 5.0F, 8.0F));

  EXPECT_TRUE(first.vertical_ready);
  EXPECT_TRUE(second.vertical_ready);
  EXPECT_EQ(lost.phase, PassageCoordinatorPhase::kVerticalAlignment);
  EXPECT_FALSE(lost.vertical_ready);
}

} // namespace
} // namespace drone_city_nav
