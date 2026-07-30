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

TEST(PassageCoordinatorTest, StartsDynamicAltitudeProfileBeforeConfiguredApproach) {
  PassageCoordinator coordinator;

  const PassageCoordinatorResult result =
      coordinator.update(inputAt(testRoute(), 0.0F, 18.0F));

  EXPECT_EQ(result.phase, PassageCoordinatorPhase::kReady);
  EXPECT_TRUE(result.active);
  EXPECT_FALSE(result.hold_xy);
  EXPECT_TRUE(result.speed_limit_active);
  EXPECT_TRUE(result.traversal_predicted_safe);
  EXPECT_LT(result.effective_approach_station_m, 4.0);
  EXPECT_GT(result.effective_speed_limit_mps, 0.5);
  EXPECT_LT(result.effective_speed_limit_mps, 5.0);
  EXPECT_EQ(result.portal_id, "portal");
  ASSERT_TRUE(result.constraint.has_value());
  const mppi::PassageConstraint upcoming_constraint =
      result.constraint.value_or(mppi::PassageConstraint{});
  EXPECT_EQ(upcoming_constraint.phase, mppi::PassagePhase::kReady);
  EXPECT_FLOAT_EQ(upcoming_constraint.approach_station_m,
                  static_cast<float>(result.effective_approach_station_m));
}

TEST(PassageCoordinatorTest, RollsSpeedDownBeforeStationaryAlignmentIsNecessary) {
  PassageCoordinator coordinator;

  const PassageCoordinatorResult result =
      coordinator.update(inputAt(testRoute(), 0.0F, 18.0F));

  EXPECT_EQ(result.phase, PassageCoordinatorPhase::kReady);
  EXPECT_FALSE(result.hold_xy);
  EXPECT_TRUE(result.speed_limit_active);
  EXPECT_GT(result.required_alignment_distance_m, result.distance_to_entry_m);
  ASSERT_TRUE(result.constraint.has_value());
  const mppi::PassageConstraint alignment_constraint =
      result.constraint.value_or(mppi::PassageConstraint{});
  EXPECT_EQ(alignment_constraint.phase, mppi::PassagePhase::kReady);
  EXPECT_GT(alignment_constraint.speed_limit_mps, 0.5F);
  EXPECT_LT(alignment_constraint.speed_limit_mps, 5.0F);
}

TEST(PassageCoordinatorTest, HoldsAtFixedStationBeforeEntryAsLastFallback) {
  PassageCoordinator coordinator;

  const PassageCoordinatorResult result =
      coordinator.update(inputAt(testRoute(), 8.0F, 18.0F));

  EXPECT_EQ(result.phase, PassageCoordinatorPhase::kVerticalAlignment);
  EXPECT_TRUE(result.hold_xy);
  EXPECT_LT(result.stationary_hold_station_m, 9.0);
  EXPECT_LT(result.hold_position.x, 9.0);
  EXPECT_DOUBLE_EQ(result.effective_speed_limit_mps, 0.0);
}

TEST(PassageCoordinatorTest, ContinuesMovingWhenPredictedTraversalStaysInSafeWindow) {
  PassageCoordinator coordinator;

  const PassageCoordinatorResult result =
      coordinator.update(inputAt(testRoute(), 8.0F, 6.0F, -1.0F));

  EXPECT_EQ(result.phase, PassageCoordinatorPhase::kReady);
  EXPECT_FALSE(result.hold_xy);
  EXPECT_TRUE(result.vertical_ready);
  EXPECT_TRUE(result.traversal_predicted_safe);
  EXPECT_GT(result.effective_speed_limit_mps, 0.0);
  EXPECT_GE(result.predicted_minimum_z_m, 2.5);
  EXPECT_LE(result.predicted_maximum_z_m, 7.5);
}

TEST(PassageCoordinatorTest, NeverStartsStationaryHoldInsidePortal) {
  PassageCoordinator coordinator;

  const PassageCoordinatorResult result =
      coordinator.update(inputAt(testRoute(), 10.0F, 8.0F));

  EXPECT_EQ(result.phase, PassageCoordinatorPhase::kTraversal);
  EXPECT_TRUE(result.entry_plane_crossed);
  EXPECT_FALSE(result.hold_xy);
  EXPECT_FALSE(result.traversal_predicted_safe);
  EXPECT_DOUBLE_EQ(result.effective_speed_limit_mps, 0.5);
}

TEST(PassageCoordinatorTest, ReleasesHoldAfterStableVerticalCapture) {
  PassageCoordinator coordinator;
  const auto route = testRoute();
  static_cast<void>(coordinator.update(inputAt(route, 8.0F, 18.0F)));

  PassageCoordinatorResult result;
  for (std::size_t cycle = 0U; cycle < 3U; ++cycle) {
    result = coordinator.update(inputAt(route, 8.0F, 5.0F, 0.1F));
  }

  EXPECT_EQ(result.phase, PassageCoordinatorPhase::kReady);
  EXPECT_FALSE(result.hold_xy);
  EXPECT_TRUE(result.vertical_ready);
  EXPECT_NEAR(result.z_reference_m, 5.0, 0.2);
}

TEST(PassageCoordinatorTest,
     StationaryFallbackIgnoresOptimisticPredictionUntilCapture) {
  PassageCoordinator coordinator;
  const auto route = testRoute();
  const PassageCoordinatorResult alignment =
      coordinator.update(inputAt(route, 8.0F, 18.0F));
  ASSERT_TRUE(alignment.hold_xy);

  const PassageCoordinatorResult still_moving =
      coordinator.update(inputAt(route, 8.0F, 6.0F, 1.0F));

  EXPECT_EQ(still_moving.phase, PassageCoordinatorPhase::kVerticalAlignment);
  EXPECT_TRUE(still_moving.hold_xy);
  EXPECT_FALSE(still_moving.vertical_ready);
  EXPECT_EQ(still_moving.capture_stable_cycles, 0U);
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
  EXPECT_EQ(replacement.phase, PassageCoordinatorPhase::kReady);
  EXPECT_EQ(replacement.route_generation, 11U);
  EXPECT_TRUE(replacement.vertical_ready);
  EXPECT_TRUE(replacement.traversal_predicted_safe);
}

TEST(PassageCoordinatorTest, PreservesValidAltitudeWhenRouteStartsInsidePortal) {
  PassageCoordinator coordinator;

  const PassageCoordinatorResult result =
      coordinator.update(inputAt(testRoute(), 10.0F, 6.0F));

  EXPECT_EQ(result.phase, PassageCoordinatorPhase::kTraversal);
  EXPECT_TRUE(result.vertical_ready);
  EXPECT_DOUBLE_EQ(result.preferred_z_m, 6.0);
}

TEST(PassageCoordinatorTest, PreservesTraversalAcrossMatchingRouteGeneration) {
  PassageCoordinator coordinator;
  const auto first_route = testRoute(10U);
  const PassageCoordinatorResult initial =
      coordinator.update(inputAt(first_route, 10.0F, 5.0F));

  const auto replacement_route = testRoute(11U);
  const PassageCoordinatorResult replacement =
      coordinator.update(inputAt(replacement_route, 10.25F, 5.0F));

  EXPECT_EQ(initial.phase, PassageCoordinatorPhase::kTraversal);
  EXPECT_EQ(replacement.phase, PassageCoordinatorPhase::kTraversal);
  EXPECT_EQ(replacement.route_generation, 11U);
  EXPECT_TRUE(replacement.entry_plane_crossed);
  EXPECT_TRUE(replacement.vertical_ready);
  EXPECT_FALSE(replacement.hold_xy);
  EXPECT_DOUBLE_EQ(replacement.preferred_z_m, 5.0);
}

TEST(PassageCoordinatorTest, RecapturesReachableAltitudeAfterInsideRouteReplacement) {
  PassageCoordinator coordinator;
  const auto first_route = testRoute(10U);
  const PassageCoordinatorResult initial =
      coordinator.update(inputAt(first_route, 10.0F, 2.51F));

  EXPECT_EQ(initial.phase, PassageCoordinatorPhase::kTraversal);
  EXPECT_TRUE(initial.vertical_ready);
  EXPECT_FALSE(initial.hold_xy);
  EXPECT_DOUBLE_EQ(initial.preferred_z_m, 2.75);

  const auto replacement_route = testRoute(11U);
  PassageCoordinatorResult result =
      coordinator.update(inputAt(replacement_route, 10.25F, 2.49F));
  EXPECT_EQ(result.route_generation, 11U);
  EXPECT_FALSE(result.vertical_ready);
  EXPECT_FALSE(result.hold_xy);

  for (std::size_t cycle = 1U; cycle < 3U; ++cycle) {
    result = coordinator.update(inputAt(replacement_route, 10.25F, 2.49F));
  }
  EXPECT_EQ(result.phase, PassageCoordinatorPhase::kTraversal);
  EXPECT_FALSE(result.hold_xy);
  EXPECT_FALSE(result.vertical_ready);
  EXPECT_DOUBLE_EQ(result.preferred_z_m, 2.75);

  for (std::size_t cycle = 0U; cycle < 3U; ++cycle) {
    result = coordinator.update(inputAt(replacement_route, 10.25F, 2.75F, 0.1F));
    EXPECT_FALSE(result.hold_xy && result.vertical_ready);
  }
  EXPECT_EQ(result.phase, PassageCoordinatorPhase::kTraversal);
  EXPECT_FALSE(result.hold_xy);
  EXPECT_TRUE(result.vertical_ready);
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

TEST(PassageCoordinatorTest, UsesSafePredictionAfterCaptureRetentionExpires) {
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
  EXPECT_EQ(lost.phase, PassageCoordinatorPhase::kReady);
  EXPECT_TRUE(lost.vertical_ready);
  EXPECT_TRUE(lost.traversal_predicted_safe);
  EXPECT_FALSE(lost.hold_xy);
}

} // namespace
} // namespace drone_city_nav
