#include "drone_city_nav/rolling_route_telemetry_3d.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

TEST(RollingRouteTelemetry3DTest, ClassifiesTypedEndpointSemantics) {
  RouteIntent3D transit{.purpose = RouteIntentPurpose3D::kMissionTransit,
                        .valid = true};
  RouteIntent3D observation{
      .purpose = RouteIntentPurpose3D::kObservationFrontier,
      .valid = true,
  };

  EXPECT_EQ(routeEndpointSemantics3D(transit, false, false, true),
            RouteEndpointSemantics3D::kContinuation);
  EXPECT_EQ(routeEndpointSemantics3D(observation, true, false, true),
            RouteEndpointSemantics3D::kObservationStop);
  EXPECT_EQ(routeEndpointSemantics3D(transit, true, true, true),
            RouteEndpointSemantics3D::kMissionStop);
  EXPECT_EQ(routeEndpointSemantics3D(transit, true, true, false),
            RouteEndpointSemantics3D::kContinuation);
  EXPECT_EQ(routeEndpointSemantics3D(observation, true, true, false),
            RouteEndpointSemantics3D::kContinuation);
  EXPECT_EQ(routeEndpointSemantics3DName(RouteEndpointSemantics3D::kEmergencyBrakeTail),
            "emergency_brake_tail");
  EXPECT_EQ(effectiveRouteEndpointSemantics3D(RouteEndpointSemantics3D::kContinuation,
                                              true, true),
            RouteEndpointSemantics3D::kEmergencyBrakeTail);
  EXPECT_EQ(effectiveRouteEndpointSemantics3D(RouteEndpointSemantics3D::kContinuation,
                                              true, false),
            RouteEndpointSemantics3D::kContinuation);
}

TEST(RollingRouteTelemetry3DTest, IncludesVerticalMotionInRouteSpeed) {
  EXPECT_DOUBLE_EQ(routeSpeed3D(Vec3{0.0, 0.0, 3.5}), 3.5);
  EXPECT_DOUBLE_EQ(routeSpeed3D(Vec3{3.0, 4.0, 12.0}), 13.0);
}

TEST(RollingRouteTelemetry3DTest, KeepsPartialTopologySegmentsNonTerminal) {
  const RouteIntent3D topology{
      .strategic_plan_id = 81U,
      .purpose = RouteIntentPurpose3D::kMissionTransit,
      .intent_reaches_mission_target = true,
      .valid = true,
  };
  const RouteIntent3D observation{
      .purpose = RouteIntentPurpose3D::kObservationFrontier,
      .valid = true,
  };

  EXPECT_EQ(routeEndpointSemantics3D(topology, false, false, true),
            RouteEndpointSemantics3D::kContinuation);
  EXPECT_EQ(routeEndpointSemantics3D(observation, false, false, true),
            RouteEndpointSemantics3D::kContinuation);
}

TEST(RollingRouteTelemetry3DTest,
     EndpointSpeedAndFiniteBoundaryPoliciesRemainIndependent) {
  EXPECT_FALSE(routeEndpointHasTerminalStop3D(RouteEndpointSemantics3D::kContinuation));
  EXPECT_TRUE(
      routeEndpointUsesLocalBoundary3D(RouteEndpointSemantics3D::kContinuation));

  EXPECT_TRUE(
      routeEndpointHasTerminalStop3D(RouteEndpointSemantics3D::kObservationStop));
  EXPECT_TRUE(
      routeEndpointUsesLocalBoundary3D(RouteEndpointSemantics3D::kObservationStop));

  EXPECT_TRUE(routeEndpointHasTerminalStop3D(RouteEndpointSemantics3D::kMissionStop));
  EXPECT_FALSE(
      routeEndpointUsesLocalBoundary3D(RouteEndpointSemantics3D::kMissionStop));

  EXPECT_TRUE(
      routeEndpointHasTerminalStop3D(RouteEndpointSemantics3D::kEmergencyBrakeTail));
  EXPECT_TRUE(
      routeEndpointUsesLocalBoundary3D(RouteEndpointSemantics3D::kEmergencyBrakeTail));

  const auto invalid = static_cast<RouteEndpointSemantics3D>(255U);
  EXPECT_TRUE(routeEndpointHasTerminalStop3D(invalid));
  EXPECT_TRUE(routeEndpointUsesLocalBoundary3D(invalid));
}

TEST(RollingRouteTelemetry3DTest, KeepsDirectReplansInOneContinuityLineage) {
  RouteIntent3D first{
      .mission_target = {100.0, 20.0, 8.0},
      .intent_target = {25.0, 5.0, 8.0},
      .source = RouteIntentSource3D::kDirect,
      .purpose = RouteIntentPurpose3D::kMissionTransit,
      .valid = true,
  };
  RouteIntent3D successor = first;
  successor.intent_target = {40.0, 8.0, 8.0};
  first.id = makeRouteIntentId3D(first.source, first.purpose, first.mission_target,
                                 first.intent_target);
  successor.id = makeRouteIntentId3D(successor.source, successor.purpose,
                                     successor.mission_target, successor.intent_target);

  EXPECT_NE(first.id, successor.id);
  EXPECT_EQ(routeContinuityId3D(first), routeContinuityId3D(successor));
  EXPECT_NE(
      routeContinuityId3D(first, RouteContinuityLineage3D{.mission_epoch = 7U}),
      routeContinuityId3D(successor, RouteContinuityLineage3D{.mission_epoch = 8U}));
}

TEST(RollingRouteTelemetry3DTest, AcceptsAContinuousUnconstrainedBoundaryTrace) {
  RollingRouteTelemetry3D telemetry;
  telemetry.observe(RollingRouteTelemetryObservation3D{
      .route_generation = 7U,
      .continuity_id = 42U,
      .geometry_revision = 100U,
      .endpoint_semantics = RouteEndpointSemantics3D::kContinuation,
      .route_remaining_m = 0.5,
      .speed_mps = 4.0,
      .resident_route_available = true,
      .execution_owner_available = true,
  });
  telemetry.observe(RollingRouteTelemetryObservation3D{
      .route_generation = 8U,
      .continuity_id = 42U,
      .geometry_revision = 101U,
      .endpoint_semantics = RouteEndpointSemantics3D::kContinuation,
      .route_remaining_m = 12.0,
      .speed_mps = 3.8,
      .resident_route_available = true,
      .execution_owner_available = true,
  });

  const RollingRouteTelemetrySnapshot3D& snapshot = telemetry.snapshot();
  EXPECT_TRUE(snapshot.regressionFree());
  EXPECT_EQ(snapshot.continuation_boundary_ticks, 1U);
  EXPECT_EQ(snapshot.continuation_zero_speed_ticks, 0U);
  EXPECT_EQ(snapshot.continuity_transition_ticks, 1U);
  EXPECT_EQ(snapshot.continuity_transition_zero_speed_ticks, 0U);
  EXPECT_DOUBLE_EQ(snapshot.minimum_continuation_boundary_speed_mps, 4.0);
  EXPECT_DOUBLE_EQ(snapshot.minimum_continuity_transition_speed_mps, 3.8);
  EXPECT_NEAR(snapshot.maximum_continuity_transition_speed_drop_mps, 0.2, 1.0e-12);
}

TEST(RollingRouteTelemetry3DTest, DetectsAStopAtAContinuationTransition) {
  RollingRouteTelemetry3D telemetry;
  telemetry.observe(RollingRouteTelemetryObservation3D{
      .route_generation = 7U,
      .continuity_id = 42U,
      .geometry_revision = 100U,
      .speed_mps = 3.0,
  });
  telemetry.observe(RollingRouteTelemetryObservation3D{
      .route_generation = 8U,
      .continuity_id = 42U,
      .geometry_revision = 101U,
      .speed_mps = 0.0,
  });

  EXPECT_FALSE(telemetry.snapshot().regressionFree());
  EXPECT_EQ(telemetry.snapshot().continuity_transition_zero_speed_ticks, 1U);
  EXPECT_DOUBLE_EQ(telemetry.snapshot().maximum_continuity_transition_speed_drop_mps,
                   3.0);
}

TEST(RollingRouteTelemetry3DTest, DetectsOwnershipGapEpisodesAndDuration) {
  RollingRouteTelemetry3D telemetry;
  const RollingRouteTelemetryObservation3D gap{
      .route_generation = 7U,
      .resident_route_available = true,
      .execution_owner_available = false,
  };
  telemetry.observe(gap);
  telemetry.observe(gap);
  telemetry.observe(RollingRouteTelemetryObservation3D{
      .route_generation = 7U,
      .resident_route_available = true,
      .execution_owner_available = true,
  });
  telemetry.observe(gap);

  const RollingRouteTelemetrySnapshot3D& snapshot = telemetry.snapshot();
  EXPECT_FALSE(snapshot.regressionFree());
  EXPECT_EQ(snapshot.ownership_gap_ticks, 3U);
  EXPECT_EQ(snapshot.ownership_gap_episodes, 2U);
  EXPECT_EQ(snapshot.maximum_consecutive_ownership_gap_ticks, 2U);
}

TEST(RollingRouteTelemetry3DTest, MovingRawInvalidationRequiresAFiniteBrakingTail) {
  RollingRouteTelemetry3D missing_tail;
  missing_tail.observe(RollingRouteTelemetryObservation3D{
      .speed_mps = 4.0,
      .raw_invalidation_active = true,
  });
  EXPECT_EQ(missing_tail.snapshot().moving_raw_invalidation_without_braking_tail_ticks,
            1U);

  RollingRouteTelemetry3D certified_tail;
  certified_tail.observe(RollingRouteTelemetryObservation3D{
      .endpoint_semantics = RouteEndpointSemantics3D::kEmergencyBrakeTail,
      .speed_mps = 4.0,
      .raw_invalidation_active = true,
      .finite_braking_tail_active = true,
  });
  EXPECT_EQ(certified_tail.snapshot().finite_braking_tail_activations, 1U);
  EXPECT_EQ(
      certified_tail.snapshot().moving_raw_invalidation_without_braking_tail_ticks, 0U);
}

TEST(RollingRouteTelemetry3DTest, DetectsReseedAcrossAContinuityPreservingUpdate) {
  RollingRouteTelemetry3D telemetry;
  telemetry.observe(RollingRouteTelemetryObservation3D{
      .route_generation = 7U,
      .continuity_id = 42U,
      .geometry_revision = 100U,
  });
  telemetry.observe(RollingRouteTelemetryObservation3D{
      .route_generation = 8U,
      .continuity_id = 42U,
      .geometry_revision = 101U,
      .nominal_reseeded = true,
  });

  const RollingRouteTelemetrySnapshot3D& snapshot = telemetry.snapshot();
  EXPECT_FALSE(snapshot.regressionFree());
  EXPECT_EQ(snapshot.route_generation_changes, 1U);
  EXPECT_EQ(snapshot.continuity_preserving_generation_changes, 1U);
  EXPECT_EQ(snapshot.geometry_revision_changes, 1U);
  EXPECT_EQ(snapshot.continuity_preserving_reseed_ticks, 1U);
}

} // namespace
} // namespace drone_city_nav
