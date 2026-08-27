#include "drone_city_nav/route_planning_3d.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace drone_city_nav {
namespace {

TEST(RoutePlanning3D, NetCoordinateProgressDoesNotRewardUnneededAltitude) {
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
  EXPECT_DOUBLE_EQ(
      routeNetCoordinateProgress3D(start, Point3{0.0, 0.0, 20.0}, horizontal_goal),
      0.0);
  EXPECT_DOUBLE_EQ(routeNetCoordinateProgress3D(start, Point3{0.0, 0.0, 15.0},
                                                Point3{0.0, 0.0, 20.0}),
                   10.0);
}

[[nodiscard]] RouteProposal3D
proposal(const RouteIntentSource3D source, const bool strategic,
         const bool physical_executable, const bool activation_eligible,
         const bool reaches_segment, const bool reaches_intent,
         const bool reaches_mission, const double objective_cost,
         const double endpoint_displacement_m, const double route_length_m,
         const std::uint64_t fingerprint, const double mission_progress_m = 0.0,
         const RouteIntentPurpose3D purpose = RouteIntentPurpose3D::kMissionTransit,
         const bool intent_reaches_mission_target = false) {
  return RouteProposal3D{
      .intent = {.id = fingerprint + 100U,
                 .strategic_plan_id =
                     source == RouteIntentSource3D::kTopology && strategic ? 1U : 0U,
                 .source = source,
                 .purpose = purpose,
                 .strategic_continuation_available = strategic,
                 .strategic_mission_continuation = intent_reaches_mission_target,
                 .intent_reaches_mission_target = intent_reaches_mission_target,
                 .valid = true},
      .evidence = {.status = physical_executable
                                 ? SegmentEvidenceStatus3D::kValid
                                 : SegmentEvidenceStatus3D::kRawCollision,
                   .route_length_m = route_length_m,
                   .endpoint_displacement_m = endpoint_displacement_m,
                   .mission_progress_m = mission_progress_m,
                   .objective_cost = objective_cost,
                   .physical_executable = physical_executable,
                   .reaches_segment_target = reaches_segment,
                   .reaches_intent_target = reaches_intent,
                   .reaches_mission_target = reaches_mission,
                   .raw_collision = !physical_executable},
      .route_fingerprint = fingerprint,
      .activation_eligible = activation_eligible,
  };
}

constexpr RouteProposalSelection3DConfig kSelectionConfig{
    .heuristic_precedence_enabled = true};

[[nodiscard]] SegmentEvidenceWorld3D world(const mppi::EsdfGrid& grid,
                                           const std::vector<float>& esdf) {
  return SegmentEvidenceWorld3D{
      .grid = &grid,
      .esdf_m = esdf,
      .footprint = {.radius_m = 0.0, .sweep_step_m = 0.25},
      .flight_envelope = {.minimum_target_z_m = -10.0, .maximum_target_z_m = 10.0},
      .validated_through_revision = 12U,
  };
}

TEST(RoutePlanning3DTest, RawRejectedDirectCandidateLosesToTopologyInSameDecision) {
  const std::vector<RouteProposal3D> proposals{
      proposal(RouteIntentSource3D::kDirect, false, false, false, false, false, false,
               1.0, 20.0, 20.0, 1U),
      proposal(RouteIntentSource3D::kTopology, true, true, true, true, false, false,
               10.0, 8.0, 30.0, 2U),
  };

  const RouteProposalSelection3D selection =
      selectRouteProposal3D(proposals, kSelectionConfig);

  ASSERT_TRUE(selection.selected_index.has_value());
  EXPECT_EQ(selection.selected_index.value_or(proposals.size()), 1U);
  EXPECT_EQ(selection.eligible_candidates, 1U);
}

TEST(RoutePlanning3DTest,
     DirectCandidateRejectedByFinalHandoffFallsBackToPreparedTopology) {
  const std::vector<RouteProposal3D> proposals{
      proposal(RouteIntentSource3D::kDirect, false, true, false, true, true, true, 1.0,
               20.0, 20.0, 15U, 20.0),
      proposal(RouteIntentSource3D::kTopology, true, true, true, true, false, false,
               10.0, 8.0, 30.0, 16U),
  };

  const RouteProposalSelection3D selection =
      selectRouteProposal3D(proposals, kSelectionConfig);

  ASSERT_TRUE(selection.selected_index.has_value());
  EXPECT_EQ(selection.selected_index.value_or(proposals.size()), 1U);
  EXPECT_EQ(selection.eligible_candidates, 1U);
  EXPECT_EQ(selection.reason, RouteProposalSelectionReason3D::kOnlyEligibleCandidate);
}

TEST(RoutePlanning3DTest, NoPreparedCandidateLeavesSelectionEmpty) {
  const std::vector<RouteProposal3D> proposals{
      proposal(RouteIntentSource3D::kDirect, false, true, false, true, true, true, 1.0,
               20.0, 20.0, 17U, 20.0),
      proposal(RouteIntentSource3D::kTopology, true, false, false, false, false, false,
               10.0, 8.0, 30.0, 18U),
  };

  const RouteProposalSelection3D selection =
      selectRouteProposal3D(proposals, kSelectionConfig);

  EXPECT_FALSE(selection.selected_index.has_value());
  EXPECT_EQ(selection.eligible_candidates, 0U);
  EXPECT_EQ(selection.reason, RouteProposalSelectionReason3D::kNoEligibleCandidate);
}

TEST(RoutePlanning3DTest,
     PermissiveSelectionPrefersNetCoordinateProgressOverLengthAndPrecedence) {
  const std::vector<RouteProposal3D> proposals{
      proposal(RouteIntentSource3D::kTopology, true, true, true, true, true, false, 1.0,
               20.0, 80.0, 21U, 4.0),
      proposal(RouteIntentSource3D::kDirect, false, true, true, false, false, false,
               20.0, 20.0, 22.0, 22U, 12.0),
  };

  const RouteProposalSelection3D selection =
      selectRouteProposal3D(proposals, RouteProposalSelection3DConfig{});

  ASSERT_EQ(selection.selected_index, std::optional<std::size_t>{1U});
  EXPECT_EQ(selection.reason, RouteProposalSelectionReason3D::kNetCoordinateProgress);
}

TEST(RoutePlanning3DTest, PermissiveSelectionDoesNotRewardPureReverseDisplacement) {
  const std::vector<RouteProposal3D> proposals{
      proposal(RouteIntentSource3D::kDirect, false, true, true, false, false, false,
               1.0, 2.0, 2.0, 23U, -1.0),
      proposal(RouteIntentSource3D::kTopology, true, true, true, false, false, false,
               2.0, 6.0, 6.0, 24U, -6.0, RouteIntentPurpose3D::kObservationFrontier),
  };

  const RouteProposalSelection3D selection =
      selectRouteProposal3D(proposals, RouteProposalSelection3DConfig{});

  ASSERT_EQ(selection.selected_index, std::optional<std::size_t>{0U});
  EXPECT_EQ(selection.reason, RouteProposalSelectionReason3D::kNetCoordinateProgress);
}

TEST(RoutePlanning3DTest,
     PermissiveSelectionLetsAProductiveLateralDetourBeatAShortPrefix) {
  const std::vector<RouteProposal3D> proposals{
      proposal(RouteIntentSource3D::kDirect, false, true, true, false, false, false,
               1.0, 2.0, 2.0, 25U, 1.0),
      proposal(RouteIntentSource3D::kTopology, true, true, true, false, false, false,
               2.0, 8.0, 8.0, 26U, -1.0, RouteIntentPurpose3D::kObservationFrontier),
  };

  const RouteProposalSelection3D selection =
      selectRouteProposal3D(proposals, RouteProposalSelection3DConfig{});

  ASSERT_EQ(selection.selected_index, std::optional<std::size_t>{1U});
  EXPECT_EQ(selection.reason, RouteProposalSelectionReason3D::kNetCoordinateProgress);
}

TEST(RoutePlanning3DTest, PermissiveSelectionDoesNotRewardLoopLength) {
  const std::vector<RouteProposal3D> proposals{
      proposal(RouteIntentSource3D::kTopology, true, true, true, false, false, false,
               1.0, 0.0, 80.0, 27U, 0.0, RouteIntentPurpose3D::kObservationFrontier),
      proposal(RouteIntentSource3D::kDirect, false, true, true, false, false, false,
               2.0, 2.0, 2.0, 28U, 1.0),
  };

  const RouteProposalSelection3D selection =
      selectRouteProposal3D(proposals, RouteProposalSelection3DConfig{});

  ASSERT_EQ(selection.selected_index, std::optional<std::size_t>{1U});
  EXPECT_EQ(selection.reason, RouteProposalSelectionReason3D::kNetCoordinateProgress);
}

TEST(RoutePlanning3DTest, StrategicBypassBeatsShorterGoalDirectedPrefix) {
  const std::vector<RouteProposal3D> proposals{
      proposal(RouteIntentSource3D::kDirect, false, true, true, false, false, false,
               1.0, 20.0, 20.0, 3U),
      proposal(RouteIntentSource3D::kTopology, true, true, true, false, false, false,
               40.0, 8.0, 35.0, 4U),
  };

  const RouteProposalSelection3D selection =
      selectRouteProposal3D(proposals, kSelectionConfig);

  ASSERT_TRUE(selection.selected_index.has_value());
  EXPECT_EQ(selection.selected_index.value_or(proposals.size()), 1U);
  EXPECT_EQ(selection.reason, RouteProposalSelectionReason3D::kStrategicContinuation);
}

TEST(RoutePlanning3DTest,
     StrategicObservationFrontierBeatsAnUnfinishedDirectMissionPrefix) {
  const std::vector<RouteProposal3D> proposals{
      proposal(RouteIntentSource3D::kDirect, false, true, true, false, false, false,
               10.0, 20.0, 24.0, 5U),
      proposal(RouteIntentSource3D::kTopology, true, true, true, true, false, false,
               1.0, 10.0, 12.0, 6U, 0.0, RouteIntentPurpose3D::kObservationFrontier),
  };

  const RouteProposalSelection3D selection =
      selectRouteProposal3D(proposals, kSelectionConfig);

  ASSERT_TRUE(selection.selected_index.has_value());
  EXPECT_EQ(selection.selected_index.value_or(proposals.size()), 1U);
  EXPECT_EQ(selection.reason, RouteProposalSelectionReason3D::kStrategicContinuation);
}

TEST(RoutePlanning3DTest,
     NonStrategicExplorationCannotReplaceExecutableMissionTransit) {
  const std::vector<RouteProposal3D> proposals{
      proposal(RouteIntentSource3D::kDirect, false, true, true, false, false, false,
               10.0, 20.0, 24.0, 7U),
      proposal(RouteIntentSource3D::kTopology, false, true, true, true, false, false,
               1.0, 10.0, 12.0, 8U, 0.0, RouteIntentPurpose3D::kObservationFrontier),
  };

  const RouteProposalSelection3D selection =
      selectRouteProposal3D(proposals, kSelectionConfig);

  ASSERT_TRUE(selection.selected_index.has_value());
  EXPECT_EQ(selection.selected_index.value_or(proposals.size()), 0U);
  EXPECT_EQ(selection.reason, RouteProposalSelectionReason3D::kRouteQuality);
}

TEST(RoutePlanning3DTest, ProductiveDirectTransitBeatsAValidatedStrategicFrontier) {
  const std::vector<RouteProposal3D> proposals{
      proposal(RouteIntentSource3D::kDirect, false, true, true, false, false, false,
               6.0, 20.0, 24.0, 9U, 8.0),
      proposal(RouteIntentSource3D::kTopology, true, true, true, true, false, false,
               1.0, 1.0, 1.0, 10U, -0.5, RouteIntentPurpose3D::kObservationFrontier),
  };

  const RouteProposalSelection3D selection =
      selectRouteProposal3D(proposals, kSelectionConfig);

  ASSERT_TRUE(selection.selected_index.has_value());
  EXPECT_EQ(selection.selected_index.value_or(proposals.size()), 0U);
  EXPECT_EQ(selection.reason, RouteProposalSelectionReason3D::kProductiveDirectTransit);
}

TEST(RoutePlanning3DTest, ProductiveDirectPrefixDefersStrategicMissionContinuation) {
  const std::vector<RouteProposal3D> proposals{
      proposal(RouteIntentSource3D::kDirect, false, true, true, false, false, false,
               1.0, 9.0, 11.0, 13U, 3.0),
      proposal(RouteIntentSource3D::kTopology, true, true, true, true, false, false,
               10.0, 3.0, 4.0, 14U, -2.0, RouteIntentPurpose3D::kMissionTransit, true),
  };

  const RouteProposalSelection3D selection =
      selectRouteProposal3D(proposals, kSelectionConfig);

  ASSERT_TRUE(selection.selected_index.has_value());
  EXPECT_EQ(selection.selected_index.value_or(proposals.size()), 0U);
  EXPECT_EQ(selection.reason, RouteProposalSelectionReason3D::kProductiveDirectTransit);
}

TEST(RoutePlanning3DTest, ProductiveDirectPrefixAlsoDefersPartialMissionContinuation) {
  RouteProposal3D partial =
      proposal(RouteIntentSource3D::kTopology, true, true, true, true, false, false,
               10.0, 3.0, 4.0, 19U, -2.0, RouteIntentPurpose3D::kMissionTransit, false);
  partial.intent.strategic_plan_id = 7U;
  partial.intent.strategic_mission_continuation = true;
  const std::vector<RouteProposal3D> proposals{
      proposal(RouteIntentSource3D::kDirect, false, true, true, false, false, false,
               1.0, 9.0, 11.0, 20U, 3.0),
      partial,
  };

  const RouteProposalSelection3D selection =
      selectRouteProposal3D(proposals, kSelectionConfig);

  ASSERT_TRUE(selection.selected_index.has_value());
  EXPECT_EQ(selection.selected_index.value_or(proposals.size()), 0U);
  EXPECT_TRUE(isStrategicMissionContinuation3D(partial));
  EXPECT_FALSE(partial.intent.intent_reaches_mission_target);
  EXPECT_FALSE(partial.evidence.reaches_mission_target);
  EXPECT_EQ(selection.reason, RouteProposalSelectionReason3D::kProductiveDirectTransit);
}

TEST(RoutePlanning3DTest, InefficientDirectPrefixYieldsToAValidatedStrategicFrontier) {
  const std::vector<RouteProposal3D> proposals{
      proposal(RouteIntentSource3D::kDirect, false, true, true, false, false, false,
               6.0, 7.0, 22.0, 11U, 0.75),
      proposal(RouteIntentSource3D::kTopology, true, true, true, true, false, false,
               1.0, 1.0, 1.0, 12U, -0.5, RouteIntentPurpose3D::kObservationFrontier),
  };

  const RouteProposalSelection3D selection =
      selectRouteProposal3D(proposals, kSelectionConfig);

  ASSERT_TRUE(selection.selected_index.has_value());
  EXPECT_EQ(selection.selected_index.value_or(proposals.size()), 1U);
  EXPECT_EQ(selection.reason, RouteProposalSelectionReason3D::kStrategicContinuation);
}

TEST(RoutePlanning3DTest, ReachingLocalBendDoesNotCompleteGlobalIntent) {
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
                       .intent_target = {10.0, 0.0, 0.0},
                       .segment_target = {2.0, 0.0, 0.0},
                       .source = RouteIntentSource3D::kTopology,
                       .purpose = RouteIntentPurpose3D::kObservationFrontier,
                       .strategic_continuation_available = true,
                       .segment_reaches_intent_target = false,
                       .valid = true};
  const std::vector<RouteSample3D> route{{.position = {0.5, 0.5, 0.5}},
                                         {.position = {2.0, 0.5, 0.5}}};

  const SegmentEvidence3D evidence = evaluateSegmentEvidence3D(
      intent, route, route.front().position, true, true, false, 1.0, world(grid, esdf));

  EXPECT_TRUE(evidence.physical_executable);
  EXPECT_TRUE(evidence.reaches_segment_target);
  EXPECT_FALSE(evidence.reaches_intent_target);
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
                       .intent_target = {3.0, 0.5, 0.5},
                       .segment_target = {3.0, 0.5, 0.5},
                       .valid = true};
  const std::vector<RouteSample3D> route{{.position = {0.5, 0.5, 0.5}},
                                         {.position = {2.5, 0.5, 0.5}}};

  const SegmentEvidence3D evidence = evaluateSegmentEvidence3D(
      intent, route, route.front().position, true, true, false, 1.0, world(grid, esdf));

  EXPECT_TRUE(evidence.physical_executable);
  EXPECT_TRUE(evidence.unknown_exposure);
  EXPECT_FALSE(evidence.known_clearance_observed);
  EXPECT_TRUE(std::isinf(evidence.minimum_known_clearance_m));
}

TEST(RoutePlanning3DTest, CollisionWinsWhenAnotherSampleIsUnknown) {
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
                       .intent_target = {3.0, 0.5, 0.5},
                       .segment_target = {3.0, 0.5, 0.5},
                       .valid = true};
  const std::vector<RouteSample3D> route{{.position = {0.5, 0.5, 0.5}},
                                         {.position = {2.5, 0.5, 0.5}}};

  const SegmentEvidence3D evidence = evaluateSegmentEvidence3D(
      intent, route, route.front().position, true, true, false, 1.0, world(grid, esdf));

  EXPECT_FALSE(evidence.physical_executable);
  EXPECT_TRUE(evidence.raw_collision);
  EXPECT_TRUE(evidence.unknown_exposure);
  EXPECT_EQ(evidence.status, SegmentEvidenceStatus3D::kRawCollision);
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
                             .intent_target = {3.0, 0.5, 0.5},
                             .segment_target = {3.0, 0.5, 0.5},
                             .valid = true};
  const std::vector<RouteSample3D> route{{.position = {0.5, 0.5, 0.5}},
                                         {.position = {2.5, 0.5, 0.5}}};
  SegmentEvidenceWorld3D evidence_world = world(grid, esdf);
  evidence_world.latest_observed_occupancy = &latest_raw;

  const SegmentEvidence3D evidence = evaluateSegmentEvidence3D(
      intent, route, route.front().position, true, true, false, 1.0, evidence_world);

  EXPECT_FALSE(evidence.physical_executable);
  EXPECT_TRUE(evidence.raw_collision);
  EXPECT_TRUE(evidence.unknown_exposure);
  EXPECT_EQ(evidence.status, SegmentEvidenceStatus3D::kRawCollision);
}

TEST(RoutePlanning3DTest, InvalidDerivedEsdfIsOptionalWhenFreshRawWorldIsSafe) {
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
                             .intent_target = {3.0, 0.5, 0.5},
                             .segment_target = {3.0, 0.5, 0.5},
                             .valid = true};
  const std::vector<RouteSample3D> route{{.position = {0.5, 0.5, 0.5}},
                                         {.position = {2.5, 0.5, 0.5}}};
  SegmentEvidenceWorld3D permissive_world = world(grid, esdf);
  permissive_world.latest_observed_occupancy = &latest_raw;

  const SegmentEvidence3D permissive = evaluateSegmentEvidence3D(
      intent, route, route.front().position, true, true, false, 1.0, permissive_world);
  SegmentEvidenceWorld3D strict_world = permissive_world;
  strict_world.reject_invalid_esdf = true;
  const SegmentEvidence3D strict = evaluateSegmentEvidence3D(
      intent, route, route.front().position, true, true, false, 1.0, strict_world);

  EXPECT_TRUE(permissive.physical_executable);
  EXPECT_TRUE(permissive.invalid_esdf_exposure);
  EXPECT_EQ(permissive.status, SegmentEvidenceStatus3D::kValid);
  EXPECT_FALSE(strict.physical_executable);
  EXPECT_EQ(strict.status, SegmentEvidenceStatus3D::kInvalidEsdf);
}

} // namespace
} // namespace drone_city_nav
