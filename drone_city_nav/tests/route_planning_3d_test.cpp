#include "drone_city_nav/route_planning_3d.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] RouteProposal3D
proposal(const RouteIntentSource3D source, const bool strategic,
         const bool physical_executable, const bool activation_eligible,
         const bool reaches_segment, const bool reaches_intent,
         const bool reaches_mission, const double objective_cost,
         const double endpoint_displacement_m, const double route_length_m,
         const std::uint64_t fingerprint) {
  return RouteProposal3D{
      .intent = {.id = fingerprint + 100U,
                 .source = source,
                 .strategic_continuation_available = strategic,
                 .valid = true},
      .evidence = {.status = physical_executable
                                 ? SegmentEvidenceStatus3D::kValid
                                 : SegmentEvidenceStatus3D::kRawCollision,
                   .route_length_m = route_length_m,
                   .endpoint_displacement_m = endpoint_displacement_m,
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

  const RouteProposalSelection3D selection = selectRouteProposal3D(proposals);

  ASSERT_TRUE(selection.selected_index.has_value());
  EXPECT_EQ(selection.selected_index.value_or(proposals.size()), 1U);
  EXPECT_EQ(selection.eligible_candidates, 1U);
}

TEST(RoutePlanning3DTest, StrategicBypassBeatsShorterGoalDirectedPrefix) {
  const std::vector<RouteProposal3D> proposals{
      proposal(RouteIntentSource3D::kDirect, false, true, true, false, false, false,
               1.0, 20.0, 20.0, 3U),
      proposal(RouteIntentSource3D::kTopology, true, true, true, false, false, false,
               40.0, 8.0, 35.0, 4U),
  };

  const RouteProposalSelection3D selection = selectRouteProposal3D(proposals);

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

} // namespace
} // namespace drone_city_nav
