#include "drone_city_nav/route_lifecycle_3d.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] MaterializedRouteProposal3D validProposal() {
  return MaterializedRouteProposal3D{
      .planned_world = {.producer_instance_id = 7U,
                        .esdf_fingerprint = 100U,
                        .esdf_source_raw_revision = 10U,
                        .esdf_source_occupied_fingerprint = 1000U,
                        .raw_validated_through_revision = 10U},
      .validated_world = {.producer_instance_id = 7U,
                          .esdf_fingerprint = 110U,
                          .esdf_source_raw_revision = 11U,
                          .esdf_source_occupied_fingerprint = 1100U,
                          .raw_validated_through_revision = 12U},
      .objective = {.goal = {20.0, 2.0, 2.0}, .mission_epoch = 3U, .available = true},
      .intent = {.id = 9U,
                 .planned_on_revision = 100U,
                 .mission_target = {20.0, 2.0, 2.0},
                 .intent_target = {10.0, 2.0, 2.0},
                 .segment_target = {10.0, 2.0, 2.0},
                 .valid = true},
      .evidence = {.planned_on_revision = 100U,
                   .validated_through_revision = 110U,
                   .status = SegmentEvidenceStatus3D::kValid,
                   .physical_executable = true},
      .route_fingerprint = 1234U,
      .route_sample_count = 5U,
      .activation_eligible = true,
  };
}

[[nodiscard]] std::vector<RouteSample3D> straightRoute() {
  return {
      {.position = {1.5, 1.5, 1.5}, .station_m = 0.0},
      {.position = {3.5, 1.5, 1.5}, .station_m = 2.0},
      {.position = {5.5, 1.5, 1.5}, .station_m = 4.0},
      {.position = {7.5, 1.5, 1.5}, .station_m = 6.0},
      {.position = {9.5, 1.5, 1.5}, .station_m = 8.0},
  };
}

[[nodiscard]] ActivatedRouteIdentity3D validActivatedRoute() {
  const std::optional<ActivatedRouteIdentity3D> activated =
      activateRouteProposal3D(validProposal(), 5U);
  if (!activated.has_value()) {
    throw std::logic_error{"valid route proposal was rejected"};
  }
  return activated.value();
}

[[nodiscard]] MaterializedRouteProposal3D strategicMissionProposal() {
  MaterializedRouteProposal3D proposal = validProposal();
  proposal.intent.source = RouteIntentSource3D::kTopology;
  proposal.intent.purpose = RouteIntentPurpose3D::kMissionTransit;
  proposal.intent.target_identity = 42U;
  proposal.intent.strategic_continuation_available = true;
  proposal.intent.intent_reaches_mission_target = true;
  return proposal;
}

[[nodiscard]] ActivatedRouteIdentity3D strategicMissionActivatedRoute() {
  const std::optional<ActivatedRouteIdentity3D> activated =
      activateRouteProposal3D(strategicMissionProposal(), 5U);
  if (!activated.has_value()) {
    throw std::logic_error{"valid strategic mission proposal was rejected"};
  }
  return activated.value();
}

[[nodiscard]] RouteExecutionObservation3D
observation(const StaticRouteObjective& objective,
            const ObservedOccupancyGrid3D* occupancy = nullptr) {
  return RouteExecutionObservation3D{
      .current_objective = objective,
      .previously_validated_through_raw_revision = 12U,
      .position = {6.5, 1.5, 1.5},
      .minimum_station_m = 0.0,
      .maximum_cross_track_m = 2.0,
      .latest_raw_occupancy = occupancy,
      .latest_raw_producer_instance_id = 7U,
      .latest_raw_revision = 13U,
      .footprint = {.radius_m = 0.0,
                    .perimeter_samples = 0U,
                    .radial_rings = 0U,
                    .axial_samples = 1U,
                    .sweep_step_m = 0.25},
  };
}

TEST(RouteLifecycle3DTest, ActivationPreservesPlannedAndValidatedWorldLineage) {
  const MaterializedRouteProposal3D proposal = validProposal();

  const std::optional<ActivatedRouteIdentity3D> activated =
      activateRouteProposal3D(proposal, 5U);

  ASSERT_TRUE(activated.has_value());
  const ActivatedRouteIdentity3D identity =
      activated.value_or(ActivatedRouteIdentity3D{});
  EXPECT_EQ(identity.generation, 5U);
  EXPECT_EQ(identity.proposal.planned_world.esdf_fingerprint, 100U);
  EXPECT_EQ(identity.proposal.validated_world.esdf_fingerprint, 110U);
  EXPECT_EQ(identity.proposal.validated_world.raw_validated_through_revision, 12U);
  EXPECT_EQ(identity.proposal.objective.mission_epoch, 3U);
}

TEST(RouteLifecycle3DTest, ActivationRejectsAProposalFromAnotherProducerLineage) {
  MaterializedRouteProposal3D proposal = validProposal();
  proposal.validated_world.producer_instance_id = 8U;

  EXPECT_FALSE(activateRouteProposal3D(proposal, 5U).has_value());
}

TEST(RouteLifecycle3DTest, ActivationRejectsValidationOlderThanItsEsdfSource) {
  MaterializedRouteProposal3D proposal = validProposal();
  proposal.validated_world.raw_validated_through_revision = 10U;

  EXPECT_FALSE(activateRouteProposal3D(proposal, 5U).has_value());
}

TEST(RouteLifecycle3DTest, ActivationRejectsAnOlderValidatedWorld) {
  MaterializedRouteProposal3D proposal = validProposal();
  proposal.validated_world.esdf_source_raw_revision = 9U;
  proposal.validated_world.raw_validated_through_revision = 12U;

  EXPECT_FALSE(activateRouteProposal3D(proposal, 5U).has_value());
}

TEST(RouteLifecycle3DTest, EquivalentStrategicSegmentRetainsTheActiveRoute) {
  const MaterializedRouteProposal3D active_proposal = strategicMissionProposal();
  const ActivatedRouteIdentity3D active = strategicMissionActivatedRoute();
  MaterializedRouteProposal3D replacement = active_proposal;
  replacement.route_fingerprint += 1U;
  replacement.planned_world.esdf_source_raw_revision += 1U;
  replacement.planned_world.raw_validated_through_revision += 1U;
  replacement.validated_world.esdf_source_raw_revision += 1U;
  replacement.validated_world.raw_validated_through_revision += 1U;

  const RouteProposalReplacementAssessment3D assessment =
      assessRouteProposalReplacement3D(&active, replacement, {});

  EXPECT_FALSE(assessment.replacementAllowed());
  EXPECT_EQ(assessment.status,
            RouteProposalReplacementStatus3D::kRetainEquivalentActiveSegment);
}

TEST(RouteLifecycle3DTest, AdvancedStrategicSegmentCanReplaceTheActiveRoute) {
  const MaterializedRouteProposal3D active_proposal = strategicMissionProposal();
  const ActivatedRouteIdentity3D active = strategicMissionActivatedRoute();
  MaterializedRouteProposal3D replacement = active_proposal;
  replacement.intent.segment_target.x += 0.75;

  const RouteProposalReplacementAssessment3D assessment =
      assessRouteProposalReplacement3D(&active, replacement, {});

  EXPECT_TRUE(assessment.replacementAllowed());
}

TEST(RouteLifecycle3DTest, SafetyReplanCanReplaceAnEquivalentStrategicSegment) {
  const MaterializedRouteProposal3D active_proposal = strategicMissionProposal();
  const ActivatedRouteIdentity3D active = strategicMissionActivatedRoute();

  const RouteProposalReplacementAssessment3D assessment =
      assessRouteProposalReplacement3D(
          &active, active_proposal,
          RouteProposalReplacementObservation3D{.safety_replan_requested = true});

  EXPECT_TRUE(assessment.replacementAllowed());
}

TEST(RouteLifecycle3DTest, ProposalCanPublishOnANewerResidentWorld) {
  const MaterializedRouteProposal3D proposal = validProposal();
  const NavigationWorldCertificate3D resident{
      .producer_instance_id = 7U,
      .esdf_fingerprint = 200U,
      .esdf_source_raw_revision = 20U,
      .esdf_source_occupied_fingerprint = 2000U,
      .raw_validated_through_revision = 20U,
  };

  const RoutePublicationAssessment3D assessment =
      assessRoutePublication3D(proposal, resident);

  EXPECT_TRUE(assessment.compatible());
  EXPECT_EQ(assessment.status, RoutePublicationStatus3D::kCompatible);
}

TEST(RouteLifecycle3DTest, ProposalCannotPublishAcrossProducerLineages) {
  const MaterializedRouteProposal3D proposal = validProposal();
  const NavigationWorldCertificate3D resident{
      .producer_instance_id = 8U,
      .esdf_fingerprint = 200U,
      .esdf_source_raw_revision = 20U,
      .esdf_source_occupied_fingerprint = 2000U,
      .raw_validated_through_revision = 20U,
  };

  const RoutePublicationAssessment3D assessment =
      assessRoutePublication3D(proposal, resident);

  EXPECT_FALSE(assessment.compatible());
  EXPECT_EQ(assessment.status, RoutePublicationStatus3D::kWorldLineageMismatch);
}

TEST(RouteLifecycle3DTest, ProposalCannotPublishOnAWorldOlderThanItsPlan) {
  const MaterializedRouteProposal3D proposal = validProposal();
  const NavigationWorldCertificate3D resident{
      .producer_instance_id = 7U,
      .esdf_fingerprint = 90U,
      .esdf_source_raw_revision = 9U,
      .esdf_source_occupied_fingerprint = 900U,
      .raw_validated_through_revision = 9U,
  };

  const RoutePublicationAssessment3D assessment =
      assessRoutePublication3D(proposal, resident);

  EXPECT_FALSE(assessment.compatible());
  EXPECT_EQ(assessment.status, RoutePublicationStatus3D::kResidentWorldPredatesPlan);
}

TEST(RouteLifecycle3DTest, PublicationStatusesHaveStableDiagnosticNames) {
  EXPECT_EQ(routePublicationStatus3DName(RoutePublicationStatus3D::kNotAssessed),
            "not_assessed");
  EXPECT_EQ(routePublicationStatus3DName(RoutePublicationStatus3D::kCompatible),
            "compatible");
  EXPECT_EQ(
      routePublicationStatus3DName(RoutePublicationStatus3D::kInvalidProposalWorld),
      "invalid_proposal_world");
  EXPECT_EQ(
      routePublicationStatus3DName(RoutePublicationStatus3D::kInvalidResidentWorld),
      "invalid_resident_world");
  EXPECT_EQ(
      routePublicationStatus3DName(RoutePublicationStatus3D::kWorldLineageMismatch),
      "world_lineage_mismatch");
  EXPECT_EQ(routePublicationStatus3DName(
                RoutePublicationStatus3D::kResidentWorldPredatesPlan),
            "resident_world_predates_plan");
}

TEST(RouteLifecycle3DTest, PassedPrefixCollisionDoesNotInvalidateRemainingSuffix) {
  const std::vector<RouteSample3D> route = straightRoute();
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 12, 4, 4}};
  ASSERT_TRUE(occupancy.setState({2, 1, 1}, ObservedVoxelState::kOccupied));
  const ActivatedRouteIdentity3D activated = validActivatedRoute();

  const RouteExecutionAssessment3D assessment = assessRouteExecution3D(
      &activated, route, observation(activated.proposal.objective, &occupancy));

  EXPECT_TRUE(assessment.usable());
  EXPECT_GE(assessment.projection.station_m, 4.0);
  EXPECT_EQ(assessment.validated_through_raw_revision, 13U);
  EXPECT_GE(assessment.raw_validation.first_validated_route_segment, 2U);
}

TEST(RouteLifecycle3DTest, NewlyObservedCollisionInRemainingSuffixRejectsExecution) {
  const std::vector<RouteSample3D> route = straightRoute();
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 12, 4, 4}};
  ASSERT_TRUE(occupancy.setState({8, 1, 1}, ObservedVoxelState::kOccupied));
  const ActivatedRouteIdentity3D activated = validActivatedRoute();

  const RouteExecutionAssessment3D assessment = assessRouteExecution3D(
      &activated, route, observation(activated.proposal.objective, &occupancy));

  EXPECT_EQ(assessment.status, RouteExecutionStatus3D::kRawCollision);
  EXPECT_EQ(assessment.raw_validation.failure_route_segment, 3U);
}

TEST(RouteLifecycle3DTest, CollisionOnConnectorToRemainingSuffixRejectsExecution) {
  const std::vector<RouteSample3D> route = straightRoute();
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 12, 5, 4}};
  ASSERT_TRUE(occupancy.setState({6, 2, 1}, ObservedVoxelState::kOccupied));
  const ActivatedRouteIdentity3D activated = validActivatedRoute();
  RouteExecutionObservation3D displaced =
      observation(activated.proposal.objective, &occupancy);
  displaced.position = {6.5, 3.5, 1.5};
  displaced.maximum_cross_track_m = 3.0;

  const RouteExecutionAssessment3D assessment =
      assessRouteExecution3D(&activated, route, displaced);

  EXPECT_EQ(assessment.status, RouteExecutionStatus3D::kRawCollision);
  EXPECT_TRUE(assessment.raw_validation.connector_validated);
}

TEST(RouteLifecycle3DTest, ObjectiveIdentityIsCheckedAtExecutionBoundary) {
  const std::vector<RouteSample3D> route = straightRoute();
  const ActivatedRouteIdentity3D activated = validActivatedRoute();
  StaticRouteObjective changed = activated.proposal.objective;
  changed.mission_epoch += 1U;

  const RouteExecutionAssessment3D assessment =
      assessRouteExecution3D(&activated, route, observation(changed));

  EXPECT_EQ(assessment.status, RouteExecutionStatus3D::kObjectiveMismatch);
  EXPECT_TRUE(assessment.replacementRequired());
}

TEST(RouteLifecycle3DTest, AStructurallyInvalidActiveRouteRequiresReplacement) {
  const ActivatedRouteIdentity3D activated = validActivatedRoute();
  const std::vector<RouteSample3D> route{
      {.position = {1.5, 1.5, 1.5}, .station_m = 0.0}};

  const RouteExecutionAssessment3D assessment = assessRouteExecution3D(
      &activated, route, observation(activated.proposal.objective));

  EXPECT_EQ(assessment.status, RouteExecutionStatus3D::kInvalidRoute);
  EXPECT_TRUE(assessment.replacementRequired());
}

TEST(RouteLifecycle3DTest, MissingRouteWaitsForInitialPlanningWithoutReplacement) {
  const RouteExecutionAssessment3D assessment =
      assessRouteExecution3D(nullptr, {}, RouteExecutionObservation3D{});

  EXPECT_EQ(assessment.status, RouteExecutionStatus3D::kNoActiveRoute);
  EXPECT_FALSE(assessment.replacementRequired());
}

TEST(RouteLifecycle3DTest, ExecutionRejectsRawWorldFromAnotherProducerLineage) {
  const std::vector<RouteSample3D> route = straightRoute();
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 12, 4, 4}};
  const ActivatedRouteIdentity3D activated = validActivatedRoute();
  RouteExecutionObservation3D mismatched =
      observation(activated.proposal.objective, &occupancy);
  mismatched.latest_raw_producer_instance_id = 8U;

  const RouteExecutionAssessment3D assessment =
      assessRouteExecution3D(&activated, route, mismatched);

  EXPECT_EQ(assessment.status, RouteExecutionStatus3D::kWorldLineageMismatch);
}

TEST(RouteLifecycle3DTest, OlderRawSnapshotCannotInvalidateNewerCertification) {
  const std::vector<RouteSample3D> route = straightRoute();
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 12, 4, 4}};
  ASSERT_TRUE(occupancy.setState({8, 1, 1}, ObservedVoxelState::kOccupied));
  const ActivatedRouteIdentity3D activated = validActivatedRoute();
  RouteExecutionObservation3D stale =
      observation(activated.proposal.objective, &occupancy);
  stale.previously_validated_through_raw_revision = 14U;
  stale.latest_raw_revision = 13U;

  const RouteExecutionAssessment3D assessment =
      assessRouteExecution3D(&activated, route, stale);

  EXPECT_TRUE(assessment.usable());
  EXPECT_EQ(assessment.validated_through_raw_revision, 14U);
  EXPECT_FALSE(assessment.raw_validation.connector_validated);
}

TEST(RouteLifecycle3DTest, SegmentCompletionUsesTheConfiguredCaptureRadius) {
  const std::vector<RouteSample3D> route = straightRoute();

  const RouteSegmentCompletionAssessment3D assessment = assessRouteSegmentCompletion3D(
      route, Point3{8.7, 1.5, 1.5},
      RouteSegmentCompletionConfig3D{.capture_radius_m = 2.0});

  ASSERT_TRUE(assessment.projection.valid);
  EXPECT_NEAR(assessment.projection.remaining_m, 0.8, 1.0e-9);
  EXPECT_NEAR(assessment.endpoint_distance_m, 0.8, 1.0e-9);
  EXPECT_TRUE(assessment.captured);
}

TEST(RouteLifecycle3DTest, SegmentCompletionRejectsPositionOutsideCaptureRadius) {
  const std::vector<RouteSample3D> route = straightRoute();

  const RouteSegmentCompletionAssessment3D assessment = assessRouteSegmentCompletion3D(
      route, Point3{7.0, 1.5, 1.5},
      RouteSegmentCompletionConfig3D{.capture_radius_m = 2.0});

  ASSERT_TRUE(assessment.projection.valid);
  EXPECT_NEAR(assessment.endpoint_distance_m, 2.5, 1.0e-9);
  EXPECT_FALSE(assessment.captured);
}

TEST(RouteLifecycle3DTest, SegmentCompletionRejectsInvalidContract) {
  const std::vector<RouteSample3D> route = straightRoute();

  const RouteSegmentCompletionAssessment3D assessment = assessRouteSegmentCompletion3D(
      route, Point3{9.5, 1.5, 1.5},
      RouteSegmentCompletionConfig3D{.capture_radius_m = -1.0});

  EXPECT_FALSE(assessment.projection.valid);
  EXPECT_FALSE(assessment.captured);
}

} // namespace
} // namespace drone_city_nav
