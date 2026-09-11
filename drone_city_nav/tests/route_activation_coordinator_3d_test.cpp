#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "execution_route_snapshot_3d_plan_test_support.hpp"
#include "execution_supervisor_horizon_3d_test_support.hpp"
#include "production_mppi_route_world.hpp"
#include "route_activation_coordinator_3d.hpp"

namespace drone_city_nav {
namespace {

struct ActivationFixture3D {
  std::shared_ptr<const WorldSnapshot3D> world;
  std::shared_ptr<const PlannerSearchTransaction3D> transaction;
  std::shared_ptr<const ProductionNavigationObjective> objective;
  ProductionRouteMaterialization3D materialization{};
  ProductionRouteActivationSnapshot3D snapshot{};
};

[[nodiscard]] std::shared_ptr<const WorldSnapshot3D>
staticWorld(const std::uint64_t generation = 3U) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 16, 16, 12};
  auto occupancy = std::make_shared<OccupancyGrid3D>(bounds, 71U + generation);
  const std::uint64_t revision = occupancy->fingerprint();
  const std::uint64_t occupied_fingerprint = occupancy->contentFingerprint();
  auto world = std::make_shared<WorldSnapshot3D>();
  world->revision = revision;
  world->source_occupied_fingerprint = revision;
  world->raw_occupied_fingerprint = occupied_fingerprint;
  world->grid = EsdfGrid3D{
      .width = bounds.width_cells,
      .height = bounds.height_cells,
      .resolution_m = static_cast<float>(bounds.resolution_m),
      .origin_x_m = static_cast<float>(bounds.origin_x),
      .origin_y_m = static_cast<float>(bounds.origin_y),
      .depth = bounds.depth_cells,
      .origin_z_m = static_cast<float>(bounds.origin_z),
      .outside_is_unknown = true,
  };
  const std::size_t voxel_count = static_cast<std::size_t>(bounds.width_cells) *
                                  static_cast<std::size_t>(bounds.height_cells) *
                                  static_cast<std::size_t>(bounds.depth_cells);
  world->distances_m = std::make_shared<const std::vector<float>>(voxel_count, 20.0F);
  world->static_occupancy = std::move(occupancy);
  world->local_world_generation = LocalWorldGeneration{
      .generation = generation,
      .raw_map =
          RawMapVersion{
              .producer_instance_id = 0U,
              .base_snapshot_revision = revision,
              .revision = revision,
          },
      .pose_revision = 5U,
      .esdf_revision = revision,
      .gpu_esdf_revision = revision,
  };
  return world;
}

[[nodiscard]] RouteActivationCoordinatorConfig3D
coordinatorConfig(const double cruise_speed_mps = 3.0) {
  RouteActivationCoordinatorConfig3D config;
  MotionDynamicsConfig3D dynamics;
  MotionAltitudeEnvelopeConfig3D altitude_envelope;
  config.route_tracking.maximum_cross_track_m = 2.0;
  config.route_extension.minimum_remaining_m = 1.0;
  config.route_extension.required_certified_overlap_m = 1.0;
  config.cruise_speed_mps = cruise_speed_mps;
  config.maximum_control_feedback_age_ms = 1000.0;
  // The shared finite-execution fixture uses a symmetric acceleration profile.
  // Zero drag keeps that profile terminally stationary when a pending route is
  // promoted to active in the successor-handoff test.
  dynamics.linear_drag_1ps = 0.0F;
  config.trajectory_compiler.trajectory.unconstrained_speed_mps =
      config.cruise_speed_mps;
  config.trajectory_compiler.trajectory.physical_footprint = config.physical_footprint;
  config.trajectory_compiler.passage_volume.flight_envelope = config.flight_envelope;
  config.trajectory_compiler.passage_volume.footprint = config.physical_footprint;
  config.route_risk = RouteRiskPolicy3D{
      .critical_distance_m = 1.0,
      .preferred_distance_m = 6.0,
  };
  config.dynamic_handoff_validator = [](const DynamicHandoffRequest3D& request) {
    return DynamicHandoffResult3D{
        .status = request.candidate_trajectory != nullptr &&
                          request.derived_distances_m != nullptr
                      ? DynamicHandoffStatus3D::kAccepted
                      : DynamicHandoffStatus3D::kInvalidInput,
    };
  };
  config.validation_policy = VersionedExecutionValidationPolicy3D::capture(
      config.flight_envelope, dynamics, altitude_envelope, config.physical_footprint,
      1000.0, 1000.0, 1000.0, true, false, true);
  return config;
}

[[nodiscard]] ActivationFixture3D
activationFixture(const ExecutionSupervisor3D& supervisor,
                  const bool continuous_tracking = false) {
  ActivationFixture3D fixture;
  fixture.world = staticWorld();
  const Point3 start{3.5, 3.5, 5.0};
  const Point3 goal{8.5, 3.5, 5.0};
  const StaticRouteObjective route_objective{
      .goal = goal,
      .mission_epoch = 7U,
      .sample_sequence = 1U,
      .assignment_generation = continuous_tracking ? 2U : 0U,
      .target_detection_id = continuous_tracking ? 3U : 0U,
      .target_track_id = continuous_tracking ? 4U : 0U,
      .continuous_tracking = continuous_tracking,
      .available = true,
  };
  fixture.transaction = makePlannerSearchTransaction3D(
      fixture.world, captureResidentPlannerWorld3D(*fixture.world), route_objective,
      StaticRouteSearchRequestIdentity{
          .kind = StaticRouteSearchRequestKind::kInitial,
          .base_route_generation = 0U,
      });
  fixture.objective = std::make_shared<const ProductionNavigationObjective>(
      ProductionNavigationObjective{
          .goal = goal,
          .tracking = std::nullopt,
          .mission_epoch = route_objective.mission_epoch,
          .sample_sequence = route_objective.sample_sequence,
          .assignment_generation = route_objective.assignment_generation,
          .target_detection_id = route_objective.target_detection_id,
          .target_track_id = route_objective.target_track_id,
          .continuous_tracking = continuous_tracking,
      });
  auto route = std::make_shared<const std::vector<RouteSample3D>>(
      sampleRoute3D(std::vector<Point3>{start, goal}, 0.5, 3.0));
  const RouteExecutionManagerSnapshot3D execution = supervisor.snapshot();
  const std::shared_ptr<const ExecutionPlan3D> execution_plan = execution.plan();
  const std::uint64_t candidate_generation =
      execution_plan != nullptr ? execution_plan->routeGenerationHighWater() + 1U : 0U;
  fixture.materialization.route = MaterializedRoute3D{
      .world = fixture.world,
      .objective = route_objective,
      .intent =
          RouteIntent3D{
              .id = makeRouteIntentId3D(goal, route_objective.mission_epoch),
              .planned_on_revision = fixture.world->revision,
              .mission_target = goal,
              .valid = true,
          },
      .segment_evidence =
          SegmentEvidence3D{
              .planned_on_revision = fixture.world->revision,
              .validated_through_revision = fixture.world->revision,
              .status = SegmentEvidenceStatus3D::kValid,
              .materialized = true,
              .planner_executable = true,
              .physical_executable = true,
              .reaches_mission_target = true,
          },
      .route = route,
      .constrained_spans = std::make_shared<const std::vector<ConstrainedRouteSpan>>(),
      .passage_volumes = std::make_shared<const std::vector<PassageVolume>>(),
      .cooperative_passage_assignments =
          std::make_shared<const std::vector<CooperativePassageAssignment>>(),
      .selected_passage_traversal_ids =
          std::make_shared<const std::vector<PassageTraversalId>>(),
      .candidate_generation = candidate_generation,
      .fingerprint = routeFingerprint(*route),
      .reaches_mission_goal = true,
      .planner_executable = true,
  };
  fixture.materialization.validation = StaticRouteCandidateValidation{
      .status = StaticRouteCandidateStatus::kAccepted,
      .accepted = true,
  };
  fixture.snapshot = ProductionRouteActivationSnapshot3D{
      .resident_world = fixture.world,
      .execution_authority = execution.authority,
      .pending_route = execution.pending,
      .navigation =
          ProductionMppiNavigation{
              .state =
                  MotionState3D{
                      .x = static_cast<float>(start.x),
                      .y = static_cast<float>(start.y),
                      .z = static_cast<float>(start.z),
                  },
              .receive_stamp_ns = 100,
              .source_timestamp_us = 90U,
              .revision = 11U,
              .valid = true,
          },
      .objective = fixture.objective,
      .raw_world = nullptr,
      .minimum_tracking_route_mission_epoch = 0U,
      .minimum_tracking_route_sample_sequence = 0U,
      .stamp_ns = 100,
  };
  return fixture;
}

void activatePending(ExecutionSupervisor3D& supervisor) {
  const std::shared_ptr<const PendingCertifiedRoute3D> pending = supervisor.pending();
  const std::shared_ptr<const CommittedExecutionAuthority3D> authority =
      supervisor.authority();
  ASSERT_NE(pending, nullptr);
  ASSERT_NE(authority, nullptr);
  const std::shared_ptr<const ExecutionPlan3D> plan = authority->plan();
  ASSERT_NE(plan, nullptr);
  FiniteExecutionPlan3D finite = SnapshotFixture3D::finitePlanForRoute(
      *plan, pending->route, FiniteExecutionKind3D::kNominal, 100U);
  const ExecutionRouteTransitionResult3D transition =
      ::drone_city_nav::activateCertifiedRoute3D(*plan, plan->version, pending->route,
                                                 std::move(finite));
  ASSERT_TRUE(transition.applied());
  ASSERT_NE(transition.next, nullptr);
  const ExecutionHorizonCommitResult3D committed = commitExecutionHorizonForTest(
      supervisor, ExecutionHorizonTestTransaction3D{
                      .kind = ExecutionHorizonCommitKind3D::kPendingTransition,
                      .expected_authority = authority,
                      .expected_plan = plan,
                      .transition = transition,
                      .expected_pending = pending,
                      .owner = SnapshotFixture3D::committedOwner(*transition.next),
                      .input = SnapshotFixture3D::committedInput(*transition.next),
                  });
  ASSERT_EQ(committed.status, ExecutionHorizonCommitStatus3D::kCommitted);
  ASSERT_NE(supervisor.plan(), nullptr);
  ASSERT_NE(supervisor.plan()->route(), nullptr);
  ASSERT_EQ(supervisor.pending(), nullptr);
}

// The executor could build no nominal horizon on the resident and retained
// what it had: the vehicle brakes or stands on it, following nothing.
void retainResidentExecution(ExecutionSupervisor3D& supervisor) {
  const std::shared_ptr<const CommittedExecutionAuthority3D> authority =
      supervisor.authority();
  ASSERT_NE(authority, nullptr);
  const std::shared_ptr<const ExecutionPlan3D> plan = authority->plan();
  ASSERT_NE(plan, nullptr);
  ASSERT_NE(plan->route(), nullptr);
  ASSERT_NE(plan->finiteExecution(), nullptr);
  FiniteExecutionPlan3D retained = SnapshotFixture3D::finitePlanForRoute(
      *plan, *plan->route(), FiniteExecutionKind3D::kRetained,
      plan->finiteExecution()->trajectory_revision + 1U);
  const ExecutionRouteTransitionResult3D transition =
      ::drone_city_nav::replaceFiniteExecutionPlan3D(
          *plan, SnapshotFixture3D::guard(*plan), std::move(retained));
  ASSERT_TRUE(transition.applied());
  ASSERT_NE(transition.next, nullptr);
  const ExecutionHorizonCommitResult3D committed = commitExecutionHorizonForTest(
      supervisor, ExecutionHorizonTestTransaction3D{
                      .kind = ExecutionHorizonCommitKind3D::kTransition,
                      .expected_authority = authority,
                      .expected_plan = plan,
                      .transition = transition,
                      .expected_pending = nullptr,
                      .owner = SnapshotFixture3D::committedOwner(*transition.next),
                      .input = SnapshotFixture3D::committedInput(*transition.next),
                  });
  ASSERT_EQ(committed.status, ExecutionHorizonCommitStatus3D::kCommitted);
  ASSERT_NE(supervisor.plan(), nullptr);
  ASSERT_NE(supervisor.plan()->finiteExecution(), nullptr);
  ASSERT_EQ(supervisor.plan()->finiteExecution()->kind,
            FiniteExecutionKind3D::kRetained);
}

[[nodiscard]] PreparedRouteActivation3D
prepare(const RouteActivationCoordinator3D& coordinator, ActivationFixture3D fixture) {
  return coordinator.prepare(RouteActivationPreparationRequest3D{
      .transaction = std::move(fixture.transaction),
      .materialization = std::move(fixture.materialization),
      .snapshot = std::move(fixture.snapshot),
  });
}

[[nodiscard]] RouteActivationCommitContext3D
commitContext(const PreparedRouteActivation3D& prepared) {
  return RouteActivationCommitContext3D{
      .resident_world = prepared.snapshot.resident_world,
      .objective = prepared.snapshot.objective,
      .raw_world = prepared.snapshot.raw_world,
      .minimum_tracking_route_mission_epoch =
          prepared.snapshot.minimum_tracking_route_mission_epoch,
      .minimum_tracking_route_sample_sequence =
          prepared.snapshot.minimum_tracking_route_sample_sequence,
  };
}

TEST(RouteActivationCoordinator3DTest,
     SealsExactSnapshotAndPublishesThroughTheExecutionSupervisor) {
  ExecutionSupervisor3D supervisor;
  RouteActivationCoordinator3D coordinator{coordinatorConfig()};
  PreparedRouteActivation3D prepared =
      prepare(coordinator, activationFixture(supervisor));

  ASSERT_NE(prepared.result.trajectory, nullptr)
      << compiledTrajectoryFailureReason3DName(
             prepared.result.admission.trajectory_validation.reason);
  EXPECT_EQ(prepared.result.trajectory->exact_initial_state.identity.revision, 11U);
  ASSERT_TRUE(prepared.result.admission.route_certified)
      << "ready="
      << prepared.result.admission.readyForArbitration(prepared.result.proposal)
      << " generation=" << prepared.result.admission.generation_matches
      << " base=" << prepared.result.admission.certification_execution_base_current
      << " replacement=" << prepared.result.admission.replacement.replacementAllowed()
      << " assessment=" << prepared.result.admission.assessment.accepted()
      << " handoff=" << prepared.result.admission.handoff.accepted();
  if (!prepared.pending_draft.has_value()) {
    FAIL() << "certified preparation did not produce a pending draft";
    return;
  }
  EXPECT_EQ(prepared.pending_draft.value().publication_sequence, 0U);

  const RouteActivationCommitContext3D context = commitContext(prepared);
  const RouteActivationCommitResult3D committed =
      coordinator.commit(std::move(prepared), context, supervisor);

  const auto& publication_status =
      committed.result.admission.pending_publication_status;
  if (!publication_status.has_value()) {
    FAIL() << "commit did not report the manager publication status";
    return;
  }
  EXPECT_EQ(publication_status.value(), PendingRoutePublicationStatus3D::kPublished);
  EXPECT_TRUE(committed.result.admission.certified_pending);
  EXPECT_EQ(committed.result.admission.activation_status,
            StaticRouteActivationStatus::kCertifiedPending);
  ASSERT_NE(supervisor.pending(), nullptr);
  EXPECT_EQ(supervisor.pending()->publication_sequence, 1U);
}

TEST(RouteActivationCoordinator3DTest,
     UsesInjectedDynamicHandoffValidatorWithoutControllerTypes) {
  ExecutionSupervisor3D supervisor;
  std::size_t validator_calls = 0U;
  RouteActivationCoordinatorConfig3D config = coordinatorConfig();
  config.dynamic_handoff_validator =
      [&validator_calls](const DynamicHandoffRequest3D& request) {
        ++validator_calls;
        EXPECT_NE(request.candidate_trajectory, nullptr);
        EXPECT_NE(request.derived_distances_m, nullptr);
        EXPECT_GT(request.reference_speed_mps, 0.0F);
        EXPECT_GT(request.maximum_cross_track_m, 0.0F);
        return DynamicHandoffResult3D{
            .status = DynamicHandoffStatus3D::kExcessiveCrossTrack,
            .cross_track_m = request.maximum_cross_track_m + 1.0F,
        };
      };
  RouteActivationCoordinator3D coordinator{config};

  const PreparedRouteActivation3D prepared =
      prepare(coordinator, activationFixture(supervisor));

  EXPECT_EQ(validator_calls, 1U);
  EXPECT_EQ(prepared.result.admission.handoff.status,
            DynamicHandoffStatus3D::kExcessiveCrossTrack);
  EXPECT_FALSE(prepared.result.admission.handoff.accepted());
  EXPECT_FALSE(prepared.pending_draft.has_value());
  EXPECT_EQ(prepared.result.admission.activation_status,
            StaticRouteActivationStatus::kDynamicHandoffRejected);
}

TEST(RouteActivationCoordinator3DTest, RejectsMissingDynamicHandoffPort) {
  RouteActivationCoordinatorConfig3D config = coordinatorConfig();
  config.dynamic_handoff_validator = {};

  EXPECT_THROW(static_cast<void>(RouteActivationCoordinator3D{config}),
               std::invalid_argument);
}

TEST(RouteActivationCoordinator3DTest, RejectsSupersededWorldBeforePendingPublication) {
  ExecutionSupervisor3D supervisor;
  RouteActivationCoordinator3D coordinator{coordinatorConfig()};
  PreparedRouteActivation3D prepared =
      prepare(coordinator, activationFixture(supervisor));
  ASSERT_TRUE(prepared.pending_draft.has_value());
  RouteActivationCommitContext3D context = commitContext(prepared);
  context.resident_world = staticWorld(4U);

  const RouteActivationCommitResult3D committed =
      coordinator.commit(std::move(prepared), context, supervisor);

  EXPECT_FALSE(committed.result.admission.snapshot_current);
  EXPECT_EQ(committed.result.admission.activation_status,
            StaticRouteActivationStatus::kActivationSnapshotSuperseded);
  EXPECT_FALSE(committed.result.admission.pending_publication_status.has_value());
  EXPECT_EQ(supervisor.pending(), nullptr);
}

TEST(RouteActivationCoordinator3DTest,
     RejectsPreparationSupersededByAResidentPendingRoute) {
  ExecutionSupervisor3D supervisor;
  RouteActivationCoordinator3D coordinator{coordinatorConfig()};
  PreparedRouteActivation3D first = prepare(coordinator, activationFixture(supervisor));
  PreparedRouteActivation3D second =
      prepare(coordinator, activationFixture(supervisor));
  ASSERT_TRUE(first.pending_draft.has_value());
  ASSERT_TRUE(second.pending_draft.has_value());
  const RouteActivationCommitContext3D first_context = commitContext(first);
  static_cast<void>(coordinator.commit(std::move(first), first_context, supervisor));
  const std::shared_ptr<const PendingCertifiedRoute3D> resident = supervisor.pending();
  ASSERT_NE(resident, nullptr);

  const RouteActivationCommitContext3D second_context = commitContext(second);
  const RouteActivationCommitResult3D committed =
      coordinator.commit(std::move(second), second_context, supervisor);

  EXPECT_FALSE(committed.result.admission.pending_snapshot_current);
  EXPECT_FALSE(committed.result.admission.snapshot_current);
  EXPECT_FALSE(committed.result.admission.pending_publication_status.has_value());
  EXPECT_EQ(committed.result.admission.activation_status,
            StaticRouteActivationStatus::kActivationSnapshotSuperseded);
  EXPECT_EQ(supervisor.pending(), resident);
}

TEST(RouteActivationCoordinator3DTest,
     AtomicallyReplacesPendingWithAMateriallyFasterIncumbent) {
  ExecutionSupervisor3D supervisor;
  RouteActivationCoordinator3D slow_coordinator{coordinatorConfig(0.5)};
  PreparedRouteActivation3D slow =
      prepare(slow_coordinator, activationFixture(supervisor));
  const RouteActivationCommitContext3D slow_context = commitContext(slow);
  const RouteActivationCommitResult3D slow_committed =
      slow_coordinator.commit(std::move(slow), slow_context, supervisor);
  ASSERT_TRUE(slow_committed.result.admission.certified_pending);
  const std::shared_ptr<const PendingCertifiedRoute3D> resident = supervisor.pending();
  ASSERT_NE(resident, nullptr);

  RouteActivationCoordinator3D fast_coordinator{coordinatorConfig(3.0)};
  PreparedRouteActivation3D fast =
      prepare(fast_coordinator, activationFixture(supervisor));
  ASSERT_TRUE(fast.result.admission.successor_improvement_required);
  ASSERT_TRUE(fast.result.admission.successor_compared_to_pending);
  ASSERT_TRUE(fast.result.admission.successor_improvement.accepted());
  ASSERT_TRUE(fast.pending_draft.has_value());
  const RouteActivationCommitContext3D fast_context = commitContext(fast);
  const RouteActivationCommitResult3D fast_committed =
      fast_coordinator.commit(std::move(fast), fast_context, supervisor);

  ASSERT_TRUE(fast_committed.result.admission.pending_publication_status.has_value());
  EXPECT_EQ(fast_committed.result.admission.pending_publication_status.value_or(
                PendingRoutePublicationStatus3D::kInvalidCandidate),
            PendingRoutePublicationStatus3D::kReplaced);
  EXPECT_TRUE(fast_committed.result.admission.certified_pending);
  ASSERT_NE(supervisor.pending(), nullptr);
  EXPECT_NE(supervisor.pending(), resident);
  EXPECT_EQ(supervisor.pending()->publication_sequence, 2U);
}

TEST(RouteActivationCoordinator3DTest,
     RetainsPendingWhenEitherImprovementThresholdIsNotCleared) {
  ExecutionSupervisor3D supervisor;
  RouteActivationCoordinator3D coordinator{coordinatorConfig()};
  PreparedRouteActivation3D first = prepare(coordinator, activationFixture(supervisor));
  const RouteActivationCommitContext3D first_context = commitContext(first);
  static_cast<void>(coordinator.commit(std::move(first), first_context, supervisor));
  const std::shared_ptr<const PendingCertifiedRoute3D> resident = supervisor.pending();
  ASSERT_NE(resident, nullptr);

  PreparedRouteActivation3D equivalent =
      prepare(coordinator, activationFixture(supervisor));
  EXPECT_TRUE(equivalent.result.admission.successor_improvement_required);
  EXPECT_TRUE(equivalent.result.admission.successor_compared_to_pending);
  EXPECT_FALSE(equivalent.result.admission.successor_improvement.accepted());
  EXPECT_EQ(equivalent.result.admission.successor_improvement.status,
            RouteSuccessorImprovementStatus3D::kInsufficientAbsoluteImprovement);
  EXPECT_FALSE(equivalent.pending_draft.has_value());
  const RouteActivationCommitContext3D equivalent_context = commitContext(equivalent);
  const RouteActivationCommitResult3D committed =
      coordinator.commit(std::move(equivalent), equivalent_context, supervisor);

  EXPECT_EQ(committed.result.admission.activation_status,
            StaticRouteActivationStatus::kInsufficientSuccessorImprovement);
  EXPECT_EQ(supervisor.pending(), resident);
}

TEST(RouteActivationCoordinator3DTest,
     KeepsContinuousTrackingOutsidePointToPointHysteresis) {
  ExecutionSupervisor3D supervisor;
  RouteActivationCoordinator3D coordinator{coordinatorConfig()};
  PreparedRouteActivation3D first =
      prepare(coordinator, activationFixture(supervisor, true));
  ASSERT_TRUE(first.pending_draft.has_value())
      << "candidate="
      << staticRouteCandidateStatusName(
             first.result.admission.candidate_validation.status)
      << " reserve="
      << certifiedRouteReserveStatus3DName(
             first.result.admission.certified_reserve.status)
      << " trajectory="
      << compiledTrajectoryFailureReason3DName(
             first.result.admission.trajectory_validation.reason)
      << " assessment=" << first.result.admission.assessment.accepted()
      << " handoff=" << first.result.admission.handoff.accepted()
      << " replacement=" << first.result.admission.replacement.replacementAllowed()
      << " generation=" << first.result.admission.generation_matches
      << " certified=" << first.result.admission.route_certified;
  const RouteActivationCommitContext3D first_context = commitContext(first);
  static_cast<void>(coordinator.commit(std::move(first), first_context, supervisor));
  const std::shared_ptr<const PendingCertifiedRoute3D> resident = supervisor.pending();
  ASSERT_NE(resident, nullptr);

  PreparedRouteActivation3D tracking_update =
      prepare(coordinator, activationFixture(supervisor, true));
  EXPECT_TRUE(tracking_update.result.admission.successor_compared_to_pending);
  EXPECT_FALSE(tracking_update.result.admission.successor_improvement_required);
  EXPECT_EQ(tracking_update.result.admission.successor_improvement.status,
            RouteSuccessorImprovementStatus3D::kNotAssessed);
  ASSERT_TRUE(tracking_update.pending_draft.has_value());
  const RouteActivationCommitContext3D tracking_context =
      commitContext(tracking_update);
  const RouteActivationCommitResult3D committed =
      coordinator.commit(std::move(tracking_update), tracking_context, supervisor);

  ASSERT_TRUE(committed.result.admission.pending_publication_status.has_value());
  EXPECT_EQ(committed.result.admission.pending_publication_status.value_or(
                PendingRoutePublicationStatus3D::kInvalidCandidate),
            PendingRoutePublicationStatus3D::kReplaced);
  ASSERT_NE(supervisor.pending(), nullptr);
  EXPECT_NE(supervisor.pending(), resident);
}

TEST(RouteActivationCoordinator3DTest,
     PublishesMateriallyFasterSameIntentAsCurrentStateRouteHandoff) {
  ExecutionSupervisor3D supervisor;
  RouteActivationCoordinator3D slow_coordinator{coordinatorConfig(0.5)};
  PreparedRouteActivation3D slow =
      prepare(slow_coordinator, activationFixture(supervisor));
  const RouteActivationCommitContext3D slow_context = commitContext(slow);
  static_cast<void>(slow_coordinator.commit(std::move(slow), slow_context, supervisor));
  activatePending(supervisor);

  RouteActivationCoordinator3D fast_coordinator{coordinatorConfig(3.0)};
  PreparedRouteActivation3D fast =
      prepare(fast_coordinator, activationFixture(supervisor));
  ASSERT_TRUE(fast.result.admission.successor_improvement_required);
  EXPECT_FALSE(fast.result.admission.successor_compared_to_pending);
  ASSERT_TRUE(fast.result.admission.successor_improvement.accepted());
  ASSERT_TRUE(fast.result.admission.replacement.replacementAllowed());
  if (!fast.pending_draft.has_value()) {
    FAIL() << "materially faster active-route successor was not certified";
    return;
  }
  EXPECT_EQ(fast.pending_draft->base_kind, PendingExecutionBaseKind3D::kRouteHandoff);
  const RouteActivationCommitContext3D fast_context = commitContext(fast);
  const RouteActivationCommitResult3D committed =
      fast_coordinator.commit(std::move(fast), fast_context, supervisor);

  EXPECT_TRUE(committed.result.admission.certified_pending);
  ASSERT_NE(supervisor.pending(), nullptr);
  EXPECT_EQ(supervisor.pending()->base_kind, PendingExecutionBaseKind3D::kRouteHandoff);
}

TEST(RouteActivationCoordinator3DTest,
     ARouteTheVehicleIsFollowingIsTheBaseASuccessorImprovesOn) {
  // The improvement requirement measures a successor against the route the
  // vehicle is following. A route the plan merely keeps -- while the vehicle
  // brakes on a continuation stop, or holds with no executable horizon -- is
  // not one it is following, and one recorded flight refused a successor there
  // for being two hundredths of a second slower while it had nothing to fly.
  ExecutionSupervisor3D supervisor;
  RouteActivationCoordinator3D coordinator{coordinatorConfig()};
  PreparedRouteActivation3D first = prepare(coordinator, activationFixture(supervisor));
  const RouteActivationCommitContext3D first_context = commitContext(first);
  static_cast<void>(coordinator.commit(std::move(first), first_context, supervisor));
  activatePending(supervisor);
  ASSERT_NE(supervisor.plan(), nullptr);
  ASSERT_EQ(supervisor.plan()->phase(), ExecutionRoutePhase3D::kFollowing);

  const PreparedRouteActivation3D equivalent =
      prepare(coordinator, activationFixture(supervisor));

  EXPECT_TRUE(equivalent.result.admission.successor_improvement_required);
  EXPECT_FALSE(equivalent.result.admission.successor_improvement.accepted());
  EXPECT_FALSE(equivalent.pending_draft.has_value());
}

TEST(RouteActivationCoordinator3DTest,
     ADivergedReleaseWaivesTheSuccessorImprovementRequirement) {
  ExecutionSupervisor3D supervisor;
  RouteActivationCoordinator3D coordinator{coordinatorConfig()};
  PreparedRouteActivation3D first = prepare(coordinator, activationFixture(supervisor));
  const RouteActivationCommitContext3D first_context = commitContext(first);
  static_cast<void>(coordinator.commit(std::move(first), first_context, supervisor));
  activatePending(supervisor);
  ASSERT_NE(supervisor.plan(), nullptr);
  const std::uint64_t generation = supervisor.plan()->routeGenerationHighWater();
  ASSERT_GT(generation, 0U);

  // The same route again, so it improves on nothing; the search that produced
  // it replaces a resident the vehicle could not follow.
  ActivationFixture3D diverged = activationFixture(supervisor);
  diverged.transaction = makePlannerSearchTransaction3D(
      diverged.world, captureResidentPlannerWorld3D(*diverged.world),
      diverged.transaction->objective,
      StaticRouteSearchRequestIdentity{
          .kind = StaticRouteSearchRequestKind::kReplan,
          .base_route_generation = generation,
      },
      std::nullopt, RouteReleaseReason3D::kDiverged);
  ASSERT_NE(diverged.transaction, nullptr);
  PreparedRouteActivation3D replacement = prepare(coordinator, diverged);
  EXPECT_FALSE(replacement.result.admission.successor_improvement_required);
  EXPECT_TRUE(replacement.result.admission.replacement.replacementAllowed());
  ASSERT_TRUE(replacement.pending_draft.has_value());
  const RouteActivationCommitContext3D replacement_context = commitContext(replacement);
  const RouteActivationCommitResult3D committed =
      coordinator.commit(std::move(replacement), replacement_context, supervisor);

  EXPECT_TRUE(committed.result.admission.certified_pending)
      << staticRouteActivationStatusName(committed.result.admission.activation_status);
  EXPECT_NE(supervisor.pending(), nullptr);
}

TEST(RouteActivationCoordinator3DTest,
     AStalledReleaseTakesASlowerSuccessorWithoutTheBlockedGrace) {
  ExecutionSupervisor3D supervisor;
  RouteActivationCoordinator3D coordinator{coordinatorConfig(3.0)};
  PreparedRouteActivation3D first = prepare(coordinator, activationFixture(supervisor));
  const RouteActivationCommitContext3D first_context = commitContext(first);
  static_cast<void>(coordinator.commit(std::move(first), first_context, supervisor));
  activatePending(supervisor);
  ASSERT_NE(supervisor.plan(), nullptr);
  const std::uint64_t generation = supervisor.plan()->routeGenerationHighWater();
  ASSERT_GT(generation, 0U);

  // The same route flown slower: it loses to the resident on remaining time,
  // so a blocked release holds it for the grace that loss earns.
  RouteActivationCoordinator3D slow_coordinator{coordinatorConfig(0.5)};
  const auto replacement_for = [&](const RouteReleaseReason3D reason) {
    ActivationFixture3D released = activationFixture(supervisor);
    released.transaction = makePlannerSearchTransaction3D(
        released.world, captureResidentPlannerWorld3D(*released.world),
        released.transaction->objective,
        StaticRouteSearchRequestIdentity{
            .kind = StaticRouteSearchRequestKind::kReplan,
            .base_route_generation = generation,
        },
        std::nullopt, reason);
    return released;
  };
  ActivationFixture3D blocked = replacement_for(RouteReleaseReason3D::kBlocked);
  ASSERT_NE(blocked.transaction, nullptr);
  const PreparedRouteActivation3D held = prepare(slow_coordinator, blocked);
  ASSERT_TRUE(held.result.admission.blocked_replacement_assessed);
  EXPECT_TRUE(held.result.admission.blocked_replacement_deferred);
  EXPECT_FALSE(held.pending_draft.has_value());

  // Released as stalled instead, the vehicle has stood through that grace
  // already: the successor is taken as found.
  ActivationFixture3D stalled = replacement_for(RouteReleaseReason3D::kStalled);
  ASSERT_NE(stalled.transaction, nullptr);
  PreparedRouteActivation3D replacement = prepare(slow_coordinator, stalled);
  EXPECT_FALSE(replacement.result.admission.successor_improvement_required);
  EXPECT_FALSE(replacement.result.admission.blocked_replacement_assessed);
  EXPECT_FALSE(replacement.result.admission.blocked_replacement_deferred);
  EXPECT_TRUE(replacement.result.admission.replacement.replacementAllowed());
  ASSERT_TRUE(replacement.pending_draft.has_value());
  const RouteActivationCommitContext3D replacement_context = commitContext(replacement);
  const RouteActivationCommitResult3D committed =
      slow_coordinator.commit(std::move(replacement), replacement_context, supervisor);

  EXPECT_TRUE(committed.result.admission.certified_pending)
      << staticRouteActivationStatusName(committed.result.admission.activation_status);
  EXPECT_NE(supervisor.pending(), nullptr);
}

TEST(RouteActivationCoordinator3DTest,
     AResidentOnARetainedHorizonHoldsNoSuccessorBack) {
  ExecutionSupervisor3D supervisor;
  RouteActivationCoordinator3D coordinator{coordinatorConfig(3.0)};
  PreparedRouteActivation3D first = prepare(coordinator, activationFixture(supervisor));
  const RouteActivationCommitContext3D first_context = commitContext(first);
  static_cast<void>(coordinator.commit(std::move(first), first_context, supervisor));
  activatePending(supervisor);
  retainResidentExecution(supervisor);
  ASSERT_NE(supervisor.plan(), nullptr);
  const std::uint64_t generation = supervisor.plan()->routeGenerationHighWater();
  ASSERT_GT(generation, 0U);

  // The same route flown slower loses to the resident on remaining time; a
  // resident under execution would hold it for the grace that loss earns.
  RouteActivationCoordinator3D slow_coordinator{coordinatorConfig(0.5)};
  ActivationFixture3D blocked = activationFixture(supervisor);
  blocked.transaction = makePlannerSearchTransaction3D(
      blocked.world, captureResidentPlannerWorld3D(*blocked.world),
      blocked.transaction->objective,
      StaticRouteSearchRequestIdentity{
          .kind = StaticRouteSearchRequestKind::kReplan,
          .base_route_generation = generation,
      },
      std::nullopt, RouteReleaseReason3D::kBlocked);
  ASSERT_NE(blocked.transaction, nullptr);
  PreparedRouteActivation3D replacement = prepare(slow_coordinator, blocked);
  EXPECT_FALSE(replacement.result.admission.successor_improvement_required);
  EXPECT_FALSE(replacement.result.admission.blocked_replacement_assessed);
  EXPECT_FALSE(replacement.result.admission.blocked_replacement_deferred);
  ASSERT_TRUE(replacement.pending_draft.has_value());
  const RouteActivationCommitContext3D replacement_context = commitContext(replacement);
  const RouteActivationCommitResult3D committed =
      slow_coordinator.commit(std::move(replacement), replacement_context, supervisor);

  EXPECT_TRUE(committed.result.admission.certified_pending)
      << staticRouteActivationStatusName(committed.result.admission.activation_status);
  EXPECT_NE(supervisor.pending(), nullptr);
}

TEST(RouteActivationCoordinator3DTest,
     InvalidPreparationFailsClosedWithoutAnExecutionDraft) {
  RouteActivationCoordinator3D coordinator{coordinatorConfig()};

  const PreparedRouteActivation3D prepared = coordinator.prepare({});

  EXPECT_EQ(prepared.result.admission.candidate_validation.status,
            StaticRouteCandidateStatus::kInvalidInput);
  EXPECT_FALSE(prepared.pending_draft.has_value());
  EXPECT_EQ(prepared.execution_base, nullptr);
}

} // namespace
} // namespace drone_city_nav
