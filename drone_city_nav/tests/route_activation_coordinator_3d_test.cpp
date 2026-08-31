#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

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
  world->grid = mppi::EsdfGrid{
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

[[nodiscard]] RouteActivationCoordinatorConfig3D coordinatorConfig() {
  RouteActivationCoordinatorConfig3D config;
  config.route_tracking.maximum_cross_track_m = 2.0;
  config.cruise_speed_mps = 3.0;
  config.maximum_control_feedback_age_ms = 1000.0;
  config.trajectory_compiler.trajectory.unconstrained_speed_mps =
      config.cruise_speed_mps;
  config.trajectory_compiler.trajectory.physical_footprint = config.physical_footprint;
  config.passage_volume.flight_envelope = config.flight_envelope;
  config.passage_volume.footprint = config.physical_footprint;
  config.trajectory_compiler.passage_volume = config.passage_volume;
  config.mppi.steps = 120U;
  config.validation_policy = VersionedExecutionValidationPolicy3D::capture(
      config.flight_envelope, config.mppi.dynamics, config.mppi.altitude_envelope,
      config.physical_footprint, 1000.0, 1000.0, 1000.0, true, false, true);
  return config;
}

[[nodiscard]] ActivationFixture3D
activationFixture(const RouteExecutionManager3D& manager) {
  ActivationFixture3D fixture;
  fixture.world = staticWorld();
  const Point3 start{3.5, 3.5, 5.0};
  const Point3 goal{8.5, 3.5, 5.0};
  const StaticRouteObjective route_objective{
      .goal = goal,
      .mission_epoch = 7U,
      .sample_sequence = 1U,
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
      });
  auto route = std::make_shared<const std::vector<RouteSample3D>>(
      sampleRoute3D(std::vector<Point3>{start, goal}, 0.5, 3.0));
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
      .candidate_generation = 1U,
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
      .execution_authority = manager.authority(),
      .navigation =
          ProductionMppiNavigation{
              .state =
                  mppi::State{
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
     SealsExactSnapshotAndPublishesThroughTheExecutionManager) {
  RouteExecutionManager3D manager;
  RouteActivationCoordinator3D coordinator{coordinatorConfig()};
  PreparedRouteActivation3D prepared = prepare(coordinator, activationFixture(manager));

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
      << " handoff=" << prepared.result.admission.handoff.accepted;
  if (!prepared.pending_draft.has_value()) {
    FAIL() << "certified preparation did not produce a pending draft";
    return;
  }
  EXPECT_EQ(prepared.pending_draft.value().publication_sequence, 0U);

  const RouteActivationCommitContext3D context = commitContext(prepared);
  const RouteActivationCommitResult3D committed =
      coordinator.commit(std::move(prepared), context, manager);

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
  ASSERT_NE(manager.pending(), nullptr);
  EXPECT_EQ(manager.pending()->publication_sequence, 1U);
}

TEST(RouteActivationCoordinator3DTest, RejectsSupersededWorldBeforePendingPublication) {
  RouteExecutionManager3D manager;
  RouteActivationCoordinator3D coordinator{coordinatorConfig()};
  PreparedRouteActivation3D prepared = prepare(coordinator, activationFixture(manager));
  ASSERT_TRUE(prepared.pending_draft.has_value());
  RouteActivationCommitContext3D context = commitContext(prepared);
  context.resident_world = staticWorld(4U);

  const RouteActivationCommitResult3D committed =
      coordinator.commit(std::move(prepared), context, manager);

  EXPECT_FALSE(committed.result.admission.snapshot_current);
  EXPECT_EQ(committed.result.admission.activation_status,
            StaticRouteActivationStatus::kActivationSnapshotSuperseded);
  EXPECT_FALSE(committed.result.admission.pending_publication_status.has_value());
  EXPECT_EQ(manager.pending(), nullptr);
}

TEST(RouteActivationCoordinator3DTest,
     ReportsOccupiedCommitWithoutDisplacingTheResidentPendingRoute) {
  RouteExecutionManager3D manager;
  RouteActivationCoordinator3D coordinator{coordinatorConfig()};
  PreparedRouteActivation3D first = prepare(coordinator, activationFixture(manager));
  PreparedRouteActivation3D second = prepare(coordinator, activationFixture(manager));
  ASSERT_TRUE(first.pending_draft.has_value());
  ASSERT_TRUE(second.pending_draft.has_value());
  const RouteActivationCommitContext3D first_context = commitContext(first);
  static_cast<void>(coordinator.commit(std::move(first), first_context, manager));
  const std::shared_ptr<const PendingCertifiedRoute3D> resident = manager.pending();
  ASSERT_NE(resident, nullptr);

  const RouteActivationCommitContext3D second_context = commitContext(second);
  const RouteActivationCommitResult3D committed =
      coordinator.commit(std::move(second), second_context, manager);

  const auto& publication_status =
      committed.result.admission.pending_publication_status;
  if (!publication_status.has_value()) {
    FAIL() << "occupied commit did not report the manager publication status";
    return;
  }
  EXPECT_EQ(publication_status.value(),
            PendingRoutePublicationStatus3D::kPendingOccupied);
  EXPECT_EQ(committed.result.admission.activation_status,
            StaticRouteActivationStatus::kActivationCommitRejected);
  EXPECT_EQ(manager.pending(), resident);
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
