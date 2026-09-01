#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "production_mppi_raw_world.hpp"
#include "production_mppi_route_world.hpp"
#include "production_planner_search_transaction_3d.hpp"
#include "production_route_pipeline_artifacts_3d.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] WorldSnapshot3D coherentObservedWorld() {
  WorldSnapshot3D world;
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 4, 4, 4};
  world.producer_instance_id = 7U;
  world.source_raw_revision = 451U;
  world.source_occupied_fingerprint = 88U;
  world.grid = EsdfGrid3D{.width = 4,
                          .height = 4,
                          .resolution_m = 1.0F,
                          .depth = 4,
                          .outside_is_unknown = true};
  world.observed_occupancy = std::make_shared<const ObservedOccupancyGrid3D>(bounds);
  const KnownObstacleDistance3DBuildResult distance =
      buildKnownObstacleDistance3D(*world.observed_occupancy, bounds, 7.0);
  world.revision = distance.field->sourceFingerprint();
  world.distances_m = distance.field->materializeDense();
  world.observed_raw_world_owner = VersionedObservedRawWorld3D::captureOwned(
      RawMapVersion{
          .producer_instance_id = 7U, .base_snapshot_revision = 400U, .revision = 451U},
      world.observed_occupancy, std::nullopt, std::nullopt);
  world.raw_occupied_fingerprint =
      world.observed_raw_world_owner->occupiedContentFingerprint();
  world.planner_full_reset = true;
  world.observed_esdf_resource = ObservedEsdfResource3D{
      .local_occupancy = std::make_shared<const ObservedOccupancyGrid3D>(bounds),
      .known_obstacle_distance = distance.field,
      .classification_override_cells =
          std::make_shared<const std::vector<GridIndex3D>>(),
      .coverage =
          ObservedEsdfCoverage3D{
              .source_raw_version = {.producer_instance_id = 7U,
                                     .base_snapshot_revision = 400U,
                                     .revision = 451U},
              .raw_local_fingerprint = 88U,
              .esdf_fingerprint = world.revision,
              .total_voxels = 64U,
              .recomputed_voxels = 64U,
              .maximum_distance_m = 7.0,
              .mode = ObservedEsdf3DBuildMode::kFull,
          },
  };
  world.local_world_generation = LocalWorldGeneration{
      .generation = 12U,
      .raw_map = {.producer_instance_id = 7U,
                  .base_snapshot_revision = 400U,
                  .revision = 451U},
      .pose_revision = 21U,
      .esdf_revision = world.revision,
      .gpu_esdf_revision = world.revision,
  };
  return world;
}

[[nodiscard]] std::shared_ptr<const PersistentPlannerWorld3D>
observedPlannerWorld(const WorldSnapshot3D& world) {
  return captureResidentPlannerWorld3D(world);
}

TEST(ProductionMppiRouteWorldTest, ExactObservedResourcesFormOneCoherentGeneration) {
  const WorldSnapshot3D world = coherentObservedWorld();

  EXPECT_EQ(assessProductionWorldGeneration(world),
            ProductionWorldGenerationStatus::kCoherent);
  EXPECT_TRUE(productionWorldGenerationCoherent(world));
  EXPECT_NE(navigationWorldCertificate3D(world).local_world_generation, 0U);
}

TEST(ProductionMppiRouteWorldTest, MixedEsdfAndRawGenerationsFailClosed) {
  WorldSnapshot3D world = coherentObservedWorld();
  ++world.local_world_generation.gpu_esdf_revision;

  EXPECT_EQ(assessProductionWorldGeneration(world),
            ProductionWorldGenerationStatus::kInvalidGeneration);
  EXPECT_EQ(navigationWorldCertificate3D(world).local_world_generation, 0U);

  world = coherentObservedWorld();
  ++world.source_raw_revision;
  EXPECT_EQ(assessProductionWorldGeneration(world),
            ProductionWorldGenerationStatus::kRawVersionMismatch);
  EXPECT_EQ(navigationWorldCertificate3D(world).local_world_generation, 0U);
}

TEST(ProductionMppiRouteWorldTest, ObservedOwnerMustMatchExactRawSnapshot) {
  WorldSnapshot3D world = coherentObservedWorld();
  const auto other_occupancy = std::make_shared<const ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 4, 4, 4});
  world.observed_raw_world_owner = VersionedObservedRawWorld3D::captureOwned(
      world.local_world_generation.raw_map, other_occupancy, std::nullopt,
      std::nullopt);

  EXPECT_EQ(assessProductionWorldGeneration(world),
            ProductionWorldGenerationStatus::kObservedOwnerMismatch);
}

TEST(ProductionMppiRouteWorldTest,
     RouteSearchUsesResidentPlannerInputWhenOverlayIsNotRequested) {
  const WorldSnapshot3D world = coherentObservedWorld();
  const std::shared_ptr<const PersistentPlannerWorld3D> resident =
      observedPlannerWorld(world);

  ASSERT_NE(resident, nullptr);
  EXPECT_EQ(resident->observed_occupancy, world.observed_occupancy);
  EXPECT_EQ(resident->revision, world.source_raw_revision);
  EXPECT_EQ(resident->occupied_fingerprint, world.raw_occupied_fingerprint);
  EXPECT_TRUE(resident->full_reset);
}

TEST(ProductionMppiRouteWorldTest,
     InitialPlannerTransactionOwnsOnlyExactImmutableInputs) {
  auto world = std::make_shared<const WorldSnapshot3D>(coherentObservedWorld());
  const std::shared_ptr<const PersistentPlannerWorld3D> planner_world =
      captureResidentPlannerWorld3D(*world);
  const StaticRouteObjective objective{
      .goal = {10.0, 20.0, 30.0},
      .mission_epoch = 9U,
      .available = true,
  };

  const std::shared_ptr<const PlannerSearchTransaction3D> transaction =
      makePlannerSearchTransaction3D(world, planner_world, objective,
                                     StaticRouteSearchRequestIdentity{
                                         .kind = StaticRouteSearchRequestKind::kInitial,
                                     });

  ASSERT_NE(transaction, nullptr);
  EXPECT_EQ(transaction->world, world);
  EXPECT_EQ(transaction->planner_world, planner_world);
  EXPECT_EQ(transaction->objective.mission_epoch, 9U);
  EXPECT_TRUE(transaction->initial());
  EXPECT_FALSE(transaction->extension());
  EXPECT_FALSE(transaction->replacement());
}

TEST(ProductionMppiRouteWorldTest,
     ExtensionPlannerTransactionRequiresAnExactContinuityBase) {
  auto world = std::make_shared<const WorldSnapshot3D>(coherentObservedWorld());
  const StaticRouteObjective objective{
      .goal = {10.0, 20.0, 30.0},
      .mission_epoch = 9U,
      .available = true,
  };

  EXPECT_EQ(makePlannerSearchTransaction3D(
                world, captureResidentPlannerWorld3D(*world), objective,
                StaticRouteSearchRequestIdentity{
                    .kind = StaticRouteSearchRequestKind::kExtension,
                    .base_route_generation = 4U,
                }),
            nullptr);
}

TEST(ProductionMppiRouteWorldTest,
     PlannerTransactionRejectsIncoherentRequestIdentityAndReason) {
  auto world = std::make_shared<const WorldSnapshot3D>(coherentObservedWorld());
  const std::shared_ptr<const PersistentPlannerWorld3D> planner_world =
      captureResidentPlannerWorld3D(*world);
  const StaticRouteObjective objective{
      .goal = {10.0, 20.0, 30.0},
      .mission_epoch = 9U,
      .available = true,
  };

  EXPECT_EQ(
      makePlannerSearchTransaction3D(world, planner_world, objective,
                                     StaticRouteSearchRequestIdentity{
                                         .kind = StaticRouteSearchRequestKind::kInitial,
                                         .base_route_generation = 4U,
                                     }),
      nullptr);
  EXPECT_EQ(
      makePlannerSearchTransaction3D(world, planner_world, objective,
                                     StaticRouteSearchRequestIdentity{
                                         .kind = StaticRouteSearchRequestKind::kReplan,
                                         .base_route_generation = 4U,
                                     }),
      nullptr);
}

TEST(ProductionMppiRouteWorldTest,
     PhysicalReplanOverlayDoesNotMisrepresentEsdfGeneration) {
  WorldSnapshot3D world = coherentObservedWorld();
  const auto newer_occupancy = std::make_shared<const ObservedOccupancyGrid3D>(
      world.observed_occupancy->bounds());
  const RawMapVersion newer_version{
      .producer_instance_id = 7U, .base_snapshot_revision = 400U, .revision = 470U};
  const std::shared_ptr<const VersionedObservedRawWorld3D> newer_owner =
      VersionedObservedRawWorld3D::captureOwned(newer_version, newer_occupancy,
                                                std::nullopt, std::nullopt);
  ASSERT_NE(newer_owner, nullptr);
  const std::shared_ptr<const ProductionMppiRawWorld3D> newer_raw =
      ProductionMppiRawWorld3D::capture(newer_owner,
                                        ProductionMppiRawWorldMetadata3D{
                                            .source_stamp_ns = 1'000'000'000,
                                            .receive_stamp_ns = 1'010'000'000,
                                            .ready_stamp_ns = 1'020'000'000,
                                            .reconstruction_ms = 1.0,
                                            .dirty_chunks = {},
                                            .full_reset = false,
                                        });
  ASSERT_NE(newer_raw, nullptr);

  const std::shared_ptr<const PersistentPlannerWorld3D> route_search_planner_world =
      captureObservedRouteSearchWorld3D(*newer_raw, std::nullopt, std::nullopt);
  const std::shared_ptr<const PersistentPlannerWorld3D> resident =
      observedPlannerWorld(world);

  ASSERT_NE(route_search_planner_world, nullptr);
  EXPECT_TRUE(route_search_planner_world->full_reset);
  EXPECT_EQ(route_search_planner_world->revision, 470U);
  EXPECT_NE(route_search_planner_world, resident);
  const std::shared_ptr<const PlannerSearchTransaction3D> transaction =
      makePlannerSearchTransaction3D(std::make_shared<const WorldSnapshot3D>(world),
                                     route_search_planner_world,
                                     StaticRouteObjective{
                                         .goal = {10.0, 20.0, 30.0},
                                         .mission_epoch = 9U,
                                         .available = true,
                                     },
                                     StaticRouteSearchRequestIdentity{
                                         .kind = StaticRouteSearchRequestKind::kReplan,
                                         .base_route_generation = 4U,
                                     },
                                     std::nullopt, RouteReleaseReason3D::kBlocked);
  ASSERT_NE(transaction, nullptr);
  EXPECT_TRUE(transaction->replacement());
  EXPECT_EQ(transaction->planner_world, route_search_planner_world);
  EXPECT_TRUE(productionWorldGenerationCoherent(world));
  EXPECT_EQ(world.local_world_generation.raw_map.revision, 451U);
  EXPECT_EQ(navigationWorldCertificate3D(world).esdf_source_raw_revision, 451U);
}

TEST(ProductionMppiRouteWorldTest, RawWorldFactoryMakesOwnerTheOnlyOccupancyAuthority) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 4, 4, 4};
  const auto authoritative_occupancy =
      std::make_shared<const ObservedOccupancyGrid3D>(bounds);
  const RawMapVersion version{
      .producer_instance_id = 7U, .base_snapshot_revision = 400U, .revision = 470U};
  const std::shared_ptr<const VersionedObservedRawWorld3D> owner =
      VersionedObservedRawWorld3D::captureOwned(version, authoritative_occupancy,
                                                std::nullopt, std::nullopt);
  ASSERT_NE(owner, nullptr);
  const std::shared_ptr<const ProductionMppiRawWorld3D> raw =
      ProductionMppiRawWorld3D::capture(owner, ProductionMppiRawWorldMetadata3D{
                                                   .source_stamp_ns = 1'000'000'000,
                                                   .receive_stamp_ns = 1'010'000'000,
                                                   .ready_stamp_ns = 1'020'000'000,
                                                   .reconstruction_ms = 1.0,
                                                   .dirty_chunks = {},
                                                   .full_reset = false,
                                               });

  ASSERT_NE(raw, nullptr);
  EXPECT_EQ(raw->occupancyOwner(), authoritative_occupancy);
  EXPECT_EQ(raw->authoritativeOwner(), owner);
  EXPECT_NE(captureObservedRouteSearchWorld3D(*raw, std::nullopt, std::nullopt),
            nullptr);
  const ProprioceptiveFreeSpaceSeed3D seed{
      .position = {1.0, 1.0, 1.0},
      .body_axis = {},
      .footprint = {},
  };
  const auto route_evidence = raw->deriveRouteEvidence(seed, std::nullopt);
  ASSERT_NE(route_evidence, nullptr);
  EXPECT_TRUE(raw->ownsRouteEvidence(*route_evidence));
  EXPECT_EQ(route_evidence->occupancyOwner(), authoritative_occupancy);
  EXPECT_EQ(ProductionMppiRawWorld3D::capture(route_evidence,
                                              ProductionMppiRawWorldMetadata3D{
                                                  .source_stamp_ns = 1'000'000'000,
                                                  .receive_stamp_ns = 1'010'000'000,
                                                  .ready_stamp_ns = 1'020'000'000,
                                                  .reconstruction_ms = 1.0,
                                                  .dirty_chunks = {},
                                                  .full_reset = false,
                                              }),
            nullptr);
  EXPECT_EQ(ProductionMppiRawWorld3D::capture(nullptr,
                                              ProductionMppiRawWorldMetadata3D{
                                                  .source_stamp_ns = 1'000'000'000,
                                                  .receive_stamp_ns = 1'010'000'000,
                                                  .ready_stamp_ns = 1'020'000'000,
                                                  .reconstruction_ms = 1.0,
                                                  .dirty_chunks = {},
                                                  .full_reset = false,
                                              }),
            nullptr);
}

TEST(ProductionMppiRouteWorldTest,
     OnlyMissingRouteAndPhysicalCollisionSearchesRequireLatestRaw) {
  EXPECT_TRUE(
      routeSearchRequiresLatestRawOverlay3D(RouteReleaseReason3D::kNoActiveRoute));
  EXPECT_TRUE(routeSearchRequiresLatestRawOverlay3D(RouteReleaseReason3D::kBlocked));
  EXPECT_FALSE(routeSearchRequiresLatestRawOverlay3D(RouteReleaseReason3D::kExhausted));
  EXPECT_FALSE(routeSearchRequiresLatestRawOverlay3D(RouteReleaseReason3D::kDiverged));
  EXPECT_FALSE(
      routeSearchRequiresLatestRawOverlay3D(RouteReleaseReason3D::kObjectiveChanged));
}

TEST(ProductionMppiRouteWorldTest, ObservedCoverageMustMatchExactWorldResources) {
  WorldSnapshot3D world = coherentObservedWorld();
  ++world.observed_esdf_resource.coverage.source_raw_version.revision;
  EXPECT_EQ(assessProductionWorldGeneration(world),
            ProductionWorldGenerationStatus::kObservedEsdfCoverageMismatch);

  world = coherentObservedWorld();
  ++world.observed_esdf_resource.coverage.raw_local_fingerprint;
  EXPECT_EQ(assessProductionWorldGeneration(world),
            ProductionWorldGenerationStatus::kObservedEsdfCoverageMismatch);

  world = coherentObservedWorld();
  --world.observed_esdf_resource.coverage.recomputed_voxels;
  EXPECT_EQ(assessProductionWorldGeneration(world),
            ProductionWorldGenerationStatus::kObservedEsdfCoverageMismatch);
}

TEST(ProductionMppiRouteWorldTest, StaticWorldUsesItsEsdfAsRawGenerationAnchor) {
  WorldSnapshot3D world;
  world.static_occupancy = std::make_shared<const OccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 4, 4, 4}, 77U);
  world.revision = world.static_occupancy->fingerprint();
  world.source_occupied_fingerprint = world.static_occupancy->contentFingerprint();
  world.raw_occupied_fingerprint = world.source_occupied_fingerprint;
  world.grid = EsdfGrid3D{4, 4, 1.0F, 0.0F, 0.0F, 4, 0.0F};
  world.distances_m = std::make_shared<const std::vector<float>>(64U, 2.0F);
  world.local_world_generation = LocalWorldGeneration{
      .generation = 3U,
      .raw_map = {.base_snapshot_revision = 77U, .revision = 77U},
      .pose_revision = 5U,
      .esdf_revision = 77U,
      .gpu_esdf_revision = 77U,
  };

  EXPECT_TRUE(productionWorldGenerationCoherent(world));
  const std::shared_ptr<const PersistentPlannerWorld3D> planner_world =
      captureResidentPlannerWorld3D(world);
  ASSERT_NE(planner_world, nullptr);
  EXPECT_EQ(planner_world->static_occupancy, world.static_occupancy);
  EXPECT_EQ(planner_world->occupied_fingerprint, world.raw_occupied_fingerprint);
  --world.local_world_generation.raw_map.base_snapshot_revision;
  EXPECT_EQ(assessProductionWorldGeneration(world),
            ProductionWorldGenerationStatus::kRawVersionMismatch);
}

TEST(ProductionMppiRouteWorldTest,
     WorldPublicationIsOnePointerSwapAndCannotOverwriteRouteState) {
  WorldSnapshot3D old_world_value;
  old_world_value.local_world_generation.generation = 117U;
  old_world_value.revision = 9001U;
  old_world_value.source_raw_revision = 447U;
  auto old_world = std::make_shared<const WorldSnapshot3D>(std::move(old_world_value));
  const auto topology = std::make_shared<const std::vector<PassageTraversalEdge>>(1U);
  WorldSnapshot3D new_world_value;
  new_world_value.local_world_generation.generation = 118U;
  new_world_value.producer_instance_id = 7U;
  new_world_value.revision = 9002U;
  new_world_value.source_raw_revision = 451U;
  new_world_value.grid.width = 17;
  new_world_value.distances_m = std::make_shared<const std::vector<float>>(4U, 2.0F);
  new_world_value.topology_passage_traversals = topology;
  auto new_world = std::make_shared<const WorldSnapshot3D>(std::move(new_world_value));
  const auto route = std::make_shared<const std::vector<RouteSample3D>>(3U);
  MaterializedRoute3D materialized;
  materialized.world = old_world;
  materialized.candidate_generation = 34U;
  materialized.route = route;
  materialized.intent.id = 1234U;
  std::shared_ptr<const WorldSnapshot3D> resident_world = old_world;

  resident_world = new_world;

  EXPECT_EQ(resident_world, new_world);
  EXPECT_EQ(resident_world->topology_passage_traversals, topology);
  EXPECT_EQ(old_world->revision, 9001U);
  EXPECT_EQ(materialized.world, old_world);
  EXPECT_EQ(materialized.candidate_generation, 34U);
  EXPECT_EQ(materialized.route, route);
  EXPECT_EQ(materialized.intent.id, 1234U);
}

TEST(ProductionMppiRouteWorldTest,
     RouteArtifactCopiesShareTheExactImmutableWorldAndTopology) {
  const auto topology = std::make_shared<const std::vector<PassageTraversalEdge>>(1U);
  WorldSnapshot3D world;
  world.revision = 91U;
  world.topology_passage_traversals = topology;
  MaterializedRoute3D source;
  source.world = std::make_shared<const WorldSnapshot3D>(std::move(world));
  source.route = std::make_shared<const std::vector<RouteSample3D>>(2U);

  MaterializedRoute3D candidate = source;
  candidate.route.reset();
  candidate.constrained_spans.reset();

  EXPECT_EQ(candidate.world, source.world);
  EXPECT_EQ(candidate.world->revision, 91U);
  EXPECT_EQ(candidate.world->topology_passage_traversals, topology);
}

TEST(ProductionMppiRouteWorldTest,
     PendingStatusRequiresACompleteMatchingPayloadBeforeItIsSatisfied) {
  const ProductionMppiPendingRawWorldUpdate pending{
      .authority_generation = 3U,
      .producer_instance_id = 7U,
      .announced_sequence = 42U,
      .minimum_source_stamp_ns = 1'000'000'000,
  };
  const ProducerEvidenceAdmissionState matching{
      .authority_generation = 3U,
      .producer_instance_id = 7U,
      .sequence = 42U,
      .source_stamp_ns = 1'000'000'000,
  };
  const RawMapVersion committed{
      .producer_instance_id = 7U,
      .base_snapshot_revision = 40U,
      .revision = 42U,
  };

  EXPECT_TRUE(pending.valid());
  EXPECT_TRUE(pending.satisfiedBy(matching, committed));

  ProducerEvidenceAdmissionState incomplete = matching;
  incomplete.sequence = 41U;
  EXPECT_FALSE(pending.satisfiedBy(incomplete, committed));
  incomplete = matching;
  incomplete.current_identity_conflicted = true;
  EXPECT_FALSE(pending.satisfiedBy(incomplete, committed));
}

TEST(ProductionMppiRouteWorldTest, CommittedPayloadOwnsObservationFreshness) {
  const auto occupancy = std::make_shared<const ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 4, 4, 4});
  const auto owner = VersionedObservedRawWorld3D::captureOwned(
      RawMapVersion{
          .producer_instance_id = 7U,
          .base_snapshot_revision = 40U,
          .revision = 42U,
      },
      occupancy, std::nullopt, std::nullopt);
  const auto committed =
      ProductionMppiRawWorld3D::capture(owner, ProductionMppiRawWorldMetadata3D{
                                                   .source_stamp_ns = 1'000'000'000,
                                                   .receive_stamp_ns = 1'010'000'000,
                                                   .ready_stamp_ns = 1'020'000'000,
                                                   .reconstruction_ms = 1.0,
                                                   .dirty_chunks = {},
                                                   .full_reset = false,
                                               });

  ASSERT_NE(committed, nullptr);
  EXPECT_DOUBLE_EQ(committedRawWorldAgeMs(committed.get(), 1'030'000'000), 30.0);
  EXPECT_TRUE(std::isinf(committedRawWorldAgeMs(nullptr, 1'030'000'000)));
}

} // namespace
} // namespace drone_city_nav
