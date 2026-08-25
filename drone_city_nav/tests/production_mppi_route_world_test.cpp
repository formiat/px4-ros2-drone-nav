#include <gtest/gtest.h>

#include <memory>
#include <optional>
#include <vector>

#include "production_mppi_node.hpp"
#include "production_mppi_route_world.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] ProductionMppiPreparedEsdf coherentObservedWorld() {
  ProductionMppiPreparedEsdf world;
  world.producer_instance_id = 7U;
  world.revision = 99U;
  world.source_raw_revision = 451U;
  world.source_occupied_fingerprint = 88U;
  world.grid = mppi::EsdfGrid{.width = 4,
                              .height = 4,
                              .resolution_m = 1.0F,
                              .depth = 4,
                              .outside_is_unknown = true};
  world.distances_m = std::make_shared<const std::vector<float>>(64U, 2.0F);
  world.observed_occupancy = std::make_shared<const ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 4, 4, 4});
  world.observed_raw_world_owner = VersionedObservedRawWorld3D::captureOwned(
      RawMapVersion{
          .producer_instance_id = 7U, .base_snapshot_revision = 400U, .revision = 451U},
      world.observed_occupancy, std::nullopt, std::nullopt);
  world.observed_esdf_resource = ObservedEsdfResource3D{
      .local_occupancy = std::make_shared<const ObservedOccupancyGrid3D>(
          GridBounds3D{0.0, 0.0, 0.0, 1.0, 4, 4, 4}),
      .nearest_obstacle_indices =
          std::make_shared<const std::vector<std::size_t>>(64U, 0U),
      .classification_override_cells =
          std::make_shared<const std::vector<GridIndex3D>>(),
      .coverage =
          ObservedEsdfCoverage3D{
              .source_raw_version = {.producer_instance_id = 7U,
                                     .base_snapshot_revision = 400U,
                                     .revision = 451U},
              .raw_local_fingerprint = 88U,
              .esdf_fingerprint = 99U,
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
      .esdf_revision = 99U,
      .gpu_esdf_revision = 99U,
  };
  return world;
}

TEST(ProductionMppiRouteWorldTest, ExactObservedResourcesFormOneCoherentGeneration) {
  const ProductionMppiPreparedEsdf world = coherentObservedWorld();

  EXPECT_EQ(assessProductionWorldGeneration(world),
            ProductionWorldGenerationStatus::kCoherent);
  EXPECT_TRUE(productionWorldGenerationCoherent(world));
  EXPECT_NE(navigationWorldCertificate3D(world).local_world_generation, 0U);
}

TEST(ProductionMppiRouteWorldTest, MixedEsdfAndRawGenerationsFailClosed) {
  ProductionMppiPreparedEsdf world = coherentObservedWorld();
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
  ProductionMppiPreparedEsdf world = coherentObservedWorld();
  const auto other_occupancy = std::make_shared<const ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 4, 4, 4});
  world.observed_raw_world_owner = VersionedObservedRawWorld3D::captureOwned(
      world.local_world_generation.raw_map, other_occupancy, std::nullopt,
      std::nullopt);

  EXPECT_EQ(assessProductionWorldGeneration(world),
            ProductionWorldGenerationStatus::kObservedOwnerMismatch);
}

TEST(ProductionMppiRouteWorldTest, ObservedCoverageMustMatchExactWorldResources) {
  ProductionMppiPreparedEsdf world = coherentObservedWorld();
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
  ProductionMppiPreparedEsdf world;
  world.revision = 77U;
  world.grid = mppi::EsdfGrid{4, 4, 1.0F, 0.0F, 0.0F, 4, 0.0F};
  world.distances_m = std::make_shared<const std::vector<float>>(64U, 2.0F);
  world.local_world_generation = LocalWorldGeneration{
      .generation = 3U,
      .raw_map = {.base_snapshot_revision = 77U, .revision = 77U},
      .pose_revision = 5U,
      .esdf_revision = 77U,
      .gpu_esdf_revision = 77U,
  };

  EXPECT_TRUE(productionWorldGenerationCoherent(world));
  --world.local_world_generation.raw_map.base_snapshot_revision;
  EXPECT_EQ(assessProductionWorldGeneration(world),
            ProductionWorldGenerationStatus::kRawVersionMismatch);
}

TEST(ProductionMppiRouteWorldTest,
     CompletedWorldBuildPreservesNewerRoutePlanningCache) {
  auto generation_33_route = std::make_shared<const std::vector<RouteSample3D>>(2U);
  auto generation_34_route = std::make_shared<const std::vector<RouteSample3D>>(3U);

  ProductionMppiPreparedEsdf completed_world_build;
  completed_world_build.local_world_generation.generation = 118U;
  completed_world_build.producer_instance_id = 7U;
  completed_world_build.revision = 9002U;
  completed_world_build.source_raw_revision = 451U;
  completed_world_build.grid.width = 17;
  completed_world_build.distances_m =
      std::make_shared<const std::vector<float>>(4U, 2.0F);
  completed_world_build.global_guide_generation = 33U;
  completed_world_build.route_3d = generation_33_route;
  completed_world_build.route_intent.id = 33U;
  completed_world_build.static_route_extension_request = true;

  ProductionMppiPreparedEsdf resident = completed_world_build;
  resident.local_world_generation.generation = 117U;
  resident.revision = 9001U;
  resident.source_raw_revision = 447U;
  resident.global_guide_generation = 34U;
  resident.route_3d = generation_34_route;
  resident.route_intent.id = 1234U;
  resident.static_route_extension_request = false;

  adoptWorldResources(resident, completed_world_build);

  EXPECT_EQ(resident.local_world_generation.generation, 118U);
  EXPECT_EQ(resident.revision, 9002U);
  EXPECT_EQ(resident.source_raw_revision, 451U);
  EXPECT_EQ(resident.grid.width, 17);
  EXPECT_EQ(resident.distances_m, completed_world_build.distances_m);
  EXPECT_EQ(resident.global_guide_generation, 34U);
  EXPECT_EQ(resident.route_3d, generation_34_route);
  EXPECT_EQ(resident.route_intent.id, 1234U);
  EXPECT_FALSE(resident.static_route_extension_request);
}

TEST(ProductionMppiRouteWorldTest,
     CompletedObservedWorldBuildRetainsExactRawObservationOwner) {
  const auto observed_occupancy = std::make_shared<const ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 4, 4, 4});
  const std::shared_ptr<const VersionedObservedRawWorld3D> observed_owner =
      VersionedObservedRawWorld3D::captureOwned(
          RawMapVersion{.producer_instance_id = 7U,
                        .base_snapshot_revision = 451U,
                        .revision = 451U},
          observed_occupancy, std::nullopt, std::nullopt);
  ASSERT_NE(observed_owner, nullptr);

  ProductionMppiPreparedEsdf completed_world_build;
  completed_world_build.observed_occupancy = observed_occupancy;
  completed_world_build.observed_raw_world_owner = observed_owner;
  completed_world_build.observed_esdf_resource = ObservedEsdfResource3D{
      .local_occupancy = observed_occupancy,
      .nearest_obstacle_indices =
          std::make_shared<const std::vector<std::size_t>>(64U, 0U),
      .classification_override_cells =
          std::make_shared<const std::vector<GridIndex3D>>(),
      .coverage = ObservedEsdfCoverage3D{.raw_local_fingerprint = 91U},
  };
  ProductionMppiPreparedEsdf resident;

  adoptWorldResources(resident, completed_world_build);

  EXPECT_EQ(resident.observed_occupancy, observed_occupancy);
  EXPECT_EQ(resident.observed_raw_world_owner, observed_owner);
  EXPECT_EQ(resident.observed_esdf_resource.local_occupancy, observed_occupancy);
  EXPECT_EQ(resident.observed_esdf_resource.coverage.raw_local_fingerprint, 91U);
  ASSERT_NE(resident.observed_raw_world_owner, nullptr);
  EXPECT_EQ(std::addressof(resident.observed_raw_world_owner->occupancy()),
            resident.observed_occupancy.get());
  EXPECT_EQ(resident.observed_raw_world_owner->occupiedSnapshot(),
            observed_owner->occupiedSnapshot());
}

TEST(ProductionMppiRouteWorldTest,
     ExactAppliedControlBodyAxisTakesPriorityOverMeasuredAcceleration) {
  ProductionMppiAppliedControl applied;
  applied.control = mppi::Control{.ax = 4.0F, .ay = 0.0F, .az = 0.0F};
  applied.source_stamp_ns = 1'010'000'000;
  applied.receive_stamp_ns = 1'020'000'000;
  applied.producer_instance_id = 91U;
  applied.horizon_producer_instance_id = 92U;
  applied.horizon_sequence = 17U;
  applied.execution_mode = msg::MppiControlFeedback::EXECUTION_MODE_PLANNED;
  applied.control_authoritative = true;
  applied.valid = true;
  const ProductionMppiExecutionHorizonOwner owner{
      .valid_from_ns = 1'000'000'000,
      .valid_until_ns = 2'000'000'000,
      .producer_instance_id = 92U,
      .target_offboard_instance_id = 91U,
      .sequence = 17U,
      .execution_mode = msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED,
      .valid = true,
  };
  ProductionMppiNavigation navigation;
  navigation.measured_equivalent_control =
      mppi::Control{.ax = -4.0F, .ay = 0.0F, .az = 0.0F};
  navigation.receive_stamp_ns = 1'025'000'000;
  navigation.linear_acceleration_authoritative = true;
  navigation.valid = true;

  const std::optional<FootprintBodyAxis> selected = authoritativeBodyAxisForExecution(
      applied, owner, navigation, 1'030'000'000, 100.0, 100.0);
  ASSERT_TRUE(selected.has_value());
  const FootprintBodyAxis expected = bodyAxisFromWorldAcceleration(Vec3{4.0, 0.0, 0.0});
  EXPECT_DOUBLE_EQ(selected->x, expected.x);
  EXPECT_DOUBLE_EQ(selected->y, expected.y);
  EXPECT_DOUBLE_EQ(selected->z, expected.z);
}

TEST(ProductionMppiRouteWorldTest,
     MeasuredBodyAxisIsFallbackForNonOwningAppliedControl) {
  ProductionMppiAppliedControl applied;
  applied.control = mppi::Control{.ax = 4.0F};
  applied.source_stamp_ns = 1'010'000'000;
  applied.receive_stamp_ns = 1'020'000'000;
  applied.producer_instance_id = 91U;
  applied.horizon_producer_instance_id = 92U;
  applied.horizon_sequence = 16U;
  applied.execution_mode = msg::MppiControlFeedback::EXECUTION_MODE_PLANNED;
  applied.control_authoritative = true;
  applied.valid = true;
  const ProductionMppiExecutionHorizonOwner owner{
      .valid_from_ns = 1'000'000'000,
      .valid_until_ns = 2'000'000'000,
      .producer_instance_id = 92U,
      .target_offboard_instance_id = 91U,
      .sequence = 17U,
      .execution_mode = msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED,
      .valid = true,
  };
  ProductionMppiNavigation navigation;
  navigation.measured_equivalent_control =
      mppi::Control{.ax = -4.0F, .ay = 1.0F, .az = 0.5F};
  navigation.receive_stamp_ns = 1'025'000'000;
  navigation.linear_acceleration_authoritative = true;
  navigation.valid = true;

  const std::optional<FootprintBodyAxis> selected = authoritativeBodyAxisForExecution(
      applied, owner, navigation, 1'030'000'000, 100.0, 100.0);
  ASSERT_TRUE(selected.has_value());
  const FootprintBodyAxis expected =
      bodyAxisFromWorldAcceleration(Vec3{-4.0, 1.0, 0.5});
  EXPECT_DOUBLE_EQ(selected->x, expected.x);
  EXPECT_DOUBLE_EQ(selected->y, expected.y);
  EXPECT_DOUBLE_EQ(selected->z, expected.z);
}

TEST(ProductionMppiRouteWorldTest,
     MissingOrStaleAccelerationEvidenceDoesNotInventBodyAxis) {
  ProductionMppiAppliedControl applied;
  ProductionMppiExecutionHorizonOwner owner;
  ProductionMppiNavigation navigation;
  navigation.receive_stamp_ns = 1'000'000'000;
  navigation.valid = true;

  EXPECT_FALSE(authoritativeBodyAxisForExecution(applied, owner, navigation,
                                                 1'010'000'000, 100.0, 100.0)
                   .has_value());

  navigation.linear_acceleration_authoritative = true;
  EXPECT_FALSE(authoritativeBodyAxisForExecution(applied, owner, navigation,
                                                 1'500'000'000, 100.0, 100.0)
                   .has_value());
}

TEST(ProductionMppiRouteWorldTest,
     AppliedControlAuthorityRequiresExactTwoSidedLiveOwnership) {
  ProductionMppiAppliedControl applied;
  applied.source_stamp_ns = 1'010'000'000;
  applied.receive_stamp_ns = 1'020'000'000;
  applied.producer_instance_id = 91U;
  applied.horizon_producer_instance_id = 92U;
  applied.horizon_sequence = 17U;
  applied.execution_mode = msg::MppiControlFeedback::EXECUTION_MODE_PLANNED;
  applied.control_authoritative = true;
  applied.valid = true;
  const ProductionMppiExecutionHorizonOwner owner{
      .valid_from_ns = 1'000'000'000,
      .valid_until_ns = 2'000'000'000,
      .producer_instance_id = 92U,
      .target_offboard_instance_id = 91U,
      .sequence = 17U,
      .execution_mode = msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED,
      .valid = true,
  };

  EXPECT_TRUE(
      appliedControlAuthoritativeForExecution(applied, owner, 1'030'000'000, 100.0));

  ProductionMppiAppliedControl bounded_clock_skew = applied;
  bounded_clock_skew.source_stamp_ns = 1'050'000'000;
  bounded_clock_skew.receive_stamp_ns = 1'045'000'000;
  EXPECT_TRUE(appliedControlAuthoritativeForExecution(bounded_clock_skew, owner,
                                                      1'040'000'000, 100.0));

  ProductionMppiAppliedControl future_feedback = bounded_clock_skew;
  future_feedback.source_stamp_ns = 1'200'000'000;
  future_feedback.receive_stamp_ns = 1'200'000'000;
  EXPECT_FALSE(appliedControlAuthoritativeForExecution(future_feedback, owner,
                                                       1'040'000'000, 100.0));

  ProductionMppiAppliedControl wrong_offboard = applied;
  ++wrong_offboard.producer_instance_id;
  EXPECT_FALSE(appliedControlAuthoritativeForExecution(wrong_offboard, owner,
                                                       1'030'000'000, 100.0));

  ProductionMppiAppliedControl wrong_planner = applied;
  ++wrong_planner.horizon_producer_instance_id;
  EXPECT_FALSE(appliedControlAuthoritativeForExecution(wrong_planner, owner,
                                                       1'030'000'000, 100.0));

  ProductionMppiAppliedControl revoked = applied;
  revoked.control_authoritative = false;
  EXPECT_FALSE(
      appliedControlAuthoritativeForExecution(revoked, owner, 1'030'000'000, 100.0));
  ProductionMppiAppliedControl delayed = applied;
  delayed.source_stamp_ns = 1'010'000'000;
  delayed.receive_stamp_ns = 1'490'000'000;
  EXPECT_FALSE(
      appliedControlAuthoritativeForExecution(delayed, owner, 1'500'000'000, 100.0));
  EXPECT_FALSE(appliedControlAuthoritativeForExecution(applied, owner,
                                                       owner.valid_until_ns, 100.0));
}

TEST(ProductionMppiRouteWorldTest,
     VehicleStatusAuthorityRequiresFreshStableArmedObservation) {
  ProductionMppiVehicleStatus status{
      .receive_stamp_ns = 1'000'000'000,
      .source_timestamp_us = 900'000U,
      .revision = 3U,
      .armed = true,
      .valid = true,
  };

  EXPECT_TRUE(
      vehicleStatusAuthoritativeForExecution(status, true, 1'050'000'000, 100.0));
  EXPECT_TRUE(
      vehicleStatusAuthoritativeForExecution(status, true, 1'500'000'000, 1000.0));
  EXPECT_TRUE(
      vehicleStatusAuthoritativeForExecution(status, true, 2'000'000'000, 1000.0));
  EXPECT_FALSE(
      vehicleStatusAuthoritativeForExecution(status, true, 2'000'000'001, 1000.0));
  EXPECT_FALSE(
      vehicleStatusAuthoritativeForExecution(status, false, 1'050'000'000, 100.0));
  status.armed = false;
  EXPECT_FALSE(
      vehicleStatusAuthoritativeForExecution(status, true, 1'050'000'000, 100.0));
  status.armed = true;
  status.valid = false;
  EXPECT_FALSE(
      vehicleStatusAuthoritativeForExecution(status, true, 1'050'000'000, 100.0));
  status.valid = true;
  EXPECT_FALSE(
      vehicleStatusAuthoritativeForExecution(status, true, 1'101'000'000, 100.0));
  EXPECT_FALSE(
      vehicleStatusAuthoritativeForExecution(status, true, 999'999'999, 100.0));
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
  const ProductionMppiRawWorld3D committed{
      .version =
          RawMapVersion{
              .producer_instance_id = 7U,
              .base_snapshot_revision = 40U,
              .revision = 42U,
          },
      .source_stamp_ns = 1'000'000'000,
      .receive_stamp_ns = 1'010'000'000,
      .ready_stamp_ns = 1'020'000'000,
      .reconstruction_ms = 1.0,
      .occupancy = nullptr,
      .execution_owner = nullptr,
      .dirty_chunks = {},
      .full_reset = false,
  };

  EXPECT_DOUBLE_EQ(committedRawWorldAgeMs(&committed, 1'030'000'000), 30.0);
  EXPECT_TRUE(std::isinf(
      committedRawWorldAgeMs<ProductionMppiRawWorld3D>(nullptr, 1'030'000'000)));
}

} // namespace
} // namespace drone_city_nav
