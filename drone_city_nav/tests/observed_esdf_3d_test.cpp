#include "drone_city_nav/observed_esdf_3d.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <numbers>

namespace drone_city_nav {
namespace {

void fillKnownFree(ObservedOccupancyGrid3D& occupancy) {
  const GridBounds3D& bounds = occupancy.bounds();
  for (int z = 0; z < bounds.depth_cells; ++z) {
    for (int y = 0; y < bounds.height_cells; ++y) {
      for (int x = 0; x < bounds.width_cells; ++x) {
        static_cast<void>(
            occupancy.setState(GridIndex3D{x, y, z}, ObservedVoxelState::kFree));
      }
    }
  }
}

[[nodiscard]] PreviousObservedEsdf3D
previousField(const ObservedEsdf3D& field,
              const ObservedOccupancyGrid3D& source_occupancy) {
  return PreviousObservedEsdf3D{
      .grid = field.grid,
      .distances_m = field.distances_m,
      .source_occupancy =
          std::make_shared<const ObservedOccupancyGrid3D>(source_occupancy),
      .local_occupancy = field.local_occupancy,
      .occupancy_fingerprint = field.occupancy_fingerprint,
      .maximum_distance_m = field.maximum_distance_m,
  };
}

TEST(ObservedEsdf3DTest, PreservesUnknownFreeAndOccupiedSemantics) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 6, 4, 3};
  ObservedOccupancyGrid3D occupancy{bounds};
  static_cast<void>(
      occupancy.setState(GridIndex3D{1, 1, 1}, ObservedVoxelState::kFree));
  static_cast<void>(
      occupancy.setState(GridIndex3D{2, 1, 1}, ObservedVoxelState::kOccupied));

  const ObservedEsdf3D field = buildObservedEsdf3D(occupancy, bounds, 10.0);
  const auto index = [&](const int x, const int y, const int z) {
    return (static_cast<std::size_t>(z) *
                static_cast<std::size_t>(bounds.height_cells) +
            static_cast<std::size_t>(y)) *
               static_cast<std::size_t>(bounds.width_cells) +
           static_cast<std::size_t>(x);
  };

  EXPECT_TRUE(field.grid.outside_is_unknown);
  ASSERT_TRUE(field.local_occupancy);
  EXPECT_EQ(field.grid.depth, bounds.depth_cells);
  EXPECT_EQ(field.distances_m.at(index(0, 0, 0)), mppi::kUnknownEsdfDistanceM);
  EXPECT_GT(field.distances_m.at(index(1, 1, 1)), 0.0F);
  EXPECT_FLOAT_EQ(field.distances_m.at(index(2, 1, 1)), 0.0F);
  EXPECT_EQ(field.stats.known_voxels, 2U);
  EXPECT_EQ(field.stats.free_voxels, 1U);
  EXPECT_EQ(field.stats.occupied_voxels, 1U);
  EXPECT_EQ(field.stats.unknown_voxels, 70U);
  EXPECT_EQ(field.local_occupancy->state(GridIndex3D{0, 0, 0}),
            ObservedVoxelState::kUnknown);
  EXPECT_EQ(field.local_occupancy->state(GridIndex3D{1, 1, 1}),
            ObservedVoxelState::kFree);
  EXPECT_EQ(field.local_occupancy->state(GridIndex3D{2, 1, 1}),
            ObservedVoxelState::kOccupied);
}

TEST(ObservedEsdf3DTest, FingerprintIsDeterministicAndSensitiveToObservedState) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 0.5, 64, 64, 8};
  const LocalObservedEsdfWindow3D window{
      .horizontal_half_extent_m = 4.0,
      .vertical_half_extent_m = 2.0,
      .horizontal_recenter_margin_m = 2.0,
      .vertical_recenter_margin_m = 1.0,
  };
  const GridBounds3D local =
      selectLocalObservedEsdfBounds(bounds, Point3{8.0, 8.0, 2.0}, window);
  ObservedOccupancyGrid3D occupancy{bounds};
  const std::uint64_t empty = observedOccupancyFingerprint(occupancy, local);

  static_cast<void>(
      occupancy.setState(GridIndex3D{16, 16, 4}, ObservedVoxelState::kFree));
  const std::uint64_t free = observedOccupancyFingerprint(occupancy, local);
  EXPECT_NE(free, empty);
  EXPECT_EQ(free, observedOccupancyFingerprint(occupancy, local));

  static_cast<void>(
      occupancy.setState(GridIndex3D{16, 16, 4}, ObservedVoxelState::kOccupied));
  EXPECT_NE(observedOccupancyFingerprint(occupancy, local), free);
}

TEST(ObservedEsdf3DTest, FingerprintIgnoresCellsOutsideExactLocalBounds) {
  const GridBounds3D world{0.0, 0.0, 0.0, 1.0, 48, 48, 32};
  const GridBounds3D local{10.0, 10.0, 8.0, 1.0, 10, 10, 10};
  ObservedOccupancyGrid3D occupancy{world};
  const std::uint64_t empty = observedOccupancyFingerprint(occupancy, local);

  static_cast<void>(
      occupancy.setState(GridIndex3D{5, 12, 12}, ObservedVoxelState::kOccupied));
  EXPECT_EQ(observedOccupancyFingerprint(occupancy, local), empty);

  static_cast<void>(
      occupancy.setState(GridIndex3D{12, 12, 12}, ObservedVoxelState::kOccupied));
  EXPECT_NE(observedOccupancyFingerprint(occupancy, local), empty);
}

TEST(ObservedEsdf3DTest,
     ProprioceptiveSeedChangesOnlyPlanningClassificationAndNeverOccupiedCells) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 0.25, 16, 16, 16};
  ObservedOccupancyGrid3D occupancy{bounds};
  static_cast<void>(
      occupancy.setState(GridIndex3D{8, 8, 8}, ObservedVoxelState::kOccupied));
  const ProprioceptiveFreeSpaceSeed3D seed{
      .position = Point3{2.0, 2.0, 2.0},
      .body_axis = FootprintBodyAxis{},
      .footprint = SweptFootprintConfig{.radius_m = 0.5,
                                        .lower_extent_m = 0.25,
                                        .upper_extent_m = 0.25},
  };
  const ObservedEsdf3D field =
      buildObservedEsdf3D(occupancy, bounds, 10.0, nullptr, &seed);

  ASSERT_TRUE(field.local_occupancy);
  EXPECT_GT(field.stats.proprioceptive_free_voxels, 0U);
  EXPECT_EQ(field.local_occupancy->state(GridIndex3D{8, 8, 8}),
            ObservedVoxelState::kOccupied);
  EXPECT_EQ(occupancy.state(GridIndex3D{7, 8, 8}), ObservedVoxelState::kUnknown);
  EXPECT_EQ(field.local_occupancy->state(GridIndex3D{7, 8, 8}),
            ObservedVoxelState::kFree);
}

TEST(ObservedEsdf3DTest, TiltedProprioceptiveSeedPromotesOnlyItsActualBodyVolume) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 0.25, 16, 16, 16};
  const ObservedOccupancyGrid3D occupancy{bounds};
  const ProprioceptiveFreeSpaceSeed3D tilted_seed{
      .position = Point3{2.125, 2.125, 2.125},
      .body_axis = FootprintBodyAxis{0.6, 0.0, 0.8},
      .footprint = SweptFootprintConfig{.radius_m = 0.2,
                                        .lower_extent_m = 0.2,
                                        .upper_extent_m = 1.0},
  };
  const GridIndex3D tilted_only_cell{10, 8, 10};

  const ObservedEsdf3D without_seed = buildObservedEsdf3D(occupancy, bounds, 10.0);
  const ObservedEsdf3D with_tilted_seed =
      buildObservedEsdf3D(occupancy, bounds, 10.0, nullptr, &tilted_seed);

  ASSERT_TRUE(without_seed.local_occupancy);
  ASSERT_TRUE(with_tilted_seed.local_occupancy);
  EXPECT_EQ(without_seed.local_occupancy->state(tilted_only_cell),
            ObservedVoxelState::kUnknown);
  EXPECT_EQ(with_tilted_seed.local_occupancy->state(tilted_only_cell),
            ObservedVoxelState::kFree);
}

TEST(ObservedEsdf3DTest,
     ProprioceptiveSupportContactIsFreeOnlyInThePreparedPlanningWorld) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 0.25, 16, 16, 16};
  ObservedOccupancyGrid3D occupancy{bounds};
  const GridIndex3D support_cell{8, 8, 7};
  static_cast<void>(occupancy.setState(support_cell, ObservedVoxelState::kOccupied));
  const ProprioceptiveFreeSpaceSeed3D seed{
      .position = Point3{2.0, 2.0, 2.0},
      .body_axis = FootprintBodyAxis{},
      .footprint = SweptFootprintConfig{.radius_m = 0.5,
                                        .lower_extent_m = 0.25,
                                        .upper_extent_m = 0.25},
  };
  const std::optional<LaunchSupportContact3D> support =
      detectLaunchSupportContact3D(occupancy, seed);
  ASSERT_TRUE(support.has_value());

  const ObservedEsdf3D field =
      buildObservedEsdf3D(occupancy, bounds, 10.0, nullptr, &seed, &*support);
  const ObservedEsdf3D unmasked =
      buildObservedEsdf3D(occupancy, bounds, 10.0, nullptr, &seed);

  ASSERT_TRUE(field.local_occupancy);
  EXPECT_GT(field.stats.launch_support_voxels, 1U);
  EXPECT_EQ(field.local_occupancy->state(support_cell), ObservedVoxelState::kFree);
  EXPECT_EQ(field.local_occupancy->state(GridIndex3D{7, 8, 7}),
            ObservedVoxelState::kFree);
  EXPECT_EQ(field.local_occupancy->state(GridIndex3D{0, 0, 0}),
            ObservedVoxelState::kUnknown);
  EXPECT_EQ(occupancy.state(support_cell), ObservedVoxelState::kOccupied);
  EXPECT_NE(field.occupancy_fingerprint, unmasked.occupancy_fingerprint);
}

TEST(ObservedEsdf3DTest, VehicleLandDetectorCreatesBoundedSupportWithoutLidarEvidence) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 0.25, 16, 16, 16};
  const ProprioceptiveFreeSpaceSeed3D seed{
      .position = Point3{2.0, 2.0, 2.0},
      .body_axis = FootprintBodyAxis{},
      .footprint = SweptFootprintConfig{.radius_m = 0.5,
                                        .lower_extent_m = 0.25,
                                        .upper_extent_m = 0.25},
  };

  const LaunchSupportContact3D support =
      makeVehicleLandedSupportContact3D(bounds, seed);

  EXPECT_FALSE(support.contact_cells.empty());
  EXPECT_EQ(support.occupied_evidence_cells, 0U);
  EXPECT_EQ(support.evidence_source, LaunchSupportEvidenceSource::kVehicleLandDetector);
  EXPECT_DOUBLE_EQ(support.maximum_lateral_departure_m, bounds.resolution_m);
  EXPECT_DOUBLE_EQ(support.maximum_axial_settling_m, bounds.resolution_m);
}

TEST(ObservedEsdf3DTest, LaunchSupportDepartureLeavesObservedFloorAlongTheBodyAxis) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 0.25, 20, 20, 20};
  ObservedOccupancyGrid3D occupancy{bounds};
  for (int z = 0; z < bounds.depth_cells; ++z) {
    for (int y = 0; y < bounds.height_cells; ++y) {
      for (int x = 0; x < bounds.width_cells; ++x) {
        static_cast<void>(
            occupancy.setState(GridIndex3D{x, y, z}, ObservedVoxelState::kFree));
      }
    }
  }
  for (int y = 0; y < bounds.height_cells; ++y) {
    for (int x = 0; x < bounds.width_cells; ++x) {
      static_cast<void>(
          occupancy.setState(GridIndex3D{x, y, 7}, ObservedVoxelState::kOccupied));
    }
  }
  const ProprioceptiveFreeSpaceSeed3D seed{
      .position = Point3{2.0, 2.0, 2.0},
      .body_axis = FootprintBodyAxis{},
      .footprint = SweptFootprintConfig{.radius_m = 0.5,
                                        .lower_extent_m = 0.25,
                                        .upper_extent_m = 0.25},
  };
  const LaunchSupportContact3D support =
      makeVehicleLandedSupportContact3D(bounds, seed);

  const LaunchSupportDeparture3D departure =
      planLaunchSupportDeparture3D(occupancy, seed.position, support, 1.0);

  ASSERT_TRUE(departure.executable);
  EXPECT_GT(departure.target.z, seed.position.z);
  EXPECT_GE(departure.axial_departure_m, 1.0);
  EXPECT_TRUE(validateRawFootprintAt(occupancy, departure.target, seed.body_axis,
                                     seed.footprint)
                  .accepted());
  EXPECT_TRUE(validateRawSweptFootprint(occupancy, seed.position, seed.body_axis,
                                        departure.target, seed.body_axis,
                                        seed.footprint, &seed, &support)
                  .accepted());
}

TEST(ObservedEsdf3DTest, DetectsQuantizedSupportAtTheEdgeOfTheLaunchFootprint) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 0.25, 16, 16, 16};
  ObservedOccupancyGrid3D occupancy{bounds};
  const GridIndex3D edge_support_cell{10, 8, 7};
  static_cast<void>(
      occupancy.setState(edge_support_cell, ObservedVoxelState::kOccupied));
  const ProprioceptiveFreeSpaceSeed3D seed{
      .position = Point3{2.0, 2.0, 2.0},
      .body_axis = FootprintBodyAxis{},
      .footprint = SweptFootprintConfig{.radius_m = 0.5,
                                        .lower_extent_m = 0.25,
                                        .upper_extent_m = 0.25},
  };

  const std::optional<LaunchSupportContact3D> support =
      detectLaunchSupportContact3D(occupancy, seed);

  ASSERT_TRUE(support.has_value());
  EXPECT_EQ(support->occupied_evidence_cells, 1U);
}

TEST(ObservedEsdf3DTest, LaunchSupportCoversTheBoundedDepartureEnvelope) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 0.25, 16, 16, 16};
  ObservedOccupancyGrid3D occupancy{bounds};
  const GridIndex3D swept_support_cell{11, 8, 7};
  static_cast<void>(
      occupancy.setState(swept_support_cell, ObservedVoxelState::kOccupied));
  const ProprioceptiveFreeSpaceSeed3D seed{
      .position = Point3{2.0, 2.0, 2.0},
      .body_axis = FootprintBodyAxis{},
      .footprint = SweptFootprintConfig{.radius_m = 0.5,
                                        .lower_extent_m = 0.25,
                                        .upper_extent_m = 0.25},
  };

  const std::optional<LaunchSupportContact3D> support =
      detectLaunchSupportContact3D(occupancy, seed);

  ASSERT_TRUE(support.has_value());
  EXPECT_EQ(support->occupied_evidence_cells, 1U);
  const ObservedEsdf3D field =
      buildObservedEsdf3D(occupancy, bounds, 10.0, nullptr, &seed, &*support);
  ASSERT_TRUE(field.local_occupancy);
  EXPECT_EQ(field.local_occupancy->state(swept_support_cell),
            ObservedVoxelState::kFree);
  EXPECT_EQ(occupancy.state(swept_support_cell), ObservedVoxelState::kOccupied);
}

TEST(ObservedEsdf3DTest, IncrementalInsertAndRemoveExactlyMatchFullRebuilds) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 48, 32, 24};
  ObservedOccupancyGrid3D occupancy{bounds};
  fillKnownFree(occupancy);
  const ObservedEsdf3D initial = buildObservedEsdf3D(occupancy, bounds, 3.0);
  const PreviousObservedEsdf3D initial_previous = previousField(initial, occupancy);
  const GridIndex3D changed_cell{20, 15, 10};
  const OccupancyChunkIndex3D dirty = ObservedOccupancyGrid3D::chunkIndex(changed_cell);

  static_cast<void>(occupancy.setState(changed_cell, ObservedVoxelState::kOccupied));
  const ObservedEsdf3D inserted = updateObservedEsdf3D(
      occupancy, bounds, 3.0, &initial_previous, std::span{&dirty, 1U}, false, 0.75);
  const ObservedEsdf3D inserted_full = buildObservedEsdf3D(occupancy, bounds, 3.0);

  EXPECT_EQ(inserted.stats.mode, ObservedEsdf3DBuildMode::kIncremental);
  EXPECT_GT(inserted.stats.reused_voxels, 0U);
  EXPECT_LT(inserted.stats.recomputed_voxels, inserted.distances_m.size());
  EXPECT_EQ(inserted.distances_m, inserted_full.distances_m);
  EXPECT_EQ(inserted.occupancy_fingerprint, inserted_full.occupancy_fingerprint);

  const PreviousObservedEsdf3D inserted_previous = previousField(inserted, occupancy);
  static_cast<void>(occupancy.setState(changed_cell, ObservedVoxelState::kFree));
  const ObservedEsdf3D removed = updateObservedEsdf3D(
      occupancy, bounds, 3.0, &inserted_previous, std::span{&dirty, 1U}, false, 0.75);
  const ObservedEsdf3D removed_full = buildObservedEsdf3D(occupancy, bounds, 3.0);

  EXPECT_EQ(removed.stats.mode, ObservedEsdf3DBuildMode::kIncremental);
  EXPECT_EQ(removed.distances_m, removed_full.distances_m);
  EXPECT_EQ(removed.occupancy_fingerprint, removed_full.occupancy_fingerprint);
}

TEST(ObservedEsdf3DTest, IncrementalClassificationChangeMatchesFullRebuild) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 48, 32, 24};
  ObservedOccupancyGrid3D occupancy{bounds};
  fillKnownFree(occupancy);
  static_cast<void>(
      occupancy.setState(GridIndex3D{24, 16, 12}, ObservedVoxelState::kOccupied));
  const ObservedEsdf3D initial = buildObservedEsdf3D(occupancy, bounds, 3.0);
  const PreviousObservedEsdf3D previous = previousField(initial, occupancy);
  const GridIndex3D changed_cell{19, 15, 10};
  const OccupancyChunkIndex3D dirty = ObservedOccupancyGrid3D::chunkIndex(changed_cell);
  static_cast<void>(occupancy.setState(changed_cell, ObservedVoxelState::kUnknown));

  const ObservedEsdf3D incremental = updateObservedEsdf3D(
      occupancy, bounds, 3.0, &previous, std::span{&dirty, 1U}, false, 0.75);
  const ObservedEsdf3D full = buildObservedEsdf3D(occupancy, bounds, 3.0);

  EXPECT_EQ(incremental.stats.mode, ObservedEsdf3DBuildMode::kIncremental);
  EXPECT_EQ(incremental.distances_m, full.distances_m);
  EXPECT_EQ(incremental.local_occupancy->state(changed_cell),
            ObservedVoxelState::kUnknown);
}

TEST(ObservedEsdf3DTest,
     EvidenceOnlyClassificationChangeIsIncrementalWithoutDirtyChunks) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 48, 32, 24};
  ObservedOccupancyGrid3D occupancy{bounds};
  static_cast<void>(
      occupancy.setState(GridIndex3D{24, 16, 12}, ObservedVoxelState::kOccupied));
  const SweptFootprintConfig footprint{
      .radius_m = 0.6, .lower_extent_m = 0.5, .upper_extent_m = 0.5};
  const ProprioceptiveFreeSpaceSeed3D first_seed{
      .position = Point3{12.5, 12.5, 10.5},
      .body_axis = FootprintBodyAxis{},
      .footprint = footprint,
  };
  const ProprioceptiveFreeSpaceSeed3D second_seed{
      .position = Point3{16.5, 12.5, 10.5},
      .body_axis = FootprintBodyAxis{},
      .footprint = footprint,
  };
  const ObservedEsdf3D initial =
      buildObservedEsdf3D(occupancy, bounds, 3.0, nullptr, &first_seed);
  const PreviousObservedEsdf3D previous = previousField(initial, occupancy);

  const ObservedEsdf3D incremental = updateObservedEsdf3D(
      occupancy, bounds, 3.0, &previous, {}, false, 0.75, nullptr, &second_seed);
  const ObservedEsdf3D full =
      buildObservedEsdf3D(occupancy, bounds, 3.0, nullptr, &second_seed);

  EXPECT_EQ(incremental.stats.mode, ObservedEsdf3DBuildMode::kIncremental);
  EXPECT_GT(incremental.stats.changed_voxels, 0U);
  EXPECT_EQ(incremental.distances_m, full.distances_m);
  EXPECT_EQ(incremental.occupancy_fingerprint, full.occupancy_fingerprint);
}

TEST(ObservedEsdf3DTest,
     UntrackedRawChangeFallsBackEvenWhenClassificationMasksTheDifference) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 24, 24, 24};
  ObservedOccupancyGrid3D occupancy{bounds};
  const GridIndex3D changed_cell{12, 12, 12};
  const ProprioceptiveFreeSpaceSeed3D seed{
      .position = Point3{12.5, 12.5, 12.5},
      .body_axis = FootprintBodyAxis{},
      .footprint = SweptFootprintConfig{.radius_m = 0.6,
                                        .lower_extent_m = 0.5,
                                        .upper_extent_m = 0.5},
  };
  const ObservedEsdf3D initial =
      buildObservedEsdf3D(occupancy, bounds, 3.0, nullptr, &seed);
  const PreviousObservedEsdf3D previous = previousField(initial, occupancy);
  ASSERT_TRUE(initial.local_occupancy);
  ASSERT_EQ(initial.local_occupancy->state(changed_cell), ObservedVoxelState::kFree);
  ASSERT_TRUE(occupancy.setState(changed_cell, ObservedVoxelState::kFree));

  const ObservedEsdf3D update = updateObservedEsdf3D(occupancy, bounds, 3.0, &previous,
                                                     {}, false, 0.75, nullptr, &seed);

  EXPECT_EQ(update.stats.mode, ObservedEsdf3DBuildMode::kFull);
  EXPECT_TRUE(update.stats.incremental_fallback);
  EXPECT_EQ(update.stats.changed_voxels, 0U);
}

TEST(ObservedEsdf3DTest, RawChangesOutsideTheExactLocalWindowDoNotForceARebuild) {
  const GridBounds3D world{0.0, 0.0, 0.0, 1.0, 48, 48, 32};
  const GridBounds3D local{10.0, 10.0, 8.0, 1.0, 10, 10, 10};
  ObservedOccupancyGrid3D occupancy{world};
  const ObservedEsdf3D initial = buildObservedEsdf3D(occupancy, local, 3.0);
  const PreviousObservedEsdf3D previous = previousField(initial, occupancy);
  ASSERT_TRUE(
      occupancy.setState(GridIndex3D{5, 12, 12}, ObservedVoxelState::kOccupied));

  const ObservedEsdf3D update =
      updateObservedEsdf3D(occupancy, local, 3.0, &previous, {}, false, 0.75);

  EXPECT_EQ(update.stats.mode, ObservedEsdf3DBuildMode::kReused);
  EXPECT_FALSE(update.stats.incremental_fallback);
  EXPECT_EQ(update.distances_m, initial.distances_m);
}

TEST(ObservedEsdf3DTest, ReusesAnExactlyUnchangedClassifiedWorld) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 24, 24, 24};
  ObservedOccupancyGrid3D occupancy{bounds};
  fillKnownFree(occupancy);
  const ObservedEsdf3D initial = buildObservedEsdf3D(occupancy, bounds, 3.0);
  const PreviousObservedEsdf3D previous = previousField(initial, occupancy);

  const ObservedEsdf3D reused =
      updateObservedEsdf3D(occupancy, bounds, 3.0, &previous, {}, false, 0.75);

  EXPECT_EQ(reused.stats.mode, ObservedEsdf3DBuildMode::kReused);
  EXPECT_EQ(reused.stats.recomputed_voxels, 0U);
  EXPECT_EQ(reused.stats.reused_voxels, reused.distances_m.size());
  EXPECT_EQ(reused.distances_m, initial.distances_m);
  EXPECT_EQ(reused.local_occupancy, initial.local_occupancy);
}

TEST(ObservedEsdf3DTest, FallsBackWhenDirtyLineageOrPatchBudgetIsInsufficient) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 48, 32, 24};
  ObservedOccupancyGrid3D occupancy{bounds};
  fillKnownFree(occupancy);
  const ObservedEsdf3D initial = buildObservedEsdf3D(occupancy, bounds, 3.0);
  const PreviousObservedEsdf3D previous = previousField(initial, occupancy);
  static_cast<void>(
      occupancy.setState(GridIndex3D{20, 15, 10}, ObservedVoxelState::kOccupied));
  const OccupancyChunkIndex3D unrelated{0, 0, 0};

  const ObservedEsdf3D missing_lineage = updateObservedEsdf3D(
      occupancy, bounds, 3.0, &previous, std::span{&unrelated, 1U}, false, 0.75);
  EXPECT_EQ(missing_lineage.stats.mode, ObservedEsdf3DBuildMode::kFull);
  EXPECT_TRUE(missing_lineage.stats.incremental_fallback);

  const OccupancyChunkIndex3D dirty =
      ObservedOccupancyGrid3D::chunkIndex(GridIndex3D{20, 15, 10});
  const ObservedEsdf3D over_budget = updateObservedEsdf3D(
      occupancy, bounds, 3.0, &previous, std::span{&dirty, 1U}, false, 0.01);
  EXPECT_EQ(over_budget.stats.mode, ObservedEsdf3DBuildMode::kFull);
  EXPECT_TRUE(over_budget.stats.incremental_fallback);
}

TEST(ObservedEsdf3DTest, CoverageCertificateRequiresExactParentAccounting) {
  const RawMapVersion source{
      .producer_instance_id = 17U, .base_snapshot_revision = 4U, .revision = 9U};
  const RawMapVersion parent{
      .producer_instance_id = 17U, .base_snapshot_revision = 4U, .revision = 8U};
  const ObservedEsdfCoverage3D incremental{
      .source_raw_version = source,
      .parent_raw_version = parent,
      .raw_local_fingerprint = 101U,
      .esdf_fingerprint = 202U,
      .parent_esdf_fingerprint = 201U,
      .total_voxels = 100U,
      .recomputed_voxels = 25U,
      .reused_voxels = 75U,
      .maximum_distance_m = 7.0,
      .mode = ObservedEsdf3DBuildMode::kIncremental,
  };
  EXPECT_TRUE(incremental.coherent());

  ObservedEsdfCoverage3D invalid = incremental;
  ++invalid.parent_raw_version.base_snapshot_revision;
  EXPECT_FALSE(invalid.coherent());
  invalid = incremental;
  ++invalid.reused_voxels;
  EXPECT_FALSE(invalid.coherent());

  ObservedEsdfCoverage3D reused = incremental;
  reused.esdf_fingerprint = reused.parent_esdf_fingerprint;
  reused.recomputed_voxels = 0U;
  reused.reused_voxels = reused.total_voxels;
  reused.mode = ObservedEsdf3DBuildMode::kReused;
  EXPECT_TRUE(reused.coherent());

  ObservedEsdfCoverage3D evidence_only = incremental;
  evidence_only.parent_raw_version = evidence_only.source_raw_version;
  EXPECT_TRUE(evidence_only.coherent());
  evidence_only.parent_esdf_fingerprint = evidence_only.esdf_fingerprint;
  EXPECT_FALSE(evidence_only.coherent());

  reused.parent_raw_version = reused.source_raw_version;
  EXPECT_FALSE(reused.coherent());
}

TEST(ObservedEsdf3DTest, MaximumDistanceCoversFootprintAndVoxelCorrection) {
  const SweptFootprintConfig footprint{
      .radius_m = 0.82, .lower_extent_m = 0.23, .upper_extent_m = 0.35};
  const double required = requiredObservedEsdfMaximumDistanceM(6.0, footprint, 0.25);

  EXPECT_DOUBLE_EQ(required, 6.0 + std::hypot(0.82, 0.35) + std::numbers::sqrt3 * 0.25);
  EXPECT_TRUE(std::isnan(requiredObservedEsdfMaximumDistanceM(-1.0, footprint, 0.25)));
}

TEST(ObservedEsdf3DTest, RecenterHonorsWorldEdges) {
  const GridBounds3D world{0.0, 0.0, 0.0, 1.0, 100, 80, 20};
  const LocalObservedEsdfWindow3D window{
      .horizontal_half_extent_m = 10.0,
      .vertical_half_extent_m = 8.0,
      .horizontal_recenter_margin_m = 4.0,
      .vertical_recenter_margin_m = 3.0,
  };
  const GridBounds3D left =
      selectLocalObservedEsdfBounds(world, Point3{2.0, 40.0, 5.0}, window);

  EXPECT_DOUBLE_EQ(left.origin_x, world.origin_x);
  EXPECT_DOUBLE_EQ(left.origin_z, world.origin_z);
  EXPECT_FALSE(
      localObservedEsdfNeedsRecenter(left, world, Point3{2.0, 40.0, 5.0}, window));
  EXPECT_TRUE(
      localObservedEsdfNeedsRecenter(left, world, Point3{10.0, 40.0, 5.0}, window));
  EXPECT_TRUE(
      localObservedEsdfNeedsRecenter(left, world, Point3{2.0, 40.0, 12.0}, window));
}

} // namespace
} // namespace drone_city_nav
