#include "drone_city_nav/launch_support_contact_3d.hpp"
#include "drone_city_nav/observed_esdf_3d.hpp"
#include "drone_city_nav/observed_esdf_footprint_3d.hpp"

#include <gtest/gtest.h>

#include <array>
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

[[nodiscard]] PreviousObservedEsdf3D previousField(const ObservedEsdf3D& field) {
  return PreviousObservedEsdf3D{
      .grid = field.grid,
      .distances_m = field.distances_m,
      .known_obstacle_distance = field.known_obstacle_distance,
      .occupancy_fingerprint = field.occupancy_fingerprint,
      .maximum_distance_m = field.maximum_distance_m,
  };
}

TEST(ObservedEsdf3DTest, MeasuresKnownObstaclesEquallyInFreeAndUnknownSpace) {
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
  ASSERT_TRUE(field.distances_m);
  ASSERT_TRUE(field.known_obstacle_distance);
  ASSERT_TRUE(field.local_occupancy);
  EXPECT_EQ(field.grid.depth, bounds.depth_cells);
  EXPECT_FLOAT_EQ(field.distances_m->at(index(0, 0, 0)), std::sqrt(6.0F));
  EXPECT_GT(field.distances_m->at(index(1, 1, 1)), 0.0F);
  EXPECT_FLOAT_EQ(field.distances_m->at(index(2, 1, 1)), 0.0F);
  EXPECT_FLOAT_EQ(field.known_obstacle_distance->distanceAt({0, 0, 0}),
                  std::sqrt(6.0F));
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

TEST(ObservedEsdf3DTest, FingerprintDependsOnlyOnKnownObstacles) {
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
  const std::uint64_t empty = knownObstacleFingerprint3D(occupancy, local);

  static_cast<void>(
      occupancy.setState(GridIndex3D{16, 16, 4}, ObservedVoxelState::kFree));
  const std::uint64_t free = knownObstacleFingerprint3D(occupancy, local);
  EXPECT_EQ(free, empty);
  EXPECT_EQ(free, knownObstacleFingerprint3D(occupancy, local));

  static_cast<void>(
      occupancy.setState(GridIndex3D{16, 16, 4}, ObservedVoxelState::kOccupied));
  EXPECT_NE(knownObstacleFingerprint3D(occupancy, local), free);
}

TEST(ObservedEsdf3DTest, FingerprintIgnoresCellsOutsideExactLocalBounds) {
  const GridBounds3D world{0.0, 0.0, 0.0, 1.0, 48, 48, 32};
  const GridBounds3D local{10.0, 10.0, 8.0, 1.0, 10, 10, 10};
  ObservedOccupancyGrid3D occupancy{world};
  const std::uint64_t empty = knownObstacleFingerprint3D(occupancy, local);

  static_cast<void>(
      occupancy.setState(GridIndex3D{5, 12, 12}, ObservedVoxelState::kOccupied));
  EXPECT_EQ(knownObstacleFingerprint3D(occupancy, local), empty);

  static_cast<void>(
      occupancy.setState(GridIndex3D{12, 12, 12}, ObservedVoxelState::kOccupied));
  EXPECT_NE(knownObstacleFingerprint3D(occupancy, local), empty);
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
  const LaunchSupportContact3D support_value =
      support.value_or(LaunchSupportContact3D{});

  const ObservedEsdf3D field =
      buildObservedEsdf3D(occupancy, bounds, 10.0, nullptr, &support_value);
  const ObservedEsdf3D unmasked = buildObservedEsdf3D(occupancy, bounds, 10.0);

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
  EXPECT_EQ(support.value_or(LaunchSupportContact3D{}).occupied_evidence_cells, 1U);
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
  const LaunchSupportContact3D support_value =
      support.value_or(LaunchSupportContact3D{});
  EXPECT_EQ(support_value.occupied_evidence_cells, 1U);
  const ObservedEsdf3D field =
      buildObservedEsdf3D(occupancy, bounds, 10.0, nullptr, &support_value);
  ASSERT_TRUE(field.local_occupancy);
  EXPECT_EQ(field.local_occupancy->state(swept_support_cell),
            ObservedVoxelState::kFree);
  EXPECT_EQ(occupancy.state(swept_support_cell), ObservedVoxelState::kOccupied);
}

TEST(ObservedEsdf3DTest, SourceChangesRebuildTheExactFieldAndRelabelsReuseIt) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 48, 32, 24};
  ObservedOccupancyGrid3D occupancy{bounds};
  fillKnownFree(occupancy);
  const ObservedEsdf3D initial = buildObservedEsdf3D(occupancy, bounds, 3.0);
  const PreviousObservedEsdf3D initial_previous = previousField(initial);
  const GridIndex3D changed_cell{20, 15, 10};

  static_cast<void>(occupancy.setState(changed_cell, ObservedVoxelState::kOccupied));
  const ObservedEsdf3D inserted =
      updateObservedEsdf3D(occupancy, bounds, 3.0, &initial_previous, false);
  const ObservedEsdf3D inserted_full = buildObservedEsdf3D(occupancy, bounds, 3.0);

  EXPECT_EQ(inserted.stats.mode, ObservedEsdf3DBuildMode::kFull);
  EXPECT_EQ(inserted.stats.recomputed_voxels, inserted.distances_m->size());
  EXPECT_EQ(*inserted.distances_m, *inserted_full.distances_m);
  EXPECT_EQ(inserted.occupancy_fingerprint, inserted_full.occupancy_fingerprint);
  EXPECT_NE(inserted.occupancy_fingerprint, initial.occupancy_fingerprint);

  const PreviousObservedEsdf3D inserted_previous = previousField(inserted);
  static_cast<void>(
      occupancy.setState(GridIndex3D{19, 15, 10}, ObservedVoxelState::kUnknown));
  const ObservedEsdf3D relabeled =
      updateObservedEsdf3D(occupancy, bounds, 3.0, &inserted_previous, false);
  EXPECT_EQ(relabeled.stats.mode, ObservedEsdf3DBuildMode::kReused);
  EXPECT_EQ(relabeled.distances_m, inserted.distances_m);
  EXPECT_EQ(relabeled.known_obstacle_distance, inserted.known_obstacle_distance);
  EXPECT_EQ(relabeled.local_occupancy->state(GridIndex3D{19, 15, 10}),
            ObservedVoxelState::kUnknown);
  EXPECT_EQ(relabeled.stats.reused_voxels, relabeled.distances_m->size());

  static_cast<void>(occupancy.setState(changed_cell, ObservedVoxelState::kFree));
  const ObservedEsdf3D removed =
      updateObservedEsdf3D(occupancy, bounds, 3.0, &inserted_previous, false);
  EXPECT_EQ(removed.stats.mode, ObservedEsdf3DBuildMode::kFull);
  EXPECT_EQ(*removed.distances_m, *initial.distances_m);
  EXPECT_EQ(removed.occupancy_fingerprint, initial.occupancy_fingerprint);
}

TEST(ObservedEsdf3DTest, FullResetRebuildsEvenWhenSourcesAreUnchanged) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 32, 32, 16};
  ObservedOccupancyGrid3D occupancy{bounds};
  static_cast<void>(
      occupancy.setState(GridIndex3D{16, 16, 8}, ObservedVoxelState::kOccupied));
  const ObservedEsdf3D initial = buildObservedEsdf3D(occupancy, bounds, 3.0);
  const PreviousObservedEsdf3D previous = previousField(initial);

  const ObservedEsdf3D reset =
      updateObservedEsdf3D(occupancy, bounds, 3.0, &previous, true);

  EXPECT_EQ(reset.stats.mode, ObservedEsdf3DBuildMode::kFull);
  EXPECT_NE(reset.distances_m, initial.distances_m);
  EXPECT_EQ(*reset.distances_m, *initial.distances_m);
  EXPECT_EQ(reset.occupancy_fingerprint, initial.occupancy_fingerprint);
}

TEST(ObservedEsdf3DTest, LaunchSupportRelabelWithoutObstacleChangeReusesDistance) {
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
  const LaunchSupportContact3D first_support =
      makeVehicleLandedSupportContact3D(bounds, first_seed);
  const LaunchSupportContact3D second_support =
      makeVehicleLandedSupportContact3D(bounds, second_seed);
  const ObservedEsdf3D initial =
      buildObservedEsdf3D(occupancy, bounds, 3.0, nullptr, &first_support);
  const PreviousObservedEsdf3D previous = previousField(initial);

  const ObservedEsdf3D relabeled = updateObservedEsdf3D(
      occupancy, bounds, 3.0, &previous, false, nullptr, &second_support);
  const ObservedEsdf3D full =
      buildObservedEsdf3D(occupancy, bounds, 3.0, nullptr, &second_support);

  EXPECT_EQ(relabeled.stats.mode, ObservedEsdf3DBuildMode::kReused);
  EXPECT_EQ(*relabeled.distances_m, *full.distances_m);
  EXPECT_EQ(relabeled.distances_m, initial.distances_m);
  EXPECT_EQ(relabeled.known_obstacle_distance, initial.known_obstacle_distance);
  EXPECT_EQ(relabeled.occupancy_fingerprint, full.occupancy_fingerprint);
  EXPECT_FALSE(relabeled.classification_override_cells.empty());
}

TEST(ObservedEsdf3DTest, SuppressedSupportCellNeverEntersTheSourceIdentity) {
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
  const LaunchSupportContact3D support =
      makeVehicleLandedSupportContact3D(bounds, seed);
  ASSERT_TRUE(occupancy.setState(changed_cell, ObservedVoxelState::kOccupied));
  const ObservedEsdf3D initial =
      buildObservedEsdf3D(occupancy, bounds, 3.0, nullptr, &support);
  const PreviousObservedEsdf3D previous = previousField(initial);
  ASSERT_TRUE(initial.local_occupancy);
  ASSERT_EQ(initial.local_occupancy->state(changed_cell), ObservedVoxelState::kFree);
  ASSERT_TRUE(occupancy.setState(changed_cell, ObservedVoxelState::kFree));

  const ObservedEsdf3D update =
      updateObservedEsdf3D(occupancy, bounds, 3.0, &previous, false, nullptr, &support);

  EXPECT_EQ(update.stats.mode, ObservedEsdf3DBuildMode::kReused);
  EXPECT_EQ(update.known_obstacle_distance, initial.known_obstacle_distance);
}

TEST(ObservedEsdf3DTest, RawChangesOutsideTheSourceHaloDoNotForceARebuild) {
  const GridBounds3D world{0.0, 0.0, 0.0, 1.0, 48, 48, 32};
  const GridBounds3D local{10.0, 10.0, 8.0, 1.0, 10, 10, 10};
  ObservedOccupancyGrid3D occupancy{world};
  const ObservedEsdf3D initial = buildObservedEsdf3D(occupancy, local, 3.0);
  const PreviousObservedEsdf3D previous = previousField(initial);
  ASSERT_TRUE(
      occupancy.setState(GridIndex3D{5, 12, 12}, ObservedVoxelState::kOccupied));

  const ObservedEsdf3D update =
      updateObservedEsdf3D(occupancy, local, 3.0, &previous, false);

  EXPECT_EQ(update.stats.mode, ObservedEsdf3DBuildMode::kReused);
  EXPECT_EQ(update.distances_m, initial.distances_m);
}

TEST(ObservedEsdf3DTest, UnalignedWindowClassificationMatchesPerVoxelLabels) {
  const GridBounds3D world{0.0, 0.0, 0.0, 1.0, 48, 48, 32};
  const GridBounds3D local{10.0, 13.0, 7.0, 1.0, 21, 19, 11};
  ObservedOccupancyGrid3D occupancy{world};
  for (int index = 0; index < 200; ++index) {
    const GridIndex3D cell{(index * 7) % 48, (index * 13) % 48, (index * 5) % 32};
    static_cast<void>(occupancy.setState(cell, index % 3 == 0
                                                   ? ObservedVoxelState::kOccupied
                                                   : ObservedVoxelState::kFree));
  }

  const ObservedEsdf3D field = buildObservedEsdf3D(occupancy, local, 3.0);

  ASSERT_TRUE(field.local_occupancy);
  for (int z = 0; z < local.depth_cells; ++z) {
    for (int y = 0; y < local.height_cells; ++y) {
      for (int x = 0; x < local.width_cells; ++x) {
        EXPECT_EQ(field.local_occupancy->state(GridIndex3D{x, y, z}),
                  occupancy.state(GridIndex3D{x + 10, y + 13, z + 7}));
      }
    }
  }
}

TEST(ObservedEsdf3DTest, ReusesAnExactlyUnchangedClassifiedWorld) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 24, 24, 24};
  ObservedOccupancyGrid3D occupancy{bounds};
  fillKnownFree(occupancy);
  const ObservedEsdf3D initial = buildObservedEsdf3D(occupancy, bounds, 3.0);
  const PreviousObservedEsdf3D previous = previousField(initial);

  const ObservedEsdf3D reused =
      updateObservedEsdf3D(occupancy, bounds, 3.0, &previous, false);

  EXPECT_EQ(reused.stats.mode, ObservedEsdf3DBuildMode::kReused);
  EXPECT_EQ(reused.stats.recomputed_voxels, 0U);
  EXPECT_EQ(reused.stats.reused_voxels, reused.distances_m->size());
  EXPECT_EQ(reused.distances_m, initial.distances_m);
  ASSERT_TRUE(reused.local_occupancy);
  EXPECT_EQ(reused.local_occupancy->knownVoxelCount(),
            initial.local_occupancy->knownVoxelCount());
  EXPECT_EQ(reused.local_occupancy->occupiedVoxelCount(),
            initial.local_occupancy->occupiedVoxelCount());
}

TEST(ObservedEsdf3DTest, CoverageCertificateRequiresExactParentAccounting) {
  const RawMapVersion source{
      .producer_instance_id = 17U, .base_snapshot_revision = 4U, .revision = 9U};
  const RawMapVersion parent{
      .producer_instance_id = 17U, .base_snapshot_revision = 4U, .revision = 8U};
  const ObservedEsdfCoverage3D full{
      .source_raw_version = source,
      .raw_local_fingerprint = 101U,
      .esdf_fingerprint = 202U,
      .total_voxels = 100U,
      .recomputed_voxels = 100U,
      .reused_voxels = 0U,
      .maximum_distance_m = 7.0,
      .mode = ObservedEsdf3DBuildMode::kFull,
  };
  EXPECT_TRUE(full.coherent());
  ObservedEsdfCoverage3D invalid_full = full;
  invalid_full.parent_esdf_fingerprint = 201U;
  EXPECT_FALSE(invalid_full.coherent());

  ObservedEsdfCoverage3D reused = full;
  reused.parent_raw_version = parent;
  reused.parent_esdf_fingerprint = reused.esdf_fingerprint;
  reused.recomputed_voxels = 0U;
  reused.reused_voxels = reused.total_voxels;
  reused.mode = ObservedEsdf3DBuildMode::kReused;
  EXPECT_TRUE(reused.coherent());

  ObservedEsdfCoverage3D invalid = reused;
  ++invalid.parent_raw_version.base_snapshot_revision;
  EXPECT_FALSE(invalid.coherent());
  invalid = reused;
  ++invalid.reused_voxels;
  EXPECT_FALSE(invalid.coherent());
  invalid = reused;
  invalid.parent_raw_version = invalid.source_raw_version;
  EXPECT_FALSE(invalid.coherent());
  invalid = reused;
  invalid.parent_esdf_fingerprint = 5U;
  EXPECT_FALSE(invalid.coherent());
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
  // The requested [0, 13) x-range and [0, 14) z-range grow to whole chunks.
  EXPECT_EQ(left.width_cells, ObservedOccupancyGrid3D::kChunkSize);
  EXPECT_EQ(left.depth_cells, ObservedOccupancyGrid3D::kChunkSize);
  EXPECT_FALSE(
      localObservedEsdfNeedsRecenter(left, world, Point3{2.0, 40.0, 5.0}, window));
  EXPECT_TRUE(
      localObservedEsdfNeedsRecenter(left, world, Point3{13.0, 40.0, 5.0}, window));
  EXPECT_TRUE(
      localObservedEsdfNeedsRecenter(left, world, Point3{2.0, 40.0, 14.0}, window));
}

} // namespace
} // namespace drone_city_nav
