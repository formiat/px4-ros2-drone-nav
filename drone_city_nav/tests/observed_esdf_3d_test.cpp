#include "drone_city_nav/observed_esdf_3d.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

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
