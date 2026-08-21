#include "drone_city_nav/observation_frontier.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] ObservedOccupancyGrid3D makeHalfObservedWorld() {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 24, 24, 16}};
  for (int z = 0; z < 16; ++z) {
    for (int y = 0; y < 24; ++y) {
      for (int x = 0; x <= 10; ++x) {
        static_cast<void>(
            occupancy.setState(GridIndex3D{x, y, z}, ObservedVoxelState::kFree));
      }
    }
  }
  return occupancy;
}

[[nodiscard]] SensorObservabilityConfig makeConfig() {
  SensorObservabilityConfig config;
  config.footprint.radius_m = 0.5;
  config.footprint.lower_extent_m = 0.5;
  config.footprint.upper_extent_m = 0.5;
  config.footprint.perimeter_samples = 12U;
  config.footprint.radial_rings = 2U;
  config.footprint.axial_samples = 3U;
  config.maximum_observation_range_m = 8.0;
  config.minimum_known_free_ray_m = 1.0;
  config.minimum_supporting_rays = 2U;
  config.minimum_information_gain_voxels = 4U;
  return config;
}

TEST(ObservationFrontierTest, AcceptsObservedFootprintFacingUnknownVolume) {
  const ObservedOccupancyGrid3D occupancy = makeHalfObservedWorld();
  const Point3 pose{8.5, 12.5, 8.5};
  const ObservationFrontierEvaluation evaluation =
      evaluateObservationFrontier(occupancy, pose, 17U, makeConfig());

  ASSERT_TRUE(evaluation.accepted());
  EXPECT_TRUE(evaluation.evidence.footprint_observed_free);
  EXPECT_GE(evaluation.evidence.supporting_rays, 2U);
  EXPECT_GE(evaluation.evidence.information_gain_voxels, 4U);
  EXPECT_GT(evaluation.frontier.observation_direction.x, 0.5);
  EXPECT_EQ(evaluation.frontier.supporting_map_revision, 17U);
  EXPECT_NE(evaluation.frontier.id.value, 0U);
}

TEST(ObservationFrontierTest, StableIdentityDoesNotDependOnMapRevision) {
  const ObservedOccupancyGrid3D occupancy = makeHalfObservedWorld();
  const Point3 pose{8.5, 12.5, 8.5};
  const ObservationFrontierEvaluation first =
      evaluateObservationFrontier(occupancy, pose, 17U, makeConfig());
  const ObservationFrontierEvaluation second =
      evaluateObservationFrontier(occupancy, pose, 23U, makeConfig());

  ASSERT_TRUE(first.accepted());
  ASSERT_TRUE(second.accepted());
  EXPECT_EQ(first.frontier.id, second.frontier.id);
  EXPECT_NE(first.frontier.supporting_map_revision,
            second.frontier.supporting_map_revision);
}

TEST(ObservationFrontierTest, RejectsPoseWhoseFootprintTouchesUnknownSpace) {
  const ObservedOccupancyGrid3D occupancy = makeHalfObservedWorld();
  SensorObservabilityConfig config = makeConfig();
  config.footprint.radius_m = 1.0;

  const ObservationFrontierEvaluation evaluation =
      evaluateObservationFrontier(occupancy, Point3{10.5, 12.5, 8.5}, 17U, config);

  EXPECT_FALSE(evaluation.accepted());
  EXPECT_EQ(evaluation.evidence.status,
            ObservationFrontierStatus::kFootprintNotObserved);
}

TEST(ObservationFrontierTest, RejectsFullyObservedVolumeWithoutFrontier) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 24, 24, 16}};
  for (int z = 0; z < 16; ++z) {
    for (int y = 0; y < 24; ++y) {
      for (int x = 0; x < 24; ++x) {
        static_cast<void>(
            occupancy.setState(GridIndex3D{x, y, z}, ObservedVoxelState::kFree));
      }
    }
  }

  const ObservationFrontierEvaluation evaluation = evaluateObservationFrontier(
      occupancy, Point3{12.5, 12.5, 8.5}, 17U, makeConfig());

  EXPECT_FALSE(evaluation.accepted());
  EXPECT_EQ(evaluation.evidence.status, ObservationFrontierStatus::kNoUnknownBoundary);
}

} // namespace
} // namespace drone_city_nav
