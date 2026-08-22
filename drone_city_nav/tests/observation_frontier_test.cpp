#include "drone_city_nav/observation_frontier.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <unordered_set>

namespace drone_city_nav {
namespace {

constexpr auto kStrictValidation = ObservedSpaceValidationPolicy::kRequireKnownFree;
constexpr auto kPermissiveValidation = ObservedSpaceValidationPolicy::kAllowUnknown;

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

[[nodiscard]] ObservedOccupancyGrid3D makeOppositeHalfObservedWorld() {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 24, 24, 16}};
  for (int z = 0; z < 16; ++z) {
    for (int y = 0; y < 24; ++y) {
      for (int x = 10; x < 24; ++x) {
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
  const ObservationFrontierEvaluation evaluation = evaluateObservationFrontier(
      occupancy, pose, 17U, makeConfig(), kStrictValidation);

  ASSERT_TRUE(evaluation.accepted());
  EXPECT_TRUE(evaluation.evidence.footprint_validation_accepted);
  EXPECT_GE(evaluation.evidence.supporting_rays, 2U);
  EXPECT_GE(evaluation.evidence.information_gain_voxels, 4U);
  EXPECT_GE(evaluation.evidence.information_gain_voxels,
            evaluation.evidence.required_information_gain_voxels);
  EXPECT_GT(evaluation.frontier.observation_direction.x, 0.5);
  EXPECT_GT(evaluation.frontier.observation_pose.x, pose.x);
  EXPECT_LT(evaluation.frontier.observation_pose.x,
            evaluation.frontier.boundary_centroid.x);
  EXPECT_TRUE(validateRawSweptFootprint(occupancy, pose, FootprintBodyAxis{},
                                        evaluation.frontier.observation_pose,
                                        FootprintBodyAxis{}, makeConfig().footprint)
                  .accepted());
  EXPECT_EQ(evaluation.frontier.supporting_map_revision, 17U);
  EXPECT_NE(evaluation.frontier.id.value, 0U);
}

TEST(ObservationFrontierTest,
     RejectsUnknownSpeckleSmallerThanThePhysicalFootprintProjection) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 0.25, 48, 48, 32}};
  for (int z = 0; z < 32; ++z) {
    for (int y = 0; y < 48; ++y) {
      for (int x = 0; x < 48; ++x) {
        static_cast<void>(occupancy.setState({x, y, z}, ObservedVoxelState::kOccupied));
      }
    }
  }
  for (int z = 10; z <= 18; ++z) {
    for (int y = 12; y <= 36; ++y) {
      for (int x = 4; x <= 28; ++x) {
        static_cast<void>(occupancy.setState({x, y, z}, ObservedVoxelState::kFree));
      }
    }
  }
  for (int x = 29; x <= 32; ++x) {
    static_cast<void>(occupancy.setState({x, 24, 14}, ObservedVoxelState::kUnknown));
    static_cast<void>(occupancy.setState({x, 25, 14}, ObservedVoxelState::kUnknown));
  }
  SensorObservabilityConfig config = makeConfig();
  config.footprint.radius_m = 0.82;
  config.footprint.lower_extent_m = 0.23;
  config.footprint.upper_extent_m = 0.35;
  config.minimum_information_gain_voxels = 1U;
  config.minimum_supporting_rays = 1U;
  config.maximum_observation_range_m = 4.0;

  const ObservationFrontierEvaluation evaluation = evaluateObservationFrontier(
      occupancy, Point3{5.625, 6.125, 3.625}, 17U, config, kStrictValidation);

  EXPECT_FALSE(evaluation.accepted());
  EXPECT_EQ(evaluation.evidence.status,
            ObservationFrontierStatus::kInsufficientInformationGain);
  EXPECT_GT(evaluation.evidence.required_information_gain_voxels,
            evaluation.evidence.information_gain_voxels);
}

TEST(ObservationFrontierTest, StableIdentityDoesNotDependOnMapRevision) {
  const ObservedOccupancyGrid3D occupancy = makeHalfObservedWorld();
  const Point3 pose{8.5, 12.5, 8.5};
  const ObservationFrontierEvaluation first = evaluateObservationFrontier(
      occupancy, pose, 17U, makeConfig(), kStrictValidation);
  const ObservationFrontierEvaluation second = evaluateObservationFrontier(
      occupancy, pose, 23U, makeConfig(), kStrictValidation);

  ASSERT_TRUE(first.accepted());
  ASSERT_TRUE(second.accepted());
  EXPECT_EQ(first.frontier.id, second.frontier.id);
  EXPECT_NE(first.frontier.supporting_map_revision,
            second.frontier.supporting_map_revision);
}

TEST(ObservationFrontierTest, NearbyPosesShareRegionalIdentity) {
  const ObservedOccupancyGrid3D occupancy = makeHalfObservedWorld();
  const SensorObservabilityConfig config = makeConfig();
  const ObservationFrontierEvaluation first = evaluateObservationFrontier(
      occupancy, Point3{8.5, 12.5, 8.5}, 17U, config, kStrictValidation);
  const ObservationFrontierEvaluation nearby = evaluateObservationFrontier(
      occupancy, Point3{8.5, 14.5, 8.5}, 17U, config, kStrictValidation);
  const ObservationFrontierEvaluation separate = evaluateObservationFrontier(
      occupancy, Point3{8.5, 18.5, 8.5}, 17U, config, kStrictValidation);

  ASSERT_TRUE(first.accepted());
  ASSERT_TRUE(nearby.accepted());
  ASSERT_TRUE(separate.accepted());
  EXPECT_EQ(first.frontier.id, nearby.frontier.id);
  EXPECT_NE(first.frontier.id, separate.frontier.id);
  EXPECT_GT(first.frontier.boundary_centroid.x, first.frontier.observation_pose.x);
}

TEST(ObservationFrontierTest, RegionalIdentityDoesNotDependOnApproachDirection) {
  SensorObservabilityConfig config = makeConfig();
  config.frontier_identity_resolution_m = 100.0;
  const ObservationFrontierEvaluation from_left = evaluateObservationFrontier(
      makeHalfObservedWorld(), Point3{8.5, 12.5, 8.5}, 17U, config, kStrictValidation);
  const ObservationFrontierEvaluation from_right = evaluateObservationFrontier(
      makeOppositeHalfObservedWorld(), Point3{12.5, 12.5, 8.5}, 17U, config,
      kStrictValidation);

  ASSERT_TRUE(from_left.accepted());
  ASSERT_TRUE(from_right.accepted());
  EXPECT_GT(from_left.frontier.observation_direction.x, 0.5);
  EXPECT_LT(from_right.frontier.observation_direction.x, -0.5);
  EXPECT_EQ(from_left.frontier.id, from_right.frontier.id);
}

TEST(ObservationFrontierTest, EvidenceRepresentsOneCoherentAngularSector) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 24, 24, 16}};
  for (int z = 3; z <= 12; ++z) {
    for (int y = 4; y <= 19; ++y) {
      for (int x = 4; x <= 19; ++x) {
        static_cast<void>(
            occupancy.setState(GridIndex3D{x, y, z}, ObservedVoxelState::kFree));
      }
    }
  }

  const ObservationFrontierEvaluation evaluation = evaluateObservationFrontier(
      occupancy, Point3{12.5, 12.5, 8.5}, 17U, makeConfig(), kStrictValidation);

  ASSERT_TRUE(evaluation.accepted());
  const Vec3 direction = evaluation.frontier.observation_direction;
  EXPECT_GT(std::hypot(std::hypot(direction.x, direction.y), direction.z), 0.99);
  EXPECT_LT(evaluation.frontier.supporting_rays, evaluation.evidence.tested_rays);
}

TEST(ObservationFrontierTest, ReturnsIndependentFrontiersForDistinctUnknownSectors) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 24, 24, 16}};
  for (int z = 5; z <= 11; ++z) {
    for (int y = 9; y <= 15; ++y) {
      for (int x = 9; x <= 15; ++x) {
        static_cast<void>(
            occupancy.setState(GridIndex3D{x, y, z}, ObservedVoxelState::kFree));
      }
    }
  }
  SensorObservabilityConfig config = makeConfig();
  config.frontier_identity_resolution_m = 2.0;

  const ObservationFrontierSetEvaluation evaluation = evaluateObservationFrontiers(
      occupancy, Point3{12.5, 12.5, 8.5}, 17U, config, kStrictValidation);

  ASSERT_GT(evaluation.frontiers.size(), 1U);
  std::ostringstream directions;
  for (const ObservationFrontier& frontier : evaluation.frontiers) {
    directions << " (" << frontier.observation_direction.x << ','
               << frontier.observation_direction.y << ','
               << frontier.observation_direction.z << ')';
  }
  EXPECT_TRUE(std::ranges::any_of(evaluation.frontiers, [](const auto& frontier) {
    return std::abs(frontier.observation_direction.z) > 0.5;
  })) << directions.str();
  EXPECT_TRUE(std::ranges::any_of(evaluation.frontiers, [](const auto& frontier) {
    return std::hypot(frontier.observation_direction.x,
                      frontier.observation_direction.y) > 0.8;
  })) << directions.str();
  std::unordered_set<std::uint64_t> ids;
  for (const ObservationFrontier& frontier : evaluation.frontiers) {
    EXPECT_TRUE(ids.insert(frontier.id.value).second);
  }
}

TEST(ObservationFrontierTest, DiscoveryDeduplicatesObservationPosesForOneBoundary) {
  ObservedOccupancyGrid3D occupancy = makeHalfObservedWorld();
  const ObservationFrontierDiscovery discovery = discoverObservationFrontiers(
      occupancy, 17U, makeConfig(), kStrictValidation, 1U, 1024U);

  ASSERT_FALSE(discovery.frontiers.empty());
  std::unordered_set<std::uint64_t> ids;
  for (const ObservationFrontier& frontier : discovery.frontiers) {
    EXPECT_TRUE(ids.insert(frontier.id.value).second);
  }
}

TEST(ObservationFrontierTest, RejectsPoseWhoseFootprintTouchesUnknownSpace) {
  const ObservedOccupancyGrid3D occupancy = makeHalfObservedWorld();
  SensorObservabilityConfig config = makeConfig();
  config.footprint.radius_m = 1.0;

  const ObservationFrontierEvaluation evaluation = evaluateObservationFrontier(
      occupancy, Point3{10.5, 12.5, 8.5}, 17U, config, kStrictValidation);

  EXPECT_FALSE(evaluation.accepted());
  EXPECT_EQ(evaluation.evidence.status,
            ObservationFrontierStatus::kFootprintNotObserved);
}

TEST(ObservationFrontierTest,
     PermissiveValidationAllowsUnknownFootprintButNeverRawOccupied) {
  ObservedOccupancyGrid3D occupancy = makeHalfObservedWorld();
  SensorObservabilityConfig config = makeConfig();
  config.footprint.radius_m = 1.0;
  const Point3 pose{10.5, 12.5, 8.5};

  const ObservationFrontierEvaluation permissive =
      evaluateObservationFrontier(occupancy, pose, 17U, config, kPermissiveValidation);
  ASSERT_TRUE(permissive.accepted());
  EXPECT_TRUE(permissive.evidence.footprint_validation_accepted);

  ASSERT_TRUE(occupancy.setState({10, 12, 8}, ObservedVoxelState::kOccupied));
  const ObservationFrontierEvaluation occupied =
      evaluateObservationFrontier(occupancy, pose, 18U, config, kPermissiveValidation);
  EXPECT_FALSE(occupied.accepted());
  EXPECT_FALSE(occupied.evidence.footprint_validation_accepted);
  EXPECT_EQ(occupied.evidence.status, ObservationFrontierStatus::kRawCollision);
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
      occupancy, Point3{12.5, 12.5, 8.5}, 17U, makeConfig(), kStrictValidation);

  EXPECT_FALSE(evaluation.accepted());
  EXPECT_EQ(evaluation.evidence.status, ObservationFrontierStatus::kNoUnknownBoundary);
}

TEST(ObservationFrontierTest, BudgetedDiscoveryIsDeterministicAndReportsCoverage) {
  const ObservedOccupancyGrid3D occupancy = makeHalfObservedWorld();

  const ObservationFrontierDiscovery first = discoverObservationFrontiers(
      occupancy, 17U, makeConfig(), kStrictValidation, 1U, 2U);
  const ObservationFrontierDiscovery second = discoverObservationFrontiers(
      occupancy, 17U, makeConfig(), kStrictValidation, 1U, 2U);

  EXPECT_GT(first.sampled_free_voxels, 0U);
  EXPECT_GT(first.boundary_candidates, first.evaluated_candidates);
  EXPECT_EQ(first.evaluated_candidates, 2U);
  EXPECT_NE(first.evaluation_sample_fingerprint, 0U);
  EXPECT_EQ(first.evaluation_sample_fingerprint, second.evaluation_sample_fingerprint);
  EXPECT_TRUE(first.evaluation_budget_exhausted);
  ASSERT_EQ(first.frontiers.size(), second.frontiers.size());
  for (std::size_t index = 0U; index < first.frontiers.size(); ++index) {
    EXPECT_EQ(first.frontiers[index].id, second.frontiers[index].id);
  }
}

TEST(ObservationFrontierTest, BudgetedDiscoveryRotatesItsSampleAcrossRevisions) {
  const ObservedOccupancyGrid3D occupancy = makeHalfObservedWorld();

  const ObservationFrontierDiscovery first = discoverObservationFrontiers(
      occupancy, 17U, makeConfig(), kStrictValidation, 1U, 2U);
  const ObservationFrontierDiscovery next = discoverObservationFrontiers(
      occupancy, 18U, makeConfig(), kStrictValidation, 1U, 2U);

  ASSERT_EQ(first.evaluated_candidates, 2U);
  ASSERT_EQ(next.evaluated_candidates, 2U);
  EXPECT_NE(first.evaluation_sample_fingerprint, next.evaluation_sample_fingerprint);
}

} // namespace
} // namespace drone_city_nav
