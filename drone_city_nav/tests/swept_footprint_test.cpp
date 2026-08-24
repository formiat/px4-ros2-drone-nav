#include "drone_city_nav/distance_field.hpp"
#include "drone_city_nav/observed_esdf_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <limits>
#include <ranges>
#include <vector>

namespace drone_city_nav {
namespace {

TEST(SweptFootprintTest, RejectsRotorSweepWhileCenterlineRemainsFree) {
  OccupancyGrid2D occupancy{GridBounds{0.0, 0.0, 1.0, 12, 6}};
  occupancy.reset(CellState::kFree);
  occupancy.setOccupied(GridIndex{6, 3});
  const DistanceField2D field =
      DistanceField2D::build(occupancy, 20.0, DistanceFieldSource::kOccupied);
  std::vector<float> esdf;
  std::ranges::transform(field.distancesM(), std::back_inserter(esdf),
                         [](const double value) { return static_cast<float>(value); });
  const mppi::EsdfGrid grid{12, 6, 1.0F, 0.0F, 0.0F};

  const SweptFootprintResult point_mass = validateSweptFootprint(
      grid, esdf, Point3{1.5, 2.5, 0.0}, Point3{10.5, 2.5, 0.0},
      SweptFootprintConfig{.radius_m = 0.0, .perimeter_samples = 0U});
  const SweptFootprintResult physical = validateSweptFootprint(
      grid, esdf, Point3{1.5, 2.5, 0.0}, Point3{10.5, 2.5, 0.0},
      SweptFootprintConfig{
          .radius_m = 1.0, .perimeter_samples = 8U, .sweep_step_m = 0.25});

  EXPECT_TRUE(point_mass.accepted());
  EXPECT_EQ(physical.status, SweptFootprintStatus::kRawCollision);
}

TEST(SweptFootprintTest, ReportsOutsideGridSeparatelyFromPhysicalCollision) {
  const mppi::EsdfGrid grid{4, 4, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(16U, 10.0F);

  const SweptFootprintResult result = validateFootprintAt(
      grid, esdf, Point3{0.1, 2.0, 0.0},
      SweptFootprintConfig{.radius_m = 0.5, .perimeter_samples = 8U});

  EXPECT_EQ(result.status, SweptFootprintStatus::kOutsideGrid);
}

TEST(SweptFootprintTest, DetectsCollisionAboveVehicleReferencePoint) {
  const mppi::EsdfGrid grid{8, 8, 1.0F, 0.0F, 0.0F, 8, 0.0F};
  std::vector<float> esdf(std::size_t{8U} * 8U * 8U,
                          std::numeric_limits<float>::infinity());
  esdf[(4U * 8U + 3U) * 8U + 3U] = 0.0F;
  const Point3 position{3.5, 3.5, 3.5};

  EXPECT_TRUE(validateFootprintAt(
                  grid, esdf, position,
                  SweptFootprintConfig{.radius_m = 0.0, .perimeter_samples = 0U})
                  .accepted());
  EXPECT_EQ(validateFootprintAt(grid, esdf, position,
                                SweptFootprintConfig{.radius_m = 0.2,
                                                     .lower_extent_m = 0.2,
                                                     .upper_extent_m = 1.0,
                                                     .perimeter_samples = 8U,
                                                     .radial_rings = 1U,
                                                     .axial_samples = 3U})
                .status,
            SweptFootprintStatus::kRawCollision);
}

TEST(SweptFootprintTest, EsdfUnknownSampleDoesNotHideOccupiedFootprintSample) {
  const mppi::EsdfGrid grid{8, 8, 1.0F, 0.0F, 0.0F, 8, 0.0F};
  std::vector<float> esdf(std::size_t{8U} * 8U * 8U,
                          std::numeric_limits<float>::infinity());
  esdf[(3U * 8U + 3U) * 8U + 3U] = mppi::kUnknownEsdfDistanceM;
  esdf[(4U * 8U + 3U) * 8U + 3U] = 0.0F;

  const SweptFootprintResult result =
      validateFootprintAt(grid, esdf, Point3{3.5, 3.5, 3.5},
                          SweptFootprintConfig{.radius_m = 0.2,
                                               .lower_extent_m = 0.2,
                                               .upper_extent_m = 1.0,
                                               .perimeter_samples = 8U,
                                               .radial_rings = 1U,
                                               .axial_samples = 3U});

  EXPECT_EQ(result.status, SweptFootprintStatus::kRawCollision);
  EXPECT_TRUE(result.evidence.raw_collision);
  EXPECT_TRUE(result.evidence.unknown_exposure);
  EXPECT_TRUE(result.evidence.known_clearance_observed);
  EXPECT_DOUBLE_EQ(result.evidence.minimum_known_clearance_m, 0.0);
}

TEST(SweptFootprintTest, EsdfUnknownPrefixDoesNotHideLaterSegmentCollision) {
  const mppi::EsdfGrid grid{8, 4, 1.0F, 0.0F, 0.0F, 4, 0.0F};
  std::vector<float> esdf(std::size_t{8U} * 4U * 4U,
                          std::numeric_limits<float>::infinity());
  esdf[(1U * 4U + 1U) * 8U + 1U] = mppi::kUnknownEsdfDistanceM;
  esdf[(1U * 4U + 1U) * 8U + 5U] = 0.0F;

  const SweptFootprintResult result = validateSweptFootprint(
      grid, esdf, Point3{1.5, 1.5, 1.5}, Point3{5.5, 1.5, 1.5},
      SweptFootprintConfig{
          .radius_m = 0.0, .perimeter_samples = 0U, .sweep_step_m = 0.25});

  EXPECT_EQ(result.status, SweptFootprintStatus::kRawCollision);
  EXPECT_TRUE(result.evidence.raw_collision);
  EXPECT_TRUE(result.evidence.unknown_exposure);
}

TEST(SweptFootprintTest, UnknownWithoutKnownSamplesHasNoClearanceEvidence) {
  const mppi::EsdfGrid grid{4, 4, 1.0F, 0.0F, 0.0F, 4, 0.0F};
  std::vector<float> esdf(std::size_t{4U} * 4U * 4U,
                          std::numeric_limits<float>::infinity());
  esdf[(1U * 4U + 1U) * 4U + 1U] = mppi::kUnknownEsdfDistanceM;

  const SweptFootprintResult result = validateFootprintAt(
      grid, esdf, Point3{1.5, 1.5, 1.5},
      SweptFootprintConfig{.radius_m = 0.0, .perimeter_samples = 0U});

  EXPECT_EQ(result.status, SweptFootprintStatus::kUnknownSpace);
  EXPECT_TRUE(result.evidence.unknown_exposure);
  EXPECT_FALSE(result.evidence.raw_collision);
  EXPECT_FALSE(result.evidence.known_clearance_observed);
}

TEST(SweptFootprintTest, RotatesPhysicalVolumeWithBodyAxis) {
  const mppi::EsdfGrid grid{8, 8, 1.0F, 0.0F, 0.0F, 8, 0.0F};
  std::vector<float> esdf(std::size_t{8U} * 8U * 8U,
                          std::numeric_limits<float>::infinity());
  esdf[(3U * 8U + 3U) * 8U + 4U] = 0.0F;
  const SweptFootprintConfig config{.radius_m = 0.2,
                                    .lower_extent_m = 0.2,
                                    .upper_extent_m = 1.0,
                                    .perimeter_samples = 8U,
                                    .radial_rings = 1U,
                                    .axial_samples = 3U};
  const Point3 position{3.5, 3.5, 3.5};

  EXPECT_TRUE(validateFootprintAt(grid, esdf, position, FootprintBodyAxis{}, config)
                  .accepted());
  EXPECT_EQ(validateFootprintAt(grid, esdf, position, FootprintBodyAxis{1.0, 0.0, 0.0},
                                config)
                .status,
            SweptFootprintStatus::kRawCollision);
}

TEST(SweptFootprintTest, ClearanceBroadPhaseReturnsAConservativeSafeBound) {
  const mppi::EsdfGrid grid{8, 8, 1.0F, 0.0F, 0.0F, 8, 0.0F};
  const std::vector<float> esdf(std::size_t{8U} * 8U * 8U, 10.0F);
  const Point3 position{3.5, 3.5, 3.5};
  SweptFootprintConfig exact_config{.radius_m = 0.82,
                                    .lower_extent_m = 0.23,
                                    .upper_extent_m = 0.35,
                                    .perimeter_samples = 12U,
                                    .radial_rings = 2U,
                                    .axial_samples = 3U};
  SweptFootprintConfig broad_phase_config = exact_config;
  broad_phase_config.safe_clearance_threshold_m = 6.0;

  const SweptFootprintResult exact =
      validateFootprintAt(grid, esdf, position, exact_config);
  const SweptFootprintResult broad_phase =
      validateFootprintAt(grid, esdf, position, broad_phase_config);

  ASSERT_TRUE(exact.accepted());
  ASSERT_TRUE(broad_phase.accepted());
  ASSERT_TRUE(exact.evidence.known_clearance_observed);
  ASSERT_TRUE(broad_phase.evidence.known_clearance_observed);
  EXPECT_GE(broad_phase.evidence.minimum_known_clearance_m, 6.0);
  EXPECT_LE(broad_phase.evidence.minimum_known_clearance_m,
            exact.evidence.minimum_known_clearance_m);
}

TEST(SweptFootprintTest, RawTwoDimensionalSweepRejectsSideContact) {
  OccupancyGrid2D occupancy{GridBounds{0.0, 0.0, 1.0, 12, 6}};
  occupancy.reset(CellState::kFree);
  occupancy.setOccupied(GridIndex{6, 3});

  const SweptFootprintResult center = validateRawSweptFootprint(
      occupancy, Point3{1.5, 2.5, 5.0}, Point3{10.5, 2.5, 5.0},
      SweptFootprintConfig{.radius_m = 0.0, .sweep_step_m = 0.25});
  const SweptFootprintResult physical = validateRawSweptFootprint(
      occupancy, Point3{1.5, 2.5, 5.0}, Point3{10.5, 2.5, 5.0},
      SweptFootprintConfig{.radius_m = 1.0, .sweep_step_m = 0.25});

  EXPECT_TRUE(center.accepted());
  EXPECT_EQ(physical.status, SweptFootprintStatus::kRawCollision);
}

TEST(SweptFootprintTest, RawTwoDimensionalSweepHasNoSamplingScallops) {
  OccupancyGrid2D occupancy{GridBounds{0.0, 0.0, 0.05, 80, 40}};
  occupancy.reset(CellState::kFree);
  occupancy.setOccupied(GridIndex{20, 14});
  const SweptFootprintConfig footprint{.radius_m = 0.25, .sweep_step_m = 1.0};

  const SweptFootprintResult result = validateRawSweptFootprint(
      occupancy, Point3{0.5, 0.5, 2.0}, Point3{2.5, 0.5, 4.0}, footprint);

  EXPECT_EQ(result.status, SweptFootprintStatus::kRawCollision);
  EXPECT_NEAR(result.failure_point.y, 0.7, 1.0e-12);
  EXPECT_GE(result.failure_point.z, 2.0);
  EXPECT_LE(result.failure_point.z, 4.0);
}

TEST(SweptFootprintTest, RawTwoDimensionalCapsuleDoesNotInflateBeyondRadius) {
  OccupancyGrid2D occupancy{GridBounds{0.0, 0.0, 0.05, 80, 40}};
  occupancy.reset(CellState::kFree);
  occupancy.setOccupied(GridIndex{20, 16});

  EXPECT_TRUE(validateRawSweptFootprint(
                  occupancy, Point3{0.5, 0.5, 2.0}, Point3{2.5, 0.5, 4.0},
                  SweptFootprintConfig{.radius_m = 0.25, .sweep_step_m = 1.0})
                  .accepted());
}

TEST(SweptFootprintTest, RawTwoDimensionalFootprintIncludesTangentCells) {
  OccupancyGrid2D occupancy{GridBounds{0.0, 0.0, 1.0, 8, 6}};
  occupancy.reset(CellState::kFree);
  occupancy.setOccupied(GridIndex{4, 2});
  const SweptFootprintConfig footprint{.radius_m = 1.0};

  EXPECT_EQ(validateRawFootprintAt(occupancy, Point3{3.0, 2.5, 5.0}, footprint).status,
            SweptFootprintStatus::kRawCollision);
  EXPECT_TRUE(
      validateRawFootprintAt(occupancy, Point3{2.999, 2.5, 5.0}, footprint).accepted());
}

TEST(SweptFootprintTest, MiddlewareRawViewMatchesOwnedGridCollisionSemantics) {
  const GridBounds bounds{0.0, 0.0, 1.0, 8, 6};
  OccupancyGrid2D owned{bounds};
  owned.reset(CellState::kFree);
  owned.setOccupied(GridIndex{4, 3});
  std::vector<std::int8_t> middleware_cells(48U, 0);
  middleware_cells[3U * 8U + 4U] = 100;
  const RawOccupancyGridView2D view{bounds, middleware_cells, 100};
  const SweptFootprintConfig footprint{.radius_m = 0.75};

  const SweptFootprintResult owned_result =
      validateRawFootprintAt(owned, Point3{3.5, 3.5, 5.0}, footprint);
  const SweptFootprintResult view_result =
      validateRawFootprintAt(view, Point3{3.5, 3.5, 5.0}, footprint);

  EXPECT_EQ(view_result.status, owned_result.status);
  EXPECT_EQ(view_result.status, SweptFootprintStatus::kRawCollision);
  EXPECT_TRUE(
      validateRawFootprintAt(view, Point3{20.0, 20.0, 5.0}, footprint).accepted());
}

TEST(SweptFootprintTest, RawThreeDimensionalSweepUsesAxialExtent) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 10, 4, 6}};
  occupancy.setOccupied(GridIndex3D{4, 1, 3});
  const SweptFootprintConfig footprint{.radius_m = 0.25,
                                       .lower_extent_m = 0.2,
                                       .upper_extent_m = 1.2,
                                       .sweep_step_m = 0.25};

  const SweptFootprintResult result =
      validateRawSweptFootprint(occupancy, Point3{1.5, 1.5, 2.5}, FootprintBodyAxis{},
                                Point3{8.5, 1.5, 2.5}, FootprintBodyAxis{}, footprint);

  EXPECT_EQ(result.status, SweptFootprintStatus::kRawCollision);
}

TEST(SweptFootprintTest, RawThreeDimensionalBodyDoesNotRoundItsAxialCaps) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 0.25, 16, 16, 16}};
  occupancy.setOccupied(GridIndex3D{8, 8, 5});
  const SweptFootprintConfig footprint{.radius_m = 0.75,
                                       .lower_extent_m = 0.25,
                                       .upper_extent_m = 0.35,
                                       .sweep_step_m = 0.25};

  EXPECT_TRUE(validateRawFootprintAt(occupancy, Point3{2.125, 2.125, 2.0},
                                     FootprintBodyAxis{}, footprint)
                  .accepted());
}

TEST(SweptFootprintTest, NonCardinalFiniteBodyIncludesExactTangentBox) {
  const Point3 position{0.0, 0.0, 0.0};
  const FootprintBodyAxis axis{0.6, 0.8, 0.0};
  const SweptFootprintConfig footprint{
      .radius_m = 0.2, .lower_extent_m = 0.4, .upper_extent_m = 0.6};

  EXPECT_TRUE(footprintIntersectsAxisAlignedBox(
      position, axis, footprint, Point3{-0.26, 0.12, 0.0}, Point3{-0.16, 0.22, 0.1}));
}

TEST(SweptFootprintTest, NonCardinalFiniteBodyIncludesShallowPenetrationBox) {
  const Point3 position{0.0, 0.0, 0.0};
  const FootprintBodyAxis axis{0.6, 0.8, 0.0};
  const SweptFootprintConfig footprint{
      .radius_m = 0.2, .lower_extent_m = 0.4, .upper_extent_m = 0.6};
  constexpr double kPenetrationM{1.0e-11};
  const Point3 tangent_corner{-0.8 * (0.2 - kPenetrationM), 0.6 * (0.2 - kPenetrationM),
                              0.0};

  EXPECT_TRUE(footprintIntersectsAxisAlignedBox(
      position, axis, footprint, Point3{tangent_corner.x - 0.1, tangent_corner.y, 0.0},
      Point3{tangent_corner.x, tangent_corner.y + 0.1, 0.1}));
}

TEST(SweptFootprintTest, RawThreeDimensionalFootprintIncludesTangentCapCells) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 0.25, 16, 16, 16};
  const SweptFootprintConfig footprint{
      .radius_m = 0.25, .lower_extent_m = 0.25, .upper_extent_m = 0.5};

  OccupancyGrid3D upper_contact{bounds};
  upper_contact.setOccupied(GridIndex3D{8, 8, 10});
  EXPECT_EQ(validateRawFootprintAt(upper_contact, Point3{2.125, 2.125, 2.0},
                                   FootprintBodyAxis{}, footprint)
                .status,
            SweptFootprintStatus::kRawCollision);
  EXPECT_TRUE(validateRawFootprintAt(upper_contact, Point3{2.125, 2.125, 1.999},
                                     FootprintBodyAxis{}, footprint)
                  .accepted());

  OccupancyGrid3D lower_contact{bounds};
  lower_contact.setOccupied(GridIndex3D{8, 8, 9});
  EXPECT_EQ(validateRawFootprintAt(lower_contact, Point3{2.125, 2.125, 2.75},
                                   FootprintBodyAxis{}, footprint)
                .status,
            SweptFootprintStatus::kRawCollision);
  EXPECT_TRUE(validateRawFootprintAt(lower_contact, Point3{2.125, 2.125, 2.751},
                                     FootprintBodyAxis{}, footprint)
                  .accepted());
}

TEST(SweptFootprintTest, RawWorldBoundaryIsNotAnArtificialObstacle) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 4, 4, 4}};

  EXPECT_TRUE(validateRawSweptFootprint(
                  occupancy, Point3{1.5, 1.5, 1.5}, FootprintBodyAxis{},
                  Point3{8.0, 1.5, 1.5}, FootprintBodyAxis{},
                  SweptFootprintConfig{.radius_m = 0.82, .sweep_step_m = 0.25})
                  .accepted());
}

TEST(SweptFootprintTest, KnownStaticWorldRejectsPartialAndCompleteBoundsExposure) {
  const OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 4, 4, 4}};
  const SweptFootprintConfig footprint{
      .radius_m = 0.82,
      .lower_extent_m = 0.23,
      .upper_extent_m = 0.35,
      .sweep_step_m = 0.25,
  };

  EXPECT_EQ(validateKnownStaticFootprintAt(occupancy, Point3{0.5, 1.5, 1.5},
                                           FootprintBodyAxis{}, footprint)
                .status,
            SweptFootprintStatus::kOutsideGrid);
  EXPECT_EQ(validateKnownStaticSweptFootprint(
                occupancy, Point3{1.5, 1.5, 1.5}, FootprintBodyAxis{},
                Point3{8.0, 1.5, 1.5}, FootprintBodyAxis{}, footprint)
                .status,
            SweptFootprintStatus::kOutsideGrid);
}

TEST(SweptFootprintTest, RotatingStaticSweepRejectsIntermediateBoundsExposure) {
  const OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 0.1, 45, 40, 20}};
  const Point3 position{3.0, 2.0, 1.0};
  const double lateral_axis = std::sqrt(0.75);
  const FootprintBodyAxis first_axis{0.5, lateral_axis, 0.0};
  const FootprintBodyAxis second_axis{0.5, -lateral_axis, 0.0};
  const SweptFootprintConfig footprint{.radius_m = 0.1,
                                       .lower_extent_m = 2.0,
                                       .upper_extent_m = 2.0,
                                       .sweep_step_m = 0.25};

  ASSERT_TRUE(validateKnownStaticFootprintAt(occupancy, position, first_axis, footprint)
                  .accepted());
  ASSERT_TRUE(
      validateKnownStaticFootprintAt(occupancy, position, second_axis, footprint)
          .accepted());
  EXPECT_EQ(validateKnownStaticSweptFootprint(occupancy, position, first_axis, position,
                                              second_axis, footprint)
                .status,
            SweptFootprintStatus::kOutsideGrid);
}

TEST(SweptFootprintTest, RotatingRawSweepRejectsIntermediateOccupiedVoxelContact) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 0.1, 60, 40, 20}};
  occupancy.setOccupied(GridIndex3D{44, 20, 10});
  const Point3 position{3.0, 2.0, 1.0};
  const double lateral_axis = std::sqrt(0.75);
  const FootprintBodyAxis first_axis{0.5, lateral_axis, 0.0};
  const FootprintBodyAxis second_axis{0.5, -lateral_axis, 0.0};
  const SweptFootprintConfig footprint{.radius_m = 0.1,
                                       .lower_extent_m = 2.0,
                                       .upper_extent_m = 2.0,
                                       .sweep_step_m = 0.25};

  ASSERT_TRUE(
      validateRawFootprintAt(occupancy, position, first_axis, footprint).accepted());
  ASSERT_TRUE(
      validateRawFootprintAt(occupancy, position, second_axis, footprint).accepted());
  EXPECT_EQ(validateRawSweptFootprint(occupancy, position, first_axis, position,
                                      second_axis, footprint)
                .status,
            SweptFootprintStatus::kRawCollision);
}

TEST(SweptFootprintTest, AmbiguousAntipodalBodyAxisSweepFailsClosed) {
  const OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 0.25, 20, 20, 20}};

  EXPECT_EQ(validateRawSweptFootprint(
                occupancy, Point3{2.0, 2.0, 2.0}, FootprintBodyAxis{1.0, 0.0, 0.0},
                Point3{2.0, 2.0, 2.0}, FootprintBodyAxis{-1.0, 0.0, 0.0},
                SweptFootprintConfig{})
                .status,
            SweptFootprintStatus::kInvalidEsdf);
}

TEST(SweptFootprintTest, ObservedWorldRequiresEntireSweptBodyToBeKnownFree) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 10, 4, 6};
  ObservedOccupancyGrid3D occupancy{bounds};
  for (int z = 0; z < bounds.depth_cells; ++z) {
    for (int y = 0; y < bounds.height_cells; ++y) {
      for (int x = 0; x < bounds.width_cells; ++x) {
        static_cast<void>(
            occupancy.setState(GridIndex3D{x, y, z}, ObservedVoxelState::kFree));
      }
    }
  }
  const SweptFootprintConfig footprint{.radius_m = 0.25,
                                       .lower_extent_m = 0.2,
                                       .upper_extent_m = 1.2,
                                       .sweep_step_m = 0.25};

  EXPECT_TRUE(validateRawSweptFootprint(occupancy, Point3{1.5, 1.5, 2.5},
                                        FootprintBodyAxis{}, Point3{8.5, 1.5, 2.5},
                                        FootprintBodyAxis{}, footprint)
                  .accepted());

  static_cast<void>(
      occupancy.setState(GridIndex3D{4, 1, 3}, ObservedVoxelState::kUnknown));
  const SweptFootprintResult unknown =
      validateRawSweptFootprint(occupancy, Point3{1.5, 1.5, 2.5}, FootprintBodyAxis{},
                                Point3{8.5, 1.5, 2.5}, FootprintBodyAxis{}, footprint);
  EXPECT_EQ(unknown.status, SweptFootprintStatus::kUnknownSpace);

  static_cast<void>(
      occupancy.setState(GridIndex3D{4, 1, 3}, ObservedVoxelState::kOccupied));
  EXPECT_EQ(validateRawSweptFootprint(occupancy, Point3{1.5, 1.5, 2.5},
                                      FootprintBodyAxis{}, Point3{8.5, 1.5, 2.5},
                                      FootprintBodyAxis{}, footprint)
                .status,
            SweptFootprintStatus::kRawCollision);
}

TEST(SweptFootprintTest, RawOccupiedQueriesAllowUnknownButStillRejectCollision) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 0.25, 40, 16, 16}};
  const SweptFootprintConfig footprint{.radius_m = 0.2,
                                       .lower_extent_m = 0.2,
                                       .upper_extent_m = 0.2,
                                       .sweep_step_m = 0.125};

  EXPECT_TRUE(rawOccupiedFootprintIsClearAt(occupancy, Point3{2.0, 2.125, 2.125},
                                            FootprintBodyAxis{}, footprint));
  EXPECT_TRUE(rawOccupiedSweptFootprintIsClear(
      occupancy, Point3{1.0, 2.125, 2.125}, FootprintBodyAxis{},
      Point3{8.0, 2.125, 2.125}, FootprintBodyAxis{}, footprint));

  static_cast<void>(
      occupancy.setState(GridIndex3D{20, 8, 8}, ObservedVoxelState::kOccupied));

  EXPECT_FALSE(rawOccupiedSweptFootprintIsClear(
      occupancy, Point3{1.0, 2.125, 2.125}, FootprintBodyAxis{},
      Point3{8.0, 2.125, 2.125}, FootprintBodyAxis{}, footprint));
}

TEST(SweptFootprintTest, ObservedSpacePolicySeparatesUnknownFromRawCollision) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 0.25, 40, 16, 16}};
  const SweptFootprintConfig footprint{.radius_m = 0.2,
                                       .lower_extent_m = 0.2,
                                       .upper_extent_m = 0.2,
                                       .sweep_step_m = 0.125};
  const Point3 first{1.0, 2.125, 2.125};
  const Point3 second{8.0, 2.125, 2.125};

  EXPECT_EQ(validateObservedSweptFootprint(
                occupancy, first, FootprintBodyAxis{}, second, FootprintBodyAxis{},
                footprint, ObservedSpaceValidationPolicy::kRequireKnownFree)
                .status,
            SweptFootprintStatus::kUnknownSpace);
  EXPECT_TRUE(validateObservedSweptFootprint(
                  occupancy, first, FootprintBodyAxis{}, second, FootprintBodyAxis{},
                  footprint, ObservedSpaceValidationPolicy::kAllowUnknown)
                  .accepted());

  static_cast<void>(
      occupancy.setState(GridIndex3D{20, 8, 8}, ObservedVoxelState::kOccupied));
  EXPECT_EQ(validateObservedSweptFootprint(occupancy, first, FootprintBodyAxis{},
                                           second, FootprintBodyAxis{}, footprint,
                                           ObservedSpaceValidationPolicy::kAllowUnknown)
                .status,
            SweptFootprintStatus::kRawCollision);
}

TEST(SweptFootprintTest, ObservedWorldIgnoresUnknownCellsBeyondAxialCaps) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 0.25, 16, 16, 16};
  ObservedOccupancyGrid3D occupancy{bounds};
  for (int z = 0; z < bounds.depth_cells; ++z) {
    for (int y = 0; y < bounds.height_cells; ++y) {
      for (int x = 0; x < bounds.width_cells; ++x) {
        static_cast<void>(
            occupancy.setState(GridIndex3D{x, y, z}, ObservedVoxelState::kFree));
      }
    }
  }
  static_cast<void>(
      occupancy.setState(GridIndex3D{8, 8, 5}, ObservedVoxelState::kUnknown));

  EXPECT_TRUE(validateRawFootprintAt(occupancy, Point3{2.125, 2.125, 2.0},
                                     FootprintBodyAxis{},
                                     SweptFootprintConfig{.radius_m = 0.75,
                                                          .lower_extent_m = 0.25,
                                                          .upper_extent_m = 0.35,
                                                          .sweep_step_m = 0.25})
                  .accepted());
}

TEST(SweptFootprintTest, ObservedWorldReportsCollisionBeforeUnknownSpace) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 0.25, 16, 16, 16};
  ObservedOccupancyGrid3D occupancy{bounds};
  for (int z = 0; z < bounds.depth_cells; ++z) {
    for (int y = 0; y < bounds.height_cells; ++y) {
      for (int x = 0; x < bounds.width_cells; ++x) {
        static_cast<void>(
            occupancy.setState(GridIndex3D{x, y, z}, ObservedVoxelState::kFree));
      }
    }
  }
  static_cast<void>(
      occupancy.setState(GridIndex3D{7, 7, 7}, ObservedVoxelState::kUnknown));
  static_cast<void>(
      occupancy.setState(GridIndex3D{8, 8, 8}, ObservedVoxelState::kOccupied));

  const SweptFootprintResult result =
      validateRawFootprintAt(occupancy, Point3{2.125, 2.125, 2.0}, FootprintBodyAxis{},
                             SweptFootprintConfig{.radius_m = 0.75,
                                                  .lower_extent_m = 0.25,
                                                  .upper_extent_m = 0.35,
                                                  .sweep_step_m = 0.25});

  EXPECT_EQ(result.status, SweptFootprintStatus::kRawCollision);
  EXPECT_TRUE(result.evidence.raw_collision);
  EXPECT_TRUE(result.evidence.unknown_exposure);
}

TEST(SweptFootprintTest, ObservedWorldReportsLaterCollisionAfterUnknownSweepPrefix) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 0.25, 40, 16, 16};
  ObservedOccupancyGrid3D occupancy{bounds};
  for (int z = 0; z < bounds.depth_cells; ++z) {
    for (int y = 0; y < bounds.height_cells; ++y) {
      for (int x = 0; x < bounds.width_cells; ++x) {
        static_cast<void>(
            occupancy.setState(GridIndex3D{x, y, z}, ObservedVoxelState::kFree));
      }
    }
  }
  static_cast<void>(
      occupancy.setState(GridIndex3D{10, 8, 8}, ObservedVoxelState::kUnknown));
  static_cast<void>(
      occupancy.setState(GridIndex3D{30, 8, 8}, ObservedVoxelState::kOccupied));

  const SweptFootprintResult result = validateRawSweptFootprint(
      occupancy, Point3{1.0, 2.125, 2.125}, FootprintBodyAxis{},
      Point3{8.0, 2.125, 2.125}, FootprintBodyAxis{},
      SweptFootprintConfig{.radius_m = 0.2,
                           .lower_extent_m = 0.2,
                           .upper_extent_m = 0.2,
                           .sweep_step_m = 0.125});

  EXPECT_EQ(result.status, SweptFootprintStatus::kRawCollision);
  EXPECT_TRUE(result.evidence.raw_collision);
  EXPECT_TRUE(result.evidence.unknown_exposure);
}

TEST(SweptFootprintTest, ObservedWorldBoundaryIsOutsideGridRatherThanUnknown) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 4, 4, 4};
  ObservedOccupancyGrid3D occupancy{bounds};
  for (int z = 0; z < bounds.depth_cells; ++z) {
    for (int y = 0; y < bounds.height_cells; ++y) {
      for (int x = 0; x < bounds.width_cells; ++x) {
        static_cast<void>(
            occupancy.setState(GridIndex3D{x, y, z}, ObservedVoxelState::kFree));
      }
    }
  }

  const SweptFootprintResult result = validateRawSweptFootprint(
      occupancy, Point3{1.5, 1.5, 1.5}, FootprintBodyAxis{}, Point3{8.0, 1.5, 1.5},
      FootprintBodyAxis{},
      SweptFootprintConfig{.radius_m = 0.82, .sweep_step_m = 0.25});

  EXPECT_EQ(result.status, SweptFootprintStatus::kOutsideGrid);
}

TEST(SweptFootprintTest, ExactObservedWorldBoundaryContactRemainsInsideGrid) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 4, 4, 4};
  ObservedOccupancyGrid3D occupancy{bounds};
  for (int z = 0; z < bounds.depth_cells; ++z) {
    for (int y = 0; y < bounds.height_cells; ++y) {
      for (int x = 0; x < bounds.width_cells; ++x) {
        static_cast<void>(
            occupancy.setState(GridIndex3D{x, y, z}, ObservedVoxelState::kFree));
      }
    }
  }

  EXPECT_TRUE(validateRawFootprintAt(
                  occupancy, Point3{0.5, 0.5, 0.5}, FootprintBodyAxis{},
                  SweptFootprintConfig{
                      .radius_m = 0.5, .lower_extent_m = 0.5, .upper_extent_m = 0.5})
                  .accepted());
}

TEST(SweptFootprintTest, ProprioceptiveSeedCoversOnlyTheAlreadyOccupiedBodyVolume) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 0.25, 20, 20, 20};
  ObservedOccupancyGrid3D occupancy{bounds};
  const SweptFootprintConfig footprint{.radius_m = 0.5,
                                       .lower_extent_m = 0.25,
                                       .upper_extent_m = 0.25,
                                       .sweep_step_m = 0.125};
  const ProprioceptiveFreeSpaceSeed3D seed{
      .position = Point3{2.0, 2.0, 2.0},
      .body_axis = FootprintBodyAxis{},
      .footprint = footprint,
  };

  EXPECT_TRUE(
      validateRawFootprintAt(occupancy, seed.position, seed.body_axis, footprint, &seed)
          .accepted());
  EXPECT_EQ(validateRawFootprintAt(occupancy, Point3{2.25, 2.0, 2.0},
                                   FootprintBodyAxis{}, footprint, &seed)
                .status,
            SweptFootprintStatus::kUnknownSpace);
  EXPECT_EQ(validateRawFootprintAt(occupancy, Point3{2.0, 2.0, 1.75},
                                   FootprintBodyAxis{}, footprint, &seed)
                .status,
            SweptFootprintStatus::kUnknownSpace);
}

TEST(SweptFootprintTest,
     ProprioceptiveSeedAllowsObservedAxialDepartureButNotUnknownDescent) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 0.25, 20, 20, 20};
  ObservedOccupancyGrid3D occupancy{bounds};
  for (int z = 9; z <= 10; ++z) {
    for (int y = 0; y < bounds.height_cells; ++y) {
      for (int x = 0; x < bounds.width_cells; ++x) {
        static_cast<void>(
            occupancy.setState(GridIndex3D{x, y, z}, ObservedVoxelState::kFree));
      }
    }
  }
  const SweptFootprintConfig footprint{.radius_m = 0.5,
                                       .lower_extent_m = 0.25,
                                       .upper_extent_m = 0.25,
                                       .sweep_step_m = 0.125};
  const ProprioceptiveFreeSpaceSeed3D seed{
      .position = Point3{2.0, 2.0, 2.0},
      .body_axis = FootprintBodyAxis{},
      .footprint = footprint,
  };

  ASSERT_TRUE(validateRawFootprintAt(occupancy, Point3{2.0, 2.0, 2.25},
                                     FootprintBodyAxis{}, footprint, &seed)
                  .accepted());
  EXPECT_TRUE(validateRawSweptFootprint(occupancy, seed.position, seed.body_axis,
                                        Point3{2.0, 2.0, 2.25}, FootprintBodyAxis{},
                                        footprint, &seed)
                  .accepted());
  EXPECT_EQ(validateRawSweptFootprint(occupancy, seed.position, seed.body_axis,
                                      Point3{2.0, 2.0, 1.75}, FootprintBodyAxis{},
                                      footprint, &seed)
                .status,
            SweptFootprintStatus::kUnknownSpace);
}

TEST(SweptFootprintTest, OccupiedEvidenceOverridesProprioceptiveSeed) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 0.25, 20, 20, 20};
  ObservedOccupancyGrid3D occupancy{bounds};
  const SweptFootprintConfig footprint{
      .radius_m = 0.5, .lower_extent_m = 0.25, .upper_extent_m = 0.25};
  const ProprioceptiveFreeSpaceSeed3D seed{
      .position = Point3{2.0, 2.0, 2.0},
      .body_axis = FootprintBodyAxis{},
      .footprint = footprint,
  };
  static_cast<void>(
      occupancy.setState(GridIndex3D{8, 8, 8}, ObservedVoxelState::kOccupied));

  EXPECT_EQ(
      validateRawFootprintAt(occupancy, seed.position, seed.body_axis, footprint, &seed)
          .status,
      SweptFootprintStatus::kRawCollision);
}

TEST(SweptFootprintTest, AnchoredLaunchSupportAllowsBoundedNonDescendingDeparture) {
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
  const SweptFootprintConfig footprint{.radius_m = 0.5,
                                       .lower_extent_m = 0.25,
                                       .upper_extent_m = 0.25,
                                       .sweep_step_m = 0.125};
  const ProprioceptiveFreeSpaceSeed3D seed{
      .position = Point3{2.0, 2.0, 2.0},
      .body_axis = FootprintBodyAxis{},
      .footprint = footprint,
  };
  for (int y = 0; y < bounds.height_cells; ++y) {
    for (int x = 0; x < bounds.width_cells; ++x) {
      static_cast<void>(
          occupancy.setState(GridIndex3D{x, y, 7}, ObservedVoxelState::kOccupied));
    }
  }
  std::optional<LaunchSupportContact3D> support =
      detectLaunchSupportContact3D(occupancy, seed);
  ASSERT_TRUE(support.has_value());
  EXPECT_EQ(support->evidence_source, LaunchSupportEvidenceSource::kObservedOccupancy);
  EXPECT_DOUBLE_EQ(support->maximum_lateral_departure_m, bounds.resolution_m);
  EXPECT_DOUBLE_EQ(support->maximum_axial_settling_m, bounds.resolution_m);

  EXPECT_TRUE(validateRawFootprintAt(occupancy, seed.position, seed.body_axis,
                                     footprint, &seed, &*support)
                  .accepted());
  const SweptFootprintResult departure = validateRawSweptFootprint(
      occupancy, seed.position, seed.body_axis, Point3{2.1, 2.0, 2.25}, seed.body_axis,
      footprint, &seed, &*support);
  EXPECT_TRUE(departure.accepted())
      << "status=" << static_cast<int>(departure.status) << " failure=("
      << departure.failure_point.x << ',' << departure.failure_point.y << ','
      << departure.failure_point.z << ')';
  EXPECT_TRUE(updateLaunchSupportSettling(*support, Point3{2.0, 2.0, 1.95}));
  EXPECT_TRUE(validateRawFootprintAt(occupancy, Point3{2.0, 2.0, 1.95}, seed.body_axis,
                                     footprint, nullptr, &*support)
                  .accepted());
  EXPECT_EQ(validateRawFootprintAt(occupancy, Point3{2.0, 2.0, 1.90}, seed.body_axis,
                                   footprint, nullptr, &*support)
                .status,
            SweptFootprintStatus::kRawCollision);
  EXPECT_FALSE(updateLaunchSupportSettling(*support, Point3{2.0, 2.0, 1.50}));
  EXPECT_EQ(validateRawSweptFootprint(occupancy, seed.position, seed.body_axis,
                                      Point3{2.0, 2.0, 1.75}, seed.body_axis, footprint,
                                      &seed, &*support)
                .status,
            SweptFootprintStatus::kRawCollision);
  EXPECT_EQ(validateRawSweptFootprint(occupancy, seed.position, seed.body_axis,
                                      Point3{2.75, 2.0, 2.0}, seed.body_axis, footprint,
                                      &seed, &*support)
                .status,
            SweptFootprintStatus::kRawCollision);
}

TEST(SweptFootprintTest, ProprioceptiveSeedSupportsArbitraryBodyAxis) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 0.25, 20, 20, 20};
  ObservedOccupancyGrid3D occupancy{bounds};
  const SweptFootprintConfig footprint{
      .radius_m = 0.25, .lower_extent_m = 0.5, .upper_extent_m = 0.75};
  const ProprioceptiveFreeSpaceSeed3D seed{
      .position = Point3{2.0, 2.0, 2.0},
      .body_axis = FootprintBodyAxis{1.0, 1.0, 0.0},
      .footprint = footprint,
  };

  EXPECT_TRUE(
      validateRawFootprintAt(occupancy, seed.position, seed.body_axis, footprint, &seed)
          .accepted());
}

TEST(SweptFootprintTest, RawPointCloudSweepRejectsPhysicalSideContact) {
  const std::vector<Point3> obstacle_points{{5.0, 0.75, 5.0}};
  const SweptFootprintConfig footprint{.radius_m = 0.82,
                                       .lower_extent_m = 0.23,
                                       .upper_extent_m = 0.35,
                                       .sweep_step_m = 0.25};

  const SweptFootprintResult result = validateRawPointCloudSweptFootprint(
      obstacle_points, Point3{0.0, 0.0, 5.0}, FootprintBodyAxis{},
      Point3{10.0, 0.0, 5.0}, FootprintBodyAxis{}, footprint);

  EXPECT_EQ(result.status, SweptFootprintStatus::kRawCollision);
  EXPECT_DOUBLE_EQ(result.failure_point.x, obstacle_points.front().x);
  EXPECT_DOUBLE_EQ(result.failure_point.y, obstacle_points.front().y);
}

TEST(SweptFootprintTest, RotatingRawPointCloudSweepRejectsIntermediateContact) {
  const std::vector<Point3> obstacle_points{{4.5, 2.0, 1.0}};
  const Point3 position{3.0, 2.0, 1.0};
  const double lateral_axis = std::sqrt(0.75);
  const FootprintBodyAxis first_axis{0.5, lateral_axis, 0.0};
  const FootprintBodyAxis second_axis{0.5, -lateral_axis, 0.0};
  const SweptFootprintConfig footprint{.radius_m = 0.1,
                                       .lower_extent_m = 2.0,
                                       .upper_extent_m = 2.0,
                                       .sweep_step_m = 0.25};

  ASSERT_TRUE(
      validateRawPointCloudFootprintAt(obstacle_points, position, first_axis, footprint)
          .accepted());
  ASSERT_TRUE(validateRawPointCloudFootprintAt(obstacle_points, position, second_axis,
                                               footprint)
                  .accepted());
  EXPECT_EQ(validateRawPointCloudSweptFootprint(obstacle_points, position, first_axis,
                                                position, second_axis, footprint)
                .status,
            SweptFootprintStatus::kRawCollision);
}

TEST(SweptFootprintTest,
     RawPointCloudLaunchSupportMatchesPersistentSupportCellContract) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 0.25, 20, 20, 20};
  ObservedOccupancyGrid3D occupancy{bounds};
  const SweptFootprintConfig footprint{.radius_m = 0.5,
                                       .lower_extent_m = 0.25,
                                       .upper_extent_m = 0.25,
                                       .sweep_step_m = 0.125};
  const ProprioceptiveFreeSpaceSeed3D seed{
      .position = Point3{2.0, 2.0, 2.0},
      .body_axis = FootprintBodyAxis{},
      .footprint = footprint,
  };
  static_cast<void>(
      occupancy.setState(GridIndex3D{10, 8, 7}, ObservedVoxelState::kOccupied));
  const std::optional<LaunchSupportContact3D> support =
      detectLaunchSupportContact3D(occupancy, seed);
  ASSERT_TRUE(support.has_value());

  const std::vector<Point3> support_return{{2.6, 2.0, 1.875}};
  ASSERT_TRUE(validateRawPointCloudFootprintAt(support_return, seed.position,
                                               seed.body_axis, footprint, &*support)
                  .accepted());
  ASSERT_EQ(validateRawPointCloudFootprintAt(support_return, Point3{2.3, 2.0, 2.0},
                                             seed.body_axis, footprint, &*support)
                .status,
            SweptFootprintStatus::kRawCollision);
  EXPECT_TRUE(validateRawPointCloudSweptFootprint(
                  support_return, seed.position, seed.body_axis, Point3{2.2, 2.0, 2.25},
                  seed.body_axis, footprint, &*support)
                  .accepted());
  EXPECT_EQ(validateRawPointCloudSweptFootprint(support_return, seed.position,
                                                seed.body_axis, Point3{2.3, 2.0, 2.0},
                                                seed.body_axis, footprint, &*support)
                .status,
            SweptFootprintStatus::kRawCollision);
}

TEST(SweptFootprintTest, RawPointCloudFootprintUsesRequestedBodyAxis) {
  const std::vector<Point3> obstacle_points{{0.8, 0.0, 0.0}};
  const SweptFootprintConfig footprint{
      .radius_m = 0.2, .lower_extent_m = 0.2, .upper_extent_m = 1.0};

  EXPECT_TRUE(validateRawPointCloudFootprintAt(obstacle_points, Point3{},
                                               FootprintBodyAxis{}, footprint)
                  .accepted());
  EXPECT_EQ(validateRawPointCloudFootprintAt(
                obstacle_points, Point3{}, FootprintBodyAxis{1.0, 0.0, 0.0}, footprint)
                .status,
            SweptFootprintStatus::kRawCollision);
}

} // namespace
} // namespace drone_city_nav
