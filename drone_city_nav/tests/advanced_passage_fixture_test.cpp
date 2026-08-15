#include "drone_city_nav/distance_field_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <vector>

#include "advanced_passage_fixture.hpp"

namespace drone_city_nav::test {
namespace {

[[nodiscard]] GridIndex3D requiredCell(const OccupancyGrid3D& occupancy,
                                       const Point3& point) {
  const std::optional<GridIndex3D> cell = occupancy.worldToCell(point);
  if (!cell.has_value()) {
    throw std::runtime_error{"advanced passage fixture point is outside the grid"};
  }
  return *cell;
}

[[nodiscard]] double clearanceAt(const DistanceField3D& field,
                                 const OccupancyGrid3D& occupancy,
                                 const Point3& point) {
  return static_cast<double>(field.distanceAt(requiredCell(occupancy, point)));
}

TEST(AdvancedPassageFixture, DefinesAllRequiredCasesWithUniqueFingerprints) {
  const std::vector<AdvancedPassageFixtureKind> kinds = advancedPassageFixtureKinds();
  ASSERT_EQ(kinds.size(), 7U);

  std::vector<std::uint64_t> fingerprints;
  for (const AdvancedPassageFixtureKind kind : kinds) {
    const AdvancedPassageFixture fixture = buildAdvancedPassageFixture(kind);
    EXPECT_FALSE(fixture.name.empty());
    EXPECT_GT(fixture.occupancy.occupiedVoxelCount(), 0U);
    EXPECT_FALSE(fixture.expectation.open_space_seeds.empty());
    EXPECT_FALSE(fixture.expectation.raw_safe_reference_paths.empty());
    fingerprints.push_back(fixture.occupancy.fingerprint());
  }
  std::ranges::sort(fingerprints);
  EXPECT_EQ(std::ranges::unique(fingerprints).begin(), fingerprints.end());
}

TEST(AdvancedPassageFixture, PositiveReferencePathsAreRawFootprintSafe) {
  const SweptFootprintConfig footprint{};
  for (const AdvancedPassageFixtureKind kind : advancedPassageFixtureKinds()) {
    const AdvancedPassageFixture fixture = buildAdvancedPassageFixture(kind);
    for (const Point3& seed : fixture.expectation.open_space_seeds) {
      EXPECT_FALSE(fixture.occupancy.isOccupied(requiredCell(fixture.occupancy, seed)))
          << fixture.name;
    }
    for (const std::vector<Point3>& path :
         fixture.expectation.raw_safe_reference_paths) {
      ASSERT_GE(path.size(), 2U) << fixture.name;
      for (std::size_t index = 1U; index < path.size(); ++index) {
        EXPECT_TRUE(validateRawSweptFootprint(fixture.occupancy, path[index - 1U],
                                              FootprintBodyAxis{}, path[index],
                                              FootprintBodyAxis{}, footprint)
                        .accepted())
            << fixture.name << " segment " << index;
      }
    }
  }
}

TEST(AdvancedPassageFixture, PositiveCasesContainClearanceBottlenecks) {
  for (const AdvancedPassageFixtureKind kind : advancedPassageFixtureKinds()) {
    const AdvancedPassageFixture fixture = buildAdvancedPassageFixture(kind);
    if (!fixture.expectation.should_extract_passage) {
      continue;
    }
    const DistanceField3D field = DistanceField3D::build(fixture.occupancy, 10.0);
    double minimum_seed_clearance_m = std::numeric_limits<double>::infinity();
    for (const Point3& seed : fixture.expectation.open_space_seeds) {
      minimum_seed_clearance_m = std::min(minimum_seed_clearance_m,
                                          clearanceAt(field, fixture.occupancy, seed));
    }
    double minimum_path_clearance_m = std::numeric_limits<double>::infinity();
    for (const std::vector<Point3>& path :
         fixture.expectation.raw_safe_reference_paths) {
      for (const Point3& point : path) {
        minimum_path_clearance_m = std::min(
            minimum_path_clearance_m, clearanceAt(field, fixture.occupancy, point));
      }
    }
    EXPECT_GT(minimum_seed_clearance_m - minimum_path_clearance_m, 1.0)
        << fixture.name << " seed_clearance=" << minimum_seed_clearance_m
        << " bottleneck_clearance=" << minimum_path_clearance_m;
  }
}

TEST(AdvancedPassageFixture, ExercisesSlopedAndVerticalConnectivity) {
  const AdvancedPassageFixture slope =
      buildAdvancedPassageFixture(AdvancedPassageFixtureKind::kSlopedTunnel);
  const std::vector<Point3>& slope_path =
      slope.expectation.raw_safe_reference_paths.front();
  EXPECT_GT(std::abs(slope_path.back().z - slope_path.front().z), 10.0);
  EXPECT_GT(std::hypot(slope_path.back().x - slope_path.front().x,
                       slope_path.back().y - slope_path.front().y),
            10.0);

  const AdvancedPassageFixture shaft =
      buildAdvancedPassageFixture(AdvancedPassageFixtureKind::kVerticalShaft);
  const std::vector<Point3>& shaft_path =
      shaft.expectation.raw_safe_reference_paths.front();
  EXPECT_GT(std::abs(shaft_path.back().z - shaft_path.front().z), 15.0);
  EXPECT_LT(std::hypot(shaft_path.back().x - shaft_path.front().x,
                       shaft_path.back().y - shaft_path.front().y),
            1.0e-9);
}

TEST(AdvancedPassageFixture, ArchCrossSectionIsNotRectangular) {
  const AdvancedPassageFixture fixture =
      buildAdvancedPassageFixture(AdvancedPassageFixtureKind::kArchTunnel);

  EXPECT_FALSE(fixture.occupancy.isOccupied(
      requiredCell(fixture.occupancy, {16.25, 16.25, 10.25})));
  EXPECT_FALSE(fixture.occupancy.isOccupied(
      requiredCell(fixture.occupancy, {16.25, 18.25, 7.75})));
  EXPECT_TRUE(fixture.occupancy.isOccupied(
      requiredCell(fixture.occupancy, {16.25, 18.25, 10.25})));
}

TEST(AdvancedPassageFixture, JunctionsExposeThreeAndFourIndependentArms) {
  const AdvancedPassageFixture t_junction =
      buildAdvancedPassageFixture(AdvancedPassageFixtureKind::kTJunction);
  const AdvancedPassageFixture x_junction =
      buildAdvancedPassageFixture(AdvancedPassageFixtureKind::kXJunction);

  EXPECT_EQ(t_junction.expectation.open_space_seeds.size(), 3U);
  EXPECT_EQ(t_junction.expectation.raw_safe_reference_paths.size(), 3U);
  EXPECT_EQ(t_junction.expectation.minimum_segment_count, 3U);
  EXPECT_EQ(x_junction.expectation.open_space_seeds.size(), 4U);
  EXPECT_EQ(x_junction.expectation.raw_safe_reference_paths.size(), 4U);
  EXPECT_EQ(x_junction.expectation.minimum_segment_count, 4U);
}

TEST(AdvancedPassageFixture, WideHangarIsExplicitNegativeCase) {
  const AdvancedPassageFixture hangar =
      buildAdvancedPassageFixture(AdvancedPassageFixtureKind::kWideHangar);
  const DistanceField3D field = DistanceField3D::build(hangar.occupancy, 10.0);
  const std::vector<Point3>& path = hangar.expectation.raw_safe_reference_paths.front();

  EXPECT_FALSE(hangar.expectation.should_extract_passage);
  EXPECT_EQ(hangar.expectation.minimum_portal_count, 0U);
  EXPECT_EQ(hangar.expectation.minimum_segment_count, 0U);
  EXPECT_GE(clearanceAt(field, hangar.occupancy, path[1U]),
            std::min(clearanceAt(field, hangar.occupancy, path.front()),
                     clearanceAt(field, hangar.occupancy, path.back())));
}

} // namespace
} // namespace drone_city_nav::test
