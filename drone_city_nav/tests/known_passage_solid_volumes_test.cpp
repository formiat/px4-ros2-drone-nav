#include "drone_city_nav/known_passage_solid_volumes.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] PassageStructure makeStructure() {
  PassageOpening opening{};
  opening.id = "main";
  opening.structure_id = "building_with_passage";
  opening.center = Point3{10.0, 20.0, 5.0};
  opening.normal_xy = Point2{1.0, 0.0};
  opening.width_m = 7.0;
  opening.height_m = 7.0;
  opening.depth_m = 24.0;
  opening.min_z_m = 1.5;
  opening.max_z_m = 8.5;
  opening.approach_distance_m = 18.0;
  opening.exit_distance_m = 18.0;

  PassageStructure structure{};
  structure.id = "building_with_passage";
  structure.center = Point2{10.0, 20.0};
  structure.size_x_m = 40.0;
  structure.size_y_m = 24.0;
  structure.z_min_m = 0.0;
  structure.z_max_m = 32.0;
  structure.openings.push_back(opening);
  return structure;
}

[[nodiscard]] const KnownPassageSolidVolume*
findPart(const std::vector<KnownPassageSolidVolume>& volumes,
         const std::string& part_id) {
  const auto iter =
      std::ranges::find_if(volumes, [&part_id](const KnownPassageSolidVolume& volume) {
        return volume.part_id == part_id;
      });
  if (iter == volumes.end()) {
    return nullptr;
  }
  return &*iter;
}

} // namespace

TEST(KnownPassageSolidVolumes, BuildsPhysicalOpeningFrameParts) {
  const std::vector<KnownPassageSolidVolume> volumes =
      knownPassageSolidVolumes(makeStructure());

  ASSERT_EQ(volumes.size(), 4U);
  const KnownPassageSolidVolume* left = findPart(volumes, "left_mass");
  const KnownPassageSolidVolume* right = findPart(volumes, "right_mass");
  const KnownPassageSolidVolume* lower = findPart(volumes, "lower_mass");
  const KnownPassageSolidVolume* upper = findPart(volumes, "upper_mass");
  ASSERT_NE(left, nullptr);
  ASSERT_NE(right, nullptr);
  ASSERT_NE(lower, nullptr);
  ASSERT_NE(upper, nullptr);

  EXPECT_EQ(left->structure_id, "building_with_passage");
  EXPECT_EQ(left->opening_id, "main");
  EXPECT_EQ(left->part_kind, KnownPassageSolidPartKind::kLeft);
  EXPECT_DOUBLE_EQ(left->depth_m, 24.0);
  EXPECT_DOUBLE_EQ(left->width_m, 16.5);
  EXPECT_DOUBLE_EQ(left->center.x, 10.0);
  EXPECT_DOUBLE_EQ(left->center.y, 8.25);
  EXPECT_DOUBLE_EQ(left->min_z_m, 0.0);
  EXPECT_DOUBLE_EQ(left->max_z_m, 32.0);

  EXPECT_DOUBLE_EQ(right->depth_m, 24.0);
  EXPECT_EQ(right->part_kind, KnownPassageSolidPartKind::kRight);
  EXPECT_DOUBLE_EQ(right->width_m, 16.5);
  EXPECT_DOUBLE_EQ(right->center.x, 10.0);
  EXPECT_DOUBLE_EQ(right->center.y, 31.75);
  EXPECT_DOUBLE_EQ(right->min_z_m, 0.0);
  EXPECT_DOUBLE_EQ(right->max_z_m, 32.0);

  EXPECT_DOUBLE_EQ(lower->depth_m, 24.0);
  EXPECT_EQ(lower->part_kind, KnownPassageSolidPartKind::kLower);
  EXPECT_DOUBLE_EQ(lower->width_m, 7.0);
  EXPECT_DOUBLE_EQ(lower->center.x, 10.0);
  EXPECT_DOUBLE_EQ(lower->center.y, 20.0);
  EXPECT_DOUBLE_EQ(lower->min_z_m, 0.0);
  EXPECT_DOUBLE_EQ(lower->max_z_m, 1.5);

  EXPECT_DOUBLE_EQ(upper->depth_m, 24.0);
  EXPECT_EQ(upper->part_kind, KnownPassageSolidPartKind::kUpper);
  EXPECT_DOUBLE_EQ(upper->width_m, 7.0);
  EXPECT_DOUBLE_EQ(upper->center.x, 10.0);
  EXPECT_DOUBLE_EQ(upper->center.y, 20.0);
  EXPECT_DOUBLE_EQ(upper->min_z_m, 8.5);
  EXPECT_DOUBLE_EQ(upper->max_z_m, 32.0);
}

TEST(KnownPassageSolidVolumes, SkipsAbsentLowerOrUpperMass) {
  PassageStructure structure = makeStructure();
  structure.openings.front().min_z_m = structure.z_min_m;
  structure.openings.front().max_z_m = structure.z_max_m;

  const std::vector<KnownPassageSolidVolume> volumes =
      knownPassageSolidVolumes(structure);

  ASSERT_EQ(volumes.size(), 2U);
  ASSERT_NE(findPart(volumes, "left_mass"), nullptr);
  ASSERT_NE(findPart(volumes, "right_mass"), nullptr);
}

} // namespace drone_city_nav
