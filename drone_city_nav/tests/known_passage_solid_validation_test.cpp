#include "drone_city_nav/known_passage_solid_validation.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] KnownPassageMap makeMap() {
  PassageOpening opening{};
  opening.id = "main";
  opening.structure_id = "connector";
  opening.center = Point3{10.0, 0.0, 5.0};
  opening.normal_xy = Point2{1.0, 0.0};
  opening.width_m = 8.0;
  opening.height_m = 7.0;
  opening.depth_m = 4.0;
  opening.min_z_m = 1.5;
  opening.max_z_m = 8.5;

  PassageStructure structure{};
  structure.id = "connector";
  structure.center = Point2{10.0, 0.0};
  structure.size_x_m = 4.0;
  structure.size_y_m = 20.0;
  structure.z_min_m = 0.0;
  structure.z_max_m = 20.0;
  structure.openings.push_back(opening);

  KnownPassageMap map{};
  map.structures.push_back(structure);
  return map;
}

[[nodiscard]] TrajectoryPointSample sample(const double s, const double x,
                                           const double y, const double z) {
  TrajectoryPointSample result{};
  result.s_m = s;
  result.point = Point2{x, y};
  result.z_m = z;
  return result;
}

} // namespace

TEST(KnownPassageSolidValidation, AcceptsContinuousPathThroughOpening) {
  const KnownPassageMap map = makeMap();
  const std::vector<TrajectoryPointSample> samples{sample(0.0, 0.0, 0.0, 5.0),
                                                   sample(20.0, 20.0, 0.0, 5.0)};

  const KnownPassageSolidValidationSummary result =
      validateTrajectoryAgainstKnownPassageSolids(samples, &map);

  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.reason, KnownPassageSolidValidationReason::kClear);
}

TEST(KnownPassageSolidValidation, RejectsLowerMassIntersectionBetweenSamples) {
  const KnownPassageMap map = makeMap();
  const std::vector<TrajectoryPointSample> samples{sample(0.0, 0.0, 0.0, 1.0),
                                                   sample(20.0, 20.0, 0.0, 1.0)};

  const KnownPassageSolidValidationSummary result =
      validateTrajectoryAgainstKnownPassageSolids(samples, &map);

  ASSERT_FALSE(result.valid);
  ASSERT_TRUE(result.has_first_intersection);
  EXPECT_EQ(result.first_intersection.part_kind, KnownPassageSolidPartKind::kLower);
  EXPECT_NEAR(result.first_intersection.point.x, 8.0, 1.0e-9);
}

TEST(KnownPassageSolidValidation, RejectsUpperMassIntersection) {
  const KnownPassageMap map = makeMap();
  const std::vector<TrajectoryPointSample> samples{sample(0.0, 0.0, 0.0, 10.0),
                                                   sample(20.0, 20.0, 0.0, 10.0)};

  const KnownPassageSolidValidationSummary result =
      validateTrajectoryAgainstKnownPassageSolids(samples, &map);

  ASSERT_FALSE(result.valid);
  ASSERT_TRUE(result.has_first_intersection);
  EXPECT_EQ(result.first_intersection.part_kind, KnownPassageSolidPartKind::kUpper);
}

TEST(KnownPassageSolidValidation, AcceptsTrajectoryWithoutKnownMap) {
  const std::vector<TrajectoryPointSample> samples{sample(0.0, 0.0, 0.0, 0.0),
                                                   sample(1.0, 1.0, 0.0, 0.0)};

  const KnownPassageSolidValidationSummary result =
      validateTrajectoryAgainstKnownPassageSolids(samples, nullptr);

  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.reason, KnownPassageSolidValidationReason::kNoMap);
}

} // namespace drone_city_nav
