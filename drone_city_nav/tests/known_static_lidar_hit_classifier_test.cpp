#include "drone_city_nav/known_static_lidar_hit_classifier.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] KnownPassageSolidVolume
makeVolume(const KnownPassageSolidPartKind kind = KnownPassageSolidPartKind::kUpper) {
  return KnownPassageSolidVolume{
      .structure_id = "building",
      .opening_id = "opening",
      .part_id = "upper_mass",
      .part_kind = kind,
      .center = Point2{5.0, 0.0},
      .normal_xy = Point2{1.0, 0.0},
      .lateral_xy = Point2{0.0, 1.0},
      .depth_m = 2.0,
      .width_m = 4.0,
      .min_z_m = 2.0,
      .max_z_m = 4.0,
  };
}

[[nodiscard]] KnownStaticLidarHitClassifier
makeClassifier(KnownPassageSolidVolume volume = makeVolume(),
               const double tolerance_m = 0.5) {
  std::vector<KnownPassageSolidVolume> volumes;
  volumes.push_back(std::move(volume));
  return KnownStaticLidarHitClassifier{
      std::move(volumes), KnownStaticLidarHitClassifierConfig{tolerance_m}};
}

} // namespace

TEST(KnownStaticLidarHitClassifier, ConfidentFaceInteriorSuppressesExpectedHit) {
  const KnownStaticLidarHitClassifier classifier = makeClassifier();

  const KnownStaticLidarHitResult result =
      classifier.classify(Point3{0.0, 0.0, 3.0}, Point3{1.0, 0.0, 0.0}, 4.2);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kExpectedStatic);
  EXPECT_TRUE(result.confident_face_interior);
  EXPECT_NEAR(result.expected_range_m, 4.0, 1.0e-9);
  EXPECT_NEAR(result.range_delta_m, 0.2, 1.0e-9);
  EXPECT_EQ(result.structure_id, "building");
  EXPECT_EQ(result.part_kind, KnownPassageSolidPartKind::kUpper);
}

TEST(KnownStaticLidarHitClassifier, CloserObjectIsRetained) {
  const KnownStaticLidarHitClassifier classifier = makeClassifier();

  const KnownStaticLidarHitResult result =
      classifier.classify(Point3{0.0, 0.0, 3.0}, Point3{1.0, 0.0, 0.0}, 3.0);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kUnexpected);
  EXPECT_TRUE(result.volume_matched);
}

TEST(KnownStaticLidarHitClassifier, FartherRangeIsAmbiguousAndRetained) {
  const KnownStaticLidarHitClassifier classifier = makeClassifier();

  const KnownStaticLidarHitResult result =
      classifier.classify(Point3{0.0, 0.0, 3.0}, Point3{1.0, 0.0, 0.0}, 5.0);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kAmbiguous);
  EXPECT_TRUE(result.volume_matched);
}

TEST(KnownStaticLidarHitClassifier, RayThroughFreeOpeningIsUnexpected) {
  const KnownStaticLidarHitClassifier classifier = makeClassifier();

  const KnownStaticLidarHitResult result =
      classifier.classify(Point3{0.0, 0.0, 1.0}, Point3{1.0, 0.0, 0.0}, 4.0);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kUnexpected);
  EXPECT_FALSE(result.volume_matched);
}

TEST(KnownStaticLidarHitClassifier, RotatedVolumeUsesItsLocalFrame) {
  KnownPassageSolidVolume volume = makeVolume(KnownPassageSolidPartKind::kLeft);
  volume.center = Point2{0.0, 5.0};
  volume.normal_xy = Point2{0.0, 1.0};
  volume.lateral_xy = Point2{-1.0, 0.0};
  const KnownStaticLidarHitClassifier classifier = makeClassifier(std::move(volume));

  const KnownStaticLidarHitResult result =
      classifier.classify(Point3{0.0, 0.0, 3.0}, Point3{0.0, 1.0, 0.0}, 4.0);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kExpectedStatic);
  EXPECT_EQ(result.part_kind, KnownPassageSolidPartKind::kLeft);
}

TEST(KnownStaticLidarHitClassifier, FaceBoundaryGrazingIsAmbiguous) {
  const KnownStaticLidarHitClassifier classifier = makeClassifier();

  const KnownStaticLidarHitResult result =
      classifier.classify(Point3{0.0, 2.0, 3.0}, Point3{1.0, 0.0, 0.0}, 4.0);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kAmbiguous);
  EXPECT_FALSE(result.confident_face_interior);
}

TEST(KnownStaticLidarHitClassifier, JustInsideBoundaryIsConfident) {
  const KnownStaticLidarHitClassifier classifier = makeClassifier();

  const KnownStaticLidarHitResult result =
      classifier.classify(Point3{0.0, 2.0 - 2.0e-6, 3.0}, Point3{1.0, 0.0, 0.0}, 4.0);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kExpectedStatic);
}

TEST(KnownStaticLidarHitClassifier, JustOutsideBoundaryDoesNotMatchSolid) {
  const KnownStaticLidarHitClassifier classifier = makeClassifier();

  const KnownStaticLidarHitResult result =
      classifier.classify(Point3{0.0, 2.0 + 2.0e-6, 3.0}, Point3{1.0, 0.0, 0.0}, 4.0);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kUnexpected);
}

TEST(KnownStaticLidarHitClassifier, EdgeEntryIsAmbiguous) {
  const KnownStaticLidarHitClassifier classifier = makeClassifier();
  const double component = std::numbers::sqrt2 / 2.0;

  const KnownStaticLidarHitResult result =
      classifier.classify(Point3{0.0, -6.0, 3.0}, Point3{component, component, 0.0},
                          4.0 * std::numbers::sqrt2);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kAmbiguous);
  EXPECT_FALSE(result.confident_face_interior);
}

TEST(KnownStaticLidarHitClassifier, CornerEntryIsAmbiguous) {
  const KnownStaticLidarHitClassifier classifier = makeClassifier();
  const double component = std::numbers::inv_sqrt3;

  const KnownStaticLidarHitResult result = classifier.classify(
      Point3{0.0, -6.0, -2.0}, Point3{component, component, component},
      4.0 * std::numbers::sqrt3);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kAmbiguous);
}

TEST(KnownStaticLidarHitClassifier, OriginInsideSolidIsAmbiguous) {
  const KnownStaticLidarHitClassifier classifier = makeClassifier();

  const KnownStaticLidarHitResult result =
      classifier.classify(Point3{5.0, 0.0, 3.0}, Point3{1.0, 0.0, 0.0}, 0.5);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kAmbiguous);
}

TEST(KnownStaticLidarHitClassifier, InvalidRayFailsOpen) {
  const KnownStaticLidarHitClassifier classifier = makeClassifier();

  const KnownStaticLidarHitResult result =
      classifier.classify(Point3{0.0, 0.0, std::numeric_limits<double>::quiet_NaN()},
                          Point3{1.0, 0.0, 0.0}, 4.0);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kAmbiguous);
}

TEST(KnownStaticLidarHitClassifier, GeometricAmbiguityDoesNotDependOnRangeTolerance) {
  const KnownStaticLidarHitClassifier narrow = makeClassifier(makeVolume(), 0.01);
  const KnownStaticLidarHitClassifier wide = makeClassifier(makeVolume(), 2.0);

  const KnownStaticLidarHitResult narrow_result =
      narrow.classify(Point3{0.0, 2.0, 3.0}, Point3{1.0, 0.0, 0.0}, 4.0);
  const KnownStaticLidarHitResult wide_result =
      wide.classify(Point3{0.0, 2.0, 3.0}, Point3{1.0, 0.0, 0.0}, 4.0);

  EXPECT_EQ(narrow_result.classification,
            KnownStaticLidarHitClassification::kAmbiguous);
  EXPECT_EQ(wide_result.classification, KnownStaticLidarHitClassification::kAmbiguous);
}

} // namespace drone_city_nav
