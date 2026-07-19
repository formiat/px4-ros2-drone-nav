#include "drone_city_nav/known_static_lidar_hit_classifier.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr double kEffectiveMaxRangeM = 20.0;

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
      .opening_center = Point2{5.0, 0.0},
      .opening_depth_m = 2.0,
      .opening_width_m = 4.0,
      .opening_min_z_m = 0.0,
      .opening_max_z_m = 2.0,
  };
}

[[nodiscard]] KnownStaticLidarHitClassifier
makeClassifier(KnownPassageSolidVolume volume = makeVolume(),
               const double closer_tolerance_m = 0.5,
               const double farther_tolerance_m = 1.5) {
  std::vector<KnownPassageSolidVolume> volumes;
  volumes.push_back(std::move(volume));
  return KnownStaticLidarHitClassifier{
      std::move(volumes), KnownStaticLidarHitClassifierConfig{
                              .closer_range_tolerance_m = closer_tolerance_m,
                              .farther_range_tolerance_m = farther_tolerance_m,
                              .endpoint_volume_tolerance_m = 0.75,
                              .opening_boundary_tolerance_m = 0.15}};
}

[[nodiscard]] KnownStaticLidarHitClassifier highOpeningClassifier() {
  KnownPassageSolidVolume lower = makeVolume(KnownPassageSolidPartKind::kLower);
  lower.part_id = "lower_mass";
  lower.min_z_m = 0.0;
  lower.max_z_m = 21.5;
  lower.opening_min_z_m = 21.5;
  lower.opening_max_z_m = 28.5;
  KnownPassageSolidVolume upper = lower;
  upper.part_id = "upper_mass";
  upper.part_kind = KnownPassageSolidPartKind::kUpper;
  upper.min_z_m = 28.5;
  upper.max_z_m = 32.0;
  return KnownStaticLidarHitClassifier{{std::move(lower), std::move(upper)}};
}

} // namespace

TEST(KnownStaticLidarHitClassifier, ConfidentFaceInteriorSuppressesExpectedHit) {
  const KnownStaticLidarHitClassifier classifier = makeClassifier();

  const KnownStaticLidarHitResult result = classifier.classify(
      Point3{0.0, 0.0, 3.0}, Point3{1.0, 0.0, 0.0}, 4.2, kEffectiveMaxRangeM);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kExpectedStatic);
  EXPECT_TRUE(result.confident_face_interior);
  EXPECT_NEAR(result.expected_range_m, 4.0, 1.0e-9);
  EXPECT_NEAR(result.range_delta_m, 0.2, 1.0e-9);
  EXPECT_EQ(result.structure_id, "building");
  EXPECT_EQ(result.part_kind, KnownPassageSolidPartKind::kUpper);
}

TEST(KnownStaticLidarHitClassifier, CloserObjectIsRetained) {
  const KnownStaticLidarHitClassifier classifier = makeClassifier();

  const KnownStaticLidarHitResult result = classifier.classify(
      Point3{0.0, 0.0, 3.0}, Point3{1.0, 0.0, 0.0}, 3.0, kEffectiveMaxRangeM);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kUnexpected);
  EXPECT_TRUE(result.volume_matched);
}

TEST(KnownStaticLidarHitClassifier, CloserEndpointNearSolidIsAmbiguous) {
  const KnownStaticLidarHitClassifier classifier = makeClassifier();

  const KnownStaticLidarHitResult result = classifier.classify(
      Point3{0.0, 0.0, 3.0}, Point3{1.0, 0.0, 0.0}, 3.34, kEffectiveMaxRangeM);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kAmbiguous);
  EXPECT_EQ(result.endpoint_relation, KnownStaticEndpointRelation::kNearSurface);
  EXPECT_TRUE(result.closer_side_fallback);
  EXPECT_NEAR(result.endpoint_solid_distance_m, 0.66, 1.0e-9);
}

TEST(KnownStaticLidarHitClassifier, EndpointInsideSameSolidUsesGeometricFallback) {
  const KnownStaticLidarHitClassifier classifier = makeClassifier();

  const KnownStaticLidarHitResult result = classifier.classify(
      Point3{0.0, 0.0, 3.0}, Point3{1.0, 0.0, 0.0}, 6.0, kEffectiveMaxRangeM);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kExpectedStatic);
  EXPECT_TRUE(result.volume_matched);
  EXPECT_TRUE(result.endpoint_volume_fallback);
}

TEST(KnownStaticLidarHitClassifier, EndpointPastSameSolidRemainsAmbiguous) {
  const KnownStaticLidarHitClassifier classifier = makeClassifier();

  const KnownStaticLidarHitResult result = classifier.classify(
      Point3{0.0, 0.0, 3.0}, Point3{1.0, 0.0, 0.0}, 6.5, kEffectiveMaxRangeM);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kAmbiguous);
  EXPECT_TRUE(result.endpoint_volume_fallback);
}

TEST(KnownStaticLidarHitClassifier, FartherKnownSurfaceReturnUsesFartherTolerance) {
  const KnownStaticLidarHitClassifier classifier = makeClassifier();

  const KnownStaticLidarHitResult result = classifier.classify(
      Point3{0.0, 0.0, 3.0}, Point3{1.0, 0.0, 0.0}, 5.4, kEffectiveMaxRangeM);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kExpectedStatic);
  EXPECT_NEAR(result.range_delta_m, 1.4, 1.0e-9);
  EXPECT_TRUE(result.endpoint_volume_fallback);
}

TEST(KnownStaticLidarHitClassifier, RayThroughFreeOpeningIsUnexpected) {
  const KnownStaticLidarHitClassifier classifier = makeClassifier();

  const KnownStaticLidarHitResult result = classifier.classify(
      Point3{0.0, 0.0, 1.0}, Point3{1.0, 0.0, 0.0}, 5.0, kEffectiveMaxRangeM);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kUnexpected);
  EXPECT_FALSE(result.volume_matched);
  EXPECT_EQ(result.endpoint_relation, KnownStaticEndpointRelation::kInsideOpening);
  EXPECT_GT(result.endpoint_opening_margin_m, 0.0);
}

TEST(KnownStaticLidarHitClassifier, RotatedVolumeUsesItsLocalFrame) {
  KnownPassageSolidVolume volume = makeVolume(KnownPassageSolidPartKind::kLeft);
  volume.center = Point2{0.0, 5.0};
  volume.normal_xy = Point2{0.0, 1.0};
  volume.lateral_xy = Point2{-1.0, 0.0};
  const KnownStaticLidarHitClassifier classifier = makeClassifier(std::move(volume));

  const KnownStaticLidarHitResult result = classifier.classify(
      Point3{0.0, 0.0, 3.0}, Point3{0.0, 1.0, 0.0}, 4.0, kEffectiveMaxRangeM);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kExpectedStatic);
  EXPECT_EQ(result.part_kind, KnownPassageSolidPartKind::kLeft);
}

TEST(KnownStaticLidarHitClassifier, FaceBoundaryGrazingIsAmbiguous) {
  const KnownStaticLidarHitClassifier classifier = makeClassifier();

  const KnownStaticLidarHitResult result = classifier.classify(
      Point3{0.0, 2.0, 3.0}, Point3{1.0, 0.0, 0.0}, 4.0, kEffectiveMaxRangeM);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kAmbiguous);
  EXPECT_FALSE(result.confident_face_interior);
}

TEST(KnownStaticLidarHitClassifier, JustInsideBoundaryIsConfident) {
  const KnownStaticLidarHitClassifier classifier = makeClassifier();

  const KnownStaticLidarHitResult result = classifier.classify(
      Point3{0.0, 2.0 - 2.0e-6, 3.0}, Point3{1.0, 0.0, 0.0}, 4.0, kEffectiveMaxRangeM);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kExpectedStatic);
}

TEST(KnownStaticLidarHitClassifier, JustOutsideBoundaryDoesNotMatchSolid) {
  const KnownStaticLidarHitClassifier classifier = makeClassifier();

  const KnownStaticLidarHitResult result = classifier.classify(
      Point3{0.0, 2.0 + 2.0e-6, 3.0}, Point3{1.0, 0.0, 0.0}, 4.0, kEffectiveMaxRangeM);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kAmbiguous);
  EXPECT_EQ(result.endpoint_relation, KnownStaticEndpointRelation::kNearSurface);
}

TEST(KnownStaticLidarHitClassifier, EdgeEntryIsAmbiguous) {
  const KnownStaticLidarHitClassifier classifier = makeClassifier();
  const double component = std::numbers::sqrt2 / 2.0;

  const KnownStaticLidarHitResult result =
      classifier.classify(Point3{0.0, -6.0, 3.0}, Point3{component, component, 0.0},
                          4.0 * std::numbers::sqrt2, kEffectiveMaxRangeM);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kAmbiguous);
  EXPECT_FALSE(result.confident_face_interior);
}

TEST(KnownStaticLidarHitClassifier, CornerEntryIsAmbiguous) {
  const KnownStaticLidarHitClassifier classifier = makeClassifier();
  const double component = std::numbers::inv_sqrt3;

  const KnownStaticLidarHitResult result = classifier.classify(
      Point3{0.0, -6.0, -2.0}, Point3{component, component, component},
      4.0 * std::numbers::sqrt3, kEffectiveMaxRangeM);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kAmbiguous);
}

TEST(KnownStaticLidarHitClassifier, OriginInsideSolidIsAmbiguous) {
  const KnownStaticLidarHitClassifier classifier = makeClassifier();

  const KnownStaticLidarHitResult result = classifier.classify(
      Point3{5.0, 0.0, 3.0}, Point3{1.0, 0.0, 0.0}, 0.5, kEffectiveMaxRangeM);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kAmbiguous);
}

TEST(KnownStaticLidarHitClassifier, InvalidRayFailsOpen) {
  const KnownStaticLidarHitClassifier classifier = makeClassifier();

  const KnownStaticLidarHitResult result =
      classifier.classify(Point3{0.0, 0.0, std::numeric_limits<double>::quiet_NaN()},
                          Point3{1.0, 0.0, 0.0}, 4.0, kEffectiveMaxRangeM);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kAmbiguous);
}

TEST(KnownStaticLidarHitClassifier, GeometricAmbiguityDoesNotDependOnRangeTolerance) {
  const KnownStaticLidarHitClassifier narrow = makeClassifier(makeVolume(), 0.01, 0.01);
  const KnownStaticLidarHitClassifier wide = makeClassifier(makeVolume(), 2.0, 2.0);

  const KnownStaticLidarHitResult narrow_result = narrow.classify(
      Point3{0.0, 2.0, 3.0}, Point3{1.0, 0.0, 0.0}, 4.0, kEffectiveMaxRangeM);
  const KnownStaticLidarHitResult wide_result = wide.classify(
      Point3{0.0, 2.0, 3.0}, Point3{1.0, 0.0, 0.0}, 4.0, kEffectiveMaxRangeM);

  EXPECT_EQ(narrow_result.classification,
            KnownStaticLidarHitClassification::kAmbiguous);
  EXPECT_EQ(wide_result.classification, KnownStaticLidarHitClassification::kAmbiguous);
}

TEST(KnownStaticLidarHitClassifier,
     DistantSurfaceOutsideEffectiveRangeDoesNotMatchMeasuredEndpoint) {
  KnownPassageSolidVolume distant = makeVolume();
  distant.center = Point2{213.0, 0.0};
  distant.opening_center = distant.center;
  const KnownStaticLidarHitClassifier classifier = makeClassifier(std::move(distant));

  const KnownStaticBeamEvaluation evaluation =
      classifier.evaluateBeam(Point3{0.0, 0.0, 3.0}, Point3{1.0, 0.0, 0.0}, 29.0, 30.0);

  EXPECT_FALSE(evaluation.in_range_surface.has_value());
  EXPECT_FALSE(evaluation.endpoint_fallback_surface.has_value());
  EXPECT_EQ(evaluation.endpoint_relation, KnownStaticEndpointRelation::kOutside);
  EXPECT_EQ(evaluation.hit_result.classification,
            KnownStaticLidarHitClassification::kUnexpected);
  EXPECT_FALSE(evaluation.hit_result.volume_matched);
  EXPECT_TRUE(std::isnan(evaluation.hit_result.expected_range_m));
  EXPECT_TRUE(std::isnan(evaluation.hit_result.range_delta_m));
}

TEST(KnownStaticLidarHitClassifier,
     OpeningLowerBoundaryMillimeterReturnsAreStaticAttached) {
  const KnownStaticLidarHitClassifier classifier = highOpeningClassifier();
  for (const double altitude_m : {21.5, 21.502, 21.505}) {
    const KnownStaticLidarHitResult result = classifier.classify(
        Point3{0.0, 0.0, altitude_m}, Point3{1.0, 0.0, 0.0}, 5.0, kEffectiveMaxRangeM);

    EXPECT_NE(result.classification, KnownStaticLidarHitClassification::kUnexpected);
    EXPECT_NE(result.endpoint_relation, KnownStaticEndpointRelation::kInsideOpening);
    if (altitude_m > 21.5) {
      EXPECT_EQ(result.endpoint_relation,
                KnownStaticEndpointRelation::kInsideOpeningBoundary);
      EXPECT_EQ(result.part_kind, KnownPassageSolidPartKind::kLower);
      EXPECT_EQ(result.part_id, "lower_mass");
      EXPECT_NEAR(result.endpoint_solid_distance_m, altitude_m - 21.5, 1.0e-9);
      EXPECT_NEAR(result.opening_min_z_m, 21.5, 1.0e-9);
      EXPECT_NEAR(result.opening_max_z_m, 28.5, 1.0e-9);
      EXPECT_NEAR(result.opening_boundary_tolerance_m, 0.15, 1.0e-9);
    }
  }
}

TEST(KnownStaticLidarHitClassifier, OpeningUpperBoundaryReturnUsesNearestUpperMass) {
  const KnownStaticLidarHitClassifier classifier = highOpeningClassifier();

  const KnownStaticLidarHitResult result = classifier.classify(
      Point3{0.0, 0.0, 28.498}, Point3{1.0, 0.0, 0.0}, 5.0, kEffectiveMaxRangeM);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kAmbiguous);
  EXPECT_EQ(result.endpoint_relation,
            KnownStaticEndpointRelation::kInsideOpeningBoundary);
  EXPECT_EQ(result.part_kind, KnownPassageSolidPartKind::kUpper);
  EXPECT_EQ(result.part_id, "upper_mass");
  EXPECT_NEAR(result.endpoint_solid_distance_m, 0.002, 1.0e-9);
}

TEST(KnownStaticLidarHitClassifier, OpeningInteriorRemainsImmediateObstacle) {
  const KnownStaticLidarHitClassifier classifier = highOpeningClassifier();

  const KnownStaticLidarHitResult result = classifier.classify(
      Point3{0.0, 0.0, 21.8}, Point3{1.0, 0.0, 0.0}, 5.0, kEffectiveMaxRangeM);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kUnexpected);
  EXPECT_EQ(result.endpoint_relation, KnownStaticEndpointRelation::kInsideOpening);
  EXPECT_NEAR(result.endpoint_solid_distance_m, 0.3, 1.0e-9);
}

TEST(KnownStaticLidarHitClassifier, OpeningEntryPlaneDoesNotCreateFalseSolidBoundary) {
  const KnownStaticLidarHitClassifier classifier = highOpeningClassifier();

  const KnownStaticLidarHitResult result = classifier.classify(
      Point3{0.0, 0.0, 25.0}, Point3{1.0, 0.0, 0.0}, 4.005, kEffectiveMaxRangeM);

  EXPECT_EQ(result.classification, KnownStaticLidarHitClassification::kUnexpected);
  EXPECT_EQ(result.endpoint_relation, KnownStaticEndpointRelation::kInsideOpening);
  EXPECT_NEAR(result.endpoint_opening_margin_m, 0.005, 1.0e-9);
  EXPECT_GT(result.endpoint_solid_distance_m, 3.0);
}

} // namespace drone_city_nav
