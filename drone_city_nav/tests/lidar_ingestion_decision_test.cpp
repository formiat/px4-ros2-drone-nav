#include "drone_city_nav/lidar_ingestion_decision.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] LidarBeamObservation observation(const Point3 direction,
                                               const double measured_range_m,
                                               const bool hit = true) {
  const Point3 origin{0.0, 0.0, 10.0};
  return LidarBeamObservation{
      .projection =
          LidarBeamProjection{
              .status = LidarBeamProjectionStatus::kAccepted,
              .hit = hit,
              .used_range_m = measured_range_m,
              .endpoint_altitude_m = origin.z + measured_range_m * direction.z,
              .endpoint = Point2{measured_range_m * direction.x,
                                 measured_range_m * direction.y},
              .ray_origin_map_m = origin,
              .ray_direction_map = direction,
              .endpoint_map_m =
                  Point3{measured_range_m * direction.x, measured_range_m * direction.y,
                         origin.z + measured_range_m * direction.z},
              .endpoint_xyz_valid = true,
              .attitude_compensation_applied = true,
          },
      .measured_range_m = measured_range_m,
      .effective_max_range_m = 30.0,
      .attitude_compensation_required = true,
      .source_attitude_valid = true,
      .source_roll_rad = 0.0,
      .source_pitch_rad = 0.0,
      .source_tilt_rad = 0.0,
  };
}

[[nodiscard]] KnownStaticLidarHitClassifier horizontalKnownSurface() {
  std::vector<KnownPassageSolidVolume> volumes;
  volumes.push_back(KnownPassageSolidVolume{
      .structure_id = "building",
      .opening_id = "opening",
      .part_id = "upper_mass",
      .part_kind = KnownPassageSolidPartKind::kUpper,
      .center = Point2{6.0, 0.0},
      .normal_xy = Point2{1.0, 0.0},
      .lateral_xy = Point2{0.0, 1.0},
      .depth_m = 2.0,
      .width_m = 4.0,
      .min_z_m = 8.0,
      .max_z_m = 12.0,
  });
  return KnownStaticLidarHitClassifier{std::move(volumes)};
}

[[nodiscard]] KnownStaticLidarHitClassifier knownSurfaceAtRange(const Point3 direction,
                                                                const double range_m) {
  const double horizontal_norm = std::hypot(direction.x, direction.y);
  const Point2 normal{direction.x / horizontal_norm, direction.y / horizontal_norm};
  const Point2 lateral{-normal.y, normal.x};
  constexpr double kDepthM = 2.0;
  std::vector<KnownPassageSolidVolume> volumes;
  volumes.push_back(KnownPassageSolidVolume{
      .structure_id = "tied_building",
      .opening_id = "tied_opening",
      .part_id = "lower_mass",
      .part_kind = KnownPassageSolidPartKind::kLower,
      .center = Point2{normal.x * (range_m * horizontal_norm + kDepthM / 2.0),
                       normal.y * (range_m * horizontal_norm + kDepthM / 2.0)},
      .normal_xy = normal,
      .lateral_xy = lateral,
      .depth_m = kDepthM,
      .width_m = 4.0,
      .min_z_m = -100.0,
      .max_z_m = 100.0,
  });
  return KnownStaticLidarHitClassifier{std::move(volumes)};
}

} // namespace

TEST(LidarIngestionDecision, SuppressesExpectedGroundHitWithoutFreeClearing) {
  const Point3 direction{0.6, 0.0, -0.8};
  const GroundLidarRejectionConfig ground{};
  const LidarIngestionDecision decision =
      evaluateLidarIngestion(observation(direction, 12.44), nullptr, &ground);

  EXPECT_EQ(decision.action, LidarIngestionAction::kSuppressAllUpdates);
  EXPECT_EQ(decision.reason, LidarIngestionReason::kExpectedGround);
  EXPECT_EQ(decision.expected_surface, LidarExpectedSurfaceKind::kGround);
  EXPECT_NEAR(decision.expected_range_m, 12.4375, 1.0e-6);
}

TEST(LidarIngestionDecision, RetainsObstacleClearlyBeforeGround) {
  const Point3 direction{0.6, 0.0, -0.8};
  const GroundLidarRejectionConfig ground{};
  const LidarIngestionDecision decision =
      evaluateLidarIngestion(observation(direction, 9.0), nullptr, &ground);

  EXPECT_EQ(decision.action, LidarIngestionAction::kIntegrateFreeAndHit);
  EXPECT_EQ(decision.reason, LidarIngestionReason::kObstacleBeforeExpectedSurface);
  EXPECT_LT(decision.range_delta_m, -3.0);
}

TEST(LidarIngestionDecision, GroundToleranceBoundariesAreAsymmetric) {
  const Point3 direction{0.6, 0.0, -0.8};
  constexpr double kExpectedRangeM = 12.4375;
  const GroundLidarRejectionConfig ground{};

  const LidarIngestionDecision at_closer_boundary = evaluateLidarIngestion(
      observation(direction, kExpectedRangeM - ground.closer_range_tolerance_m),
      nullptr, &ground);
  EXPECT_EQ(at_closer_boundary.reason, LidarIngestionReason::kExpectedGround);

  const LidarIngestionDecision at_farther_boundary = evaluateLidarIngestion(
      observation(direction, kExpectedRangeM + ground.farther_range_tolerance_m),
      nullptr, &ground);
  EXPECT_EQ(at_farther_boundary.reason, LidarIngestionReason::kExpectedGround);

  const LidarIngestionDecision beyond_farther_boundary = evaluateLidarIngestion(
      observation(direction,
                  kExpectedRangeM + ground.farther_range_tolerance_m + 0.001),
      nullptr, &ground);
  EXPECT_EQ(beyond_farther_boundary.reason, LidarIngestionReason::kAmbiguousGround);
}

TEST(LidarIngestionDecision, SuppressesDownwardNoReturnThatReachesGround) {
  const Point3 direction{0.6, 0.0, -0.8};
  const GroundLidarRejectionConfig ground{};
  const LidarIngestionDecision decision =
      evaluateLidarIngestion(observation(direction, 30.0, false), nullptr, &ground);

  EXPECT_EQ(decision.action, LidarIngestionAction::kSuppressAllUpdates);
  EXPECT_EQ(decision.reason, LidarIngestionReason::kExpectedGround);
}

TEST(LidarIngestionDecision, LeavesUpwardBeamOnLegacyPath) {
  const Point3 direction{0.6, 0.0, 0.8};
  const GroundLidarRejectionConfig ground{};
  const LidarIngestionDecision decision =
      evaluateLidarIngestion(observation(direction, 8.0), nullptr, &ground);

  EXPECT_EQ(decision.action, LidarIngestionAction::kIntegrateFreeAndHit);
  EXPECT_EQ(decision.reason, LidarIngestionReason::kNoExpectedSurface);
  EXPECT_FALSE(decision.ground_candidate_considered);
}

TEST(LidarIngestionDecision, DisabledGroundIsDistinctFromUnavailableGround) {
  const Point3 direction{0.6, 0.0, -0.8};
  GroundLidarRejectionConfig disabled{};
  disabled.enabled = false;
  const LidarIngestionDecision disabled_decision =
      evaluateLidarIngestion(observation(direction, 12.44), nullptr, &disabled);
  EXPECT_EQ(disabled_decision.ground_provider,
            LidarExpectedSurfaceProviderStatus::kDisabled);
  EXPECT_EQ(disabled_decision.action, LidarIngestionAction::kIntegrateFreeAndHit);

  GroundLidarRejectionConfig invalid{};
  invalid.ground_altitude_m = std::numeric_limits<double>::quiet_NaN();
  const LidarIngestionDecision invalid_decision =
      evaluateLidarIngestion(observation(direction, 12.44), nullptr, &invalid);
  EXPECT_EQ(invalid_decision.ground_provider,
            LidarExpectedSurfaceProviderStatus::kUnavailable);
  EXPECT_EQ(invalid_decision.action, LidarIngestionAction::kIntegrateFreeAndHit);
}

TEST(LidarIngestionDecision, InvalidGroundDoesNotDisableKnownStaticProvider) {
  const KnownStaticLidarHitClassifier classifier = horizontalKnownSurface();
  GroundLidarRejectionConfig invalid{};
  invalid.closer_range_tolerance_m = -1.0;
  const LidarIngestionDecision decision = evaluateLidarIngestion(
      observation(Point3{1.0, 0.0, 0.0}, 5.0), &classifier, &invalid);

  EXPECT_EQ(decision.ground_provider, LidarExpectedSurfaceProviderStatus::kUnavailable);
  EXPECT_EQ(decision.known_static_provider, LidarExpectedSurfaceProviderStatus::kReady);
  EXPECT_EQ(decision.action, LidarIngestionAction::kIntegrateFreeOnly);
  ASSERT_TRUE(decision.known_static_result_available);
  EXPECT_EQ(decision.known_static_result.classification,
            KnownStaticLidarHitClassification::kExpectedStatic);
}

TEST(LidarIngestionDecision, MissingRequiredAttitudeMakesGroundUnavailable) {
  LidarBeamObservation beam = observation(Point3{0.6, 0.0, -0.8}, 12.44);
  beam.source_attitude_valid = false;
  beam.projection.attitude_compensation_applied = false;
  const GroundLidarRejectionConfig ground{};
  const LidarIngestionDecision decision =
      evaluateLidarIngestion(beam, nullptr, &ground);

  EXPECT_EQ(decision.ground_provider, LidarExpectedSurfaceProviderStatus::kUnavailable);
  EXPECT_EQ(decision.reason, LidarIngestionReason::kClassificationUnavailable);
  EXPECT_EQ(decision.action, LidarIngestionAction::kIntegrateFreeAndHit);
}

TEST(LidarIngestionDecision, TiedExpectedSurfacesSuppressAmbiguousHit) {
  const Point3 direction{0.6, 0.0, -0.8};
  constexpr double kExpectedGroundRangeM = 12.4375;
  const KnownStaticLidarHitClassifier classifier =
      knownSurfaceAtRange(direction, kExpectedGroundRangeM);
  const GroundLidarRejectionConfig ground{};
  const LidarIngestionDecision decision = evaluateLidarIngestion(
      observation(direction, kExpectedGroundRangeM), &classifier, &ground);

  EXPECT_EQ(decision.expected_surface, LidarExpectedSurfaceKind::kTied);
  EXPECT_EQ(decision.reason, LidarIngestionReason::kTiedExpectedSurfaces);
  EXPECT_EQ(decision.action, LidarIngestionAction::kSuppressAllUpdates);
}

TEST(LidarIngestionDecision, NearestProviderSelectionUsesNearestSurface) {
  const Point3 direction{0.6, 0.0, -0.8};
  const GroundLidarRejectionConfig ground{};

  const KnownStaticLidarHitClassifier nearer_known =
      knownSurfaceAtRange(direction, 8.0);
  const LidarIngestionDecision known_decision =
      evaluateLidarIngestion(observation(direction, 8.0), &nearer_known, &ground);
  EXPECT_EQ(known_decision.expected_surface, LidarExpectedSurfaceKind::kKnownStatic);
  EXPECT_EQ(known_decision.reason, LidarIngestionReason::kExpectedKnownStatic);
  EXPECT_EQ(known_decision.action, LidarIngestionAction::kIntegrateFreeOnly);

  const KnownStaticLidarHitClassifier farther_known =
      knownSurfaceAtRange(direction, 20.0);
  const LidarIngestionDecision ground_decision =
      evaluateLidarIngestion(observation(direction, 12.4375), &farther_known, &ground);
  EXPECT_EQ(ground_decision.expected_surface, LidarExpectedSurfaceKind::kGround);
  EXPECT_EQ(ground_decision.reason, LidarIngestionReason::kExpectedGround);
  EXPECT_EQ(ground_decision.action, LidarIngestionAction::kSuppressAllUpdates);
}

TEST(LidarIngestionDecision, ClearlyCloserHitWinsBeforeTiedExpectedSurfaces) {
  const Point3 direction{0.6, 0.0, -0.8};
  constexpr double kExpectedGroundRangeM = 12.4375;
  const KnownStaticLidarHitClassifier classifier =
      knownSurfaceAtRange(direction, kExpectedGroundRangeM);
  const GroundLidarRejectionConfig ground{};
  const LidarIngestionDecision decision =
      evaluateLidarIngestion(observation(direction, 10.0), &classifier, &ground);

  EXPECT_EQ(decision.reason, LidarIngestionReason::kObstacleBeforeExpectedSurface);
  EXPECT_EQ(decision.action, LidarIngestionAction::kIntegrateFreeAndHit);
}

} // namespace drone_city_nav
