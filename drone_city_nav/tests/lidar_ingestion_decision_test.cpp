#include "drone_city_nav/lidar_ingestion_decision.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
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
              .applied_roll_rad = 0.1,
              .applied_pitch_rad = 0.2,
              .applied_tilt_rad = 0.3,
          },
      .measured_range_m = measured_range_m,
      .effective_max_range_m = 30.0,
      .attitude_compensation_required = true,
      .source_attitude_valid = true,
      .source_roll_rad = 0.1,
      .source_pitch_rad = 0.2,
      .source_tilt_rad = 0.3,
  };
}

[[nodiscard]] std::size_t countOccurrences(const std::string& text,
                                           const std::string& needle) {
  std::size_t count = 0U;
  std::size_t offset = 0U;
  while ((offset = text.find(needle, offset)) != std::string::npos) {
    ++count;
    offset += needle.size();
  }
  return count;
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
  ASSERT_TRUE(decision.expected_ground_range_m.has_value());
  EXPECT_NEAR(decision.expected_ground_range_m.value_or(0.0), 12.4375, 1.0e-6);
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
  EXPECT_EQ(decision.reason, LidarIngestionReason::kAmbiguousGround);
}

TEST(LidarIngestionDecision, RejectsGroundIntersectionAtRayOrigin) {
  const Point3 direction{0.6, 0.0, -0.8};
  LidarBeamObservation beam = observation(direction, 1.0);
  beam.projection.ray_origin_map_m.z = 0.05;
  beam.projection.endpoint_map_m.z = 0.05 + direction.z;
  beam.projection.endpoint_altitude_m = beam.projection.endpoint_map_m.z;
  const GroundLidarRejectionConfig ground{};

  const LidarIngestionDecision decision =
      evaluateLidarIngestion(beam, nullptr, &ground);

  EXPECT_FALSE(decision.ground_candidate_considered);
  EXPECT_EQ(decision.reason, LidarIngestionReason::kNoExpectedSurface);
  EXPECT_EQ(decision.action, LidarIngestionAction::kIntegrateFreeAndHit);
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

TEST(LidarIngestionDecision, AmbiguousEvidenceCountsScansRatherThanIndividualBeams) {
  AmbiguousLidarHitTracker tracker;
  LidarIngestionDecision ambiguous{};
  ambiguous.action = LidarIngestionAction::kSuppressAllUpdates;
  ambiguous.reason = LidarIngestionReason::kAmbiguousKnownStatic;
  ambiguous.expected_surface = LidarExpectedSurfaceKind::kKnownStatic;
  ambiguous.known_static_result_available = true;
  ambiguous.known_static_result.classification =
      KnownStaticLidarHitClassification::kAmbiguous;
  ambiguous.known_static_result.endpoint_relation =
      KnownStaticEndpointRelation::kNearSurface;
  ambiguous.known_static_result.structure_id = "building";
  ambiguous.known_static_result.part_id = "upper_mass";

  LidarBeamObservation first_beam = observation(Point3{1.0, 0.0, 0.0}, 5.0);
  first_beam.scan_stamp_ns = 100'000'000;
  first_beam.scan_stamp_valid = true;
  first_beam.acquisition_stamp_ns = 100'100'000;
  first_beam.acquisition_stamp_valid = true;
  const LidarIngestionDecision first =
      resolveAmbiguousKnownStaticIngestion(first_beam, ambiguous, &tracker);
  EXPECT_EQ(first.ambiguous_evidence_count, 1U);

  LidarBeamObservation later_beam_same_scan = first_beam;
  later_beam_same_scan.acquisition_stamp_ns = 100'900'000;
  later_beam_same_scan.projection.ray_origin_map_m.x = 1.0;
  later_beam_same_scan.projection.ray_direction_map = Point3{0.0, 1.0, 0.0};
  const LidarIngestionDecision same_scan =
      resolveAmbiguousKnownStaticIngestion(later_beam_same_scan, ambiguous, &tracker);
  EXPECT_EQ(same_scan.ambiguous_evidence_count, 1U);

  LidarBeamObservation next_scan = later_beam_same_scan;
  next_scan.scan_stamp_ns = 200'000'000;
  const LidarIngestionDecision second =
      resolveAmbiguousKnownStaticIngestion(next_scan, ambiguous, &tracker);
  EXPECT_EQ(second.ambiguous_evidence_count, 2U);
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
  EXPECT_EQ(decision.action, LidarIngestionAction::kSuppressAllUpdates);
  ASSERT_TRUE(decision.known_static_result_available);
  EXPECT_EQ(decision.known_static_result.classification,
            KnownStaticLidarHitClassification::kExpectedStatic);
}

TEST(LidarIngestionDecision, CloserSideStaticCountersDistinguishDecisionState) {
  const KnownStaticLidarHitClassifier classifier = horizontalKnownSurface();
  const LidarBeamObservation suppressed_beam = observation(Point3{1.0, 0.0, 0.0}, 4.7);
  const LidarIngestionDecision suppressed =
      evaluateLidarIngestion(suppressed_beam, &classifier, nullptr);
  ASSERT_EQ(suppressed.reason, LidarIngestionReason::kExpectedKnownStatic);

  const LidarBeamObservation pending_beam = observation(Point3{1.0, 0.0, 0.0}, 4.34);
  const LidarIngestionDecision pending =
      evaluateLidarIngestion(pending_beam, &classifier, nullptr);
  ASSERT_EQ(pending.reason, LidarIngestionReason::kAmbiguousKnownStatic);

  LidarIngestionDecision confirmed = pending;
  confirmed.reason = LidarIngestionReason::kExpectedKnownStatic;
  confirmed.ambiguous_resolution =
      AmbiguousLidarHitResolution::kConfirmedStaticAttached;
  confirmed.known_static_result.classification =
      KnownStaticLidarHitClassification::kExpectedStatic;

  LidarIngestionDecisionStats stats;
  recordLidarIngestionDecision(suppressed_beam, suppressed, false, stats);
  recordLidarIngestionDecision(pending_beam, pending, false, stats);
  recordLidarIngestionDecision(pending_beam, confirmed, false, stats);

  EXPECT_EQ(stats.closer_side_static_suppressed, 1U);
  EXPECT_EQ(stats.closer_side_static_pending, 1U);
  EXPECT_EQ(stats.closer_side_static_confirmed, 1U);
}

TEST(LidarIngestionDecision,
     EndpointNearKnownSurfaceBeyondEffectiveRangeIsStillSuppressed) {
  constexpr double kMeasuredRangeM = 34.94;
  constexpr double kEffectiveMaxRangeM = 35.0;
  constexpr double kKnownSurfaceRangeM = 35.1;
  const Point3 direction{1.0, 0.0, 0.0};
  const KnownStaticLidarHitClassifier classifier =
      knownSurfaceAtRange(direction, kKnownSurfaceRangeM);
  LidarBeamObservation beam = observation(direction, kMeasuredRangeM);
  beam.effective_max_range_m = kEffectiveMaxRangeM;

  const LidarIngestionDecision decision =
      evaluateLidarIngestion(beam, &classifier, nullptr);

  EXPECT_EQ(decision.expected_surface, LidarExpectedSurfaceKind::kKnownStatic);
  EXPECT_EQ(decision.reason, LidarIngestionReason::kExpectedKnownStatic);
  EXPECT_EQ(decision.action, LidarIngestionAction::kSuppressAllUpdates);
  EXPECT_TRUE(decision.known_static_result_available);
}

TEST(LidarIngestionDecision,
     DistantKnownSurfaceOutsideLidarRangeIsAnOrdinaryUnknownObstacle) {
  const Point3 direction{1.0, 0.0, 0.0};
  const KnownStaticLidarHitClassifier classifier =
      knownSurfaceAtRange(direction, 212.0);
  LidarBeamObservation beam = observation(direction, 29.0);
  beam.effective_max_range_m = 30.0;

  const LidarIngestionDecision decision =
      evaluateLidarIngestion(beam, &classifier, nullptr);

  EXPECT_EQ(decision.action, LidarIngestionAction::kIntegrateFreeAndHit);
  EXPECT_EQ(decision.reason, LidarIngestionReason::kNoExpectedSurface);
  EXPECT_EQ(decision.expected_surface, LidarExpectedSurfaceKind::kNone);
  EXPECT_TRUE(std::isnan(decision.expected_range_m));
  EXPECT_TRUE(std::isnan(decision.range_delta_m));
  EXPECT_FALSE(decision.known_static_result_available);
  EXPECT_EQ(validateAcceptedLidarIngestionDecision(
                makeLidarIngestionDecisionSnapshot(decision)),
            LidarIngestionDecisionValidation::kValid);
}

TEST(LidarIngestionDecision,
     MalformedAcceptedKnownStaticDecisionFallsBackWithoutDroppingObstacle) {
  const LidarBeamObservation beam = observation(Point3{1.0, 0.0, 0.0}, 29.0);
  LidarIngestionDecision malformed{};
  malformed.action = LidarIngestionAction::kIntegrateFreeAndHit;
  malformed.reason = LidarIngestionReason::kUnexpectedKnownStatic;
  malformed.expected_surface = LidarExpectedSurfaceKind::kKnownStatic;
  malformed.expected_range_m = std::numeric_limits<double>::quiet_NaN();
  malformed.range_delta_m = -183.0;
  malformed.known_static_result_available = true;
  malformed.known_static_result.volume_matched = true;
  malformed.known_static_result.structure_id = "distant_building";
  LidarIngestionDecisionStats stats;

  const LidarIngestionDecision normalized =
      normalizeAcceptedLidarIngestionDecision(beam, malformed, stats);

  EXPECT_EQ(normalized.action, LidarIngestionAction::kIntegrateFreeAndHit);
  EXPECT_EQ(normalized.reason, LidarIngestionReason::kNoExpectedSurface);
  EXPECT_EQ(normalized.expected_surface, LidarExpectedSurfaceKind::kNone);
  EXPECT_TRUE(std::isnan(normalized.expected_range_m));
  EXPECT_TRUE(std::isnan(normalized.range_delta_m));
  EXPECT_FALSE(normalized.known_static_result_available);
  EXPECT_EQ(stats.invariant_fallbacks, 1U);
  ASSERT_EQ(stats.diagnostics.size(), 1U);
  EXPECT_EQ(stats.diagnostics.front().diagnostic_class,
            LidarIngestionDiagnosticClass::kInvariantFallback);
  EXPECT_EQ(stats.diagnostics.front().reason,
            LidarIngestionReason::kUnexpectedKnownStatic);
}

TEST(LidarIngestionDecision, AcceptedEvaluatorDecisionsSatisfySharedInvariant) {
  const Point3 horizontal{1.0, 0.0, 0.0};
  const Point3 downward{0.6, 0.0, -0.8};
  const GroundLidarRejectionConfig ground{};
  const KnownStaticLidarHitClassifier known_surface = horizontalKnownSurface();

  std::vector<KnownPassageSolidVolume> opening_volumes;
  opening_volumes.push_back(KnownPassageSolidVolume{
      .structure_id = "opening_building",
      .opening_id = "opening",
      .part_id = "upper_mass",
      .part_kind = KnownPassageSolidPartKind::kUpper,
      .center = Point2{6.0, 0.0},
      .normal_xy = Point2{1.0, 0.0},
      .lateral_xy = Point2{0.0, 1.0},
      .depth_m = 2.0,
      .width_m = 4.0,
      .min_z_m = 12.0,
      .max_z_m = 16.0,
      .opening_center = Point2{6.0, 0.0},
      .opening_depth_m = 2.0,
      .opening_width_m = 4.0,
      .opening_min_z_m = 8.0,
      .opening_max_z_m = 12.0,
  });
  const KnownStaticLidarHitClassifier opening_surface{std::move(opening_volumes)};

  const std::array<LidarIngestionDecision, 4U> decisions{
      evaluateLidarIngestion(observation(horizontal, 8.0), nullptr, nullptr),
      evaluateLidarIngestion(observation(downward, 9.0), nullptr, &ground),
      evaluateLidarIngestion(observation(horizontal, 3.0), &known_surface, nullptr),
      evaluateLidarIngestion(observation(horizontal, 6.0), &opening_surface, nullptr),
  };
  for (const LidarIngestionDecision& decision : decisions) {
    ASSERT_EQ(decision.action, LidarIngestionAction::kIntegrateFreeAndHit);
    EXPECT_EQ(validateAcceptedLidarIngestionDecision(
                  makeLidarIngestionDecisionSnapshot(decision)),
              LidarIngestionDecisionValidation::kValid);
  }
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
  EXPECT_EQ(known_decision.action, LidarIngestionAction::kSuppressAllUpdates);

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

TEST(LidarIngestionDecision, TiedGroundAltitudeRejectionIsNotCountedAsNonGround) {
  const Point3 direction{0.6, 0.0, -0.8};
  constexpr double kExpectedGroundRangeM = 12.4375;
  const KnownStaticLidarHitClassifier classifier =
      knownSurfaceAtRange(direction, kExpectedGroundRangeM);
  const GroundLidarRejectionConfig ground{};
  const LidarBeamObservation beam = observation(direction, kExpectedGroundRangeM);
  const LidarIngestionDecision decision =
      evaluateLidarIngestion(beam, &classifier, &ground);
  LidarIngestionDecisionStats stats;

  recordLidarIngestionDecision(beam, decision, true, stats);

  EXPECT_EQ(stats.ambiguous_ground_suppressed, 1U);
  EXPECT_EQ(stats.non_ground_altitude_rejected, 0U);
}

TEST(LidarIngestionDecision, DiagnosticClassesHaveIndependentBounds) {
  const Point3 direction{0.6, 0.0, -0.8};
  const GroundLidarRejectionConfig ground{};
  const LidarBeamObservation expected_beam = observation(direction, 12.44);
  const LidarIngestionDecision expected =
      evaluateLidarIngestion(expected_beam, nullptr, &ground);
  LidarIngestionDecisionStats stats;
  for (std::size_t i = 0U; i < 32U; ++i) {
    recordLidarIngestionDecision(expected_beam, expected, false, stats);
  }

  const LidarBeamObservation closer_beam = observation(direction, 9.0);
  recordLidarIngestionDecision(
      closer_beam, evaluateLidarIngestion(closer_beam, nullptr, &ground), false, stats);
  const LidarBeamObservation ambiguous_beam = observation(direction, 30.0, false);
  recordLidarIngestionDecision(ambiguous_beam,
                               evaluateLidarIngestion(ambiguous_beam, nullptr, &ground),
                               false, stats);

  GroundLidarRejectionConfig invalid_ground{};
  invalid_ground.ground_altitude_m = std::numeric_limits<double>::quiet_NaN();
  const KnownStaticLidarHitClassifier classifier = horizontalKnownSurface();
  const LidarBeamObservation known_beam = observation(Point3{1.0, 0.0, 0.0}, 5.0);
  const LidarIngestionDecision known_decision =
      evaluateLidarIngestion(known_beam, &classifier, &invalid_ground);
  recordLidarIngestionDecision(known_beam, known_decision, false, stats);

  const auto count_class = [&stats](const LidarIngestionDiagnosticClass target) {
    return std::count_if(stats.diagnostics.begin(), stats.diagnostics.end(),
                         [target](const LidarIngestionDecisionDiagnostic& diagnostic) {
                           return diagnostic.diagnostic_class == target;
                         });
  };
  EXPECT_EQ(count_class(LidarIngestionDiagnosticClass::kExpectedGround), 4);
  EXPECT_EQ(count_class(LidarIngestionDiagnosticClass::kCloserObstacle), 1);
  EXPECT_EQ(count_class(LidarIngestionDiagnosticClass::kAmbiguousGround), 1);
  EXPECT_EQ(count_class(LidarIngestionDiagnosticClass::kClassificationUnavailable), 1);

  const LidarIngestionRepresentativeDiagnostics representatives =
      representativeLidarIngestionDiagnostics(stats);
  EXPECT_EQ(representatives.count, 4U);
  const std::string formatted = formatLidarIngestionRepresentativeDiagnostics(stats);
  EXPECT_NE(
      formatted.find("class=classification_unavailable reason=expected_known_static"),
      std::string::npos);
  EXPECT_EQ(countOccurrences(formatted, "ray_origin=("), representatives.count);
  EXPECT_EQ(countOccurrences(formatted, "ray_dir=("), representatives.count);
  EXPECT_EQ(countOccurrences(formatted, "source_attitude=(valid=true"),
            representatives.count);
  EXPECT_EQ(countOccurrences(formatted, "applied_attitude=(applied=true"),
            representatives.count);
  EXPECT_EQ(countOccurrences(formatted, "roll=0.1 pitch=0.2 tilt=0.3"),
            representatives.count * 2U);
}

} // namespace drone_city_nav
