#include "drone_city_nav/static_route_extension.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::vector<RouteSample3D> route(const double endpoint_x) {
  return {
      RouteSample3D{.position = Point3{1.5, 1.5, 1.5}},
      RouteSample3D{.position = Point3{endpoint_x, 1.5, 1.5}},
  };
}

TEST(StaticRouteExtensionTest, RequestsResidentExtensionUsingSearchLatency) {
  const StaticRouteExtensionDecision decision = evaluateStaticRouteExtension(
      StaticRouteExtensionConfig{},
      StaticRouteExtensionObservation{.route_generation = 4U,
                                      .route_station_m = 70.0,
                                      .route_remaining_m = 55.0,
                                      .horizontal_speed_mps = 20.0,
                                      .guide_search_latency_ms = 100.0,
                                      .esdf_build_latency_ms = 6000.0});

  EXPECT_TRUE(decision.request_extension);
  EXPECT_FALSE(decision.request_roi_refresh);
  EXPECT_DOUBLE_EQ(decision.extension_trigger_remaining_m, 57.0);
}

TEST(StaticRouteExtensionTest, RequestsEarlyRoiRefreshOnlyWhenGoalLeavesEsdf) {
  StaticRouteExtensionObservation observation{
      .route_generation = 4U,
      .route_station_m = 10.0,
      .route_remaining_m = 120.0,
      .horizontal_speed_mps = 15.0,
      .guide_search_latency_ms = 100.0,
      .esdf_build_latency_ms = 6000.0,
      .next_planning_goal_inside_esdf = false,
  };
  StaticRouteExtensionDecision decision =
      evaluateStaticRouteExtension(StaticRouteExtensionConfig{}, observation);
  EXPECT_TRUE(decision.request_roi_refresh);
  EXPECT_FALSE(decision.request_extension);

  observation.next_planning_goal_inside_esdf = true;
  decision = evaluateStaticRouteExtension(StaticRouteExtensionConfig{}, observation);
  EXPECT_FALSE(decision.request_roi_refresh);
  EXPECT_FALSE(decision.request_extension);
}

TEST(StaticRouteExtensionTest, DoesNotDuplicateRequestForSameGenerationAndStation) {
  const StaticRouteExtensionDecision decision = evaluateStaticRouteExtension(
      StaticRouteExtensionConfig{},
      StaticRouteExtensionObservation{.route_generation = 4U,
                                      .route_station_m = 80.0,
                                      .route_remaining_m = 10.0,
                                      .last_request_generation = 4U,
                                      .last_request_station_m = 70.0});
  EXPECT_FALSE(decision.request_extension);
  EXPECT_FALSE(decision.request_roi_refresh);
}

TEST(StaticRouteExtensionTest, RetriesAfterIntervalWithoutVehicleProgress) {
  const StaticRouteExtensionDecision decision = evaluateStaticRouteExtension(
      StaticRouteExtensionConfig{.minimum_retry_interval_s = 1.0},
      StaticRouteExtensionObservation{.route_generation = 4U,
                                      .route_station_m = 80.0,
                                      .route_remaining_m = 10.0,
                                      .last_request_generation = 4U,
                                      .last_request_station_m = 80.0,
                                      .request_stamp_ns = 2'000'000'000,
                                      .last_request_stamp_ns = 500'000'000});
  EXPECT_TRUE(decision.request_extension);
}

TEST(StaticRouteExtensionTest, MissionTerminalRouteIsNeverExtended) {
  const StaticRouteExtensionDecision decision = evaluateStaticRouteExtension(
      StaticRouteExtensionConfig{},
      StaticRouteExtensionObservation{.route_generation = 4U,
                                      .route_remaining_m = 0.0,
                                      .route_reaches_mission_goal = true});
  EXPECT_FALSE(decision.request_extension);
  EXPECT_FALSE(decision.request_roi_refresh);
}

TEST(StaticRouteExtensionTest, DefersLifecycleReleaseButNeverRawBlockedRelease) {
  EXPECT_TRUE(deferStaticRouteReleaseDuringExtension(
      true, GlobalGuideReleaseReason::kExhausted));
  EXPECT_TRUE(deferStaticRouteReleaseDuringExtension(
      true, GlobalGuideReleaseReason::kPersistentSafetyRejection));
  EXPECT_FALSE(
      deferStaticRouteReleaseDuringExtension(true, GlobalGuideReleaseReason::kBlocked));
  EXPECT_FALSE(deferStaticRouteReleaseDuringExtension(
      false, GlobalGuideReleaseReason::kStalled));
}

TEST(StaticRouteExtensionTest, CandidateMustImproveEndpointAndAvoidRawOccupancy) {
  const mppi::EsdfGrid grid{12, 4, 1.0F, 0.0F, 0.0F, 4, 0.0F};
  std::vector<float> esdf(static_cast<std::size_t>(12U) * 4U * 4U,
                          std::numeric_limits<float>::infinity());
  const std::vector<RouteSample3D> active = route(5.5);
  const std::vector<RouteSample3D> improved = route(8.5);

  EXPECT_TRUE(validateStaticRouteCandidate(active, improved, grid, esdf,
                                           Point3{11.5, 1.5, 1.5}, 2.0, false, {})
                  .accepted);
  EXPECT_EQ(validateStaticRouteCandidate(active, route(6.5), grid, esdf,
                                         Point3{11.5, 1.5, 1.5}, 2.0, false, {})
                .status,
            StaticRouteCandidateStatus::kNoEndpointImprovement);

  const std::size_t occupied_index = (1U * 4U + 1U) * 12U + 8U;
  esdf[occupied_index] = 0.0F;
  EXPECT_EQ(validateStaticRouteCandidate(active, improved, grid, esdf,
                                         Point3{11.5, 1.5, 1.5}, 2.0, false, {})
                .status,
            StaticRouteCandidateStatus::kRawCollision);
}

TEST(StaticRouteExtensionTest, RequiredContinuationMayTemporarilyLoseGoalProgress) {
  const mppi::EsdfGrid grid{12, 4, 1.0F, 0.0F, 0.0F, 4, 0.0F};
  const std::vector<float> esdf(static_cast<std::size_t>(12U) * 4U * 4U,
                                std::numeric_limits<float>::infinity());

  const StaticRouteCandidateValidation result = validateStaticRouteCandidate(
      route(8.5), route(6.5), grid, esdf, Point3{11.5, 1.5, 1.5}, 5.0, false,
      FlightEnvelopeConfig{}, true);

  EXPECT_TRUE(result.accepted);
}

TEST(StaticRouteExtensionTest, RejectsRouteOutsideFlightEnvelope) {
  const mppi::EsdfGrid grid{12, 4, 1.0F, 0.0F, 0.0F, 40, 0.0F};
  const std::vector<float> esdf(static_cast<std::size_t>(12U) * 4U * 40U,
                                std::numeric_limits<float>::infinity());
  std::vector<RouteSample3D> invalid = route(8.5);
  invalid.back().position.z = 32.0;
  EXPECT_EQ(validateStaticRouteCandidate({}, invalid, grid, esdf,
                                         Point3{11.5, 1.5, 1.5}, 0.0, false, {})
                .status,
            StaticRouteCandidateStatus::kOutsideFlightEnvelope);
}

TEST(StaticRouteExtensionTest, PlanningGoalAndEsdfBoundaryAreExplicit) {
  const Point3 planning_goal =
      staticRoutePlanningGoal(Point3{0.0, 0.0, 2.0}, Point3{100.0, 0.0, 2.0}, 40.0);
  EXPECT_DOUBLE_EQ(planning_goal.x, 40.0);
  const mppi::EsdfGrid grid{40, 20, 1.0F, 0.0F, -10.0F, 10, 0.0F};
  EXPECT_FALSE(staticRoutePointInsideEsdf(grid, planning_goal));
  EXPECT_TRUE(staticRoutePointInsideEsdf(grid, Point3{39.5, 0.0, 2.0}));
}

TEST(StaticRouteExtensionTest, CoalescesReplanForOneRouteGeneration) {
  StaticRouteReplanGate gate;

  EXPECT_TRUE(gate.tryBegin(7U));
  EXPECT_FALSE(gate.tryBegin(7U));
  EXPECT_FALSE(gate.tryBegin(8U));
  EXPECT_TRUE(gate.inFlight());
  EXPECT_EQ(gate.generation(), 7U);

  gate.finish(8U);
  EXPECT_TRUE(gate.inFlight());
  gate.finish(7U);
  EXPECT_FALSE(gate.inFlight());
  EXPECT_TRUE(gate.tryBegin(7U));
}

TEST(StaticRouteExtensionTest, SuppressesEquivalentFailedSearchUntilRetryInterval) {
  StaticRouteFailedSearchLatch latch;
  const StaticRouteSearchContext failure{
      .base_route_generation = 7U,
      .search_start = Point3{10.0, 20.0, 18.0},
      .objective = StaticRouteObjective{.goal = Point3{100.0, 200.0, 18.0},
                                        .mission_epoch = 3U,
                                        .sample_sequence = 40U,
                                        .continuous_tracking = true,
                                        .available = true},
      .minimum_tracking_sample_sequence = 30U,
      .stamp_ns = 1'000'000'000,
  };
  latch.recordFailure(failure);

  StaticRouteSearchContext retry = failure;
  retry.objective.sample_sequence = 41U;
  retry.stamp_ns = 1'500'000'000;
  StaticRouteSearchRetryDecision decision =
      latch.evaluate(StaticRouteSearchRetryConfig{}, retry);
  EXPECT_FALSE(decision.allow);
  EXPECT_EQ(decision.trigger, StaticRouteSearchRetryTrigger::kSuppressed);

  retry.stamp_ns = 2'000'000'000;
  decision = latch.evaluate(StaticRouteSearchRetryConfig{}, retry);
  EXPECT_TRUE(decision.allow);
  EXPECT_EQ(decision.trigger, StaticRouteSearchRetryTrigger::kRetryIntervalElapsed);
}

TEST(StaticRouteExtensionTest, FailedSearchRetriesAfterMeaningfulStateChange) {
  StaticRouteFailedSearchLatch latch;
  const StaticRouteSearchContext failure{
      .base_route_generation = 7U,
      .search_start = Point3{10.0, 20.0, 18.0},
      .objective = StaticRouteObjective{.goal = Point3{100.0, 200.0, 18.0},
                                        .mission_epoch = 3U,
                                        .sample_sequence = 40U,
                                        .continuous_tracking = true,
                                        .available = true},
      .minimum_tracking_sample_sequence = 30U,
      .stamp_ns = 1'000'000'000,
  };
  latch.recordFailure(failure);

  StaticRouteSearchContext changed = failure;
  changed.search_start.x += 2.0;
  changed.stamp_ns += 100'000'000;
  EXPECT_EQ(latch.evaluate(StaticRouteSearchRetryConfig{}, changed).trigger,
            StaticRouteSearchRetryTrigger::kPoseChanged);

  changed = failure;
  changed.objective.goal.y += 5.0;
  changed.stamp_ns += 100'000'000;
  EXPECT_EQ(latch.evaluate(StaticRouteSearchRetryConfig{}, changed).trigger,
            StaticRouteSearchRetryTrigger::kObjectiveChanged);

  changed = failure;
  changed.minimum_tracking_sample_sequence += 1U;
  changed.stamp_ns += 100'000'000;
  EXPECT_EQ(latch.evaluate(StaticRouteSearchRetryConfig{}, changed).trigger,
            StaticRouteSearchRetryTrigger::kObjectiveChanged);

  changed = failure;
  changed.base_route_generation += 1U;
  changed.stamp_ns += 100'000'000;
  EXPECT_EQ(latch.evaluate(StaticRouteSearchRetryConfig{}, changed).trigger,
            StaticRouteSearchRetryTrigger::kRouteGenerationChanged);
}

TEST(StaticRouteExtensionTest, SuccessfulSearchClearsFailureLatch) {
  StaticRouteFailedSearchLatch latch;
  latch.recordFailure(StaticRouteSearchContext{.base_route_generation = 7U});
  ASSERT_TRUE(latch.latched());

  latch.clear();

  EXPECT_FALSE(latch.latched());
  EXPECT_EQ(latch.evaluate(StaticRouteSearchRetryConfig{}, {}).trigger,
            StaticRouteSearchRetryTrigger::kNoFailure);
}

TEST(StaticRouteExtensionTest, RepeatedRoiRefreshUsesUniqueRequestSequence) {
  StaticRouteRoiRefreshLifecycle lifecycle;

  const StaticRouteRoiRefreshRequest first = lifecycle.queue(7U);
  EXPECT_TRUE(lifecycle.pending(first));
  lifecycle.complete(first.sequence);
  EXPECT_FALSE(lifecycle.pending(first));

  const StaticRouteRoiRefreshRequest second = lifecycle.queue(7U);
  EXPECT_GT(second.sequence, first.sequence);
  EXPECT_EQ(second.base_route_generation, first.base_route_generation);
  EXPECT_TRUE(lifecycle.pending(second));
}

TEST(StaticRouteExtensionTest, PreservesTrackingRefreshPurpose) {
  StaticRouteRoiRefreshLifecycle lifecycle;

  const StaticRouteRoiRefreshRequest request =
      lifecycle.queue(9U, StaticRouteRoiRefreshRequest::Purpose::kTrackingObjective);

  EXPECT_EQ(request.purpose, StaticRouteRoiRefreshRequest::Purpose::kTrackingObjective);
  EXPECT_EQ(lifecycle.latest().purpose,
            StaticRouteRoiRefreshRequest::Purpose::kTrackingObjective);
}

TEST(StaticRouteExtensionTest, RejectsRouteOlderThanLosHandoffSample) {
  const StaticRouteObjective current{.goal = Point3{100.0, 50.0, 18.0},
                                     .mission_epoch = 7U,
                                     .sample_sequence = 120U,
                                     .continuous_tracking = true,
                                     .available = true};
  StaticRouteObjective route = current;
  route.sample_sequence = 119U;

  EXPECT_FALSE(staticRouteObjectiveMatches(route, current, 120U, 5.0));
  route.sample_sequence = 120U;
  EXPECT_TRUE(staticRouteObjectiveMatches(route, current, 120U, 5.0));
}

TEST(StaticRouteExtensionTest, RequiresSameEpochAndNearbyTrackingObjective) {
  const StaticRouteObjective current{.goal = Point3{100.0, 50.0, 18.0},
                                     .mission_epoch = 7U,
                                     .sample_sequence = 120U,
                                     .continuous_tracking = true,
                                     .available = true};
  StaticRouteObjective route = current;
  route.mission_epoch = 6U;
  EXPECT_FALSE(staticRouteObjectiveMatches(route, current, 0U, 5.0));

  route = current;
  route.goal.x += 6.0;
  EXPECT_FALSE(staticRouteObjectiveMatches(route, current, 0U, 5.0));
}

TEST(StaticRouteExtensionTest, ProtectsConstrainedSuffixThroughDeparture) {
  const std::vector<RouteSample3D> active =
      sampleRoute3D(std::vector<Point3>{{0.0, 0.0, 5.0}, {20.0, 0.0, 5.0}}, 1.0, 10.0);
  const std::vector<ConstrainedRouteSpan> spans{ConstrainedRouteSpan{
      .channel_id = "channel",
      .route_generation = 3U,
      .direction_sign = 1,
      .begin_station_m = 5.0,
      .end_station_m = 12.0,
      .envelope = {},
  }};

  EXPECT_TRUE(staticRouteHasProtectedConstrainedSuffix(active, spans,
                                                       Point3{8.0, 0.0, 5.0}, 3.0));
  EXPECT_TRUE(staticRouteHasProtectedConstrainedSuffix(active, spans,
                                                       Point3{14.0, 0.0, 5.0}, 3.0));
  EXPECT_FALSE(staticRouteHasProtectedConstrainedSuffix(active, spans,
                                                        Point3{16.0, 0.0, 5.0}, 3.0));
}

TEST(StaticRouteExtensionTest, ActivationStatusesHaveStableDiagnosticNames) {
  EXPECT_EQ(staticRouteActivationStatusName(
                StaticRouteActivationStatus::kCandidateValidationRejected),
            "candidate_validation_rejected");
  EXPECT_EQ(
      staticRouteActivationStatusName(StaticRouteActivationStatus::kStaleWorldRevision),
      "stale_world_revision");
  EXPECT_EQ(staticRouteActivationStatusName(
                StaticRouteActivationStatus::kStaleRouteGeneration),
            "stale_route_generation");
  EXPECT_EQ(
      staticRouteActivationStatusName(StaticRouteActivationStatus::kStaleObjective),
      "stale_objective");
}

} // namespace
} // namespace drone_city_nav
