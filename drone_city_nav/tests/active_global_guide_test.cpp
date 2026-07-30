#include "drone_city_nav/active_global_guide.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] mppi::EsdfGrid grid() {
  return mppi::EsdfGrid{80, 40, 1.0F, 0.0F, 0.0F};
}

[[nodiscard]] std::vector<float> clearEsdf() {
  const mppi::EsdfGrid model = grid();
  return std::vector<float>(static_cast<std::size_t>(model.width * model.height),
                            20.0F);
}

[[nodiscard]] std::shared_ptr<const std::vector<Point2>> straightGuide() {
  return std::make_shared<const std::vector<Point2>>(
      std::vector<Point2>{{2.5, 10.5}, {22.5, 10.5}, {42.5, 10.5}});
}

TEST(ActiveGlobalGuideTest, RetainsClearGuideAcrossWorldUpdates) {
  ActiveGlobalGuideLifecycle lifecycle;
  const std::vector<float> esdf = clearEsdf();

  ASSERT_TRUE(lifecycle.accept(straightGuide(), false, grid(), esdf, Point2{2.5, 10.5})
                  .accepted);
  const ActiveGlobalGuideUpdate update =
      lifecycle.update(grid(), esdf, Point2{8.5, 10.5}, 0U);

  EXPECT_TRUE(update.active);
  EXPECT_TRUE(update.retained);
  EXPECT_FALSE(update.requires_replan);
  EXPECT_EQ(update.release_reason, GlobalGuideReleaseReason::kNone);
  EXPECT_NEAR(update.projection.station_m, 6.0, 1.0e-9);
  EXPECT_NEAR(update.projection.remaining_m, 34.0, 1.0e-9);
}

TEST(ActiveGlobalGuideTest, RetainsGuideAndRequestsBackgroundReplanForCriticalBand) {
  ActiveGlobalGuideLifecycle lifecycle;
  std::vector<float> esdf = clearEsdf();
  ASSERT_TRUE(lifecycle.accept(straightGuide(), false, grid(), esdf, Point2{2.5, 10.5})
                  .accepted);
  esdf[10U * 80U + 20U] = 0.9F;

  const ActiveGlobalGuideUpdate update =
      lifecycle.update(grid(), esdf, Point2{8.5, 10.5}, 0U);

  EXPECT_TRUE(update.active);
  EXPECT_TRUE(update.retained);
  EXPECT_TRUE(update.requires_replan);
  EXPECT_EQ(update.release_reason, GlobalGuideReleaseReason::kNone);
}

TEST(ActiveGlobalGuideTest, KeepsGuideAcceptedInsideCriticalBand) {
  ActiveGlobalGuideLifecycle lifecycle;
  std::vector<float> esdf = clearEsdf();
  esdf[10U * 80U + 20U] = 0.9F;
  ASSERT_TRUE(lifecycle.accept(straightGuide(), false, grid(), esdf, Point2{2.5, 10.5})
                  .accepted);

  const ActiveGlobalGuideUpdate update =
      lifecycle.update(grid(), esdf, Point2{8.5, 10.5}, 0U);

  EXPECT_TRUE(update.active);
  EXPECT_TRUE(update.requires_replan);
  EXPECT_EQ(update.current_risk, GlobalGuideRiskTier::kCritical);
}

TEST(ActiveGlobalGuideTest, AcceptsPositiveInfinityAsTruncatedPreferredClearance) {
  ActiveGlobalGuideLifecycle lifecycle;
  const mppi::EsdfGrid model = grid();
  const std::vector<float> esdf(static_cast<std::size_t>(model.width * model.height),
                                std::numeric_limits<float>::infinity());

  const GlobalGuideAcceptanceResult result =
      lifecycle.accept(straightGuide(), false, model, esdf, Point2{2.5, 10.5});

  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.reason, GlobalGuideAcceptanceReason::kAccepted);
  EXPECT_EQ(result.risk, GlobalGuideRiskTier::kPreferred);
}

TEST(ActiveGlobalGuideTest, RejectsNanClearanceAsInvalid) {
  ActiveGlobalGuideLifecycle lifecycle;
  std::vector<float> esdf = clearEsdf();
  esdf[10U * 80U + 20U] = std::numeric_limits<float>::quiet_NaN();

  const GlobalGuideAcceptanceResult result =
      lifecycle.accept(straightGuide(), false, grid(), esdf, Point2{2.5, 10.5});

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, GlobalGuideAcceptanceReason::kInvalidClearance);
  EXPECT_EQ(result.risk, GlobalGuideRiskTier::kCollision);
}

TEST(ActiveGlobalGuideTest, RejectsNegativeInfinityClearanceAsInvalid) {
  ActiveGlobalGuideLifecycle lifecycle;
  std::vector<float> esdf = clearEsdf();
  esdf[10U * 80U + 20U] = -std::numeric_limits<float>::infinity();

  const GlobalGuideAcceptanceResult result =
      lifecycle.accept(straightGuide(), false, grid(), esdf, Point2{2.5, 10.5});

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, GlobalGuideAcceptanceReason::kInvalidClearance);
  EXPECT_EQ(result.risk, GlobalGuideRiskTier::kCollision);
}

TEST(ActiveGlobalGuideTest, RejectsGuideLeavingGrid) {
  ActiveGlobalGuideLifecycle lifecycle;
  const std::vector<float> esdf = clearEsdf();
  const auto guide = std::make_shared<const std::vector<Point2>>(
      std::vector<Point2>{{2.5, 10.5}, {82.5, 10.5}});

  const GlobalGuideAcceptanceResult result =
      lifecycle.accept(guide, false, grid(), esdf, Point2{2.5, 10.5});

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, GlobalGuideAcceptanceReason::kOutsideGrid);
  EXPECT_EQ(result.risk, GlobalGuideRiskTier::kCollision);
}

TEST(ActiveGlobalGuideTest, RetainsNonTerminalGuideWhileRequestingExtension) {
  ActiveGlobalGuideConfig config;
  config.minimum_remaining_m = 15.0;
  ActiveGlobalGuideLifecycle lifecycle{config};
  const std::vector<float> esdf = clearEsdf();
  ASSERT_TRUE(lifecycle.accept(straightGuide(), false, grid(), esdf, Point2{2.5, 10.5})
                  .accepted);

  const ActiveGlobalGuideUpdate update =
      lifecycle.update(grid(), esdf, Point2{32.5, 10.5}, 0U);

  EXPECT_EQ(update.release_reason, GlobalGuideReleaseReason::kExhausted);
  EXPECT_TRUE(update.active);
  EXPECT_TRUE(update.retained);
  EXPECT_TRUE(update.requires_replan);
}

TEST(ActiveGlobalGuideTest, KeepsExactMissionGoalGuideNearItsEnd) {
  ActiveGlobalGuideLifecycle lifecycle;
  const std::vector<float> esdf = clearEsdf();
  ASSERT_TRUE(lifecycle.accept(straightGuide(), true, grid(), esdf, Point2{2.5, 10.5})
                  .accepted);

  const ActiveGlobalGuideUpdate update =
      lifecycle.update(grid(), esdf, Point2{40.5, 10.5}, 0U);

  EXPECT_TRUE(update.active);
  EXPECT_TRUE(update.reaches_mission_goal);
  EXPECT_EQ(update.release_reason, GlobalGuideReleaseReason::kNone);
}

TEST(ActiveGlobalGuideTest, ReleasesGuideForNewStallGeneration) {
  ActiveGlobalGuideLifecycle lifecycle;
  const std::vector<float> esdf = clearEsdf();
  ASSERT_TRUE(lifecycle.accept(straightGuide(), false, grid(), esdf, Point2{2.5, 10.5})
                  .accepted);

  const ActiveGlobalGuideUpdate update =
      lifecycle.update(grid(), esdf, Point2{8.5, 10.5}, 1U);

  EXPECT_EQ(update.release_reason, GlobalGuideReleaseReason::kStalled);
  EXPECT_TRUE(update.requires_replan);
}

TEST(ActiveGlobalGuideTest, DoesNotApplyStallGenerationFromBeforeAcceptance) {
  ActiveGlobalGuideLifecycle lifecycle;
  const std::vector<float> esdf = clearEsdf();
  EXPECT_EQ(lifecycle.update(grid(), esdf, Point2{2.5, 10.5}, 4U).release_reason,
            GlobalGuideReleaseReason::kNoActiveGuide);
  ASSERT_TRUE(lifecycle.accept(straightGuide(), false, grid(), esdf, Point2{2.5, 10.5})
                  .accepted);

  const ActiveGlobalGuideUpdate update =
      lifecycle.update(grid(), esdf, Point2{8.5, 10.5}, 4U);

  EXPECT_TRUE(update.active);
  EXPECT_EQ(update.release_reason, GlobalGuideReleaseReason::kNone);
}

TEST(ActiveGlobalGuideTest, UsesVelocityHeadingAtCruiseSpeed) {
  ActiveGlobalGuideLifecycle lifecycle;
  mppi::State state;
  state.vx = 3.0F;
  state.vy = 4.0F;

  const GlobalGuideHeading heading =
      lifecycle.selectPlanningHeading(state, Point2{10.0, 10.0});

  EXPECT_EQ(heading.source, GlobalGuideHeadingSource::kVelocity);
  EXPECT_NEAR(heading.heading_rad, std::atan2(4.0, 3.0), 1.0e-9);
}

TEST(ActiveGlobalGuideTest, UsesAcceptedGuideHeadingWhileStationary) {
  ActiveGlobalGuideLifecycle lifecycle;
  const std::vector<float> esdf = clearEsdf();
  ASSERT_TRUE(lifecycle.accept(straightGuide(), false, grid(), esdf, Point2{2.5, 10.5})
                  .accepted);

  const GlobalGuideHeading heading =
      lifecycle.selectPlanningHeading(mppi::State{}, Point2{10.0, 10.0});

  EXPECT_EQ(heading.source, GlobalGuideHeadingSource::kActiveGuide);
  EXPECT_NEAR(heading.heading_rad, 0.0, 1.0e-9);
}

TEST(ActiveGlobalGuideTest, BlendsGuideAndVelocityAcrossSpeedTransition) {
  ActiveGlobalGuideLifecycle lifecycle;
  const std::vector<float> esdf = clearEsdf();
  ASSERT_TRUE(lifecycle.accept(straightGuide(), false, grid(), esdf, Point2{2.5, 10.5})
                  .accepted);
  mppi::State state;
  state.vy = 1.0F;

  const GlobalGuideHeading heading =
      lifecycle.selectPlanningHeading(state, Point2{10.0, 10.0});

  EXPECT_EQ(heading.source, GlobalGuideHeadingSource::kBlended);
  EXPECT_NEAR(heading.heading_rad, std::numbers::pi / 4.0, 1.0e-9);
}

TEST(ActiveGlobalGuideTest, UsesGoalDirectionWithoutVelocityOrGuide) {
  ActiveGlobalGuideLifecycle lifecycle;
  mppi::State state;
  state.x = 2.0F;
  state.y = 3.0F;
  state.yaw = -0.75F;

  const GlobalGuideHeading heading =
      lifecycle.selectPlanningHeading(state, Point2{2.0, 13.0});

  EXPECT_EQ(heading.source, GlobalGuideHeadingSource::kGoalDirection);
  EXPECT_NEAR(heading.heading_rad, std::numbers::pi / 2.0, 1.0e-9);
}

TEST(ActiveGlobalGuideTest, BodyYawNeverChangesPlanningHeading) {
  ActiveGlobalGuideLifecycle lifecycle;
  mppi::State first;
  first.x = 2.0F;
  first.y = 3.0F;
  first.yaw = -2.5F;
  mppi::State second = first;
  second.yaw = 1.7F;

  const GlobalGuideHeading first_heading =
      lifecycle.selectPlanningHeading(first, Point2{12.0, 13.0});
  const GlobalGuideHeading second_heading =
      lifecycle.selectPlanningHeading(second, Point2{12.0, 13.0});

  EXPECT_EQ(first_heading.source, GlobalGuideHeadingSource::kGoalDirection);
  EXPECT_EQ(second_heading.source, GlobalGuideHeadingSource::kGoalDirection);
  EXPECT_DOUBLE_EQ(first_heading.heading_rad, second_heading.heading_rad);
}

TEST(ActiveGlobalGuideTest, ProjectionDoesNotMoveBehindMinimumStation) {
  const std::vector<Point2> guide{{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}};

  const GlobalGuideProjection projection =
      projectOntoGlobalGuide(guide, Point2{2.0, 0.0}, 15.0);

  ASSERT_TRUE(projection.valid);
  EXPECT_NEAR(projection.station_m, 15.0, 1.0e-9);
  EXPECT_NEAR(projection.point.x, 15.0, 1.0e-9);
}

TEST(GlobalGuideProgressTrackerTest, ReseedsThenReleasesPredictionMismatch) {
  GlobalGuideProgressTracker tracker;
  EXPECT_FALSE(tracker
                   .evaluate(GlobalGuideProgressObservation{
                       .stamp_ns = 1000000000LL,
                       .guide_generation = 3U,
                       .station_m = 5.0,
                       .predicted_head_progress_m = 2.0,
                       .controller_active = true,
                   })
                   .stalled);

  const GlobalGuideProgressUpdate reseed =
      tracker.evaluate(GlobalGuideProgressObservation{
          .stamp_ns = 2100000000LL,
          .guide_generation = 3U,
          .station_m = 5.2,
          .predicted_head_progress_m = 2.0,
          .controller_active = true,
      });

  EXPECT_FALSE(reseed.stalled);
  EXPECT_TRUE(reseed.local_reseed_requested);
  EXPECT_EQ(reseed.action, GlobalGuideProgressAction::kReseedLocalMppi);
  EXPECT_EQ(reseed.local_reseed_generation, 1U);
  EXPECT_NEAR(reseed.progress_m, 0.2, 1.0e-9);

  const GlobalGuideProgressUpdate release =
      tracker.evaluate(GlobalGuideProgressObservation{
          .stamp_ns = 3'200'000'000LL,
          .guide_generation = 3U,
          .station_m = 5.3,
          .predicted_head_progress_m = 2.0,
          .controller_active = true,
      });

  EXPECT_TRUE(release.stalled);
  EXPECT_FALSE(release.local_reseed_requested);
  EXPECT_EQ(release.action, GlobalGuideProgressAction::kReleasePredictionMismatch);
  EXPECT_EQ(release.stall_generation, 1U);
  EXPECT_NEAR(release.progress_m, 0.1, 1.0e-9);
}

TEST(GlobalGuideProgressTrackerTest, ReleasesLowPredictedProgressAfterWindow) {
  GlobalGuideProgressTracker tracker;
  (void)tracker.evaluate(GlobalGuideProgressObservation{
      .stamp_ns = 1'000'000'000LL,
      .guide_generation = 3U,
      .station_m = 5.0,
      .predicted_head_progress_m = 0.1,
      .controller_active = true,
  });

  const GlobalGuideProgressUpdate update =
      tracker.evaluate(GlobalGuideProgressObservation{
          .stamp_ns = 2'100'000'000LL,
          .guide_generation = 3U,
          .station_m = 5.2,
          .predicted_head_progress_m = 0.1,
          .controller_active = true,
      });

  EXPECT_TRUE(update.stalled);
  EXPECT_FALSE(update.local_reseed_requested);
  EXPECT_EQ(update.action, GlobalGuideProgressAction::kReleaseLowPredictedProgress);
  EXPECT_EQ(update.stall_generation, 1U);
}

TEST(GlobalGuideProgressTrackerTest, ResetsAfterUsefulProgress) {
  GlobalGuideProgressTracker tracker;
  (void)tracker.evaluate(GlobalGuideProgressObservation{
      .stamp_ns = 1000000000LL,
      .guide_generation = 3U,
      .station_m = 5.0,
      .predicted_head_progress_m = 2.0,
      .controller_active = true,
  });

  const GlobalGuideProgressUpdate update =
      tracker.evaluate(GlobalGuideProgressObservation{
          .stamp_ns = 2100000000LL,
          .guide_generation = 3U,
          .station_m = 5.6,
          .predicted_head_progress_m = 2.0,
          .controller_active = true,
      });

  EXPECT_FALSE(update.stalled);
  EXPECT_FALSE(update.local_reseed_requested);
  EXPECT_NEAR(update.progress_m, 0.6, 1.0e-9);
}

TEST(GlobalGuideProgressTrackerTest, UsefulProgressClearsPendingReseed) {
  GlobalGuideProgressTracker tracker;
  (void)tracker.evaluate(GlobalGuideProgressObservation{
      .stamp_ns = 1'000'000'000LL,
      .guide_generation = 3U,
      .station_m = 5.0,
      .predicted_head_progress_m = 2.0,
      .controller_active = true,
  });
  ASSERT_TRUE(tracker
                  .evaluate(GlobalGuideProgressObservation{
                      .stamp_ns = 2'100'000'000LL,
                      .guide_generation = 3U,
                      .station_m = 5.1,
                      .predicted_head_progress_m = 2.0,
                      .controller_active = true,
                  })
                  .local_reseed_requested);
  EXPECT_FALSE(tracker
                   .evaluate(GlobalGuideProgressObservation{
                       .stamp_ns = 3'200'000'000LL,
                       .guide_generation = 3U,
                       .station_m = 5.7,
                       .predicted_head_progress_m = 2.0,
                       .controller_active = true,
                   })
                   .stalled);

  const GlobalGuideProgressUpdate reseed =
      tracker.evaluate(GlobalGuideProgressObservation{
          .stamp_ns = 4'300'000'000LL,
          .guide_generation = 3U,
          .station_m = 5.8,
          .predicted_head_progress_m = 2.0,
          .controller_active = true,
      });

  EXPECT_TRUE(reseed.local_reseed_requested);
  EXPECT_FALSE(reseed.stalled);
  EXPECT_EQ(reseed.local_reseed_generation, 2U);
}

TEST(GlobalGuideProgressTrackerTest, ReleasesAfterPersistentSafetyRejection) {
  GlobalGuideProgressConfig config;
  config.persistent_safety_rejection_window_s = 1.0;
  GlobalGuideProgressTracker tracker{config};
  EXPECT_FALSE(tracker
                   .evaluate(GlobalGuideProgressObservation{
                       .stamp_ns = 1'000'000'000LL,
                       .guide_generation = 7U,
                       .station_m = 5.0,
                       .predicted_head_progress_m = 2.0,
                       .controller_active = true,
                       .emergency_braking = true,
                   })
                   .stalled);

  const GlobalGuideProgressUpdate update =
      tracker.evaluate(GlobalGuideProgressObservation{
          .stamp_ns = 2'100'000'000LL,
          .guide_generation = 7U,
          .station_m = 5.0,
          .predicted_head_progress_m = 2.0,
          .controller_active = true,
          .emergency_braking = true,
      });

  EXPECT_TRUE(update.stalled);
  EXPECT_TRUE(update.persistent_safety_rejection);
  EXPECT_EQ(update.action,
            GlobalGuideProgressAction::kReleasePersistentSafetyRejection);
  EXPECT_EQ(update.stall_generation, 1U);
}

TEST(ActiveGlobalGuideTest, UsesPersistentSafetyRejectionReleaseReason) {
  ActiveGlobalGuideLifecycle lifecycle;
  const std::vector<float> esdf = clearEsdf();
  ASSERT_TRUE(lifecycle.accept(straightGuide(), false, grid(), esdf, Point2{2.5, 10.5})
                  .accepted);

  const ActiveGlobalGuideUpdate update =
      lifecycle.update(grid(), esdf, Point2{8.5, 10.5}, 1U,
                       GlobalGuideReleaseReason::kPersistentSafetyRejection);

  EXPECT_FALSE(update.active);
  EXPECT_TRUE(update.requires_replan);
  EXPECT_EQ(update.release_reason,
            GlobalGuideReleaseReason::kPersistentSafetyRejection);
}

} // namespace
} // namespace drone_city_nav
