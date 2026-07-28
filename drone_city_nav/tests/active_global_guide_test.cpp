#include "drone_city_nav/active_global_guide.hpp"

#include <gtest/gtest.h>

#include <cmath>
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

  ASSERT_TRUE(
      lifecycle.accept(straightGuide(), false, grid(), esdf, Point2{2.5, 10.5}));
  const ActiveGlobalGuideUpdate update =
      lifecycle.update(grid(), esdf, Point2{8.5, 10.5}, 0U);

  EXPECT_TRUE(update.active);
  EXPECT_TRUE(update.retained);
  EXPECT_FALSE(update.requires_replan);
  EXPECT_EQ(update.release_reason, GlobalGuideReleaseReason::kNone);
  EXPECT_NEAR(update.projection.station_m, 6.0, 1.0e-9);
  EXPECT_NEAR(update.projection.remaining_m, 34.0, 1.0e-9);
}

TEST(ActiveGlobalGuideTest, ReleasesGuideWhenNewCriticalBandAppears) {
  ActiveGlobalGuideLifecycle lifecycle;
  std::vector<float> esdf = clearEsdf();
  ASSERT_TRUE(
      lifecycle.accept(straightGuide(), false, grid(), esdf, Point2{2.5, 10.5}));
  esdf[10U * 80U + 20U] = 0.75F;

  const ActiveGlobalGuideUpdate update =
      lifecycle.update(grid(), esdf, Point2{8.5, 10.5}, 0U);

  EXPECT_FALSE(update.active);
  EXPECT_TRUE(update.requires_replan);
  EXPECT_EQ(update.release_reason, GlobalGuideReleaseReason::kBlocked);
}

TEST(ActiveGlobalGuideTest, KeepsGuideAcceptedInsideCriticalBand) {
  ActiveGlobalGuideLifecycle lifecycle;
  std::vector<float> esdf = clearEsdf();
  esdf[10U * 80U + 20U] = 0.75F;
  ASSERT_TRUE(
      lifecycle.accept(straightGuide(), false, grid(), esdf, Point2{2.5, 10.5}));

  const ActiveGlobalGuideUpdate update =
      lifecycle.update(grid(), esdf, Point2{8.5, 10.5}, 0U);

  EXPECT_TRUE(update.active);
  EXPECT_EQ(update.current_risk, GlobalGuideRiskTier::kCritical);
}

TEST(ActiveGlobalGuideTest, ReleasesNonTerminalGuideNearItsEnd) {
  ActiveGlobalGuideConfig config;
  config.minimum_remaining_m = 15.0;
  ActiveGlobalGuideLifecycle lifecycle{config};
  const std::vector<float> esdf = clearEsdf();
  ASSERT_TRUE(
      lifecycle.accept(straightGuide(), false, grid(), esdf, Point2{2.5, 10.5}));

  const ActiveGlobalGuideUpdate update =
      lifecycle.update(grid(), esdf, Point2{32.5, 10.5}, 0U);

  EXPECT_EQ(update.release_reason, GlobalGuideReleaseReason::kExhausted);
  EXPECT_TRUE(update.requires_replan);
}

TEST(ActiveGlobalGuideTest, KeepsMissionGoalGuideNearItsEnd) {
  ActiveGlobalGuideLifecycle lifecycle;
  const std::vector<float> esdf = clearEsdf();
  ASSERT_TRUE(lifecycle.accept(straightGuide(), true, grid(), esdf, Point2{2.5, 10.5}));

  const ActiveGlobalGuideUpdate update =
      lifecycle.update(grid(), esdf, Point2{40.5, 10.5}, 1U);

  EXPECT_TRUE(update.active);
  EXPECT_TRUE(update.mission_goal_hold);
  EXPECT_EQ(update.release_reason, GlobalGuideReleaseReason::kNone);
}

TEST(ActiveGlobalGuideTest, ReleasesGuideForNewStallGeneration) {
  ActiveGlobalGuideLifecycle lifecycle;
  const std::vector<float> esdf = clearEsdf();
  ASSERT_TRUE(
      lifecycle.accept(straightGuide(), false, grid(), esdf, Point2{2.5, 10.5}));

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
  ASSERT_TRUE(
      lifecycle.accept(straightGuide(), false, grid(), esdf, Point2{2.5, 10.5}));

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

  const GlobalGuideHeading heading = lifecycle.selectPlanningHeading(state, 0.0);

  EXPECT_EQ(heading.source, GlobalGuideHeadingSource::kVelocity);
  EXPECT_NEAR(heading.heading_rad, std::atan2(4.0, 3.0), 1.0e-9);
}

TEST(ActiveGlobalGuideTest, UsesAcceptedGuideHeadingWhileStationary) {
  ActiveGlobalGuideLifecycle lifecycle;
  const std::vector<float> esdf = clearEsdf();
  ASSERT_TRUE(
      lifecycle.accept(straightGuide(), false, grid(), esdf, Point2{2.5, 10.5}));

  const GlobalGuideHeading heading =
      lifecycle.selectPlanningHeading(mppi::State{}, std::numbers::pi / 2.0);

  EXPECT_EQ(heading.source, GlobalGuideHeadingSource::kActiveGuide);
  EXPECT_NEAR(heading.heading_rad, 0.0, 1.0e-9);
}

TEST(ActiveGlobalGuideTest, BlendsGuideAndVelocityAcrossSpeedTransition) {
  ActiveGlobalGuideLifecycle lifecycle;
  const std::vector<float> esdf = clearEsdf();
  ASSERT_TRUE(
      lifecycle.accept(straightGuide(), false, grid(), esdf, Point2{2.5, 10.5}));
  mppi::State state;
  state.vy = 1.0F;

  const GlobalGuideHeading heading = lifecycle.selectPlanningHeading(state, 0.0);

  EXPECT_EQ(heading.source, GlobalGuideHeadingSource::kBlended);
  EXPECT_NEAR(heading.heading_rad, std::numbers::pi / 4.0, 1.0e-9);
}

TEST(ActiveGlobalGuideTest, FallsBackToYawWithoutVelocityOrGuide) {
  ActiveGlobalGuideLifecycle lifecycle;

  const GlobalGuideHeading heading =
      lifecycle.selectPlanningHeading(mppi::State{}, -0.75);

  EXPECT_EQ(heading.source, GlobalGuideHeadingSource::kYawFallback);
  EXPECT_DOUBLE_EQ(heading.heading_rad, -0.75);
}

TEST(ActiveGlobalGuideTest, ProjectionDoesNotMoveBehindMinimumStation) {
  const std::vector<Point2> guide{{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}};

  const GlobalGuideProjection projection =
      projectOntoGlobalGuide(guide, Point2{2.0, 0.0}, 15.0);

  ASSERT_TRUE(projection.valid);
  EXPECT_NEAR(projection.station_m, 15.0, 1.0e-9);
  EXPECT_NEAR(projection.point.x, 15.0, 1.0e-9);
}

TEST(GlobalGuideProgressTrackerTest, DetectsMissingAlongGuideProgress) {
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

  const GlobalGuideProgressUpdate update =
      tracker.evaluate(GlobalGuideProgressObservation{
          .stamp_ns = 2100000000LL,
          .guide_generation = 3U,
          .station_m = 5.2,
          .predicted_head_progress_m = 2.0,
          .controller_active = true,
      });

  EXPECT_TRUE(update.stalled);
  EXPECT_EQ(update.stall_generation, 1U);
  EXPECT_NEAR(update.progress_m, 0.2, 1.0e-9);
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
  EXPECT_NEAR(update.progress_m, 0.6, 1.0e-9);
}

} // namespace
} // namespace drone_city_nav
