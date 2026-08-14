#include "drone_city_nav/noncooperative_collision_avoidance.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace drone_city_nav {
namespace {

constexpr std::int64_t kSecondNs{1000000000LL};

[[nodiscard]] NonCooperativeAircraftTrack
track(const Point3 position, const Vec3 velocity,
      const std::int64_t stamp_ns = kSecondNs) {
  return NonCooperativeAircraftTrack{
      .local_track_id = 1U,
      .position = position,
      .velocity = velocity,
      .measurement_stamp_ns = stamp_ns,
      .position_valid = true,
      .velocity_valid = true,
  };
}

TEST(NonCooperativeCollisionAvoidanceTest,
     PredictedClosestApproachActivatesWithoutBlockingTheCurrentPath) {
  NonCooperativeCollisionAvoidance avoidance{NonCooperativeAvoidanceConfig{}};
  const NonCooperativeAvoidanceUpdate update =
      avoidance.update(NonCooperativeAvoidanceInput{
          .ownship = mppi::State{.vx = 10.0F},
          .tracks = {track(Point3{30.0, 8.0, 0.0}, Vec3{-5.0, 0.0, 0.0})},
          .now_ns = kSecondNs,
          .horizon_steps = 80U,
          .step_s = 0.05,
      });

  ASSERT_TRUE(update.primary_threat.has_value());
  const NonCooperativeClosestApproach threat =
      update.primary_threat.value_or(NonCooperativeClosestApproach{});
  EXPECT_TRUE(update.active);
  EXPECT_EQ(update.lifecycle_state, NonCooperativeAvoidanceLifecycleState::kEntered);
  EXPECT_EQ(threat.reason, NonCooperativeThreatReason::kStrongPredictedClosestApproach);
  EXPECT_GT(threat.current_range_m, 10.0);
  EXPECT_LT(threat.closest_approach_distance_m, 10.0);
  EXPECT_GT(threat.closing_speed_mps, 0.0);
  ASSERT_EQ(update.trajectories.size(), 1U);
  EXPECT_EQ(update.trajectories.front().active_steps, 80U);
  EXPECT_TRUE(update.acquisition.has_value());
}

TEST(NonCooperativeCollisionAvoidanceTest,
     SideCrossingTrackUsesTheSameClosestApproachContract) {
  NonCooperativeCollisionAvoidance avoidance{NonCooperativeAvoidanceConfig{}};
  const NonCooperativeAvoidanceUpdate update =
      avoidance.update(NonCooperativeAvoidanceInput{
          .ownship = mppi::State{},
          .tracks = {track(Point3{0.0, 18.0, 0.0}, Vec3{0.0, -6.0, 0.0})},
          .now_ns = kSecondNs,
          .horizon_steps = 80U,
          .step_s = 0.05,
      });

  ASSERT_TRUE(update.primary_threat.has_value());
  const NonCooperativeClosestApproach threat =
      update.primary_threat.value_or(NonCooperativeClosestApproach{});
  EXPECT_TRUE(update.active);
  EXPECT_EQ(threat.reason, NonCooperativeThreatReason::kStrongPredictedClosestApproach);
  EXPECT_NEAR(threat.time_to_closest_approach_s, 3.0, 1.0e-6);
  EXPECT_NEAR(threat.closest_approach_distance_m, 0.0, 1.0e-6);
}

TEST(NonCooperativeCollisionAvoidanceTest,
     ModerateAnticipationDoesNotTriggerAnAcquisitionReseed) {
  NonCooperativeCollisionAvoidance avoidance{NonCooperativeAvoidanceConfig{}};
  const NonCooperativeAvoidanceUpdate update =
      avoidance.update(NonCooperativeAvoidanceInput{
          .ownship = mppi::State{},
          .tracks = {track(Point3{0.0, 15.0, 0.0}, Vec3{})},
          .now_ns = kSecondNs,
          .horizon_steps = 80U,
          .step_s = 0.05,
      });

  ASSERT_TRUE(update.primary_threat.has_value());
  const NonCooperativeClosestApproach threat =
      update.primary_threat.value_or(NonCooperativeClosestApproach{});
  EXPECT_EQ(threat.reason, NonCooperativeThreatReason::kNearby);
  EXPECT_FALSE(update.active);
  EXPECT_FALSE(update.acquisition.has_value());
  EXPECT_EQ(update.fresh_track_count, 1U);
}

TEST(NonCooperativeCollisionAvoidanceTest, ReleaseRequiresContinuousClearHysteresis) {
  NonCooperativeCollisionAvoidance avoidance{NonCooperativeAvoidanceConfig{}};
  const auto update_at = [&](const std::int64_t now_ns, const Point3 position) {
    return avoidance.update(NonCooperativeAvoidanceInput{
        .ownship = mppi::State{},
        .tracks = {track(position, Vec3{}, now_ns)},
        .now_ns = now_ns,
        .horizon_steps = 80U,
        .step_s = 0.05,
    });
  };

  EXPECT_TRUE(update_at(kSecondNs, Point3{5.0, 0.0, 0.0}).active);
  EXPECT_TRUE(update_at(2 * kSecondNs, Point3{30.0, 0.0, 0.0}).active);
  EXPECT_TRUE(update_at(2 * kSecondNs + 900000000LL, Point3{30.0, 0.0, 0.0}).active);
  const NonCooperativeAvoidanceUpdate released =
      update_at(3 * kSecondNs + 100000000LL, Point3{30.0, 0.0, 0.0});
  EXPECT_FALSE(released.active);
  EXPECT_EQ(released.lifecycle_state, NonCooperativeAvoidanceLifecycleState::kReleased);
}

TEST(NonCooperativeCollisionAvoidanceTest, StaleTracksAreNotCoastedIndefinitely) {
  NonCooperativeCollisionAvoidance avoidance{NonCooperativeAvoidanceConfig{}};
  const NonCooperativeAvoidanceUpdate update =
      avoidance.update(NonCooperativeAvoidanceInput{
          .ownship = mppi::State{},
          .tracks = {track(Point3{5.0, 0.0, 0.0}, Vec3{}, kSecondNs)},
          .now_ns = 2 * kSecondNs,
          .horizon_steps = 80U,
          .step_s = 0.05,
      });

  EXPECT_EQ(update.received_track_count, 1U);
  EXPECT_EQ(update.fresh_track_count, 0U);
  EXPECT_TRUE(update.trajectories.empty());
  EXPECT_FALSE(update.primary_threat.has_value());
}

} // namespace
} // namespace drone_city_nav
