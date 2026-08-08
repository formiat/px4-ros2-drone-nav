#include "drone_city_nav/direct_tracking_maneuver_lifecycle.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace drone_city_nav {
namespace {

[[nodiscard]] DirectTrackingManeuverObservation
observation(const std::int64_t stamp_ns, const Point3 target_position,
            const Vec3 interceptor_velocity = {},
            const Vec3 target_velocity = {}) noexcept {
  return DirectTrackingManeuverObservation{
      .interceptor_position = {},
      .interceptor_velocity = interceptor_velocity,
      .target_position = target_position,
      .target_velocity = target_velocity,
      .stamp_ns = stamp_ns,
      .line_of_sight_generation = 1U,
      .active = true,
  };
}

TEST(DirectTrackingManeuverLifecycle, ReseedsOnceAfterTargetBearingChanges) {
  DirectTrackingManeuverLifecycle lifecycle;
  static_cast<void>(lifecycle.update(observation(1'000'000'000LL, {20.0, 0.0, 0.0})));

  const DirectTrackingManeuverUpdate turned =
      lifecycle.update(observation(1'600'000'000LL, {10.0, 20.0, 0.0}));
  const DirectTrackingManeuverUpdate repeated =
      lifecycle.update(observation(1'700'000'000LL, {10.0, 20.0, 0.0}));

  EXPECT_TRUE(turned.reseed_requested);
  EXPECT_EQ(turned.reason, DirectTrackingReseedReason::kBearingChange);
  EXPECT_GT(std::abs(turned.bearing_change_rad), std::acos(-1.0) / 6.0);
  EXPECT_FALSE(repeated.reseed_requested);
  EXPECT_EQ(repeated.reseed_generation, turned.reseed_generation);
}

TEST(DirectTrackingManeuverLifecycle, ReseedsOncePerSustainedNoClosingEpisode) {
  DirectTrackingManeuverLifecycle lifecycle;
  static_cast<void>(lifecycle.update(observation(1'000'000'000LL, {20.0, 0.0, 0.0},
                                                 {5.0, 0.0, 0.0}, {5.0, 0.0, 0.0})));
  const DirectTrackingManeuverUpdate stalled = lifecycle.update(
      observation(2'100'000'000LL, {20.0, 0.0, 0.0}, {5.0, 0.0, 0.0}, {5.0, 0.0, 0.0}));
  const DirectTrackingManeuverUpdate repeated = lifecycle.update(
      observation(3'200'000'000LL, {20.0, 0.0, 0.0}, {5.0, 0.0, 0.0}, {5.0, 0.0, 0.0}));

  EXPECT_TRUE(stalled.reseed_requested);
  EXPECT_EQ(stalled.reason, DirectTrackingReseedReason::kNoClosing);
  EXPECT_FALSE(repeated.reseed_requested);

  static_cast<void>(lifecycle.update(observation(3'300'000'000LL, {20.0, 0.0, 0.0},
                                                 {8.0, 0.0, 0.0}, {5.0, 0.0, 0.0})));
  static_cast<void>(lifecycle.update(observation(3'400'000'000LL, {20.0, 0.0, 0.0},
                                                 {5.0, 0.0, 0.0}, {5.0, 0.0, 0.0})));
  const DirectTrackingManeuverUpdate next_episode = lifecycle.update(
      observation(4'500'000'000LL, {20.0, 0.0, 0.0}, {5.0, 0.0, 0.0}, {5.0, 0.0, 0.0}));

  EXPECT_TRUE(next_episode.reseed_requested);
  EXPECT_EQ(next_episode.reason, DirectTrackingReseedReason::kNoClosing);
  EXPECT_EQ(next_episode.reseed_generation, stalled.reseed_generation + 1U);
}

TEST(DirectTrackingManeuverLifecycle, LosingLineOfSightStartsANewEpisode) {
  DirectTrackingManeuverLifecycle lifecycle;
  static_cast<void>(lifecycle.update(observation(1'000'000'000LL, {20.0, 0.0, 0.0})));
  const DirectTrackingManeuverUpdate first =
      lifecycle.update(observation(1'600'000'000LL, {0.0, 20.0, 0.0}));
  DirectTrackingManeuverObservation hidden;
  hidden.stamp_ns = 1'700'000'000LL;
  static_cast<void>(lifecycle.update(hidden));
  DirectTrackingManeuverObservation reacquired =
      observation(2'000'000'000LL, {0.0, 20.0, 0.0});
  reacquired.line_of_sight_generation = 2U;
  const DirectTrackingManeuverUpdate new_episode = lifecycle.update(reacquired);

  EXPECT_TRUE(first.reseed_requested);
  EXPECT_FALSE(new_episode.reseed_requested);
  EXPECT_EQ(new_episode.reseed_generation, first.reseed_generation);
}

} // namespace
} // namespace drone_city_nav
