#include "drone_city_nav/vehicle_destruction_disarm_lifecycle.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

TEST(VehicleDestructionDisarmLifecycleTest, DoesNothingBeforeDestruction) {
  VehicleDestructionDisarmLifecycle lifecycle;
  const VehicleDestructionDisarmUpdate update = lifecycle.update(1, true, true);
  EXPECT_FALSE(update.latched);
  EXPECT_FALSE(update.confirmed);
  EXPECT_FALSE(update.force_disarm_requested);
}

TEST(VehicleDestructionDisarmLifecycleTest, RetriesUntilDisarmed) {
  VehicleDestructionDisarmLifecycle lifecycle{
      VehicleDestructionDisarmConfig{.retry_period_s = 0.2}};
  lifecycle.latch(1'000'000'000LL);

  const auto first = lifecycle.update(1'000'000'000LL, true, true);
  const auto early = lifecycle.update(1'100'000'000LL, true, true);
  const auto retry = lifecycle.update(1'200'000'000LL, true, true);
  const auto confirmed = lifecycle.update(1'210'000'000LL, true, false);
  const auto after_confirmation = lifecycle.update(2'000'000'000LL, true, false);

  EXPECT_TRUE(first.force_disarm_requested);
  EXPECT_FALSE(early.force_disarm_requested);
  EXPECT_TRUE(retry.force_disarm_requested);
  EXPECT_TRUE(confirmed.confirmed);
  EXPECT_FALSE(confirmed.force_disarm_requested);
  EXPECT_FALSE(after_confirmation.force_disarm_requested);
}

TEST(VehicleDestructionDisarmLifecycleTest, RequiresAtLeastOneCommand) {
  VehicleDestructionDisarmLifecycle lifecycle;
  lifecycle.latch(1'000'000'000LL);
  const auto first = lifecycle.update(1'000'000'000LL, true, false);
  const auto confirmed = lifecycle.update(1'010'000'000LL, true, false);
  EXPECT_TRUE(first.force_disarm_requested);
  EXPECT_FALSE(first.confirmed);
  EXPECT_TRUE(confirmed.confirmed);
}

TEST(VehicleDestructionDisarmLifecycleTest, LatchCannotBeCleared) {
  VehicleDestructionDisarmLifecycle lifecycle;
  lifecycle.latch(100);
  lifecycle.latch(200);
  EXPECT_TRUE(lifecycle.latched());
  EXPECT_TRUE(lifecycle.update(100, true, true).force_disarm_requested);
}

} // namespace
} // namespace drone_city_nav
