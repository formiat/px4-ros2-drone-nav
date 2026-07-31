#include "drone_city_nav/crash_disarm_lifecycle.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

TEST(CrashDisarmLifecycleTest, DoesNothingBeforeCrash) {
  CrashDisarmLifecycle lifecycle;

  const CrashDisarmUpdate update = lifecycle.update(1000000000LL, true, true);

  EXPECT_FALSE(update.latched);
  EXPECT_FALSE(update.confirmed);
  EXPECT_FALSE(update.force_disarm_requested);
}

TEST(CrashDisarmLifecycleTest, RetriesForceDisarmUntilDisarmed) {
  CrashDisarmLifecycle lifecycle{CrashDisarmLifecycleConfig{.retry_period_s = 0.2}};
  lifecycle.latch(1000000000LL);

  const CrashDisarmUpdate first = lifecycle.update(1000000000LL, true, true);
  const CrashDisarmUpdate early = lifecycle.update(1100000000LL, true, true);
  const CrashDisarmUpdate retry = lifecycle.update(1200000000LL, true, true);
  const CrashDisarmUpdate confirmed = lifecycle.update(1210000000LL, true, false);
  const CrashDisarmUpdate after_confirmation =
      lifecycle.update(1500000000LL, true, true);

  EXPECT_TRUE(first.force_disarm_requested);
  EXPECT_FALSE(early.force_disarm_requested);
  EXPECT_TRUE(retry.force_disarm_requested);
  EXPECT_TRUE(confirmed.confirmed);
  EXPECT_FALSE(confirmed.force_disarm_requested);
  EXPECT_TRUE(after_confirmation.confirmed);
  EXPECT_FALSE(after_confirmation.force_disarm_requested);
}

TEST(CrashDisarmLifecycleTest, SendsForceDisarmWhenArmingStateIsUnknown) {
  CrashDisarmLifecycle lifecycle;
  lifecycle.latch(1000000000LL);

  const CrashDisarmUpdate update = lifecycle.update(1000000000LL, false, false);

  EXPECT_TRUE(update.latched);
  EXPECT_FALSE(update.confirmed);
  EXPECT_TRUE(update.force_disarm_requested);
}

TEST(CrashDisarmLifecycleTest, SendsAtLeastOneCommandBeforeConfirmingDisarm) {
  CrashDisarmLifecycle lifecycle;
  lifecycle.latch(1000000000LL);

  const CrashDisarmUpdate first = lifecycle.update(1000000000LL, true, false);
  const CrashDisarmUpdate confirmed = lifecycle.update(1010000000LL, true, false);

  EXPECT_TRUE(first.force_disarm_requested);
  EXPECT_FALSE(first.confirmed);
  EXPECT_TRUE(confirmed.confirmed);
}

TEST(CrashDisarmLifecycleTest, CrashLatchCannotBeCleared) {
  CrashDisarmLifecycle lifecycle;
  lifecycle.latch(1000000000LL);
  lifecycle.latch(2000000000LL);

  EXPECT_TRUE(lifecycle.latched());
  EXPECT_TRUE(lifecycle.update(1000000000LL, true, true).force_disarm_requested);
}

} // namespace
} // namespace drone_city_nav
