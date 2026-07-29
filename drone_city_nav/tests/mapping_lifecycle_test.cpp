#include "drone_city_nav/mapping_lifecycle.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

TEST(MappingLifecycleTest, RemainsActiveBelowThresholdAfterTakeoff) {
  MappingLifecycle lifecycle{5.0};
  lifecycle.updateArmed(true);

  EXPECT_FALSE(lifecycle.updateAltitude(4.9, true));
  EXPECT_TRUE(lifecycle.updateAltitude(5.0, true));
  EXPECT_TRUE(lifecycle.updateAltitude(2.0, true));
}

TEST(MappingLifecycleTest, ResetsOnlyOnArmedToDisarmedTransition) {
  MappingLifecycle lifecycle{5.0};
  lifecycle.updateArmed(false);
  EXPECT_TRUE(lifecycle.updateAltitude(6.0, true));
  lifecycle.updateArmed(true);
  lifecycle.updateArmed(false);

  EXPECT_FALSE(lifecycle.active());
  EXPECT_FALSE(lifecycle.updateAltitude(2.0, true));
}

} // namespace
} // namespace drone_city_nav
