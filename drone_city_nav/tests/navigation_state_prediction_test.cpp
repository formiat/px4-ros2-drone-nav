#include "drone_city_nav/navigation_state_prediction.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

TEST(NavigationStatePredictionTest, ExtrapolatesConstantVelocityAndYawRate) {
  const mppi::State state{
      .x = 1.0F,
      .y = 2.0F,
      .z = 3.0F,
      .vx = 4.0F,
      .vy = -2.0F,
      .vz = 1.0F,
      .yaw = 0.2F,
      .yaw_rate = 0.5F,
  };

  const NavigationStatePredictionResult result =
      predictNavigationState(state, 0.25, 1.0);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.predicted);
  EXPECT_FLOAT_EQ(result.state.x, 2.0F);
  EXPECT_FLOAT_EQ(result.state.y, 1.5F);
  EXPECT_FLOAT_EQ(result.state.z, 3.25F);
  EXPECT_NEAR(result.state.yaw, 0.325F, 1.0e-6F);
}

TEST(NavigationStatePredictionTest, RejectsPredictionPastConfiguredAge) {
  const NavigationStatePredictionResult result =
      predictNavigationState(mppi::State{}, 1.01, 1.0);

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.predicted);
}

} // namespace
} // namespace drone_city_nav
