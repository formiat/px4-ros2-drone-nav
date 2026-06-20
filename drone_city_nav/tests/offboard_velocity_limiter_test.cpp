#include "drone_city_nav/offboard_velocity_limiter.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace drone_city_nav {
namespace {

[[nodiscard]] double speed(const Point2 velocity) {
  return std::hypot(velocity.x, velocity.y);
}

} // namespace

TEST(OffboardVelocityLimiter, FirstCommandPassesThrough) {
  OffboardVelocityLimiter limiter{VelocityLimiterConfig{2.0, 1.0}};

  const VelocityLimiterOutput output = limiter.update(Point2{3.0, 0.0}, 0.1);

  EXPECT_DOUBLE_EQ(output.velocity_mps.x, 3.0);
  EXPECT_DOUBLE_EQ(output.velocity_mps.y, 0.0);
  EXPECT_FALSE(output.vector_delta_limited);
  EXPECT_FALSE(output.heading_rate_limited);
}

TEST(OffboardVelocityLimiter, LimitsVectorAcceleration) {
  OffboardVelocityLimiter limiter{VelocityLimiterConfig{2.0, 0.0}};
  (void)limiter.update(Point2{0.0, 0.0}, 0.1);

  const VelocityLimiterOutput output = limiter.update(Point2{5.0, 0.0}, 0.1);

  EXPECT_NEAR(output.velocity_mps.x, 0.2, 1.0e-9);
  EXPECT_NEAR(output.velocity_mps.y, 0.0, 1.0e-9);
  EXPECT_NEAR(output.raw_delta_mps, 5.0, 1.0e-9);
  EXPECT_NEAR(output.applied_delta_mps, 0.2, 1.0e-9);
  EXPECT_TRUE(output.vector_delta_limited);
}

TEST(OffboardVelocityLimiter, LimitsHeadingRate) {
  OffboardVelocityLimiter limiter{VelocityLimiterConfig{100.0, 1.0}};
  (void)limiter.update(Point2{5.0, 0.0}, 0.1);

  const VelocityLimiterOutput output = limiter.update(Point2{0.0, 5.0}, 0.1);

  EXPECT_TRUE(output.heading_rate_limited);
  EXPECT_NEAR(speed(output.velocity_mps), 5.0, 1.0e-9);
  EXPECT_NEAR(std::atan2(output.velocity_mps.y, output.velocity_mps.x), 0.1, 1.0e-9);
}

TEST(OffboardVelocityLimiter, VectorLimitAppliesAfterHeadingLimit) {
  OffboardVelocityLimiter limiter{VelocityLimiterConfig{1.0, 1.0}};
  (void)limiter.update(Point2{5.0, 0.0}, 0.1);

  const VelocityLimiterOutput output = limiter.update(Point2{0.0, 5.0}, 0.1);

  EXPECT_TRUE(output.heading_rate_limited);
  EXPECT_TRUE(output.vector_delta_limited);
  EXPECT_LE(output.applied_delta_mps, 0.1 + 1.0e-9);
  EXPECT_LE(speed(output.velocity_mps), 5.0);
}

TEST(OffboardVelocityLimiter, ResetClearsPreviousVelocity) {
  OffboardVelocityLimiter limiter{VelocityLimiterConfig{1.0, 1.0}};
  (void)limiter.update(Point2{5.0, 0.0}, 0.1);

  limiter.reset();
  const VelocityLimiterOutput output = limiter.update(Point2{0.0, 5.0}, 0.1);

  EXPECT_DOUBLE_EQ(output.velocity_mps.x, 0.0);
  EXPECT_DOUBLE_EQ(output.velocity_mps.y, 5.0);
  EXPECT_FALSE(output.vector_delta_limited);
  EXPECT_FALSE(output.heading_rate_limited);
}

TEST(OffboardVelocityLimiter, InvalidInputResetsToZero) {
  OffboardVelocityLimiter limiter{VelocityLimiterConfig{1.0, 1.0}};
  (void)limiter.update(Point2{5.0, 0.0}, 0.1);

  const VelocityLimiterOutput output = limiter.update(Point2{0.0, 5.0}, 0.0);

  EXPECT_DOUBLE_EQ(output.velocity_mps.x, 0.0);
  EXPECT_DOUBLE_EQ(output.velocity_mps.y, 0.0);
}

} // namespace drone_city_nav
