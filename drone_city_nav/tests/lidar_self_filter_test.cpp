#include "drone_city_nav/lidar_self_filter.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace drone_city_nav {
namespace {

TEST(LidarSelfFilter, RejectsOwnBodyAndRotorReturns) {
  const LidarSelfFilterConfig config{};

  EXPECT_TRUE(isLidarSelfReturn(Point3{0.0, 0.0, 0.0}, config));
  EXPECT_TRUE(isLidarSelfReturn(Point3{0.48, 0.48, 0.1}, config));
  EXPECT_TRUE(isLidarSelfReturn(Point3{0.9, 0.0, -0.5}, config));
  EXPECT_TRUE(isLidarSelfReturn(Point3{0.0, 0.0, 0.5}, config));
}

TEST(LidarSelfFilter, PreservesNearbyExternalReturns) {
  const LidarSelfFilterConfig config{};

  EXPECT_FALSE(isLidarSelfReturn(Point3{0.91, 0.0, 0.0}, config));
  EXPECT_FALSE(isLidarSelfReturn(Point3{0.0, 0.0, -0.51}, config));
  EXPECT_FALSE(isLidarSelfReturn(Point3{0.0, 0.0, 0.51}, config));
}

TEST(LidarSelfFilter, SeparatesDirectSelfReturnsFromVoxelQuantization) {
  const LidarSelfFilterConfig config{};

  EXPECT_EQ(
      classifyLidarSelfHit(Point3{0.89, 0.0, 0.0}, Point3{0.875, 0.0, 0.0}, config),
      LidarSelfFilterDisposition::kDiscardFromAllObstacleInputs);
  EXPECT_EQ(
      classifyLidarSelfHit(Point3{0.91, 0.0, 0.0}, Point3{0.875, 0.0, 0.0}, config),
      LidarSelfFilterDisposition::kDiscardFromPersistentMemory);
  EXPECT_EQ(
      classifyLidarSelfHit(Point3{0.91, 0.0, 0.0}, Point3{1.125, 0.0, 0.0}, config),
      LidarSelfFilterDisposition::kKeep);
  EXPECT_EQ(classifyLidarSelfHit(Point3{0.91, 0.0, 0.0}, std::nullopt, config),
            LidarSelfFilterDisposition::kKeep);
}

TEST(LidarSelfFilter, InvalidInputCannotHideAHit) {
  LidarSelfFilterConfig config{};
  config.horizontal_radius_m = -1.0;

  EXPECT_FALSE(lidarSelfFilterConfigIsValid(config));
  EXPECT_FALSE(isLidarSelfReturn(Point3{}, config));
  EXPECT_FALSE(
      isLidarSelfReturn(Point3{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
                        LidarSelfFilterConfig{}));
}

} // namespace
} // namespace drone_city_nav
