#include "drone_city_nav/lidar_debug_node_config.hpp"
#include "drone_city_nav/navigation_pose.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace drone_city_nav {

TEST(LidarDebugHeadingGate, RequiresConfiguredPx4HeadingBeforeProjection) {
  EXPECT_FALSE(lidarDebugProjectionHeadingReady(true, false));
  EXPECT_TRUE(lidarDebugProjectionHeadingReady(true, true));
  EXPECT_TRUE(lidarDebugProjectionHeadingReady(false, false));
}

TEST(LidarDebugHeadingGate, RequiresBoundedEstimatorVariance) {
  EXPECT_FALSE(px4HeadingReadyForMapping(true, 1.82, 0.04, 0.01));
  EXPECT_TRUE(px4HeadingReadyForMapping(true, 1.64, 0.0007, 0.01));
  EXPECT_FALSE(px4HeadingReadyForMapping(false, 1.64, 0.0007, 0.01));
  EXPECT_FALSE(px4HeadingReadyForMapping(
      true, 1.64, std::numeric_limits<double>::quiet_NaN(), 0.01));
  EXPECT_FALSE(px4HeadingReadyForMapping(true, std::numeric_limits<double>::quiet_NaN(),
                                         0.0007, 0.01));
}

} // namespace drone_city_nav
