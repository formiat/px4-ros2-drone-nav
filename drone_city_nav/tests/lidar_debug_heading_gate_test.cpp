#include "drone_city_nav/lidar_debug_node_config.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {

TEST(LidarDebugHeadingGate, RequiresConfiguredPx4HeadingBeforeProjection) {
  EXPECT_FALSE(lidarDebugProjectionHeadingReady(true, false));
  EXPECT_TRUE(lidarDebugProjectionHeadingReady(true, true));
  EXPECT_TRUE(lidarDebugProjectionHeadingReady(false, false));
}

} // namespace drone_city_nav
