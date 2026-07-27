#include "drone_city_nav/lidar_debug_node_config.hpp"
#include "drone_city_nav/navigation_pose.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <numbers>

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

TEST(LidarDebugHeadingGate, UsesKnownMapHeadingUntilPx4HeadingAligns) {
  MappingYawTracker tracker{true, std::numbers::pi / 2.0, 0.15};

  const MappingYawSelection unavailable_px4 = tracker.update(false, 1.9);
  EXPECT_TRUE(unavailable_px4.valid);
  EXPECT_EQ(unavailable_px4.source, MappingYawSource::kInitialMapHeading);
  EXPECT_NEAR(unavailable_px4.yaw_rad, std::numbers::pi / 2.0, 1.0e-9);

  const MappingYawSelection misaligned_px4 = tracker.update(true, 1.82);
  EXPECT_TRUE(misaligned_px4.valid);
  EXPECT_EQ(misaligned_px4.source, MappingYawSource::kInitialMapHeading);
  EXPECT_FALSE(tracker.px4Aligned());

  const MappingYawSelection aligned_px4 = tracker.update(true, 1.68);
  EXPECT_TRUE(aligned_px4.valid);
  EXPECT_EQ(aligned_px4.source, MappingYawSource::kPx4Heading);
  EXPECT_NEAR(aligned_px4.yaw_rad, 1.68, 1.0e-9);
  EXPECT_TRUE(tracker.px4Aligned());
}

TEST(LidarDebugHeadingGate, RejectsInvalidPx4HeadingAfterHandoff) {
  MappingYawTracker tracker{true, std::numbers::pi / 2.0, 0.15};
  ASSERT_TRUE(tracker.update(true, 1.60).valid);

  const MappingYawSelection selection = tracker.update(false, 1.60);

  EXPECT_FALSE(selection.valid);
  EXPECT_EQ(selection.source, MappingYawSource::kUnavailable);
}

} // namespace drone_city_nav
