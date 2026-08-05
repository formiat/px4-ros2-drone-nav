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

TEST(LidarDebugHeadingGate, RequiresStablePx4SamplesWithoutInitialHeadingAlignment) {
  MappingYawTracker tracker{true, std::numbers::pi / 2.0, 3U, 0.05};

  const MappingYawSelection unavailable_px4 = tracker.update(false, 1.9);
  EXPECT_FALSE(unavailable_px4.valid);
  EXPECT_EQ(unavailable_px4.source, MappingYawSource::kUnavailable);

  EXPECT_FALSE(tracker.update(true, 1.82).valid);
  EXPECT_FALSE(tracker.update(true, 1.84).valid);
  EXPECT_EQ(tracker.stableSampleCount(), 2U);
  EXPECT_FALSE(tracker.px4Stable());

  const MappingYawSelection stable_px4 = tracker.update(true, 1.83);
  EXPECT_TRUE(stable_px4.valid);
  EXPECT_EQ(stable_px4.source, MappingYawSource::kPx4Heading);
  EXPECT_NEAR(stable_px4.yaw_rad, 1.83, 1.0e-9);
  EXPECT_TRUE(tracker.px4Stable());
}

TEST(LidarDebugHeadingGate, UsesInitialMapHeadingOnlyWhenPx4HeadingIsDisabled) {
  MappingYawTracker tracker{false, std::numbers::pi / 2.0, 3U, 0.05};

  const MappingYawSelection selection = tracker.update(false, 1.9);

  EXPECT_TRUE(selection.valid);
  EXPECT_EQ(selection.source, MappingYawSource::kInitialMapHeading);
  EXPECT_NEAR(selection.yaw_rad, std::numbers::pi / 2.0, 1.0e-9);
  EXPECT_FALSE(tracker.px4Stable());
}

TEST(LidarDebugHeadingGate, RejectsInvalidPx4HeadingAfterHandoff) {
  MappingYawTracker tracker{true, std::numbers::pi / 2.0, 2U, 0.05};
  ASSERT_FALSE(tracker.update(true, 1.60).valid);
  ASSERT_TRUE(tracker.update(true, 1.61).valid);

  const MappingYawSelection selection = tracker.update(false, 1.60);

  EXPECT_FALSE(selection.valid);
  EXPECT_EQ(selection.source, MappingYawSource::kUnavailable);
  EXPECT_FALSE(tracker.px4Stable());
}

TEST(LidarDebugHeadingGate, RequiresFreshAlignmentAfterReset) {
  MappingYawTracker tracker{true, std::numbers::pi / 2.0, 2U, 0.05};
  ASSERT_FALSE(tracker.update(true, 1.60).valid);
  ASSERT_TRUE(tracker.update(true, 1.61).valid);

  tracker.reset();
  const MappingYawSelection selection = tracker.update(false, 1.60);

  EXPECT_FALSE(selection.valid);
  EXPECT_EQ(selection.source, MappingYawSource::kUnavailable);
  EXPECT_FALSE(tracker.px4Stable());
}

TEST(LidarDebugHeadingGate, ValidTurnAfterHandoffKeepsPx4HeadingAvailable) {
  MappingYawTracker tracker{true, std::numbers::pi / 2.0, 2U, 0.05};
  EXPECT_FALSE(tracker.update(true, 1.80).valid);
  ASSERT_TRUE(tracker.update(true, 1.81).valid);

  const MappingYawSelection turn = tracker.update(true, 2.10);
  EXPECT_TRUE(turn.valid);
  EXPECT_NEAR(turn.yaw_rad, 2.10, 1.0e-9);
  EXPECT_TRUE(tracker.px4Stable());
}

TEST(LidarDebugHeadingGate, LossOfValidityRequiresFreshStableSamples) {
  MappingYawTracker tracker{true, std::numbers::pi / 2.0, 2U, 0.05};
  EXPECT_FALSE(tracker.update(true, 1.80).valid);
  ASSERT_TRUE(tracker.update(true, 1.81).valid);
  ASSERT_FALSE(tracker.update(false, 1.81).valid);

  EXPECT_FALSE(tracker.update(true, 2.10).valid);
  EXPECT_FALSE(tracker.px4Stable());
  EXPECT_TRUE(tracker.update(true, 2.11).valid);
}

} // namespace drone_city_nav
