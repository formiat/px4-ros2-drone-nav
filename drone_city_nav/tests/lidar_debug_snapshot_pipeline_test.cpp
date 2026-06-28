#include "drone_city_nav/lidar_debug_snapshot_pipeline.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>

namespace drone_city_nav {

TEST(LidarDebugSnapshotPipeline, BuildsStableSnapshotPrefix) {
  EXPECT_EQ(lidarSnapshotPrefix(1U), "snapshot_000001");
  EXPECT_EQ(lidarSnapshotPrefix(123456U), "snapshot_123456");
}

TEST(LidarDebugSnapshotPipeline, ComputesAgesAndYawDelta) {
  EXPECT_DOUBLE_EQ(ageSecondsOrNan(1'000'000'000LL, 2'250'000'000LL), 1.25);
  EXPECT_DOUBLE_EQ(ageSecondsOrNan(3'000'000'000LL, 2'000'000'000LL), 0.0);
  EXPECT_TRUE(std::isnan(ageSecondsOrNan(0, 2'000'000'000LL)));

  EXPECT_NEAR(yawDeltaRad(std::numbers::pi, -std::numbers::pi), 0.0, 1.0e-12);
  EXPECT_TRUE(std::isnan(yawDeltaRad(std::numeric_limits<double>::quiet_NaN(), 0.0)));
}

TEST(LidarDebugSnapshotPipeline, ComputesScanTiming) {
  EXPECT_DOUBLE_EQ(lidarScanDurationSeconds(0.1, 0.02, 10U, 0.3), 0.3);
  EXPECT_DOUBLE_EQ(lidarScanDurationSeconds(0.1, 0.02, 10U, 0.0), 0.1);
  EXPECT_DOUBLE_EQ(lidarScanDurationSeconds(0.0, 0.02, 6U, 0.0), 0.1);
  EXPECT_DOUBLE_EQ(lidarScanDurationSeconds(0.0, 0.0, 1U, 0.0), 0.0);

  EXPECT_DOUBLE_EQ(lidarScanTimeIncrementSeconds(0.1, 0.02, 6U), 0.02);
  EXPECT_DOUBLE_EQ(lidarScanTimeIncrementSeconds(0.1, 0.0, 6U), 0.02);
  EXPECT_DOUBLE_EQ(lidarScanTimeIncrementSeconds(0.0, 0.0, 6U), 0.0);
}

} // namespace drone_city_nav
