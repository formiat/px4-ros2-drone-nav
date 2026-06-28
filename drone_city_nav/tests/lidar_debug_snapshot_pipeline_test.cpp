#include "drone_city_nav/lidar_debug_snapshot_pipeline.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] LidarBeamProjection
acceptedHitProjection(std::size_t beam_index, float raw_range, const void* context) {
  (void)context;
  LidarBeamProjection projection;
  projection.status = LidarBeamProjectionStatus::kAccepted;
  projection.hit = true;
  projection.used_range_m = raw_range;
  projection.endpoint = Point2{static_cast<double>(beam_index), raw_range};
  projection.endpoint_altitude_m = 2.0;
  return projection;
}

[[nodiscard]] LidarBeamProjection altitudeRejectedProjection(std::size_t beam_index,
                                                             float raw_range,
                                                             const void* context) {
  (void)beam_index;
  (void)context;
  LidarBeamProjection projection;
  projection.status = LidarBeamProjectionStatus::kAltitudeRejected;
  projection.used_range_m = raw_range;
  projection.endpoint = Point2{1.0, 2.0};
  projection.endpoint_altitude_m = -1.0;
  return projection;
}

} // namespace

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

TEST(LidarDebugSnapshotPipeline, ReportsNoScanOrPoseReadiness) {
  const LidarDebugSnapshotOutput no_scan = buildLidarDebugSnapshotOutput(
      LidarDebugSnapshotInput{}, acceptedHitProjection, nullptr);
  EXPECT_FALSE(no_scan.ready);
  EXPECT_EQ(no_scan.readiness, LidarDebugSnapshotReadiness::kNoScan);

  const std::vector<float> ranges{1.0F};
  LidarDebugSnapshotInput input;
  input.scan_seen = true;
  input.ranges = ranges;
  const LidarDebugSnapshotOutput no_pose =
      buildLidarDebugSnapshotOutput(input, acceptedHitProjection, nullptr);
  EXPECT_FALSE(no_pose.ready);
  EXPECT_EQ(no_pose.readiness, LidarDebugSnapshotReadiness::kNoPose);
}

TEST(LidarDebugSnapshotPipeline, CountsAcceptedHitRowsAndRememberedHits) {
  const std::vector<float> ranges{3.0F, 4.0F};
  const LidarDebugSnapshotOutput output = buildLidarDebugSnapshotOutput(
      LidarDebugSnapshotInput{true, true, true, ranges, 0.1, 10.0, 0.0, 0.5, 1U, 17U},
      acceptedHitProjection, nullptr);

  ASSERT_TRUE(output.ready);
  EXPECT_EQ(output.readiness, LidarDebugSnapshotReadiness::kReady);
  EXPECT_EQ(output.rows.size(), 2U);
  EXPECT_EQ(output.stats.processed_beams, 2U);
  EXPECT_EQ(output.stats.accepted_beams, 2U);
  EXPECT_EQ(output.stats.hit_beams, 2U);
  EXPECT_EQ(output.stats.hit_points.size(), 2U);
  EXPECT_EQ(output.hit_points.size(), 2U);
  EXPECT_EQ(output.remembered_hits_count, 17U);
  EXPECT_DOUBLE_EQ(output.rows[1U].angle_rad, 0.5);
}

TEST(LidarDebugSnapshotPipeline, CountsAltitudeRejectedBeams) {
  const std::vector<float> ranges{3.0F};
  const LidarDebugSnapshotOutput output = buildLidarDebugSnapshotOutput(
      LidarDebugSnapshotInput{true, true, true, ranges, 0.1, 10.0, 0.0, 0.5, 1U, 0U},
      altitudeRejectedProjection, nullptr);

  ASSERT_TRUE(output.ready);
  ASSERT_EQ(output.rows.size(), 1U);
  EXPECT_EQ(output.stats.processed_beams, 1U);
  EXPECT_EQ(output.stats.accepted_beams, 0U);
  EXPECT_EQ(output.stats.altitude_rejected_beams, 1U);
  EXPECT_EQ(output.stats.hit_beams, 0U);
  EXPECT_EQ(output.rows.front().status, LidarBeamProjectionStatus::kAltitudeRejected);
  EXPECT_DOUBLE_EQ(output.rows.front().end_x_m, 1.0);
}

} // namespace drone_city_nav
